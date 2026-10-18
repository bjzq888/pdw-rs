//! Built-In Live HTTP + Server-Sent Events (SSE) Web Server, Browser Caching Engine,
//! Parallel Frequency Monitor & Capcode Catalog API (`web.rs`).
//!
//! Serves a real-time dark-mode paging monitor web application on port `8008` by default
//! (configurable and toggleable at runtime in the GUI and `pdw_config.toml`).
//!
//! Speed, Mobile & Browser Caching Architecture:
//! 1. **Instant `localStorage` Hydration**: The browser caches up to `N` messages locally
//!    (`pdw_rs_msg_cache_v1`) along with the server `session_id` and `last_seq` watermark,
//!    rendering cached messages in `< 5 ms` before network round-trips complete.
//! 2. **Delta-Only Bootstrap Sync (`/api/bootstrap?session_id=<sid>&since_seq=<seq>`)**:
//!    When reloading or reconnecting, the browser requests only messages with `seq > last_seq`
//!    (`delta_mode: true`), eliminating redundant transfers of already-cached messages.
//! 3. **Sub-Millisecond Server-Sent Events (`/api/events`)**: Newly decoded POCSAG & FLEX pages
//!    (including multi-fragment reassembled dispatches, RF carrier frequency, and FLEX groupcall
//!    temporary addresses) push immediately over a persistent `text/event-stream` connection.
//! 4. **Responsive Mobile Card Layout & Touch Inspector**: Automatically switches to a
//!    multi-line wrapped Dispatch Card feed on mobile viewports (`<= 820px`, or when toggled
//!    manually) with touch-safe DOM updates, full-row/card tap delegation, and a slide-up
//!    Capcode Directory & Message Inspector sheet.
//! 5. **Capcode Catalog & Parallel Frequency Endpoints**: Exposes `/api/catalog`, `/api/catalog/detail`,
//!    `/api/catalog/delete`, `/api/catalog/clear`, `/api/catalog/alias`, and `/api/frequencies/*`.

use crate::catalog::CapcodeCatalogDb;
use crate::config::{MonitoredFrequency, StatsTracker};
use crate::pocsag::DecodedPagerMessage;
use crossbeam_channel::{bounded, Receiver, Sender};
use eframe::egui;
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub const DEFAULT_WEB_PORT: u16 = 8008;
pub const DEFAULT_WEB_BIND_ADDR: &str = "0.0.0.0";
pub const MAX_SERVER_CACHED_MESSAGES: usize = 2000;

#[derive(Clone, Debug)]
pub struct WebMessageEntry {
    pub seq: u64,
    pub timestamp_ms: i64,
    pub in_monitor: bool,
    pub in_filtered: bool,
    pub msg: DecodedPagerMessage,
}

impl WebMessageEntry {
    pub fn to_json(&self) -> String {
        let label_json = match &self.msg.filter_label {
            Some(l) => format!("\"{}\"", escape_json_str(l)),
            None => "null".to_string(),
        };
        let color_json = match self.msg.filter_color_idx {
            Some(c) => format!("{}", c),
            None => "null".to_string(),
        };
        let freq_json = match self.msg.freq_hz {
            Some(hz) if hz > 0 => format!("{}", hz),
            _ => "null".to_string(),
        };
        let group_json = match &self.msg.group_address {
            Some(g) => format!("\"{}\"", escape_json_str(g)),
            None => "null".to_string(),
        };
        let frag_json = match &self.msg.fragment_info {
            Some(f) => format!("\"{}\"", escape_json_str(f)),
            None => "null".to_string(),
        };
        format!(
            "{{\"seq\":{},\"ts\":{},\"mon\":{},\"flt\":{},\"capcode\":\"{}\",\"time\":\"{}\",\"date\":\"{}\",\"mode\":\"{}\",\"msg_type\":\"{}\",\"bitrate\":{},\"message\":\"{}\",\"has_bit_errors\":{},\"soft_recovered_words\":{},\"filter_label\":{},\"filter_color_idx\":{},\"freq_hz\":{},\"group_address\":{},\"fragment_info\":{}}}",
            self.seq,
            self.timestamp_ms,
            self.in_monitor,
            self.in_filtered,
            escape_json_str(&self.msg.capcode),
            escape_json_str(&self.msg.time),
            escape_json_str(&self.msg.date),
            escape_json_str(&self.msg.mode),
            escape_json_str(&self.msg.msg_type),
            self.msg.bitrate,
            escape_json_str(&self.msg.message),
            self.msg.has_bit_errors,
            self.msg.soft_recovered_words,
            label_json,
            color_json,
            freq_json,
            group_json,
            frag_json
        )
    }
}

#[derive(Clone, Debug)]
pub struct WebTelemetrySnapshot {
    pub rx_quality_pct: f64,
    pub total_codewords: u64,
    pub soft_recoveries: u64,
    pub active_mode: String,
    pub dc_center: f32,
    pub deviation: f32,
    pub invert_active: bool,
    pub input_source: String,
    pub audio_status: String,
    pub sdr_connected: bool,
    pub vfo_freq_hz: Option<u64>,
    pub iq_rssi_dbfs: Option<f32>,
    pub afc_offset_hz: Option<f32>,
    pub paused: bool,
    pub monitored_frequencies: Vec<MonitoredFrequency>,
    pub active_span_hz: u64,
    pub catalog_capcode_count: usize,
}

impl Default for WebTelemetrySnapshot {
    fn default() -> Self {
        Self {
            rx_quality_pct: 100.0,
            total_codewords: 0,
            soft_recoveries: 0,
            active_mode: "IDLE (48000 Hz)".to_string(),
            dc_center: 0.0,
            deviation: 0.5,
            invert_active: false,
            input_source: "Soundcard".to_string(),
            audio_status: "Initializing...".to_string(),
            sdr_connected: false,
            vfo_freq_hz: None,
            iq_rssi_dbfs: None,
            afc_offset_hz: None,
            paused: false,
            monitored_frequencies: Vec::new(),
            active_span_hz: 0,
            catalog_capcode_count: 0,
        }
    }
}

impl WebTelemetrySnapshot {
    pub fn to_json(&self) -> String {
        let vfo_json = self
            .vfo_freq_hz
            .map(|v| v.to_string())
            .unwrap_or_else(|| "null".to_string());
        let rssi_json = self
            .iq_rssi_dbfs
            .map(|v| format!("{:.1}", v))
            .unwrap_or_else(|| "null".to_string());
        let afc_json = self
            .afc_offset_hz
            .map(|v| format!("{:.1}", v))
            .unwrap_or_else(|| "null".to_string());
        let freqs_json = self
            .monitored_frequencies
            .iter()
            .map(|f| {
                format!(
                    "{{\"enabled\":{},\"freq_hz\":{},\"label\":\"{}\"}}",
                    f.enabled,
                    f.freq_hz,
                    escape_json_str(&f.label)
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        format!(
            "{{\"rx_quality_pct\":{:.1},\"total_codewords\":{},\"soft_recoveries\":{},\"active_mode\":\"{}\",\"dc_center\":{:.4},\"deviation\":{:.4},\"invert_active\":{},\"input_source\":\"{}\",\"audio_status\":\"{}\",\"sdr_connected\":{},\"vfo_freq_hz\":{},\"iq_rssi_dbfs\":{},\"afc_offset_hz\":{},\"paused\":{},\"active_span_hz\":{},\"catalog_capcode_count\":{},\"monitored_frequencies\":[{}]}}",
            self.rx_quality_pct.clamp(0.0, 100.0),
            self.total_codewords,
            self.soft_recoveries,
            escape_json_str(&self.active_mode),
            self.dc_center,
            self.deviation,
            self.invert_active,
            escape_json_str(&self.input_source),
            escape_json_str(&self.audio_status),
            self.sdr_connected,
            vfo_json,
            rssi_json,
            afc_json,
            self.paused,
            self.active_span_hz,
            self.catalog_capcode_count,
            freqs_json
        )
    }
}

fn stats_to_json(stats: &StatsTracker) -> String {
    format!(
        "{{\"total\":{},\"pocsag_512_alpha\":{},\"pocsag_512_num\":{},\"pocsag_1200_alpha\":{},\"pocsag_1200_num\":{},\"pocsag_2400_alpha\":{},\"pocsag_2400_num\":{},\"flex_1600_alpha\":{},\"flex_1600_num\":{},\"flex_3200_alpha\":{},\"flex_3200_num\":{},\"flex_6400_alpha\":{},\"flex_6400_num\":{},\"rejected\":{},\"duplicates\":{},\"encrypted_hidden\":{}}}",
        stats.total_messages(),
        stats.pocsag_512_alpha,
        stats.pocsag_512_num,
        stats.pocsag_1200_alpha,
        stats.pocsag_1200_num,
        stats.pocsag_2400_alpha,
        stats.pocsag_2400_num,
        stats.flex_1600_alpha,
        stats.flex_1600_num,
        stats.flex_3200_alpha,
        stats.flex_3200_num,
        stats.flex_6400_alpha,
        stats.flex_6400_num,
        stats.rejected_count,
        stats.blocked_duplicates,
        stats.encrypted_hidden_count
    )
}

#[derive(Clone, Debug)]
pub enum WebUiCommand {
    TriggerTestPocsag,
    TriggerTestFlex,
    TogglePause,
    ClearMessages,
    ToggleFrequency(usize),
    AddFrequency { freq_hz: u64, label: String },
    RemoveFrequency(usize),
}

struct WebStateInner {
    messages: VecDeque<WebMessageEntry>,
    telemetry: WebTelemetrySnapshot,
    stats: StatsTracker,
    sse_clients: Vec<Sender<String>>,
    last_status_broadcast: Instant,
    last_broadcast_total: u64,
    last_broadcast_catalog_count: usize,
}

pub struct WebServerSharedState {
    pub session_id: u64,
    next_seq: AtomicU64,
    inner: Mutex<WebStateInner>,
    pub catalog_db: Arc<Mutex<CapcodeCatalogDb>>,
    ui_cmd_tx: Sender<WebUiCommand>,
    egui_ctx: Option<egui::Context>,
}

impl WebServerSharedState {
    pub fn new(
        catalog_db: Arc<Mutex<CapcodeCatalogDb>>,
        egui_ctx: Option<egui::Context>,
    ) -> (Arc<Self>, Receiver<WebUiCommand>) {
        let (ui_cmd_tx, ui_cmd_rx) = bounded::<WebUiCommand>(32);
        let session_id = (chrono::Local::now().timestamp_millis().unsigned_abs()).max(1);
        let state = Arc::new(Self {
            session_id,
            next_seq: AtomicU64::new(1),
            inner: Mutex::new(WebStateInner {
                messages: VecDeque::with_capacity(MAX_SERVER_CACHED_MESSAGES),
                telemetry: WebTelemetrySnapshot::default(),
                stats: StatsTracker::default(),
                sse_clients: Vec::new(),
                last_status_broadcast: Instant::now(),
                last_broadcast_total: 0,
                last_broadcast_catalog_count: 0,
            }),
            catalog_db,
            ui_cmd_tx,
            egui_ctx,
        });
        (state, ui_cmd_rx)
    }

    pub fn latest_seq(&self) -> u64 {
        self.next_seq.load(Ordering::SeqCst).saturating_sub(1)
    }

    pub fn record_message(
        &self,
        msg: &DecodedPagerMessage,
        in_monitor: bool,
        in_filtered: bool,
    ) -> u64 {
        let seq = self.next_seq.fetch_add(1, Ordering::SeqCst);
        let timestamp_ms = chrono::Local::now().timestamp_millis();
        let entry = WebMessageEntry {
            seq,
            timestamp_ms,
            in_monitor,
            in_filtered,
            msg: msg.clone(),
        };
        let entry_json = entry.to_json();
        let sse_frame = format!(
            "event: message\ndata: {{\"session_id\":{},\"entry\":{}}}\n\n",
            self.session_id, entry_json
        );

        if let Ok(mut guard) = self.inner.lock() {
            if guard.messages.len() >= MAX_SERVER_CACHED_MESSAGES {
                guard.messages.pop_front();
            }
            guard.messages.push_back(entry);
            guard
                .sse_clients
                .retain(|tx| tx.try_send(sse_frame.clone()).is_ok());
        }
        seq
    }

    pub fn clear_messages(&self) {
        let sse_frame = format!(
            "event: clear\ndata: {{\"session_id\":{}}}\n\n",
            self.session_id
        );
        if let Ok(mut guard) = self.inner.lock() {
            guard.messages.clear();
            guard
                .sse_clients
                .retain(|tx| tx.try_send(sse_frame.clone()).is_ok());
        }
    }

    pub fn notify_catalog_updated(&self) {
        let count = self
            .catalog_db
            .lock()
            .map(|db| db.total_capcodes())
            .unwrap_or(0);
        let sse_frame = format!(
            "event: catalog_updated\ndata: {{\"total_capcodes\":{}}}\n\n",
            count
        );
        if let Ok(mut guard) = self.inner.lock() {
            guard.telemetry.catalog_capcode_count = count;
            guard
                .sse_clients
                .retain(|tx| tx.try_send(sse_frame.clone()).is_ok());
        }
    }

    pub fn update_telemetry(&self, telem: WebTelemetrySnapshot, stats: &StatsTracker) {
        let latest = self.latest_seq();
        if let Ok(mut guard) = self.inner.lock() {
            let mode_changed = guard.telemetry.active_mode != telem.active_mode;
            let total_msgs = stats.total_messages();
            let count_changed = guard.last_broadcast_total != total_msgs
                || guard.last_broadcast_catalog_count != telem.catalog_capcode_count
                || guard.telemetry.monitored_frequencies != telem.monitored_frequencies;
            let elapsed_ok = guard.last_status_broadcast.elapsed() >= Duration::from_millis(500);

            guard.telemetry = telem;
            guard.stats = stats.clone();

            if !guard.sse_clients.is_empty() && (mode_changed || count_changed || elapsed_ok) {
                guard.last_status_broadcast = Instant::now();
                guard.last_broadcast_total = total_msgs;
                guard.last_broadcast_catalog_count = guard.telemetry.catalog_capcode_count;
                let sse_frame = format!(
                    "event: status\ndata: {{\"session_id\":{},\"latest_seq\":{},\"telemetry\":{},\"stats\":{}}}\n\n",
                    self.session_id,
                    latest,
                    guard.telemetry.to_json(),
                    stats_to_json(&guard.stats)
                );
                guard
                    .sse_clients
                    .retain(|tx| tx.try_send(sse_frame.clone()).is_ok());
            }
        }
    }

    pub fn active_sse_client_count(&self) -> usize {
        self.inner
            .lock()
            .map(|mut g| {
                g.sse_clients
                    .retain(|tx| tx.try_send(": ping\n\n".to_string()).is_ok());
                g.sse_clients.len()
            })
            .unwrap_or(0)
    }

    pub fn close_all_sse_clients(&self) {
        if let Ok(mut guard) = self.inner.lock() {
            guard.sse_clients.clear();
        }
    }

    pub fn register_sse_client(&self) -> (Receiver<String>, String) {
        let (tx, rx) = bounded::<String>(256);
        let initial_frame = if let Ok(mut guard) = self.inner.lock() {
            if guard.sse_clients.len() >= 64 {
                guard.sse_clients.remove(0);
            }
            guard.sse_clients.push(tx);
            format!(
                "event: status\ndata: {{\"session_id\":{},\"latest_seq\":{},\"telemetry\":{},\"stats\":{}}}\n\n",
                self.session_id,
                self.latest_seq(),
                guard.telemetry.to_json(),
                stats_to_json(&guard.stats)
            )
        } else {
            String::new()
        };
        (rx, initial_frame)
    }

    pub fn dispatch_ui_command(&self, cmd: WebUiCommand) {
        let _ = self.ui_cmd_tx.try_send(cmd);
        if let Some(ctx) = &self.egui_ctx {
            ctx.request_repaint();
        }
    }

    pub fn build_bootstrap_json(&self, client_session_id: Option<u64>, since_seq: u64) -> String {
        let latest = self.latest_seq();
        if let Ok(guard) = self.inner.lock() {
            let same_session = client_session_id == Some(self.session_id) && since_seq > 0;
            let ring_covers_since = guard
                .messages
                .front()
                .map(|first| first.seq <= since_seq.saturating_add(1))
                .unwrap_or(latest <= since_seq);
            let delta_mode = same_session && ring_covers_since;

            let mut msgs_json = String::with_capacity(4096);
            msgs_json.push('[');
            let mut first = true;
            for entry in &guard.messages {
                if delta_mode && entry.seq <= since_seq {
                    continue;
                }
                if !first {
                    msgs_json.push(',');
                }
                first = false;
                msgs_json.push_str(&entry.to_json());
            }
            msgs_json.push(']');

            format!(
                "{{\"session_id\":{},\"latest_seq\":{},\"delta_mode\":{},\"telemetry\":{},\"stats\":{},\"messages\":{}}}",
                self.session_id,
                latest,
                delta_mode,
                guard.telemetry.to_json(),
                stats_to_json(&guard.stats),
                msgs_json
            )
        } else {
            format!(
                "{{\"session_id\":{},\"latest_seq\":{},\"delta_mode\":false,\"messages\":[]}}",
                self.session_id, latest
            )
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WebServerStatus {
    pub running: bool,
    pub bind_addr: String,
    pub port: u16,
    pub error: Option<String>,
    pub requests_served: u64,
}

pub struct WebServerHandle {
    pub bind_addr: String,
    pub port: u16,
    stop_flag: Arc<AtomicBool>,
    shared: Arc<WebServerSharedState>,
    pub status: Arc<Mutex<WebServerStatus>>,
    worker: Option<JoinHandle<()>>,
}

impl WebServerHandle {
    pub fn start(bind_addr: &str, port: u16, shared: Arc<WebServerSharedState>) -> Self {
        let clean_bind = if bind_addr.trim().is_empty() {
            DEFAULT_WEB_BIND_ADDR.to_string()
        } else {
            bind_addr.trim().to_string()
        };
        let stop_flag = Arc::new(AtomicBool::new(false));
        let status = Arc::new(Mutex::new(WebServerStatus {
            running: false,
            bind_addr: clean_bind.clone(),
            port,
            error: None,
            requests_served: 0,
        }));

        let stop_clone = Arc::clone(&stop_flag);
        let status_clone = Arc::clone(&status);
        let shared_clone = Arc::clone(&shared);
        let bind_clone = clean_bind.clone();

        let worker = thread::Builder::new()
            .name(format!("pdw-web-{}", port))
            .spawn(move || {
                run_http_listener(bind_clone, port, stop_clone, status_clone, shared_clone);
            })
            .ok();

        Self {
            bind_addr: clean_bind,
            port,
            stop_flag,
            shared,
            status,
            worker,
        }
    }

    pub fn stop(&mut self) {
        self.stop_flag.store(true, Ordering::SeqCst);
        self.shared.close_all_sse_clients();
        let wake_addr = SocketAddr::from(([127, 0, 0, 1], self.port));
        let _ = TcpStream::connect_timeout(&wake_addr, Duration::from_millis(50));
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
        if let Ok(mut st) = self.status.lock() {
            st.running = false;
        }
    }

    pub fn display_url(&self) -> String {
        let host = if self.bind_addr == "0.0.0.0" || self.bind_addr.is_empty() {
            "localhost"
        } else {
            &self.bind_addr
        };
        format!("http://{}:{}/", host, self.port)
    }
}

impl Drop for WebServerHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

fn run_http_listener(
    bind_addr: String,
    port: u16,
    stop_flag: Arc<AtomicBool>,
    status: Arc<Mutex<WebServerStatus>>,
    shared: Arc<WebServerSharedState>,
) {
    let target = format!("{}:{}", bind_addr, port);
    let listener = match TcpListener::bind(&target)
        .or_else(|_| TcpListener::bind(("127.0.0.1", port)))
    {
        Ok(l) => l,
        Err(e) => {
            if let Ok(mut st) = status.lock() {
                st.running = false;
                st.error = Some(format!("Failed to bind port {}: {}", port, e));
            }
            return;
        }
    };

    if let Ok(mut st) = status.lock() {
        st.running = true;
        st.error = None;
    }

    for stream_res in listener.incoming() {
        if stop_flag.load(Ordering::Relaxed) {
            break;
        }
        match stream_res {
            Ok(stream) => {
                if stop_flag.load(Ordering::Relaxed) {
                    break;
                }
                if let Ok(mut st) = status.try_lock() {
                    st.requests_served = st.requests_served.wrapping_add(1);
                }
                let shared_conn = Arc::clone(&shared);
                let stop_conn = Arc::clone(&stop_flag);
                thread::spawn(move || {
                    handle_http_connection(stream, shared_conn, stop_conn);
                });
            }
            Err(_) => {
                if stop_flag.load(Ordering::Relaxed) {
                    break;
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
    }

    if let Ok(mut st) = status.lock() {
        st.running = false;
    }
}

fn handle_http_connection(
    mut stream: TcpStream,
    shared: Arc<WebServerSharedState>,
    stop_flag: Arc<AtomicBool>,
) {
    let _ = stream.set_nodelay(true);
    let _ = stream.set_read_timeout(Some(Duration::from_millis(1500)));
    let _ = stream.set_write_timeout(Some(Duration::from_millis(2000)));

    let mut req_buf = [0u8; 4096];
    let n = match stream.read(&mut req_buf) {
        Ok(0) | Err(_) => return,
        Ok(n) => n,
    };

    let req_str = String::from_utf8_lossy(&req_buf[..n]);
    let first_line = match req_str.lines().next() {
        Some(l) => l.trim(),
        None => return,
    };

    let mut parts = first_line.split_whitespace();
    let method = parts.next().unwrap_or("GET");
    let raw_target = parts.next().unwrap_or("/");
    let (path, query) = match raw_target.split_once('?') {
        Some((p, q)) => (p, q),
        None => (raw_target, ""),
    };

    match (method, path) {
        ("GET", "/") | ("GET", "/index.html") => {
            write_http_response(
                &mut stream,
                "200 OK",
                "text/html; charset=utf-8",
                "no-cache",
                INDEX_HTML.as_bytes(),
            );
        }
        ("GET", "/api/bootstrap") | ("GET", "/api/messages") => {
            let session_id = parse_query_u64(query, "session_id");
            let since_seq = parse_query_u64(query, "since_seq").unwrap_or(0);
            let body = shared.build_bootstrap_json(session_id, since_seq);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                body.as_bytes(),
            );
        }
        ("GET", "/api/catalog") => {
            let body = shared
                .catalog_db
                .lock()
                .map(|db| db.all_summaries_to_json())
                .unwrap_or_else(|_| "{\"total_capcodes\":0,\"profiles\":[]}".to_string());
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                body.as_bytes(),
            );
        }
        ("GET", "/api/catalog/detail") => {
            let capcode = parse_query_str(query, "capcode").unwrap_or_default();
            let body = shared
                .catalog_db
                .lock()
                .map(|db| db.capcode_detail_to_json(&capcode))
                .unwrap_or_else(|_| "{\"profile\":null,\"messages\":[]}".to_string());
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                body.as_bytes(),
            );
        }
        ("POST", "/api/catalog/delete") => {
            let capcode = parse_query_str(query, "capcode").unwrap_or_default();
            let removed = shared
                .catalog_db
                .lock()
                .map(|mut db| db.remove_capcode(&capcode))
                .unwrap_or(false);
            shared.notify_catalog_updated();
            if let Some(ctx) = &shared.egui_ctx {
                ctx.request_repaint();
            }
            let body = format!("{{\"ok\":true,\"removed\":{}}}", removed);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                body.as_bytes(),
            );
        }
        ("POST", "/api/catalog/clear") => {
            if let Ok(mut db) = shared.catalog_db.lock() {
                db.clear_all();
            }
            shared.notify_catalog_updated();
            if let Some(ctx) = &shared.egui_ctx {
                ctx.request_repaint();
            }
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true}",
            );
        }
        ("POST", "/api/catalog/alias") => {
            let capcode = parse_query_str(query, "capcode").unwrap_or_default();
            let alias = parse_query_str(query, "alias").unwrap_or_default();
            let updated = shared
                .catalog_db
                .lock()
                .map(|mut db| db.set_alias(&capcode, &alias))
                .unwrap_or(false);
            shared.notify_catalog_updated();
            if let Some(ctx) = &shared.egui_ctx {
                ctx.request_repaint();
            }
            let body = format!("{{\"ok\":{}}}", updated);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                body.as_bytes(),
            );
        }
        ("POST", "/api/frequencies/toggle") => {
            if let Some(idx) = parse_query_u64(query, "idx") {
                shared.dispatch_ui_command(WebUiCommand::ToggleFrequency(idx as usize));
            }
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true}",
            );
        }
        ("POST", "/api/frequencies/add") => {
            if let Some(freq_hz) = parse_query_u64(query, "freq_hz") {
                let label = parse_query_str(query, "label").unwrap_or_default();
                shared.dispatch_ui_command(WebUiCommand::AddFrequency { freq_hz, label });
            }
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true}",
            );
        }
        ("POST", "/api/frequencies/remove") => {
            if let Some(idx) = parse_query_u64(query, "idx") {
                shared.dispatch_ui_command(WebUiCommand::RemoveFrequency(idx as usize));
            }
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true}",
            );
        }
        ("GET", "/api/events") => {
            let headers = concat!(
                "HTTP/1.1 200 OK\r\n",
                "Content-Type: text/event-stream; charset=utf-8\r\n",
                "Cache-Control: no-cache, no-store\r\n",
                "Connection: keep-alive\r\n",
                "Access-Control-Allow-Origin: *\r\n",
                "X-Accel-Buffering: no\r\n\r\n"
            );
            if stream.write_all(headers.as_bytes()).is_err() || stream.flush().is_err() {
                return;
            }

            let (rx, initial_frame) = shared.register_sse_client();
            if !initial_frame.is_empty()
                && (stream.write_all(initial_frame.as_bytes()).is_err() || stream.flush().is_err())
            {
                return;
            }

            while !stop_flag.load(Ordering::Relaxed) {
                match rx.recv_timeout(Duration::from_millis(2000)) {
                    Ok(frame) => {
                        if stream.write_all(frame.as_bytes()).is_err() || stream.flush().is_err() {
                            break;
                        }
                    }
                    Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                        if stream.write_all(b": keepalive\n\n").is_err() || stream.flush().is_err()
                        {
                            break;
                        }
                    }
                    Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                }
            }
        }
        ("POST", "/api/action/test_pocsag") => {
            shared.dispatch_ui_command(WebUiCommand::TriggerTestPocsag);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true,\"action\":\"test_pocsag\"}",
            );
        }
        ("POST", "/api/action/test_flex") => {
            shared.dispatch_ui_command(WebUiCommand::TriggerTestFlex);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true,\"action\":\"test_flex\"}",
            );
        }
        ("POST", "/api/action/toggle_pause") => {
            shared.dispatch_ui_command(WebUiCommand::TogglePause);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true,\"action\":\"toggle_pause\"}",
            );
        }
        ("POST", "/api/action/clear") => {
            shared.dispatch_ui_command(WebUiCommand::ClearMessages);
            write_http_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"ok\":true,\"action\":\"clear\"}",
            );
        }
        ("GET", "/favicon.ico") => {
            write_http_response(&mut stream, "204 No Content", "image/x-icon", "max-age=86400", b"");
        }
        _ => {
            write_http_response(
                &mut stream,
                "404 Not Found",
                "application/json; charset=utf-8",
                "no-store",
                b"{\"error\":\"Not found\"}",
            );
        }
    }
}

fn write_http_response(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    cache_control: &str,
    body: &[u8],
) {
    let header = format!(
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nCache-Control: {}\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n",
        status,
        content_type,
        body.len(),
        cache_control
    );
    let _ = stream.write_all(header.as_bytes());
    if !body.is_empty() {
        let _ = stream.write_all(body);
    }
    let _ = stream.flush();
}

fn parse_query_u64(query: &str, key: &str) -> Option<u64> {
    for pair in query.split('&') {
        if let Some((k, v)) = pair.split_once('=') {
            if k == key {
                if let Ok(val) = v.trim().parse::<u64>() {
                    return Some(val);
                }
            }
        }
    }
    None
}

fn parse_query_str(query: &str, key: &str) -> Option<String> {
    for pair in query.split('&') {
        if let Some((k, v)) = pair.split_once('=') {
            if k == key {
                return Some(url_decode(v.trim()));
            }
        }
    }
    None
}

fn url_decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'+' {
            out.push(b' ');
            i += 1;
        } else if bytes[i] == b'%' && i + 2 < bytes.len() {
            if let Ok(hex_str) = std::str::from_utf8(&bytes[i + 1..i + 3]) {
                if let Ok(b) = u8::from_str_radix(hex_str, 16) {
                    out.push(b);
                    i += 3;
                    continue;
                }
            }
            out.push(bytes[i]);
            i += 1;
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

pub fn escape_json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 8);
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                out.push_str(&format!("\\u{:04x}", c as u32));
            }
            c => out.push(c),
        }
    }
    out
}

/// Embedded single-page Live Web Monitor with local browser caching (`localStorage`),
/// incremental `since_seq` delta synchronization, live SSE streaming, Parallel Frequency
/// Manager (`<= 2.0 MHz` span), responsive Mobile Dispatch Card feed, and Capcode Catalog.
pub const INDEX_HTML: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=5.0, viewport-fit=cover" />
<title>PDW-RS Live Paging Monitor &amp; Capcode Catalog</title>
<style>
  :root {
    --bg-main: #0c1017;
    --bg-panel: #131924;
    --bg-header: #192231;
    --bg-row-alt: #101620;
    --bg-row-hover: #1e2a3d;
    --bg-row-sel: #24354f;
    --border: #263347;
    --text-main: #e6edf3;
    --text-muted: #8b9bb4;
    --capcode: #f5f7ff;
    --time-col: #6ebeff;
    --freq-col: #a5f3fc;
    --mode-col: #ff6e6e;
    --type-col: #ffb45a;
    --alpha-msg: #78ffff;
    --num-msg: #ff7878;
    --err-msg: #b4b4b4;
    --accent: #38bdf8;
    --ok: #46e664;
    --warn: #ffc83c;
    --danger: #ff5555;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: rgba(56, 189, 248, 0.18); }
  html, body {
    background: var(--bg-main);
    color: var(--text-main);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-size: 13px;
    width: 100%;
    max-width: 100vw;
    height: 100vh;
    height: 100dvh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }
  .toolbar {
    background: var(--bg-panel);
    border-bottom: 1px solid var(--border);
    padding: 7px 12px;
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 7px;
    width: 100%;
    max-width: 100vw;
    flex-shrink: 0;
  }
  .brand {
    font-weight: 700;
    font-size: 14px;
    color: #fff;
    letter-spacing: 0.3px;
    display: flex;
    align-items: center;
    gap: 7px;
    margin-right: 4px;
  }
  .dot {
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--warn);
    display: inline-block;
    box-shadow: 0 0 6px var(--warn);
    flex-shrink: 0;
  }
  .dot.live { background: var(--ok); box-shadow: 0 0 8px var(--ok); }
  .dot.offline { background: var(--danger); box-shadow: 0 0 8px var(--danger); }
  .sep { width: 1px; height: 18px; background: var(--border); margin: 0 2px; }
  button, select, input[type="text"] {
    background: var(--bg-header);
    color: var(--text-main);
    border: 1px solid var(--border);
    border-radius: 5px;
    padding: 5px 9px;
    font-size: 12px;
    outline: none;
    min-height: 28px;
  }
  button { cursor: pointer; transition: background 0.12s, border-color 0.12s; user-select: none; }
  button:hover { background: #243247; border-color: #3b5070; }
  button:active { transform: translateY(0.5px); }
  button.active { background: #1d4ed8; border-color: #3b82f6; color: #fff; font-weight: 600; }
  button.danger { background: #3b181e; border-color: #7f1d1d; color: #fca5a5; }
  button.danger:hover { background: #5c1f27; }
  input[type="text"] { width: 170px; }
  input[type="text"]:focus { border-color: var(--accent); }
  .badge {
    font-family: "JetBrains Mono", Consolas, "Courier New", monospace;
    font-size: 11.5px;
    padding: 3px 7px;
    border-radius: 4px;
    background: #0f1622;
    border: 1px solid var(--border);
    white-space: nowrap;
    max-width: 100%;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .badge.mode { color: #64c8ff; font-weight: 600; }
  .badge.cache { color: #a5f3fc; }
  .badge.freq { color: #fde047; cursor: pointer; }
  .mobile-rf-summary {
    display: none;
    align-items: center;
    gap: 6px;
    margin-left: auto;
  }
  .telem-bar {
    background: #0f151f;
    border-bottom: 1px solid var(--border);
    padding: 5px 12px;
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 7px;
    font-size: 12px;
    color: var(--text-muted);
    width: 100%;
    max-width: 100vw;
    flex-shrink: 0;
  }
  .workspace {
    flex: 1;
    display: flex;
    flex-direction: column;
    min-height: 0;
    width: 100%;
    max-width: 100vw;
    overflow: hidden;
  }
  .pane {
    display: flex;
    flex-direction: column;
    min-height: 0;
    width: 100%;
    max-width: 100vw;
    overflow: hidden;
  }
  .pane-monitor { flex: 65; }
  .pane-filtered { flex: 35; border-top: 2px solid var(--border); }
  .pane-header {
    background: var(--bg-panel);
    padding: 6px 12px;
    font-weight: 600;
    font-size: 12.5px;
    border-bottom: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
    flex-shrink: 0;
  }
  .pane-hint { font-weight: 400; font-size: 11.5px; color: var(--text-muted); }
  .table-wrap {
    flex: 1;
    width: 100%;
    max-width: 100vw;
    overflow-y: auto;
    overflow-x: auto;
    -webkit-overflow-scrolling: touch;
  }
  table.desktop-table {
    width: 100%;
    min-width: 880px;
    border-collapse: collapse;
    table-layout: fixed;
    font-family: "JetBrains Mono", Consolas, "Liberation Mono", monospace;
    font-size: 12.5px;
  }
  thead th {
    position: sticky;
    top: 0;
    z-index: 2;
    background: var(--bg-header);
    color: var(--text-main);
    text-align: left;
    padding: 6px 8px;
    border-bottom: 1px solid var(--border);
    border-right: 1px solid #202b3d;
    cursor: pointer;
    user-select: none;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 12px;
    white-space: nowrap;
  }
  thead th:hover { background: #233044; }
  tbody tr {
    border-bottom: 1px solid #151d2a;
    cursor: pointer;
  }
  tbody tr:nth-child(even) { background: var(--bg-row-alt); }
  tbody tr:hover { background: var(--bg-row-hover); }
  tbody tr.selected { background: var(--bg-row-sel) !important; }
  td {
    padding: 4px 8px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    vertical-align: middle;
  }
  td.col-addr { color: var(--capcode); cursor: pointer; font-weight: 700; }
  td.col-addr:hover { text-decoration: underline; color: var(--accent); }
  td.col-time, td.col-date { color: var(--time-col); }
  td.col-freq { color: var(--freq-col); font-size: 11.5px; }
  td.col-mode, td.col-rate { color: var(--mode-col); }
  td.col-type { color: var(--type-col); }
  td.col-msg { cursor: pointer; }
  .msg-alpha { color: var(--alpha-msg); }
  .msg-num { color: var(--num-msg); }
  .msg-err { color: var(--err-msg); }
  .flt-tag { display: inline-block; margin-left: 6px; font-weight: 600; font-size: 11.5px; }
  .grp-tag { display: inline-block; margin-left: 6px; color: #facc15; font-size: 11px; font-weight: 600; }
  .frag-tag { display: inline-block; margin-left: 6px; color: #c084fc; font-size: 11px; font-weight: 600; }
  .soft-tag { display: inline-block; margin-left: 6px; color: #8cff8c; font-size: 11px; }

  /* Mobile & Responsive Dispatch Card Feed */
  .card-feed {
    display: flex;
    flex-direction: column;
    width: 100%;
    padding: 6px 8px;
    gap: 6px;
  }
  .msg-card {
    background: #121824;
    border: 1px solid #223047;
    border-left: 3px solid var(--accent);
    border-radius: 6px;
    padding: 8px 10px;
    display: flex;
    flex-direction: column;
    gap: 5px;
    cursor: pointer;
    transition: background 0.1s, border-color 0.1s;
    width: 100%;
  }
  .msg-card:nth-child(even) { background: #101620; }
  .msg-card:hover { background: var(--bg-row-hover); }
  .msg-card.selected {
    background: var(--bg-row-sel) !important;
    border-color: var(--accent);
    box-shadow: 0 0 0 1px var(--accent);
  }
  .msg-card.card-num { border-left-color: var(--num-msg); }
  .msg-card.card-err { border-left-color: var(--err-msg); }
  .msg-card-head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 6px;
    font-family: "JetBrains Mono", Consolas, monospace;
    font-size: 12px;
  }
  .card-capcode {
    color: #fde047;
    background: #1d283a;
    border: 1px solid #334766;
    border-radius: 4px;
    padding: 1px 7px;
    font-weight: 700;
    font-size: 12.5px;
    cursor: pointer;
  }
  .card-capcode:active, .card-capcode:hover {
    background: #2b3d59;
    color: #fff;
  }
  .card-pill {
    padding: 1px 6px;
    border-radius: 4px;
    font-size: 11px;
    background: #0d131d;
    border: 1px solid #202c40;
  }
  .card-pill.mode { color: var(--mode-col); font-weight: 600; }
  .card-pill.type { color: var(--type-col); }
  .card-pill.freq { color: var(--freq-col); }
  .card-time {
    margin-left: auto;
    color: var(--time-col);
    font-size: 11.5px;
    white-space: nowrap;
  }
  .msg-card-body {
    font-family: "JetBrains Mono", Consolas, "Liberation Mono", monospace;
    font-size: 13.5px;
    line-height: 1.42;
    white-space: pre-wrap;
    word-break: break-word;
    overflow-wrap: anywhere;
    padding-top: 2px;
  }
  .msg-card-badges {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
    align-items: center;
  }
  .msg-card-badges .flt-tag,
  .msg-card-badges .grp-tag,
  .msg-card-badges .frag-tag,
  .msg-card-badges .soft-tag {
    margin-left: 0;
  }
  .empty-state {
    padding: 26px 16px;
    text-align: center;
    color: var(--text-muted);
    font-size: 13px;
  }

  /* Inspector Bottom Bar / Mobile Slide-Up Sheet */
  .inspector {
    background: var(--bg-panel);
    border-top: 2px solid var(--accent);
    padding: 9px 12px;
    display: none;
    flex-direction: column;
    gap: 7px;
    max-height: 230px;
    flex-shrink: 0;
    z-index: 40;
  }
  .inspector.open { display: flex; }
  .inspector-top {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    font-size: 12px;
  }
  .inspector-actions {
    margin-left: auto;
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
  }
  .inspector-profile {
    font-family: "JetBrains Mono", Consolas, monospace;
    font-size: 11.5px;
    color: #a5f3fc;
    background: #0e1624;
    border: 1px solid #22324c;
    border-radius: 4px;
    padding: 5px 8px;
    line-height: 1.38;
    word-break: break-word;
  }
  .inspector-body {
    font-family: "JetBrains Mono", Consolas, monospace;
    font-size: 13.5px;
    line-height: 1.42;
    color: #d7fff5;
    background: #0a0e14;
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 8px 10px;
    white-space: pre-wrap;
    word-break: break-word;
    overflow-wrap: anywhere;
    overflow-y: auto;
    max-height: 105px;
  }
  .freq-drawer {
    background: #101826;
    border-bottom: 2px solid var(--accent);
    padding: 8px 12px;
    display: none;
    flex-direction: column;
    gap: 7px;
    width: 100%;
    max-width: 100vw;
    flex-shrink: 0;
  }
  .freq-drawer.open { display: flex; }
  .freq-chips { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
  .freq-chip {
    background: #172234;
    border: 1px solid #2b3d59;
    border-radius: 5px;
    padding: 4px 9px;
    display: flex;
    align-items: center;
    gap: 7px;
    font-family: "JetBrains Mono", Consolas, monospace;
    font-size: 12px;
  }
  .toolbar-row-primary,
  .toolbar-row-modes,
  .toolbar-row-search,
  .toolbar-row-extra {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 6px;
  }

  /* Dedicated Mobile & Narrow Viewport Styles */
  @media (max-width: 820px) {
    .toolbar {
      padding: 6px 8px;
      gap: 5px;
    }
    .sep { display: none; }
    .toolbar-row-primary {
      width: 100%;
      justify-content: space-between;
    }
    .mobile-rf-summary {
      display: flex;
    }
    .toolbar-row-modes {
      width: 100%;
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 4px;
    }
    .toolbar-row-modes button {
      padding: 6px 4px;
      font-size: 11.5px;
      text-align: center;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .toolbar-row-search {
      width: 100%;
      display: flex;
      gap: 5px;
    }
    .toolbar-row-search input[type="text"] {
      flex: 1;
      width: auto;
      min-width: 0;
      font-size: 13px;
      padding: 6px 8px;
    }
    .toolbar-row-extra {
      display: none;
      width: 100%;
      padding-top: 4px;
      border-top: 1px solid var(--border);
    }
    .toolbar-row-extra.mobile-open {
      display: flex;
    }
    .telem-bar {
      display: none;
      padding: 6px 8px;
    }
    .telem-bar.mobile-open {
      display: flex;
    }
    .badge {
      white-space: normal;
      word-break: break-word;
      font-size: 11px;
    }
    .pane-hint { display: none; }
    .pane-header { padding: 5px 8px; font-size: 12px; }
    .inspector {
      position: fixed;
      left: 0;
      right: 0;
      bottom: 0;
      max-height: 64dvh;
      box-shadow: 0 -8px 28px rgba(0, 0, 0, 0.75);
      border-top: 2px solid var(--accent);
      padding: 10px;
    }
    .inspector-actions {
      width: 100%;
      margin-left: 0;
      justify-content: flex-start;
    }
    .inspector-actions button {
      padding: 6px 9px;
      font-size: 12px;
    }
    .inspector-body {
      max-height: 26dvh;
      font-size: 13.5px;
    }
  }
</style>
</head>
<body>

<div class="toolbar">
  <div class="toolbar-row-primary">
    <div class="brand">
      <span id="connDot" class="dot"></span>
      <span>PDW-RS</span>
      <span id="connText" style="font-size:11.5px;font-weight:400;color:var(--text-muted);">Connecting...</span>
    </div>
    <div class="mobile-rf-summary">
      <span id="mobileRxQBadge" class="badge" style="color:var(--ok);font-weight:700;white-space:nowrap;" onclick="toggleMobileTools()">RX-Q: 100%</span>
      <button id="btnMobileTools" onclick="toggleMobileTools()">RF &amp; Tools ▾</button>
    </div>
  </div>

  <div class="sep"></div>

  <div class="toolbar-row-modes">
    <button id="btnViewMon" onclick="setViewMode('monitor')">Monitor</button>
    <button id="btnViewSplit" class="active" onclick="setViewMode('split')">Split View</button>
    <button id="btnViewFlt" onclick="setViewMode('filtered')">Filtered</button>
    <button id="btnViewCatalog" onclick="setViewMode('catalog')">Catalog (0)</button>
  </div>

  <div class="sep"></div>

  <div class="toolbar-row-search">
    <input type="text" id="searchInput" placeholder="Search capcode, message, group, freq..." oninput="onSearchInput()" />
    <button onclick="clearSearch()" title="Clear search filter">×</button>
    <button id="btnAutoScroll" class="active" onclick="toggleAutoScroll()" title="Toggle auto-scroll to newest messages">Auto-Scroll</button>
    <button id="btnPause" onclick="toggleLocalPause()">Pause</button>
  </div>

  <div class="sep"></div>

  <div id="toolbarExtra" class="toolbar-row-extra">
    <button id="btnFreqDrawer" onclick="toggleFreqDrawer()">Parallel Freqs (2)</button>
    <button id="btnLayoutMode" onclick="cycleLayoutMode()" title="Switch between Automatic (Cards on mobile, Table on desktop), Dispatch Cards, or Table">View: Auto</button>
    <button id="btnHideTone" onclick="toggleHideTone()">Tone-Only: Show</button>
    <button onclick="clearAllMessages()">Clear View</button>
    <button onclick="triggerTest('pocsag')" title="Inject synthetic 48 kHz POCSAG-1200 test burst">Test POCSAG</button>
    <button onclick="triggerTest('flex')" title="Inject synthetic 48 kHz FLEX 4-FSK test burst">Test FLEX</button>
    <button onclick="exportCsv()" title="Export cached &amp; visible messages to CSV">Export CSV</button>
  </div>
</div>

<div id="freqDrawer" class="freq-drawer">
  <div style="display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px;">
    <div>
      <strong style="color:#fff;">Parallel Monitored SDR Frequencies (Max Span: 2.000 MHz)</strong>
      <span id="freqSpanInfo" style="margin-left:8px;color:#a5f3fc;font-family:monospace;font-size:12px;">Active Span: 126.12 kHz / 2000.00 kHz</span>
    </div>
    <div style="display:flex;flex-wrap:wrap;gap:6px;align-items:center;">
      <input type="text" id="newFreqMhz" placeholder="e.g. 929.73862" style="width:120px;" />
      <input type="text" id="newFreqLabel" placeholder="Label (optional)" style="width:130px;" />
      <button onclick="addParallelFreq()">+ Add Freq (MHz)</button>
      <span id="freqErrMsg" style="color:var(--danger);font-size:12px;"></span>
    </div>
  </div>
  <div id="freqListContainer" class="freq-chips"></div>
</div>

<div id="telemBar" class="telem-bar">
  <span id="telemMode" class="badge mode">IDLE (48000 Hz)</span>
  <span id="telemRxQ" class="badge" style="color:var(--ok);font-weight:700;">RX-Q: 100.0%</span>
  <span id="telemSignal" class="badge">Pol: NORM | DC: +0.000 | Dev: ±0.500 | Soft BCH: 0</span>
  <span id="telemSource" class="badge">Input: Soundcard</span>
  <span id="telemFreqs" class="badge freq" onclick="toggleFreqDrawer()" title="Click to manage parallel monitored frequencies">Freqs: 929.61250, 929.73862 MHz</span>
  <span id="telemStats" class="badge">Decoded: 0</span>

  <div style="margin-left:auto;display:flex;flex-wrap:wrap;align-items:center;gap:6px;">
    <span id="cacheStatus" class="badge cache" title="Messages cached in browser localStorage for instant startup &amp; delta sync">Browser Cache: 0 msgs (0.0 KB)</span>
    <label style="font-size:11.5px;">Cache:</label>
    <select id="cacheLimitSelect" onchange="onCacheLimitChange()">
      <option value="0">Off</option>
      <option value="250">250 msgs</option>
      <option value="500">500 msgs</option>
      <option value="1000" selected>1,000 msgs</option>
      <option value="2500">2,500 msgs</option>
      <option value="5000">5,000 msgs</option>
    </select>
    <button onclick="clearBrowserCache()" title="Purge browser localStorage message cache">Clear Cache</button>
  </div>
</div>

<!-- Live Monitor & Filtered Dual-Pane Workspace -->
<div id="liveWorkspace" class="workspace">
  <div id="paneMonitor" class="pane pane-monitor" style="flex:65;">
    <div class="pane-header">
      <span id="monTitle">Monitored Messages (0)</span>
      <span class="pane-hint">Tap any message or capcode to inspect profile &amp; full text</span>
    </div>
    <div id="monScroll" class="table-wrap">
      <div id="monCards" class="card-feed" style="display:none;"></div>
      <table id="monTable" class="desktop-table">
        <thead>
          <tr>
            <th style="width:94px;" onclick="setSort('capcode')">Address <span id="sort_capcode"></span></th>
            <th style="width:72px;" onclick="setSort('seq')">Time <span id="sort_seq">▲</span></th>
            <th style="width:72px;" onclick="setSort('seq')">Date</th>
            <th style="width:88px;" onclick="setSort('freq_hz')">Freq (MHz) <span id="sort_freq_hz"></span></th>
            <th style="width:80px;" onclick="setSort('mode')">Mode <span id="sort_mode"></span></th>
            <th style="width:68px;" onclick="setSort('msg_type')">Type <span id="sort_msg_type"></span></th>
            <th style="width:60px;" onclick="setSort('bitrate')">Bitrate <span id="sort_bitrate"></span></th>
            <th style="min-width:340px;" onclick="setSort('message')">Message <span id="sort_message"></span></th>
          </tr>
        </thead>
        <tbody id="monBody"></tbody>
      </table>
    </div>
  </div>

  <div id="paneFiltered" class="pane pane-filtered" style="flex:35;">
    <div class="pane-header">
      <span id="fltTitle">Filtered Messages (0)</span>
      <span class="pane-hint">Matched against configured PDW-RS filter rules</span>
    </div>
    <div id="fltScroll" class="table-wrap">
      <div id="fltCards" class="card-feed" style="display:none;"></div>
      <table id="fltTable" class="desktop-table">
        <thead>
          <tr>
            <th style="width:94px;" onclick="setSort('capcode')">Address</th>
            <th style="width:72px;" onclick="setSort('seq')">Time</th>
            <th style="width:72px;" onclick="setSort('seq')">Date</th>
            <th style="width:88px;" onclick="setSort('freq_hz')">Freq (MHz)</th>
            <th style="width:80px;" onclick="setSort('mode')">Mode</th>
            <th style="width:68px;" onclick="setSort('msg_type')">Type</th>
            <th style="width:60px;" onclick="setSort('bitrate')">Bitrate</th>
            <th style="min-width:340px;" onclick="setSort('message')">Message</th>
          </tr>
        </thead>
        <tbody id="fltBody"></tbody>
      </table>
    </div>
  </div>
</div>

<!-- Capcode Directory & Message Catalog Workspace -->
<div id="catalogWorkspace" class="workspace" style="display:none;">
  <div id="catListPane" class="pane" style="flex:52;">
    <div class="pane-header">
      <span id="catTitle">Capcode Directory &amp; Catalog (0 capcodes)</span>
      <div style="display:flex;flex-wrap:wrap;gap:6px;align-items:center;">
        <input type="text" id="catSearchInput" placeholder="Search capcode / group / alias..." oninput="renderCatalogTable()" style="width:190px;" />
        <button onclick="loadCatalog()">Refresh</button>
        <button class="danger" onclick="clearEntireCatalog()">Clear DB</button>
      </div>
    </div>
    <div class="table-wrap">
      <div id="catCards" class="card-feed" style="display:none;"></div>
      <table id="catTable" class="desktop-table">
        <thead>
          <tr>
            <th style="width:98px;" onclick="setCatSort('capcode')">Capcode</th>
            <th style="width:130px;" onclick="setCatSort('alias')">Alias / Label</th>
            <th style="width:64px;" onclick="setCatSort('message_count')">Msgs ▼</th>
            <th style="width:128px;" onclick="setCatSort('first_seen')">First Seen</th>
            <th style="width:128px;" onclick="setCatSort('last_seen')">Last Seen</th>
            <th style="width:95px;">Baud Rates</th>
            <th style="width:95px;">Modes</th>
            <th style="width:115px;">Freqs (MHz)</th>
            <th style="width:130px;">FLEX Groupcalls</th>
            <th style="min-width:260px;">Latest Message</th>
            <th style="width:70px;">Action</th>
          </tr>
        </thead>
        <tbody id="catBody"></tbody>
      </table>
    </div>
  </div>

  <div id="catDetailPane" class="pane pane-filtered" style="flex:48;">
    <div class="pane-header">
      <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap;">
        <button id="btnCatBackMobile" onclick="closeMobileCatalogDetail()" style="display:none;">← Capcode List</button>
        <span id="catDetailTitle">Select a capcode above to view its profile &amp; message history</span>
      </div>
      <div id="catDetailControls" style="display:none;flex-wrap:wrap;gap:6px;align-items:center;">
        <label style="font-size:11.5px;color:var(--text-muted);">Alias:</label>
        <input type="text" id="catAliasInput" placeholder="Friendly name / notes..." style="width:150px;" />
        <button onclick="saveCapcodeAlias()">Save Alias</button>
        <button class="danger" onclick="deleteSelectedCatalogCapcode()">Remove from DB</button>
      </div>
    </div>
    <div class="table-wrap">
      <div id="catMsgCards" class="card-feed" style="display:none;"></div>
      <table id="catMsgTable" class="desktop-table">
        <thead>
          <tr>
            <th style="width:76px;">Time</th>
            <th style="width:76px;">Date</th>
            <th style="width:90px;">Freq (MHz)</th>
            <th style="width:82px;">Mode</th>
            <th style="width:70px;">Type</th>
            <th style="width:64px;">Bitrate</th>
            <th style="width:115px;">Group / Frags</th>
            <th style="min-width:300px;">Message</th>
          </tr>
        </thead>
        <tbody id="catMsgBody"></tbody>
      </table>
    </div>
  </div>
</div>

<!-- Bottom Selected Message & Capcode Profile Inspector -->
<div id="inspector" class="inspector">
  <div class="inspector-top">
    <strong id="inspCapcode" style="color:#fde047;font-size:14px;">Selected: 0000000</strong>
    <span id="inspMeta" style="color:#96cdff;"></span>
    <span id="inspLabel"></span>
    <div class="inspector-actions">
      <button onclick="openSelectedInCatalog()">Open in Catalog</button>
      <button id="btnIsolate" onclick="isolateSelectedCapcode()">Filter Capcode</button>
      <button onclick="copySelectedMsg()">Copy Msg</button>
      <button onclick="copySelectedCapcode()">Copy Capcode</button>
      <button class="danger" onclick="deleteInspectedCapcodeFromDb()">Delete DB Entry</button>
      <button onclick="closeInspector()">Close ×</button>
    </div>
  </div>
  <div id="inspProfileBar" class="inspector-profile">Loading capcode directory profile...</div>
  <div id="inspMessage" class="inspector-body"></div>
</div>

<script>
const STORAGE_CACHE_KEY = 'pdw_rs_msg_cache_v1';
const STORAGE_META_KEY  = 'pdw_rs_meta_v1';
const LABEL_PALETTE = [
  '#50b4ff', '#ffeb3c', '#ff5555', '#ffaa00',
  '#50ff78', '#50ffff', '#ff82dc', '#dcdcdc'
];

let state = {
  sessionId: 0,
  lastSeq: 0,
  maxCacheSize: 1000,
  viewMode: (typeof window !== 'undefined' && window.innerWidth <= 820) ? 'monitor' : 'split',
  layoutMode: 'auto', // 'auto' | 'cards' | 'table'
  mobileToolsOpen: false,
  autoScroll: true,
  localPaused: false,
  hideTone: false,
  searchQuery: '',
  sortCol: 'seq',
  sortAsc: true,
  messages: [],
  uidSet: new Set(),
  selectedUid: null,
  inspectedCapcode: null,
  lastDeltaCount: 0,
  cacheBytes: 0,
  monitoredFrequencies: [],
  activeSpanHz: 0,
  catalogProfiles: [],
  selectedCatalogCapcode: null,
  catSortCol: 'message_count',
  catSortAsc: false
};

let renderPending = false;
let saveCacheTimer = null;
let evtSource = null;
let pointerHoldUntil = 0;

function isMobileViewport() {
  return typeof window !== 'undefined' && window.innerWidth <= 820;
}

function useCardLayout() {
  if (state.layoutMode === 'cards') return true;
  if (state.layoutMode === 'table') return false;
  return isMobileViewport();
}

function makeUid(sessionId, seq, entry) {
  if (sessionId && seq) return `${sessionId}:${seq}`;
  return `${entry.date}_${entry.time}_${entry.capcode}_${entry.mode}_${entry.message}`;
}

function formatFreqMhz(freqHz) {
  if (!freqHz || freqHz <= 0) return 'Audio';
  return (freqHz / 1e6).toFixed(5);
}

// Guard against mid-tap DOM rebuilds canceling touch clicks during fast FLEX 6400 bursts
function registerTouchGuard() {
  const holdHandler = () => {
    pointerHoldUntil = Date.now() + 260;
  };
  ['monScroll', 'fltScroll', 'catListPane', 'catDetailPane'].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('touchstart', holdHandler, { passive: true });
    el.addEventListener('pointerdown', holdHandler, { passive: true });
  });

  // Container-level click delegation so tapping ANY cell or part of a row/card always works
  ['monScroll', 'fltScroll'].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('click', (ev) => {
      if (!ev || !ev.target || typeof ev.target.closest !== 'function') return;
      const capEl = ev.target.closest('[data-capcode-filter]');
      if (capEl) {
        ev.stopPropagation();
        const cap = capEl.getAttribute('data-capcode-filter');
        const uid = capEl.getAttribute('data-uid');
        onCapcodeClick(cap, uid);
        return;
      }
      const rowEl = ev.target.closest('[data-uid]');
      if (rowEl) {
        const uid = rowEl.getAttribute('data-uid');
        if (uid) onSelectRow(uid);
      }
    });
  });

  if (typeof window !== 'undefined') {
    window.addEventListener('resize', () => {
      applyToolbarState();
      scheduleRender();
      if (state.viewMode === 'catalog') renderCatalogTable();
    });
  }
}

// 1. Instant hydration from browser localStorage before network round-trip
function loadBrowserCache() {
  try {
    const rawMeta = localStorage.getItem(STORAGE_META_KEY);
    if (rawMeta) {
      const m = JSON.parse(rawMeta);
      if (typeof m.sessionId === 'number') state.sessionId = m.sessionId;
      if (typeof m.lastSeq === 'number') state.lastSeq = m.lastSeq;
      if (typeof m.maxCacheSize === 'number') state.maxCacheSize = m.maxCacheSize;
      if (typeof m.viewMode === 'string') state.viewMode = m.viewMode;
      if (typeof m.layoutMode === 'string') state.layoutMode = m.layoutMode;
      if (typeof m.autoScroll === 'boolean') state.autoScroll = m.autoScroll;
      if (typeof m.hideTone === 'boolean') state.hideTone = m.hideTone;
    }
    const sel = document.getElementById('cacheLimitSelect');
    if (sel) sel.value = String(state.maxCacheSize);
    applyToolbarState();

    if (state.maxCacheSize > 0) {
      const rawMsgs = localStorage.getItem(STORAGE_CACHE_KEY);
      if (rawMsgs) {
        state.cacheBytes = rawMsgs.length;
        const arr = JSON.parse(rawMsgs);
        if (Array.isArray(arr)) {
          for (const item of arr) {
            const uid = item.uid || makeUid(item.sid || state.sessionId, item.seq, item);
            if (!state.uidSet.has(uid)) {
              item.uid = uid;
              state.uidSet.add(uid);
              state.messages.push(item);
            }
          }
        }
      }
    }
  } catch (e) {
    console.warn('Browser cache hydration warning:', e);
  }
  updateCacheBadge();
  scheduleRender();
}

function saveMetaNow() {
  try {
    localStorage.setItem(STORAGE_META_KEY, JSON.stringify({
      sessionId: state.sessionId,
      lastSeq: state.lastSeq,
      maxCacheSize: state.maxCacheSize,
      viewMode: state.viewMode,
      layoutMode: state.layoutMode,
      autoScroll: state.autoScroll,
      hideTone: state.hideTone
    }));
  } catch (e) {}
}

function scheduleSaveCache() {
  if (saveCacheTimer) return;
  saveCacheTimer = setTimeout(flushCacheToStorage, 250);
}

function flushCacheToStorage() {
  saveCacheTimer = null;
  saveMetaNow();
  if (state.maxCacheSize <= 0) {
    try { localStorage.removeItem(STORAGE_CACHE_KEY); } catch (e) {}
    state.cacheBytes = 0;
    updateCacheBadge();
    return;
  }
  let slice = state.messages.slice(-state.maxCacheSize);
  while (slice.length > 0) {
    try {
      const serialized = JSON.stringify(slice);
      localStorage.setItem(STORAGE_CACHE_KEY, serialized);
      state.cacheBytes = serialized.length;
      break;
    } catch (quotaErr) {
      slice = slice.slice(Math.max(1, Math.floor(slice.length * 0.25)));
    }
  }
  updateCacheBadge();
}

window.addEventListener('beforeunload', () => {
  if (saveCacheTimer) flushCacheToStorage();
});

function updateCacheBadge() {
  const el = document.getElementById('cacheStatus');
  if (!el) return;
  if (state.maxCacheSize <= 0) {
    el.textContent = 'Browser Cache: OFF';
    return;
  }
  const cachedCnt = Math.min(state.messages.length, state.maxCacheSize);
  const kb = (state.cacheBytes / 1024).toFixed(1);
  const deltaTxt = state.lastDeltaCount > 0 ? ` | Delta +${state.lastDeltaCount}` : '';
  el.textContent = `Browser Cache: ${cachedCnt} msgs (${kb} KB)${deltaTxt}`;
}

// 2. Delta Bootstrap Sync: requests ONLY messages newer than state.lastSeq
async function bootstrapFromServer() {
  try {
    const url = `/api/bootstrap?session_id=${encodeURIComponent(state.sessionId)}&since_seq=${encodeURIComponent(state.lastSeq)}`;
    const res = await fetch(url, { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();

    const serverSid = data.session_id || 0;
    if (serverSid !== state.sessionId) {
      state.sessionId = serverSid;
      state.lastSeq = 0;
    }

    let added = 0;
    if (Array.isArray(data.messages)) {
      for (const m of data.messages) {
        if (ingestEntry(serverSid, m)) added++;
      }
    }
    if (typeof data.latest_seq === 'number' && data.latest_seq > state.lastSeq) {
      state.lastSeq = data.latest_seq;
    }
    state.lastDeltaCount = added;

    if (data.telemetry) updateTelemetryUi(data.telemetry);
    if (data.stats) updateStatsUi(data.stats);

    scheduleSaveCache();
    scheduleRender();
  } catch (err) {
    setConnectionState('offline', 'Server unreachable');
  }
}

function ingestEntry(sessionId, m) {
  const uid = makeUid(sessionId, m.seq, m);
  if (state.uidSet.has(uid)) return false;
  m.sid = sessionId;
  m.uid = uid;
  state.uidSet.add(uid);
  state.messages.push(m);
  if (typeof m.seq === 'number' && sessionId === state.sessionId && m.seq > state.lastSeq) {
    state.lastSeq = m.seq;
  }
  const keepLimit = Math.max(state.maxCacheSize, 1500);
  if (state.messages.length > keepLimit + 250) {
    const removed = state.messages.splice(0, state.messages.length - keepLimit);
    for (const r of removed) state.uidSet.delete(r.uid);
  }
  return true;
}

// 3. Real-Time SSE Connection
function connectSse() {
  if (evtSource) {
    try { evtSource.close(); } catch (e) {}
  }
  setConnectionState('sync', 'Connecting SSE...');
  evtSource = new EventSource('/api/events');

  evtSource.onopen = () => {
    const portStr = window.location.port || '8008';
    setConnectionState('live', `LIVE (:${portStr})`);
    bootstrapFromServer();
  };

  evtSource.addEventListener('message', (ev) => {
    try {
      const payload = JSON.parse(ev.data);
      const sid = payload.session_id || state.sessionId;
      if (sid && sid !== state.sessionId) {
        state.sessionId = sid;
        state.lastSeq = 0;
      }
      if (payload.entry && ingestEntry(sid, payload.entry)) {
        scheduleSaveCache();
        if (!state.localPaused) scheduleRender();
        if (state.viewMode === 'catalog') loadCatalog();
      }
    } catch (e) {}
  });

  evtSource.addEventListener('status', (ev) => {
    try {
      const payload = JSON.parse(ev.data);
      const portStr = window.location.port || '8008';
      setConnectionState('live', `LIVE (:${portStr})`);
      if (payload.telemetry) updateTelemetryUi(payload.telemetry);
      if (payload.stats) updateStatsUi(payload.stats);
    } catch (e) {}
  });

  evtSource.addEventListener('catalog_updated', () => {
    loadCatalog();
    if (state.inspectedCapcode) fetchCapcodeProfileForInspector(state.inspectedCapcode);
  });

  evtSource.addEventListener('clear', () => {
    state.messages = [];
    state.uidSet.clear();
    scheduleSaveCache();
    scheduleRender();
  });

  evtSource.onerror = () => {
    setConnectionState('offline', 'Reconnecting...');
  };
}

function setConnectionState(kind, text) {
  const dot = document.getElementById('connDot');
  const lbl = document.getElementById('connText');
  if (dot) {
    dot.className = 'dot ' + (kind === 'live' ? 'live' : (kind === 'offline' ? 'offline' : ''));
  }
  if (lbl) lbl.textContent = text;
}

function toggleMobileTools() {
  state.mobileToolsOpen = !state.mobileToolsOpen;
  const extra = document.getElementById('toolbarExtra');
  const telem = document.getElementById('telemBar');
  const btn = document.getElementById('btnMobileTools');
  if (extra) extra.classList.toggle('mobile-open', state.mobileToolsOpen);
  if (telem) telem.classList.toggle('mobile-open', state.mobileToolsOpen);
  if (btn) btn.className = state.mobileToolsOpen ? 'active' : '';
}

function cycleLayoutMode() {
  if (state.layoutMode === 'auto') state.layoutMode = 'cards';
  else if (state.layoutMode === 'cards') state.layoutMode = 'table';
  else state.layoutMode = 'auto';
  applyToolbarState();
  saveMetaNow();
  scheduleRender();
  if (state.viewMode === 'catalog') renderCatalogTable();
}

function updateTelemetryUi(t) {
  const modeEl = document.getElementById('telemMode');
  if (modeEl) modeEl.textContent = t.active_mode || 'IDLE';

  const rxqEl = document.getElementById('telemRxQ');
  const mobRxqEl = document.getElementById('mobileRxQBadge');
  if (typeof t.rx_quality_pct === 'number') {
    const qColor = t.rx_quality_pct >= 96.0 ? 'var(--ok)' : (t.rx_quality_pct >= 88.0 ? 'var(--warn)' : 'var(--danger)');
    if (rxqEl) {
      rxqEl.textContent = `RX-Q: ${t.rx_quality_pct.toFixed(1)}%`;
      rxqEl.style.color = qColor;
    }
    if (mobRxqEl) {
      mobRxqEl.textContent = `RX-Q: ${t.rx_quality_pct.toFixed(1)}%`;
      mobRxqEl.style.color = qColor;
    }
  }

  const sigEl = document.getElementById('telemSignal');
  if (sigEl) {
    const pol = t.invert_active ? 'INV' : 'NORM';
    const dc = (t.dc_center >= 0 ? '+' : '') + Number(t.dc_center || 0).toFixed(3);
    const dev = Number(t.deviation || 0).toFixed(3);
    let extra = '';
    if (typeof t.iq_rssi_dbfs === 'number') {
      extra += ` | RSSI: ${t.iq_rssi_dbfs.toFixed(1)} dBFS`;
    }
    sigEl.textContent = `Pol: ${pol} | DC: ${dc} | Dev: ±${dev} | Soft BCH: ${t.soft_recoveries || 0}${extra}`;
  }

  const srcEl = document.getElementById('telemSource');
  if (srcEl) srcEl.textContent = t.audio_status || t.input_source || 'Soundcard';

  if (typeof t.catalog_capcode_count === 'number') {
    const btnCat = document.getElementById('btnViewCatalog');
    if (btnCat) {
      btnCat.textContent = isMobileViewport()
        ? `Catalog (${t.catalog_capcode_count})`
        : `Capcode Catalog (${t.catalog_capcode_count})`;
    }
  }

  if (Array.isArray(t.monitored_frequencies)) {
    state.monitoredFrequencies = t.monitored_frequencies;
    state.activeSpanHz = t.active_span_hz || 0;
    renderFrequencyDrawer();
  }
}

function renderFrequencyDrawer() {
  const enabledList = state.monitoredFrequencies.filter(f => f.enabled);
  const btnFreq = document.getElementById('btnFreqDrawer');
  if (btnFreq) btnFreq.textContent = `Parallel Freqs (${enabledList.length})`;

  const telemFreqs = document.getElementById('telemFreqs');
  if (telemFreqs) {
    if (enabledList.length === 0) {
      telemFreqs.textContent = 'Freqs: None';
    } else {
      telemFreqs.textContent = 'Freqs: ' + enabledList.map(f => (f.freq_hz / 1e6).toFixed(5) + ' MHz').join(' + ');
    }
  }

  const spanEl = document.getElementById('freqSpanInfo');
  if (spanEl) {
    const spanKhz = (state.activeSpanHz / 1000).toFixed(2);
    spanEl.textContent = `Active Parallel Span: ${spanKhz} kHz / 2000.00 kHz (${enabledList.length} active)`;
  }

  const container = document.getElementById('freqListContainer');
  if (!container) return;
  let html = '';
  for (let i = 0; i < state.monitoredFrequencies.length; i++) {
    const f = state.monitoredFrequencies[i];
    const mhz = (f.freq_hz / 1e6).toFixed(5);
    const checked = f.enabled ? 'checked' : '';
    const lbl = f.label ? ` (${escapeHtml(f.label)})` : '';
    html += `<div class="freq-chip">` +
      `<input type="checkbox" ${checked} onchange="toggleParallelFreq(${i})" />` +
      `<span style="color:${f.enabled ? '#fde047' : '#8b9bb4'}">${mhz} MHz${lbl}</span>` +
      `<button onclick="removeParallelFreq(${i})" style="padding:1px 5px;font-size:11px;" title="Remove frequency">×</button>` +
      `</div>`;
  }
  container.innerHTML = html;
}

function toggleFreqDrawer() {
  const dr = document.getElementById('freqDrawer');
  if (dr) dr.classList.toggle('open');
}

async function toggleParallelFreq(idx) {
  try {
    await fetch(`/api/frequencies/toggle?idx=${idx}`, { method: 'POST' });
    setTimeout(bootstrapFromServer, 120);
  } catch (e) {}
}

async function addParallelFreq() {
  const mhzStr = (document.getElementById('newFreqMhz').value || '').trim();
  const labelStr = (document.getElementById('newFreqLabel').value || '').trim();
  const errEl = document.getElementById('freqErrMsg');
  if (errEl) errEl.textContent = '';

  const mhz = parseFloat(mhzStr);
  if (isNaN(mhz) || mhz < 0.1) {
    if (errEl) errEl.textContent = 'Enter a valid frequency in MHz (e.g. 929.73862)';
    return;
  }
  const candHz = Math.round(mhz * 1e6);

  // Client-side 2.0 MHz span check
  let minHz = candHz;
  let maxHz = candHz;
  for (const f of state.monitoredFrequencies) {
    if (f.enabled && f.freq_hz > 0) {
      if (f.freq_hz < minHz) minHz = f.freq_hz;
      if (f.freq_hz > maxHz) maxHz = f.freq_hz;
    }
  }
  if (maxHz - minHz > 2000000) {
    if (errEl) {
      errEl.textContent = `Rejected: ${mhz.toFixed(5)} MHz is ${((maxHz - minHz) / 1e6).toFixed(3)} MHz apart from enabled channels (max receiver bandwidth is 2.000 MHz).`;
    }
    return;
  }

  try {
    await fetch(`/api/frequencies/add?freq_hz=${candHz}&label=${encodeURIComponent(labelStr)}`, { method: 'POST' });
    document.getElementById('newFreqMhz').value = '';
    document.getElementById('newFreqLabel').value = '';
    setTimeout(bootstrapFromServer, 120);
  } catch (e) {}
}

async function removeParallelFreq(idx) {
  try {
    await fetch(`/api/frequencies/remove?idx=${idx}`, { method: 'POST' });
    setTimeout(bootstrapFromServer, 120);
  } catch (e) {}
}

function updateStatsUi(s) {
  const stEl = document.getElementById('telemStats');
  if (stEl) {
    stEl.textContent = `Decoded: ${s.total || 0} | Rejected: ${s.rejected || 0} | DupBlocked: ${s.duplicates || 0}`;
  }
}

function scheduleRender(forceNow) {
  if (renderPending) return;
  const now = Date.now();
  if (!forceNow && now < pointerHoldUntil) {
    renderPending = true;
    setTimeout(() => {
      renderPending = false;
      scheduleRender(true);
    }, pointerHoldUntil - now + 20);
    return;
  }
  renderPending = true;
  requestAnimationFrame(() => {
    renderPending = false;
    renderTables();
  });
}

function matchesFilter(m, q) {
  if (state.hideTone && (m.message === 'TONE ONLY' || m.message === 'TONE-ONLY')) {
    return false;
  }
  if (!q) return true;
  const freqStr = formatFreqMhz(m.freq_hz).toLowerCase();
  return (m.capcode && m.capcode.toLowerCase().includes(q)) ||
         (m.message && m.message.toLowerCase().includes(q)) ||
         (m.mode && m.mode.toLowerCase().includes(q)) ||
         (m.msg_type && m.msg_type.toLowerCase().includes(q)) ||
         (m.filter_label && m.filter_label.toLowerCase().includes(q)) ||
         (m.group_address && m.group_address.toLowerCase().includes(q)) ||
         freqStr.includes(q);
}

function compareMsgs(a, b) {
  let cmp = 0;
  switch (state.sortCol) {
    case 'capcode': cmp = (a.capcode || '').localeCompare(b.capcode || ''); break;
    case 'freq_hz': cmp = (a.freq_hz || 0) - (b.freq_hz || 0); break;
    case 'mode': cmp = (a.mode || '').localeCompare(b.mode || ''); break;
    case 'msg_type': cmp = (a.msg_type || '').localeCompare(b.msg_type || ''); break;
    case 'bitrate': cmp = (a.bitrate || 0) - (b.bitrate || 0); break;
    case 'message': cmp = (a.message || '').localeCompare(b.message || ''); break;
    default: cmp = (a.ts || 0) - (b.ts || 0) || (a.seq || 0) - (b.seq || 0); break;
  }
  return state.sortAsc ? cmp : -cmp;
}

function escapeHtml(s) {
  if (!s) return '';
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

function buildBadgesHtml(m) {
  let badgeHtml = '';
  if (m.filter_label) {
    const cIdx = (m.filter_color_idx || 0) % LABEL_PALETTE.length;
    badgeHtml += `<span class="flt-tag" style="color:${LABEL_PALETTE[cIdx]}">[${escapeHtml(m.filter_label)}]</span>`;
  }
  if (m.group_address) {
    badgeHtml += `<span class="grp-tag">[GRP:${escapeHtml(m.group_address)}]</span>`;
  }
  if (m.fragment_info) {
    badgeHtml += `<span class="frag-tag">[${escapeHtml(m.fragment_info)}]</span>`;
  }
  if (m.soft_recovered_words > 0) {
    badgeHtml += `<span class="soft-tag">(Soft+${m.soft_recovered_words})</span>`;
  }
  return badgeHtml;
}

function buildRowsHtml(list) {
  let html = '';
  for (let i = 0; i < list.length; i++) {
    const m = list[i];
    const selClass = m.uid === state.selectedUid ? ' class="selected"' : '';
    const isNum = (m.msg_type || '').includes('NUM') || (m.msg_type || '').includes('TONE');
    const msgClass = m.has_bit_errors ? 'msg-err' : (isNum ? 'msg-num' : 'msg-alpha');
    const badgeHtml = buildBadgesHtml(m);

    const safeUid = escapeHtml(m.uid);
    const safeCap = escapeHtml(m.capcode);
    const freqTxt = formatFreqMhz(m.freq_hz);
    html += `<tr${selClass} data-uid="${safeUid}" data-capcode="${safeCap}" onclick="onSelectRow('${safeUid}')">` +
      `<td class="col-addr" data-capcode-filter="${safeCap}" data-uid="${safeUid}" onclick="event.stopPropagation();onCapcodeClick('${safeCap}', '${safeUid}')" title="Tap to inspect capcode ${safeCap} profile">${safeCap}</td>` +
      `<td class="col-time">${escapeHtml(m.time)}</td>` +
      `<td class="col-date">${escapeHtml(m.date)}</td>` +
      `<td class="col-freq">${escapeHtml(freqTxt)}</td>` +
      `<td class="col-mode">${escapeHtml(m.mode)}</td>` +
      `<td class="col-type">${escapeHtml(m.msg_type)}</td>` +
      `<td class="col-rate">${m.bitrate || ''}</td>` +
      `<td class="col-msg ${msgClass}">${escapeHtml(m.message)}${badgeHtml}</td>` +
      `</tr>`;
  }
  return html;
}

function buildCardsHtml(list) {
  if (list.length === 0) {
    return `<div class="empty-state">No decoded messages match the current view filter.</div>`;
  }
  let html = '';
  for (let i = 0; i < list.length; i++) {
    const m = list[i];
    const isSel = m.uid === state.selectedUid;
    const isNum = (m.msg_type || '').includes('NUM') || (m.msg_type || '').includes('TONE');
    const msgClass = m.has_bit_errors ? 'msg-err' : (isNum ? 'msg-num' : 'msg-alpha');
    const borderClass = m.has_bit_errors ? ' card-err' : (isNum ? ' card-num' : '');
    const selClass = isSel ? ' selected' : '';
    const badgeHtml = buildBadgesHtml(m);

    const safeUid = escapeHtml(m.uid);
    const safeCap = escapeHtml(m.capcode);
    const freqTxt = formatFreqMhz(m.freq_hz);
    const rateTxt = m.bitrate ? ` ${m.bitrate}` : '';

    html += `<div class="msg-card${borderClass}${selClass}" data-uid="${safeUid}" data-capcode="${safeCap}" onclick="onSelectRow('${safeUid}')">` +
      `<div class="msg-card-head">` +
        `<span class="card-capcode" data-capcode-filter="${safeCap}" data-uid="${safeUid}" onclick="event.stopPropagation();onCapcodeClick('${safeCap}', '${safeUid}')" title="Tap to inspect &amp; filter capcode ${safeCap}">${safeCap}</span>` +
        `<span class="card-pill mode">${escapeHtml(m.mode)}${rateTxt}</span>` +
        `<span class="card-pill type">${escapeHtml(m.msg_type)}</span>` +
        `<span class="card-pill freq">${escapeHtml(freqTxt)} MHz</span>` +
        `<span class="card-time">${escapeHtml(m.time)} <span style="opacity:0.75;font-size:10.5px;">${escapeHtml(m.date)}</span></span>` +
      `</div>` +
      (badgeHtml ? `<div class="msg-card-badges">${badgeHtml}</div>` : '') +
      `<div class="msg-card-body ${msgClass}">${escapeHtml(m.message)}</div>` +
      `</div>`;
  }
  return html;
}

function renderTables() {
  const q = state.searchQuery.trim().toLowerCase();
  const monAll = [];
  const fltAll = [];
  const monVis = [];
  const fltVis = [];

  for (let i = 0; i < state.messages.length; i++) {
    const m = state.messages[i];
    if (m.mon !== false) {
      monAll.push(m);
      if (matchesFilter(m, q)) monVis.push(m);
    }
    if (m.flt === true) {
      fltAll.push(m);
      if (matchesFilter(m, q)) fltVis.push(m);
    }
  }

  if (state.sortCol !== 'seq' || !state.sortAsc) {
    monVis.sort(compareMsgs);
    fltVis.sort(compareMsgs);
  }

  const monTitle = document.getElementById('monTitle');
  if (monTitle) {
    monTitle.textContent = q ? `Monitored Messages (${monVis.length} / ${monAll.length})` : `Monitored Messages (${monVis.length})`;
  }
  const fltTitle = document.getElementById('fltTitle');
  if (fltTitle) {
    fltTitle.textContent = q ? `Filtered Messages (${fltVis.length} / ${fltAll.length})` : `Filtered Messages (${fltVis.length})`;
  }

  const cardsActive = useCardLayout();
  const monCards = document.getElementById('monCards');
  const monTable = document.getElementById('monTable');
  const monBody  = document.getElementById('monBody');
  const fltCards = document.getElementById('fltCards');
  const fltTable = document.getElementById('fltTable');
  const fltBody  = document.getElementById('fltBody');

  if (cardsActive) {
    if (monCards) {
      monCards.style.display = 'flex';
      monCards.innerHTML = buildCardsHtml(monVis);
    }
    if (monTable) monTable.style.display = 'none';
    if (fltCards) {
      fltCards.style.display = 'flex';
      fltCards.innerHTML = buildCardsHtml(fltVis);
    }
    if (fltTable) fltTable.style.display = 'none';
  } else {
    if (monCards) monCards.style.display = 'none';
    if (monTable) monTable.style.display = 'table';
    if (monBody) monBody.innerHTML = buildRowsHtml(monVis);
    if (fltCards) fltCards.style.display = 'none';
    if (fltTable) fltTable.style.display = 'table';
    if (fltBody) fltBody.innerHTML = buildRowsHtml(fltVis);
  }

  if (state.autoScroll && !state.selectedUid && state.sortCol === 'seq' && state.sortAsc) {
    const ms = document.getElementById('monScroll');
    if (ms) ms.scrollTop = ms.scrollHeight;
    const fs = document.getElementById('fltScroll');
    if (fs) fs.scrollTop = fs.scrollHeight;
  }
}

function setViewMode(mode) {
  state.viewMode = mode;
  applyToolbarState();
  saveMetaNow();
  if (mode === 'catalog') {
    loadCatalog();
  }
}

function applyToolbarState() {
  const bSplit = document.getElementById('btnViewSplit');
  const bMon   = document.getElementById('btnViewMon');
  const bFlt   = document.getElementById('btnViewFlt');
  const bCat   = document.getElementById('btnViewCatalog');

  if (bSplit) bSplit.className = state.viewMode === 'split' ? 'active' : '';
  if (bMon)   bMon.className   = state.viewMode === 'monitor' ? 'active' : '';
  if (bFlt)   bFlt.className   = state.viewMode === 'filtered' ? 'active' : '';
  if (bCat)   bCat.className   = state.viewMode === 'catalog' ? 'active' : '';

  const bLayout = document.getElementById('btnLayoutMode');
  if (bLayout) {
    const eff = useCardLayout() ? 'Cards' : 'Table';
    const labelMap = { auto: `View: Auto (${eff})`, cards: 'View: Cards', table: 'View: Table' };
    bLayout.textContent = labelMap[state.layoutMode] || 'View: Auto';
    bLayout.className = state.layoutMode !== 'auto' ? 'active' : '';
  }

  const liveWs = document.getElementById('liveWorkspace');
  const catWs  = document.getElementById('catalogWorkspace');
  const pMon   = document.getElementById('paneMonitor');
  const pFlt   = document.getElementById('paneFiltered');

  if (state.viewMode === 'catalog') {
    if (liveWs) liveWs.style.display = 'none';
    if (catWs) catWs.style.display = 'flex';
    applyMobileCatalogPaneState();
  } else {
    if (liveWs) liveWs.style.display = 'flex';
    if (catWs) catWs.style.display = 'none';
    if (state.viewMode === 'monitor') {
      if (pMon) pMon.style.display = 'flex';
      if (pFlt) pFlt.style.display = 'none';
    } else if (state.viewMode === 'filtered') {
      if (pMon) pMon.style.display = 'none';
      if (pFlt) pFlt.style.display = 'flex';
    } else {
      if (pMon) pMon.style.display = 'flex';
      if (pFlt) pFlt.style.display = 'flex';
    }
  }

  const btnAS = document.getElementById('btnAutoScroll');
  if (btnAS) {
    btnAS.className = state.autoScroll ? 'active' : '';
    btnAS.textContent = isMobileViewport()
      ? (state.autoScroll ? 'Auto-Scroll' : 'Scroll: Off')
      : (state.autoScroll ? 'Auto-Scroll: ON' : 'Auto-Scroll: OFF');
  }
  const btnHT = document.getElementById('btnHideTone');
  if (btnHT) {
    btnHT.className = state.hideTone ? 'active' : '';
    btnHT.textContent = state.hideTone ? 'Tone-Only: Hidden' : 'Tone-Only: Show';
  }
}

function toggleAutoScroll() {
  state.autoScroll = !state.autoScroll;
  applyToolbarState();
  saveMetaNow();
  if (state.autoScroll) scheduleRender(true);
}

function toggleLocalPause() {
  state.localPaused = !state.localPaused;
  const btn = document.getElementById('btnPause');
  if (btn) {
    btn.className = state.localPaused ? 'active' : '';
    btn.textContent = state.localPaused ? 'Resume' : 'Pause';
  }
  if (!state.localPaused) scheduleRender(true);
}

function toggleHideTone() {
  state.hideTone = !state.hideTone;
  applyToolbarState();
  saveMetaNow();
  scheduleRender(true);
}

function onSearchInput() {
  state.searchQuery = document.getElementById('searchInput').value || '';
  scheduleRender(true);
}

function clearSearch() {
  document.getElementById('searchInput').value = '';
  state.searchQuery = '';
  scheduleRender(true);
}

function onCapcodeClick(cap, uid) {
  if (uid) {
    onSelectRow(uid);
  }
  const inp = document.getElementById('searchInput');
  if (state.searchQuery.toLowerCase() === cap.toLowerCase()) {
    state.searchQuery = '';
    if (inp) inp.value = '';
  } else {
    state.searchQuery = cap;
    if (inp) inp.value = cap;
  }
  scheduleRender(true);
}

function setSort(col) {
  if (state.sortCol === col) {
    state.sortAsc = !state.sortAsc;
  } else {
    state.sortCol = col;
    state.sortAsc = true;
  }
  for (const c of ['capcode', 'seq', 'freq_hz', 'mode', 'msg_type', 'bitrate', 'message']) {
    const el = document.getElementById('sort_' + c);
    if (el) el.textContent = (state.sortCol === c) ? (state.sortAsc ? '▲' : '▼') : '';
  }
  scheduleRender(true);
}

function onSelectRow(uid) {
  state.selectedUid = uid;
  const msg = state.messages.find(m => m.uid === uid);
  const insp = document.getElementById('inspector');
  if (!msg || !insp) return;

  state.inspectedCapcode = msg.capcode;
  document.getElementById('inspCapcode').textContent = `Capcode: ${msg.capcode}`;
  const freqTxt = formatFreqMhz(msg.freq_hz);
  const grpTxt = msg.group_address ? ` | Group: ${msg.group_address}` : '';
  const fragTxt = msg.fragment_info ? ` | ${msg.fragment_info}` : '';
  document.getElementById('inspMeta').textContent = `${msg.date} ${msg.time} | ${freqTxt} MHz | ${msg.mode} | ${msg.msg_type} | ${msg.bitrate} bps${grpTxt}${fragTxt}`;
  const lblEl = document.getElementById('inspLabel');
  if (msg.filter_label) {
    const cIdx = (msg.filter_color_idx || 0) % LABEL_PALETTE.length;
    lblEl.style.color = LABEL_PALETTE[cIdx];
    lblEl.textContent = `[${msg.filter_label}]`;
  } else {
    lblEl.textContent = '';
  }
  document.getElementById('inspMessage').textContent = msg.message || '';
  insp.classList.add('open');
  fetchCapcodeProfileForInspector(msg.capcode);
  scheduleRender(true);
}

async function fetchCapcodeProfileForInspector(capcode) {
  const bar = document.getElementById('inspProfileBar');
  if (!bar) return;
  try {
    const res = await fetch(`/api/catalog/detail?capcode=${encodeURIComponent(capcode)}`, { cache: 'no-store' });
    if (!res.ok) return;
    const data = await res.json();
    if (!data.profile) {
      bar.textContent = `Capcode ${capcode}: Not currently indexed in Capcode Catalog DB`;
      return;
    }
    const p = data.profile;
    const rates = (p.seen_bitrates || []).join(', ') || String(p.last_bitrate || '-');
    const modes = (p.modes_seen || []).join(', ') || '-';
    const freqs = (p.frequencies_hz || []).map(f => (f / 1e6).toFixed(5) + ' MHz').join(', ') || 'Audio';
    const grps  = (p.flex_group_addresses || []).join(', ') || 'None';
    const aliasTxt = p.alias ? ` (${p.alias})` : '';
    bar.textContent = `Directory Profile ${p.capcode}${aliasTxt} — Total Msgs: ${p.message_count} | First Seen: ${p.first_seen_date} ${p.first_seen_time} | Last Seen: ${p.last_seen_date} ${p.last_seen_time} | Baud: ${rates} bps | Modes: ${modes} | Freqs: ${freqs} | FLEX Groupcalls: ${grps}`;
  } catch (e) {}
}

function openSelectedInCatalog() {
  if (!state.inspectedCapcode) return;
  const cap = state.inspectedCapcode;
  closeInspector();
  setViewMode('catalog');
  selectCatalogCapcode(cap);
}

async function deleteInspectedCapcodeFromDb() {
  if (!state.inspectedCapcode) return;
  try {
    await fetch(`/api/catalog/delete?capcode=${encodeURIComponent(state.inspectedCapcode)}`, { method: 'POST' });
    fetchCapcodeProfileForInspector(state.inspectedCapcode);
    if (state.viewMode === 'catalog') loadCatalog();
  } catch (e) {}
}

function closeInspector() {
  state.selectedUid = null;
  state.inspectedCapcode = null;
  document.getElementById('inspector').classList.remove('open');
  scheduleRender(true);
}

function copySelectedMsg() {
  const msg = state.messages.find(m => m.uid === state.selectedUid);
  if (msg && navigator.clipboard) navigator.clipboard.writeText(msg.message || '');
}

function copySelectedCapcode() {
  const msg = state.messages.find(m => m.uid === state.selectedUid);
  if (msg && navigator.clipboard) navigator.clipboard.writeText(msg.capcode || '');
}

function isolateSelectedCapcode() {
  const msg = state.messages.find(m => m.uid === state.selectedUid);
  if (msg) onCapcodeClick(msg.capcode);
}

// 4. Capcode Catalog View Logic (Desktop Table + Mobile Cards)
function applyMobileCatalogPaneState() {
  const listPane = document.getElementById('catListPane');
  const detailPane = document.getElementById('catDetailPane');
  const backBtn = document.getElementById('btnCatBackMobile');
  if (!listPane || !detailPane) return;

  if (isMobileViewport()) {
    if (state.selectedCatalogCapcode) {
      listPane.style.display = 'none';
      detailPane.style.display = 'flex';
      detailPane.style.flex = '1';
      if (backBtn) backBtn.style.display = 'inline-block';
    } else {
      listPane.style.display = 'flex';
      listPane.style.flex = '1';
      detailPane.style.display = 'none';
      if (backBtn) backBtn.style.display = 'none';
    }
  } else {
    listPane.style.display = 'flex';
    listPane.style.flex = '52';
    detailPane.style.display = 'flex';
    detailPane.style.flex = '48';
    if (backBtn) backBtn.style.display = 'none';
  }
}

function closeMobileCatalogDetail() {
  state.selectedCatalogCapcode = null;
  applyMobileCatalogPaneState();
  renderCatalogTable();
}

async function loadCatalog() {
  try {
    const res = await fetch('/api/catalog', { cache: 'no-store' });
    if (!res.ok) return;
    const data = await res.json();
    state.catalogProfiles = Array.isArray(data.profiles) ? data.profiles : [];
    const btnCat = document.getElementById('btnViewCatalog');
    if (btnCat) {
      btnCat.textContent = isMobileViewport()
        ? `Catalog (${state.catalogProfiles.length})`
        : `Capcode Catalog (${state.catalogProfiles.length})`;
    }
    renderCatalogTable();
    if (state.selectedCatalogCapcode) {
      selectCatalogCapcode(state.selectedCatalogCapcode);
    }
  } catch (e) {}
}

function setCatSort(col) {
  if (state.catSortCol === col) {
    state.catSortAsc = !state.catSortAsc;
  } else {
    state.catSortCol = col;
    state.catSortAsc = col === 'capcode' || col === 'alias';
  }
  renderCatalogTable();
}

function renderCatalogTable() {
  const q = ((document.getElementById('catSearchInput') || {}).value || '').trim().toLowerCase();
  const filtered = state.catalogProfiles.filter(p => {
    if (!q) return true;
    return (p.capcode && p.capcode.toLowerCase().includes(q)) ||
           (p.alias && p.alias.toLowerCase().includes(q)) ||
           (p.latest_message && p.latest_message.toLowerCase().includes(q)) ||
           (p.flex_group_addresses || []).some(g => g.toLowerCase().includes(q)) ||
           (p.modes_seen || []).some(m => m.toLowerCase().includes(q));
  });

  filtered.sort((a, b) => {
    let cmp = 0;
    if (state.catSortCol === 'message_count') cmp = (a.message_count || 0) - (b.message_count || 0);
    else if (state.catSortCol === 'alias') cmp = (a.alias || '').localeCompare(b.alias || '');
    else if (state.catSortCol === 'first_seen') cmp = (`${a.first_seen_date} ${a.first_seen_time}`).localeCompare(`${b.first_seen_date} ${b.first_seen_time}`);
    else if (state.catSortCol === 'last_seen') cmp = (`${a.last_seen_date} ${a.last_seen_time}`).localeCompare(`${b.last_seen_date} ${b.last_seen_time}`);
    else cmp = (a.capcode || '').localeCompare(b.capcode || '');
    return state.catSortAsc ? cmp : -cmp;
  });

  const titleEl = document.getElementById('catTitle');
  if (titleEl) {
    titleEl.textContent = q
      ? `Capcode Catalog (${filtered.length} / ${state.catalogProfiles.length})`
      : `Capcode Catalog (${state.catalogProfiles.length} capcodes)`;
  }

  const cardsActive = useCardLayout();
  const catCards = document.getElementById('catCards');
  const catTable = document.getElementById('catTable');
  const tbody    = document.getElementById('catBody');

  if (cardsActive) {
    if (catTable) catTable.style.display = 'none';
    if (catCards) {
      catCards.style.display = 'flex';
      if (filtered.length === 0) {
        catCards.innerHTML = `<div class="empty-state">No capcodes in catalog match the search query.</div>`;
      } else {
        let cHtml = '';
        for (const p of filtered) {
          const isSel = p.capcode === state.selectedCatalogCapcode;
          const safeCap = escapeHtml(p.capcode);
          const rates = (p.seen_bitrates || []).join(', ') || String(p.last_bitrate || '-');
          const modes = (p.modes_seen || []).join(', ') || '-';
          const freqs = (p.frequencies_hz || []).map(f => (f / 1e6).toFixed(5)).join(', ') || 'Audio';
          const grps  = (p.flex_group_addresses || []).join(', ');
          cHtml += `<div class="msg-card${isSel ? ' selected' : ''}" onclick="selectCatalogCapcode('${safeCap}')">` +
            `<div class="msg-card-head">` +
              `<span class="card-capcode">${safeCap}</span>` +
              (p.alias ? `<span style="color:#50ff78;font-weight:600;">${escapeHtml(p.alias)}</span>` : '') +
              `<span class="card-pill mode">${p.message_count || 0} msgs</span>` +
              `<span class="card-pill freq">${escapeHtml(freqs)} MHz</span>` +
              `<button class="danger" style="margin-left:auto;padding:2px 7px;font-size:11px;min-height:22px;" onclick="event.stopPropagation();deleteCapcodeRow('${safeCap}')">Delete</button>` +
            `</div>` +
            `<div style="font-size:11.5px;color:var(--text-muted);font-family:monospace;">` +
              `Last: ${escapeHtml(p.last_seen_date)} ${escapeHtml(p.last_seen_time)} • ${escapeHtml(modes)} @ ${escapeHtml(rates)} bps` +
              (grps ? ` • <span style="color:#facc15;">GRP: ${escapeHtml(grps)}</span>` : '') +
            `</div>` +
            (p.latest_message ? `<div class="msg-card-body msg-alpha">${escapeHtml(p.latest_message)}</div>` : '') +
          `</div>`;
        }
        catCards.innerHTML = cHtml;
      }
    }
    return;
  }

  if (catCards) catCards.style.display = 'none';
  if (catTable) catTable.style.display = 'table';
  if (!tbody) return;
  let html = '';
  for (const p of filtered) {
    const sel = p.capcode === state.selectedCatalogCapcode ? ' class="selected"' : '';
    const safeCap = escapeHtml(p.capcode);
    const rates = (p.seen_bitrates || []).join(', ') || String(p.last_bitrate || '-');
    const modes = (p.modes_seen || []).join(', ') || '-';
    const freqs = (p.frequencies_hz || []).map(f => (f / 1e6).toFixed(5)).join(', ') || 'Audio';
    const grps  = (p.flex_group_addresses || []).join(', ') || '-';

    html += `<tr${sel} onclick="selectCatalogCapcode('${safeCap}')">` +
      `<td class="col-addr">${safeCap}</td>` +
      `<td style="color:#50ff78;">${escapeHtml(p.alias || '')}</td>` +
      `<td style="color:#fff;font-weight:600;">${p.message_count || 0}</td>` +
      `<td class="col-time">${escapeHtml(p.first_seen_date)} ${escapeHtml(p.first_seen_time)}</td>` +
      `<td class="col-time">${escapeHtml(p.last_seen_date)} ${escapeHtml(p.last_seen_time)}</td>` +
      `<td class="col-rate">${escapeHtml(rates)}</td>` +
      `<td class="col-mode">${escapeHtml(modes)}</td>` +
      `<td class="col-freq">${escapeHtml(freqs)}</td>` +
      `<td style="color:#facc15;">${escapeHtml(grps)}</td>` +
      `<td class="msg-alpha">${escapeHtml(p.latest_message || '')}</td>` +
      `<td><button class="danger" style="padding:1px 6px;font-size:11px;" onclick="event.stopPropagation();deleteCapcodeRow('${safeCap}')">Delete</button></td>` +
      `</tr>`;
  }
  tbody.innerHTML = html;
}

async function selectCatalogCapcode(capcode) {
  state.selectedCatalogCapcode = capcode;
  applyMobileCatalogPaneState();
  renderCatalogTable();
  try {
    const res = await fetch(`/api/catalog/detail?capcode=${encodeURIComponent(capcode)}`, { cache: 'no-store' });
    if (!res.ok) return;
    const data = await res.json();
    const p = data.profile;
    const msgs = Array.isArray(data.messages) ? data.messages : [];
    const titleEl = document.getElementById('catDetailTitle');
    const ctrls = document.getElementById('catDetailControls');
    const msgCards = document.getElementById('catMsgCards');
    const msgTable = document.getElementById('catMsgTable');
    const msgBody  = document.getElementById('catMsgBody');

    if (!p) {
      if (titleEl) titleEl.textContent = 'Select a capcode above to view its profile & message history';
      if (ctrls) ctrls.style.display = 'none';
      if (msgBody) msgBody.innerHTML = '';
      if (msgCards) msgCards.innerHTML = '';
      return;
    }
    if (ctrls) ctrls.style.display = 'flex';
    document.getElementById('catAliasInput').value = p.alias || '';
    const grps = (p.flex_group_addresses || []).join(', ') || 'None';
    const rates = (p.seen_bitrates || []).join(', ') || String(p.last_bitrate || '-');
    if (titleEl) {
      titleEl.textContent = `Capcode ${p.capcode} — ${p.message_count} Msgs | First: ${p.first_seen_date} ${p.first_seen_time} | Last: ${p.last_seen_date} ${p.last_seen_time} | Baud: ${rates} | Groupcalls: ${grps}`;
    }

    if (useCardLayout()) {
      if (msgTable) msgTable.style.display = 'none';
      if (msgCards) {
        msgCards.style.display = 'flex';
        let cHtml = '';
        for (const m of msgs) {
          const freqTxt = formatFreqMhz(m.freq_hz);
          const badgeHtml = buildBadgesHtml(m);
          cHtml += `<div class="msg-card">` +
            `<div class="msg-card-head">` +
              `<span class="card-pill mode">${escapeHtml(m.mode)} ${m.bitrate || ''}</span>` +
              `<span class="card-pill type">${escapeHtml(m.msg_type)}</span>` +
              `<span class="card-pill freq">${escapeHtml(freqTxt)} MHz</span>` +
              `<span class="card-time">${escapeHtml(m.time)} ${escapeHtml(m.date)}</span>` +
            `</div>` +
            (badgeHtml ? `<div class="msg-card-badges">${badgeHtml}</div>` : '') +
            `<div class="msg-card-body msg-alpha">${escapeHtml(m.message)}</div>` +
          `</div>`;
        }
        msgCards.innerHTML = cHtml || `<div class="empty-state">No stored messages for this capcode.</div>`;
      }
    } else {
      if (msgCards) msgCards.style.display = 'none';
      if (msgTable) msgTable.style.display = 'table';
      let mHtml = '';
      for (const m of msgs) {
        const freqTxt = formatFreqMhz(m.freq_hz);
        const grpFrag = [m.group_address ? `GRP:${m.group_address}` : '', m.fragment_info || ''].filter(Boolean).join(' ');
        mHtml += `<tr>` +
          `<td class="col-time">${escapeHtml(m.time)}</td>` +
          `<td class="col-date">${escapeHtml(m.date)}</td>` +
          `<td class="col-freq">${escapeHtml(freqTxt)}</td>` +
          `<td class="col-mode">${escapeHtml(m.mode)}</td>` +
          `<td class="col-type">${escapeHtml(m.msg_type)}</td>` +
          `<td class="col-rate">${m.bitrate || ''}</td>` +
          `<td style="color:#facc15;">${escapeHtml(grpFrag)}</td>` +
          `<td class="msg-alpha">${escapeHtml(m.message)}</td>` +
          `</tr>`;
      }
      if (msgBody) msgBody.innerHTML = mHtml;
    }
  } catch (e) {}
}

async function saveCapcodeAlias() {
  if (!state.selectedCatalogCapcode) return;
  const alias = document.getElementById('catAliasInput').value || '';
  try {
    await fetch(`/api/catalog/alias?capcode=${encodeURIComponent(state.selectedCatalogCapcode)}&alias=${encodeURIComponent(alias)}`, { method: 'POST' });
    loadCatalog();
  } catch (e) {}
}

async function deleteCapcodeRow(capcode) {
  try {
    await fetch(`/api/catalog/delete?capcode=${encodeURIComponent(capcode)}`, { method: 'POST' });
    if (state.selectedCatalogCapcode === capcode) {
      state.selectedCatalogCapcode = null;
      const mb = document.getElementById('catMsgBody');
      if (mb) mb.innerHTML = '';
      const mc = document.getElementById('catMsgCards');
      if (mc) mc.innerHTML = '';
      applyMobileCatalogPaneState();
    }
    loadCatalog();
  } catch (e) {}
}

async function deleteSelectedCatalogCapcode() {
  if (!state.selectedCatalogCapcode) return;
  await deleteCapcodeRow(state.selectedCatalogCapcode);
}

async function clearEntireCatalog() {
  try {
    await fetch('/api/catalog/clear', { method: 'POST' });
    state.selectedCatalogCapcode = null;
    const mb = document.getElementById('catMsgBody');
    if (mb) mb.innerHTML = '';
    const mc = document.getElementById('catMsgCards');
    if (mc) mc.innerHTML = '';
    applyMobileCatalogPaneState();
    loadCatalog();
  } catch (e) {}
}

function onCacheLimitChange() {
  const val = parseInt(document.getElementById('cacheLimitSelect').value, 10);
  state.maxCacheSize = isNaN(val) ? 1000 : val;
  flushCacheToStorage();
}

function clearBrowserCache() {
  try {
    localStorage.removeItem(STORAGE_CACHE_KEY);
    localStorage.removeItem(STORAGE_META_KEY);
  } catch (e) {}
  state.cacheBytes = 0;
  state.lastDeltaCount = 0;
  updateCacheBadge();
}

async function clearAllMessages() {
  state.messages = [];
  state.uidSet.clear();
  closeInspector();
  clearBrowserCache();
  scheduleRender(true);
  try { await fetch('/api/action/clear', { method: 'POST' }); } catch (e) {}
}

async function triggerTest(kind) {
  try {
    await fetch(`/api/action/test_${kind}`, { method: 'POST' });
  } catch (e) {}
}

function exportCsv() {
  const rows = [['Address', 'Time', 'Date', 'Freq_MHz', 'Mode', 'Type', 'Bitrate', 'Group_Addr', 'Fragments', 'Label', 'Message']];
  for (const m of state.messages) {
    rows.push([
      m.capcode || '',
      m.time || '',
      m.date || '',
      formatFreqMhz(m.freq_hz),
      m.mode || '',
      m.msg_type || '',
      String(m.bitrate || ''),
      m.group_address || '',
      m.fragment_info || '',
      m.filter_label || '',
      '"' + String(m.message || '').replace(/"/g, '""') + '"'
    ]);
  }
  const csv = rows.map(r => r.join(',')).join('\r\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `pdw_rs_messages_${Date.now()}.csv`;
  a.click();
}

registerTouchGuard();
loadBrowserCache();
bootstrapFromServer();
connectSse();
</script>
</body>
</html>
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::catalog::CapcodeCatalogDb;
    use crate::pocsag::DecodedPagerMessage;
    use std::path::PathBuf;

    #[test]
    fn test_web_server_delta_bootstrap_and_catalog_integration() {
        let catalog_db = Arc::new(Mutex::new(CapcodeCatalogDb::new(PathBuf::from(
            "test_pdw_capcode_db.dat",
        ))));
        let (shared, cmd_rx) = WebServerSharedState::new(Arc::clone(&catalog_db), None);
        let msg1 = DecodedPagerMessage {
            capcode: "1234567".to_string(),
            time: "12:00:01".to_string(),
            date: "16-10-26".to_string(),
            mode: "POCSAG-1".to_string(),
            msg_type: "ALPHA".to_string(),
            bitrate: 1200,
            message: "FIRST WEB MESSAGE \"QUOTED\"".to_string(),
            has_bit_errors: false,
            soft_recovered_words: 0,
            filter_label: Some("DISPATCH".to_string()),
            filter_color_idx: Some(1),
            freq_hz: Some(929_612_500),
            group_address: None,
            fragment_info: None,
        };
        let msg2 = DecodedPagerMessage {
            capcode: "7654321".to_string(),
            time: "12:00:02".to_string(),
            date: "16-10-26".to_string(),
            mode: "FLEX-A".to_string(),
            msg_type: "ALPHA".to_string(),
            bitrate: 3200,
            message: "SECOND DELTA MESSAGE".to_string(),
            has_bit_errors: false,
            soft_recovered_words: 1,
            filter_label: None,
            filter_color_idx: None,
            freq_hz: Some(929_738_620),
            group_address: Some("2029568".to_string()),
            fragment_info: Some("2 frags".to_string()),
        };

        let seq1 = shared.record_message(&msg1, true, true);
        let seq2 = shared.record_message(&msg2, true, false);
        assert_eq!(seq1, 1);
        assert_eq!(seq2, 2);

        let full_json = shared.build_bootstrap_json(None, 0);
        assert!(full_json.contains("\"delta_mode\":false"));
        assert!(full_json.contains("FIRST WEB MESSAGE \\\"QUOTED\\\""));
        assert!(full_json.contains("\"freq_hz\":929738620"));
        assert!(full_json.contains("\"group_address\":\"2029568\""));
        assert!(full_json.contains("\"fragment_info\":\"2 frags\""));

        let delta_json = shared.build_bootstrap_json(Some(shared.session_id), 1);
        assert!(delta_json.contains("\"delta_mode\":true"));
        assert!(!delta_json.contains("FIRST WEB MESSAGE"));
        assert!(delta_json.contains("SECOND DELTA MESSAGE"));

        shared.dispatch_ui_command(WebUiCommand::TriggerTestPocsag);
        assert!(matches!(
            cmd_rx.try_recv(),
            Ok(WebUiCommand::TriggerTestPocsag)
        ));
    }
}
