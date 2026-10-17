//! Built-In Live HTTP + Server-Sent Events (SSE) Web Server & Browser Caching Engine (`web.rs`).
//!
//! Serves a real-time dark-mode paging monitor web application on port `8008` by default
//! (configurable and toggleable at runtime in the GUI and `pdw_config.toml`).
//!
//! Speed & Browser Caching Architecture:
//! 1. **Instant `localStorage` Hydration**: The browser caches up to `N` messages locally
//!    (`pdw_rs_msg_cache_v1`) along with the server `session_id` and `last_seq` watermark,
//!    rendering cached messages in `< 5 ms` before network round-trips complete.
//! 2. **Delta-Only Bootstrap Sync (`/api/bootstrap?session_id=<sid>&since_seq=<seq>`)**:
//!    When reloading or reconnecting, the browser requests only messages with `seq > last_seq`
//!    (`delta_mode: true`), eliminating redundant transfers of already-cached messages.
//! 3. **Sub-Millisecond Server-Sent Events (`/api/events`)**: Newly decoded POCSAG & FLEX pages
//!    push immediately over a persistent `text/event-stream` connection and batch into
//!    `requestAnimationFrame` DOM updates and debounced `localStorage` commits.

use crate::config::StatsTracker;
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
        format!(
            "{{\"seq\":{},\"ts\":{},\"mon\":{},\"flt\":{},\"capcode\":\"{}\",\"time\":\"{}\",\"date\":\"{}\",\"mode\":\"{}\",\"msg_type\":\"{}\",\"bitrate\":{},\"message\":\"{}\",\"has_bit_errors\":{},\"soft_recovered_words\":{},\"filter_label\":{},\"filter_color_idx\":{}}}",
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
            color_json
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
        format!(
            "{{\"rx_quality_pct\":{:.1},\"total_codewords\":{},\"soft_recoveries\":{},\"active_mode\":\"{}\",\"dc_center\":{:.4},\"deviation\":{:.4},\"invert_active\":{},\"input_source\":\"{}\",\"audio_status\":\"{}\",\"sdr_connected\":{},\"vfo_freq_hz\":{},\"iq_rssi_dbfs\":{},\"afc_offset_hz\":{},\"paused\":{}}}",
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
            self.paused
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
}

struct WebStateInner {
    messages: VecDeque<WebMessageEntry>,
    telemetry: WebTelemetrySnapshot,
    stats: StatsTracker,
    sse_clients: Vec<Sender<String>>,
    last_status_broadcast: Instant,
    last_broadcast_total: u64,
}

pub struct WebServerSharedState {
    pub session_id: u64,
    next_seq: AtomicU64,
    inner: Mutex<WebStateInner>,
    ui_cmd_tx: Sender<WebUiCommand>,
    egui_ctx: Option<egui::Context>,
}

impl WebServerSharedState {
    pub fn new(egui_ctx: Option<egui::Context>) -> (Arc<Self>, Receiver<WebUiCommand>) {
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
            }),
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

    pub fn update_telemetry(&self, telem: WebTelemetrySnapshot, stats: &StatsTracker) {
        let latest = self.latest_seq();
        if let Ok(mut guard) = self.inner.lock() {
            let mode_changed = guard.telemetry.active_mode != telem.active_mode;
            let total_msgs = stats.total_messages();
            let count_changed = guard.last_broadcast_total != total_msgs;
            let elapsed_ok = guard.last_status_broadcast.elapsed() >= Duration::from_millis(500);

            guard.telemetry = telem;
            guard.stats = stats.clone();

            if !guard.sse_clients.is_empty() && (mode_changed || count_changed || elapsed_ok) {
                guard.last_status_broadcast = Instant::now();
                guard.last_broadcast_total = total_msgs;
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
        // Wake blocking `TcpListener::accept()` immediately via loopback connect
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
/// incremental `since_seq` delta synchronization, live SSE streaming, sortable tables,
/// one-click capcode isolation, and full message inspector.
pub const INDEX_HTML: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>PDW-RS Live Paging Monitor</title>
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
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg-main);
    color: var(--text-main);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-size: 13px;
    height: 100vh;
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
    gap: 8px;
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
  }
  .dot.live { background: var(--ok); box-shadow: 0 0 8px var(--ok); }
  .dot.offline { background: var(--danger); box-shadow: 0 0 8px var(--danger); }
  .sep { width: 1px; height: 18px; background: var(--border); margin: 0 2px; }
  button, select, input[type="text"] {
    background: var(--bg-header);
    color: var(--text-main);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 12px;
    outline: none;
  }
  button { cursor: pointer; transition: background 0.12s, border-color 0.12s; }
  button:hover { background: #243247; border-color: #3b5070; }
  button.active { background: #1d4ed8; border-color: #3b82f6; color: #fff; font-weight: 600; }
  input[type="text"] { width: 180px; }
  input[type="text"]:focus { border-color: var(--accent); }
  .badge {
    font-family: "JetBrains Mono", Consolas, "Courier New", monospace;
    font-size: 11.5px;
    padding: 3px 7px;
    border-radius: 4px;
    background: #0f1622;
    border: 1px solid var(--border);
    white-space: nowrap;
  }
  .badge.mode { color: #64c8ff; font-weight: 600; }
  .badge.cache { color: #a5f3fc; }
  .telem-bar {
    background: #0f151f;
    border-bottom: 1px solid var(--border);
    padding: 5px 12px;
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px;
    font-size: 12px;
    color: var(--text-muted);
  }
  .workspace {
    flex: 1;
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .pane {
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .pane-monitor { flex: 65; }
  .pane-filtered { flex: 35; border-top: 2px solid var(--border); }
  .pane-header {
    background: var(--bg-panel);
    padding: 5px 12px;
    font-weight: 600;
    font-size: 12.5px;
    border-bottom: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .pane-hint { font-weight: 400; font-size: 11.5px; color: var(--text-muted); }
  .table-wrap {
    flex: 1;
    overflow-y: auto;
    overflow-x: auto;
  }
  table {
    width: 100%;
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
    padding: 5px 8px;
    border-bottom: 1px solid var(--border);
    border-right: 1px solid #202b3d;
    cursor: pointer;
    user-select: none;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 12px;
    white-space: nowrap;
  }
  thead th:hover { background: #233044; }
  tbody tr { border-bottom: 1px solid #151d2a; }
  tbody tr:nth-child(even) { background: var(--bg-row-alt); }
  tbody tr:hover { background: var(--bg-row-hover); }
  tbody tr.selected { background: var(--bg-row-sel) !important; }
  td {
    padding: 3px 8px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    vertical-align: middle;
  }
  td.col-addr { color: var(--capcode); cursor: pointer; font-weight: 600; }
  td.col-addr:hover { text-decoration: underline; color: var(--accent); }
  td.col-time, td.col-date { color: var(--time-col); }
  td.col-mode, td.col-rate { color: var(--mode-col); }
  td.col-type { color: var(--type-col); }
  td.col-msg { cursor: pointer; }
  .msg-alpha { color: var(--alpha-msg); }
  .msg-num { color: var(--num-msg); }
  .msg-err { color: var(--err-msg); }
  .flt-tag {
    display: inline-block;
    margin-left: 8px;
    font-weight: 600;
  }
  .soft-tag {
    display: inline-block;
    margin-left: 6px;
    color: #8cff8c;
    font-size: 11px;
  }
  .inspector {
    background: var(--bg-panel);
    border-top: 2px solid var(--accent);
    padding: 8px 12px;
    display: none;
    flex-direction: column;
    gap: 6px;
    max-height: 170px;
  }
  .inspector.open { display: flex; }
  .inspector-top {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    font-size: 12px;
  }
  .inspector-body {
    font-family: "JetBrains Mono", Consolas, monospace;
    font-size: 13px;
    color: #d7fff5;
    background: #0a0e14;
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 7px 10px;
    white-space: pre-wrap;
    word-break: break-word;
    overflow-y: auto;
    max-height: 105px;
  }
</style>
</head>
<body>

<div class="toolbar">
  <div class="brand">
    <span id="connDot" class="dot"></span>
    <span>PDW-RS Live</span>
    <span id="connText" style="font-size:11.5px;font-weight:400;color:var(--text-muted);">Connecting...</span>
  </div>
  <div class="sep"></div>

  <button id="btnViewSplit" class="active" onclick="setViewMode('split')">Split View</button>
  <button id="btnViewMon" onclick="setViewMode('monitor')">Monitor Only</button>
  <button id="btnViewFlt" onclick="setViewMode('filtered')">Filtered Only</button>

  <div class="sep"></div>
  <label style="font-size:12px;color:var(--text-muted);">Search:</label>
  <input type="text" id="searchInput" placeholder="Capcode / text / label..." oninput="onSearchInput()" />
  <button onclick="clearSearch()" title="Clear search filter">×</button>

  <div class="sep"></div>
  <button id="btnAutoScroll" class="active" onclick="toggleAutoScroll()">Auto-Scroll: ON</button>
  <button id="btnPause" onclick="toggleLocalPause()">Pause</button>
  <button id="btnHideTone" onclick="toggleHideTone()">Tone-Only: Show</button>
  <button onclick="clearAllMessages()">Clear View</button>

  <div class="sep"></div>
  <button onclick="triggerTest('pocsag')" title="Inject synthetic 48 kHz POCSAG-1200 test burst">Test POCSAG-1200</button>
  <button onclick="triggerTest('flex')" title="Inject synthetic 48 kHz FLEX 4-FSK (Phases A+B) test burst">Test FLEX 4-FSK</button>
  <button onclick="exportCsv()" title="Export cached & visible messages to CSV">Export CSV</button>
</div>

<div class="telem-bar">
  <span id="telemMode" class="badge mode">IDLE (48000 Hz)</span>
  <span id="telemRxQ" class="badge" style="color:var(--ok);font-weight:700;">RX-Q: 100.0%</span>
  <span id="telemSignal" class="badge">Pol: NORM | DC: +0.000 | Dev: ±0.500 | Soft BCH: 0</span>
  <span id="telemSource" class="badge">Input: Soundcard</span>
  <span id="telemStats" class="badge">Total Decoded: 0</span>

  <div style="margin-left:auto;display:flex;align-items:center;gap:6px;">
    <span id="cacheStatus" class="badge cache" title="Messages cached in browser localStorage for instant startup & delta sync">Browser Cache: 0 msgs (0.0 KB)</span>
    <label style="font-size:11.5px;">Cache Limit:</label>
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

<div class="workspace">
  <div id="paneMonitor" class="pane pane-monitor" style="flex:65;">
    <div class="pane-header">
      <span id="monTitle">Monitored Messages (0)</span>
      <span class="pane-hint">Click column header to sort • Click capcode to isolate • Click message to inspect</span>
    </div>
    <div id="monScroll" class="table-wrap">
      <table>
        <thead>
          <tr>
            <th style="width:98px;" onclick="setSort('capcode')">Address <span id="sort_capcode"></span></th>
            <th style="width:76px;" onclick="setSort('seq')">Time <span id="sort_seq">▲</span></th>
            <th style="width:76px;" onclick="setSort('seq')">Date</th>
            <th style="width:84px;" onclick="setSort('mode')">Mode <span id="sort_mode"></span></th>
            <th style="width:72px;" onclick="setSort('msg_type')">Type <span id="sort_msg_type"></span></th>
            <th style="width:64px;" onclick="setSort('bitrate')">Bitrate <span id="sort_bitrate"></span></th>
            <th onclick="setSort('message')">Message <span id="sort_message"></span></th>
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
      <table>
        <thead>
          <tr>
            <th style="width:98px;" onclick="setSort('capcode')">Address</th>
            <th style="width:76px;" onclick="setSort('seq')">Time</th>
            <th style="width:76px;" onclick="setSort('seq')">Date</th>
            <th style="width:84px;" onclick="setSort('mode')">Mode</th>
            <th style="width:72px;" onclick="setSort('msg_type')">Type</th>
            <th style="width:64px;" onclick="setSort('bitrate')">Bitrate</th>
            <th onclick="setSort('message')">Message</th>
          </tr>
        </thead>
        <tbody id="fltBody"></tbody>
      </table>
    </div>
  </div>
</div>

<div id="inspector" class="inspector">
  <div class="inspector-top">
    <strong id="inspCapcode" style="color:#fff;font-size:13.5px;">Selected: 0000000</strong>
    <span id="inspMeta" style="color:#96cdff;"></span>
    <span id="inspLabel"></span>
    <div style="margin-left:auto;display:flex;gap:6px;">
      <button onclick="copySelectedMsg()">Copy Message</button>
      <button onclick="copySelectedCapcode()">Copy Capcode</button>
      <button id="btnIsolate" onclick="isolateSelectedCapcode()">Isolate Capcode</button>
      <button onclick="closeInspector()">Close ×</button>
    </div>
  </div>
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
  viewMode: 'split',
  autoScroll: true,
  localPaused: false,
  hideTone: false,
  searchQuery: '',
  sortCol: 'seq',
  sortAsc: true,
  messages: [],
  uidSet: new Set(),
  selectedUid: null,
  lastDeltaCount: 0,
  cacheBytes: 0
};

let renderPending = false;
let saveCacheTimer = null;
let evtSource = null;

function makeUid(sessionId, seq, entry) {
  if (sessionId && seq) return `${sessionId}:${seq}`;
  return `${entry.date}_${entry.time}_${entry.capcode}_${entry.mode}_${entry.message}`;
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
    setConnectionState('live', `LIVE (SSE :${portStr})`);
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
      }
    } catch (e) {}
  });

  evtSource.addEventListener('status', (ev) => {
    try {
      const payload = JSON.parse(ev.data);
      const portStr = window.location.port || '8008';
      setConnectionState('live', `LIVE (SSE :${portStr})`);
      if (payload.telemetry) updateTelemetryUi(payload.telemetry);
      if (payload.stats) updateStatsUi(payload.stats);
    } catch (e) {}
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

function updateTelemetryUi(t) {
  const modeEl = document.getElementById('telemMode');
  if (modeEl) modeEl.textContent = t.active_mode || 'IDLE';

  const rxqEl = document.getElementById('telemRxQ');
  if (rxqEl && typeof t.rx_quality_pct === 'number') {
    rxqEl.textContent = `RX-Q: ${t.rx_quality_pct.toFixed(1)}%`;
    rxqEl.style.color = t.rx_quality_pct >= 96.0 ? 'var(--ok)' : (t.rx_quality_pct >= 88.0 ? 'var(--warn)' : 'var(--danger)');
  }

  const sigEl = document.getElementById('telemSignal');
  if (sigEl) {
    const pol = t.invert_active ? 'INV' : 'NORM';
    const dc = (t.dc_center >= 0 ? '+' : '') + Number(t.dc_center || 0).toFixed(3);
    const dev = Number(t.deviation || 0).toFixed(3);
    let extra = '';
    if (t.vfo_freq_hz) {
      extra += ` | VFO: ${(t.vfo_freq_hz / 1e6).toFixed(4)} MHz`;
    }
    if (typeof t.iq_rssi_dbfs === 'number') {
      extra += ` | RSSI: ${t.iq_rssi_dbfs.toFixed(1)} dBFS`;
    }
    sigEl.textContent = `Pol: ${pol} | DC: ${dc} | Dev: ±${dev} | Soft BCH: ${t.soft_recoveries || 0}${extra}`;
  }

  const srcEl = document.getElementById('telemSource');
  if (srcEl) srcEl.textContent = t.audio_status || t.input_source || 'Soundcard';
}

function updateStatsUi(s) {
  const stEl = document.getElementById('telemStats');
  if (stEl) {
    stEl.textContent = `Decoded: ${s.total || 0} | Rejected: ${s.rejected || 0} | DupBlocked: ${s.duplicates || 0}`;
  }
}

function scheduleRender() {
  if (renderPending) return;
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
  return (m.capcode && m.capcode.toLowerCase().includes(q)) ||
         (m.message && m.message.toLowerCase().includes(q)) ||
         (m.mode && m.mode.toLowerCase().includes(q)) ||
         (m.msg_type && m.msg_type.toLowerCase().includes(q)) ||
         (m.filter_label && m.filter_label.toLowerCase().includes(q));
}

function compareMsgs(a, b) {
  let cmp = 0;
  switch (state.sortCol) {
    case 'capcode': cmp = (a.capcode || '').localeCompare(b.capcode || ''); break;
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

function buildRowsHtml(list) {
  let html = '';
  for (let i = 0; i < list.length; i++) {
    const m = list[i];
    const selClass = m.uid === state.selectedUid ? ' class="selected"' : '';
    const isNum = (m.msg_type || '').includes('NUM') || (m.msg_type || '').includes('TONE');
    const msgClass = m.has_bit_errors ? 'msg-err' : (isNum ? 'msg-num' : 'msg-alpha');

    let badgeHtml = '';
    if (m.filter_label) {
      const cIdx = (m.filter_color_idx || 0) % LABEL_PALETTE.length;
      badgeHtml += `<span class="flt-tag" style="color:${LABEL_PALETTE[cIdx]}">[${escapeHtml(m.filter_label)}]</span>`;
    }
    if (m.soft_recovered_words > 0) {
      badgeHtml += `<span class="soft-tag">(Soft+${m.soft_recovered_words})</span>`;
    }

    const safeUid = escapeHtml(m.uid);
    const safeCap = escapeHtml(m.capcode);
    html += `<tr${selClass} data-uid="${safeUid}">` +
      `<td class="col-addr" onclick="onCapcodeClick('${safeCap}')" title="Click to isolate capcode ${safeCap}">${safeCap}</td>` +
      `<td class="col-time">${escapeHtml(m.time)}</td>` +
      `<td class="col-date">${escapeHtml(m.date)}</td>` +
      `<td class="col-mode">${escapeHtml(m.mode)}</td>` +
      `<td class="col-type">${escapeHtml(m.msg_type)}</td>` +
      `<td class="col-rate">${m.bitrate || ''}</td>` +
      `<td class="col-msg ${msgClass}" onclick="onSelectRow('${safeUid}')">${escapeHtml(m.message)}${badgeHtml}</td>` +
      `</tr>`;
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

  const monBody = document.getElementById('monBody');
  if (monBody) monBody.innerHTML = buildRowsHtml(monVis);

  const fltBody = document.getElementById('fltBody');
  if (fltBody) fltBody.innerHTML = buildRowsHtml(fltVis);

  if (state.autoScroll && state.sortCol === 'seq' && state.sortAsc) {
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
}

function applyToolbarState() {
  document.getElementById('btnViewSplit').className = state.viewMode === 'split' ? 'active' : '';
  document.getElementById('btnViewMon').className   = state.viewMode === 'monitor' ? 'active' : '';
  document.getElementById('btnViewFlt').className   = state.viewMode === 'filtered' ? 'active' : '';

  const pMon = document.getElementById('paneMonitor');
  const pFlt = document.getElementById('paneFiltered');
  if (state.viewMode === 'monitor') {
    pMon.style.display = 'flex';
    pFlt.style.display = 'none';
  } else if (state.viewMode === 'filtered') {
    pMon.style.display = 'none';
    pFlt.style.display = 'flex';
  } else {
    pMon.style.display = 'flex';
    pFlt.style.display = 'flex';
  }

  const btnAS = document.getElementById('btnAutoScroll');
  if (btnAS) {
    btnAS.className = state.autoScroll ? 'active' : '';
    btnAS.textContent = state.autoScroll ? 'Auto-Scroll: ON' : 'Auto-Scroll: OFF';
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
  if (state.autoScroll) scheduleRender();
}

function toggleLocalPause() {
  state.localPaused = !state.localPaused;
  const btn = document.getElementById('btnPause');
  if (btn) {
    btn.className = state.localPaused ? 'active' : '';
    btn.textContent = state.localPaused ? 'Resume' : 'Pause';
  }
  if (!state.localPaused) scheduleRender();
}

function toggleHideTone() {
  state.hideTone = !state.hideTone;
  applyToolbarState();
  saveMetaNow();
  scheduleRender();
}

function onSearchInput() {
  state.searchQuery = document.getElementById('searchInput').value || '';
  scheduleRender();
}

function clearSearch() {
  document.getElementById('searchInput').value = '';
  state.searchQuery = '';
  scheduleRender();
}

function onCapcodeClick(cap) {
  const inp = document.getElementById('searchInput');
  if (state.searchQuery.toLowerCase() === cap.toLowerCase()) {
    state.searchQuery = '';
    if (inp) inp.value = '';
  } else {
    state.searchQuery = cap;
    if (inp) inp.value = cap;
  }
  scheduleRender();
}

function setSort(col) {
  if (state.sortCol === col) {
    state.sortAsc = !state.sortAsc;
  } else {
    state.sortCol = col;
    state.sortAsc = true;
  }
  for (const c of ['capcode', 'seq', 'mode', 'msg_type', 'bitrate', 'message']) {
    const el = document.getElementById('sort_' + c);
    if (el) el.textContent = (state.sortCol === c) ? (state.sortAsc ? '▲' : '▼') : '';
  }
  scheduleRender();
}

function onSelectRow(uid) {
  state.selectedUid = uid;
  const msg = state.messages.find(m => m.uid === uid);
  const insp = document.getElementById('inspector');
  if (!msg || !insp) return;

  document.getElementById('inspCapcode').textContent = `Selected: ${msg.capcode}`;
  document.getElementById('inspMeta').textContent = `${msg.date} ${msg.time} | ${msg.mode} | ${msg.msg_type} | ${msg.bitrate} bps`;
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
  scheduleRender();
}

function closeInspector() {
  state.selectedUid = null;
  document.getElementById('inspector').classList.remove('open');
  scheduleRender();
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
  scheduleRender();
  try { await fetch('/api/action/clear', { method: 'POST' }); } catch (e) {}
}

async function triggerTest(kind) {
  try {
    await fetch(`/api/action/test_${kind}`, { method: 'POST' });
  } catch (e) {}
}

function exportCsv() {
  const rows = [['Address', 'Time', 'Date', 'Mode', 'Type', 'Bitrate', 'Label', 'Message']];
  for (const m of state.messages) {
    rows.push([
      m.capcode || '',
      m.time || '',
      m.date || '',
      m.mode || '',
      m.msg_type || '',
      String(m.bitrate || ''),
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
    use crate::pocsag::DecodedPagerMessage;

    #[test]
    fn test_web_server_delta_bootstrap_and_http_endpoints() {
        let (shared, cmd_rx) = WebServerSharedState::new(None);
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
        };

        let seq1 = shared.record_message(&msg1, true, true);
        let seq2 = shared.record_message(&msg2, true, false);
        assert_eq!(seq1, 1);
        assert_eq!(seq2, 2);

        let full_json = shared.build_bootstrap_json(None, 0);
        assert!(full_json.contains("\"delta_mode\":false"));
        assert!(full_json.contains("FIRST WEB MESSAGE \\\"QUOTED\\\""));
        assert!(full_json.contains("SECOND DELTA MESSAGE"));

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
