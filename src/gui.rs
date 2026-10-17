//! Local Desktop GUI (`eframe` / `egui`) for `pdw-rs`
//!
//! Optimized for ultra-low CPU usage (< 1% CPU on software `opengl32.dll` / Mesa rasterizers):
//! - Event-driven GUI repainting: wakes immediately when a message is decoded, and otherwise
//!   repaints only once per second (1 FPS) when the live oscilloscope is toggled off.
//! - Toggleable `Scope` checkbox directly on the top toolbar (off by default to save CPU).
//! - Idle-gated POCSAG bit clock: skips POCSAG symbol clock division when `pocbit == 0`.
//! - Throttled telemetry mutex updates (~10 Hz instead of 100 Hz).

use crate::audio::{
    list_input_devices, start_audio_capture, synthesize_flex_4fsk_burst, synthesize_pocsag_burst,
    AudioCaptureStream,
};
use crate::bch::BchDecoder;
use crate::config::{
    is_human_readable_message, AppConfig, DuplicateBlocker, FilterAction, FilterRule, FilterType, FilterVerdict,
    InputSource, StatsTracker,
};
use crate::sdrconnect::{
    start_sdrconnect_iq_capture, synthesize_sdrconnect_iq_frame_from_baseband, IqFmDemodulator,
    SdrConnectControlCmd, SdrConnectParams, SdrConnectStream,
};
use crate::dsp::{
    AdaptiveEnvelopeTracker, AtbSlicer, AudioPreset, RxQualityTracker, COURSE_CLKT_HI,
    COURSE_CLKT_LO, INDEX1200, INDEX1600, INDEX2400, INDEX512, SUPPORTED_SAMPLE_RATES,
};
use crate::flex::FlexDecoder;
use crate::pocsag::{DecodedPagerMessage, PocsagChannelDecoder};
use crossbeam_channel::{bounded, Receiver, Sender};
use eframe::egui::{self, Color32, RichText, Stroke};
use egui_extras::{Column, TableBuilder};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;

/// Palette for filter labels (matching PDW's filter label colors)
const LABEL_PALETTE: [Color32; 8] = [
    Color32::from_rgb(80, 180, 255),  // Light Blue
    Color32::from_rgb(255, 235, 60),  // Yellow
    Color32::from_rgb(255, 85, 85),   // Red
    Color32::from_rgb(255, 170, 0),   // Orange
    Color32::from_rgb(80, 255, 120),  // Light Green
    Color32::from_rgb(80, 255, 255),  // Cyan
    Color32::from_rgb(255, 130, 220), // Pink/Magenta
    Color32::from_rgb(220, 220, 220), // White/Gray
];

/// Shared state snapshot sent from the real-time DSP worker thread to the GUI thread.
#[derive(Clone, Debug)]
pub struct DspTelemetry {
    pub rx_quality_pct: f64,
    pub total_codewords: u64,
    pub soft_recoveries: u64,
    pub active_mode: String,
    pub dc_center: f32,
    pub deviation: f32,
    pub invert_active: bool,
    pub scope_samples: Vec<f32>,
}

impl Default for DspTelemetry {
    fn default() -> Self {
        Self {
            rx_quality_pct: 100.0,
            total_codewords: 0,
            soft_recoveries: 0,
            active_mode: "IDLE (Listening 48 kHz)".to_string(),
            dc_center: 0.0,
            deviation: 0.5,
            invert_active: false,
            scope_samples: Vec::new(),
        }
    }
}

pub enum WorkerCommand {
    UpdateConfig(AppConfig),
    ResetRxQuality,
    InjectSamples(Vec<f32>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SortColumn {
    Arrival,
    Address,
    Mode,
    MsgType,
    Bitrate,
    Message,
}

#[derive(Default)]
struct TableUiEvents {
    clicked_capcode: Option<String>,
    selected_msg: Option<DecodedPagerMessage>,
    add_filter_for_msg: Option<DecodedPagerMessage>,
}

pub struct PdwApp {
    config_path: PathBuf,
    config: AppConfig,
    audio_stream: Option<AudioCaptureStream>,
    sdrconnect_stream: Option<SdrConnectStream>,
    audio_status: String,
    available_devices: Vec<String>,
    vfo_mhz_input: String,

    // Channels between audio thread, DSP worker thread, and GUI
    sample_tx: Sender<Vec<f32>>,
    cmd_tx: Sender<WorkerCommand>,
    msg_rx: Receiver<DecodedPagerMessage>,
    telemetry: Arc<Mutex<DspTelemetry>>,

    // UI message buffers, sorting, search & inspector state
    paused: bool,
    monitor_messages: VecDeque<DecodedPagerMessage>,
    filtered_messages: VecDeque<DecodedPagerMessage>,
    dup_blocker: DuplicateBlocker,
    stats: StatsTracker,
    search_query: String,
    sort_column: SortColumn,
    sort_ascending: bool,
    selected_message: Option<DecodedPagerMessage>,

    // Dialog visibility flags
    show_audio_setup: bool,
    show_options: bool,
    show_filters: bool,
    show_stats: bool,

    // Filter editor scratch state
    editing_filter: FilterRule,
    editing_filter_idx: Option<usize>,
    filter_ini_path: String,
    status_toast: String,
}

impl PdwApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Force dark theme so white capcodes and cyan/red message colors are always high-contrast
        cc.egui_ctx.set_visuals(egui::Visuals::dark());

        let config_path = PathBuf::from("pdw_config.toml");
        let mut config = AppConfig::load_or_default(&config_path);

        // Auto-import legacy `filters.ini` / `FILTERS.INI` if present and filters list is empty
        for flt_name in ["filters.ini", "FILTERS.INI", "Filters.ini"] {
            let legacy_ini = PathBuf::from(flt_name);
            if config.filters.is_empty() && legacy_ini.exists() {
                let _ = config.import_legacy_filters_ini(&legacy_ini);
                break;
            }
        }

        let (sample_tx, sample_rx) = bounded::<Vec<f32>>(256);
        let (cmd_tx, cmd_rx) = bounded::<WorkerCommand>(32);
        let (msg_tx, msg_rx) = bounded::<DecodedPagerMessage>(1024);
        let telemetry = Arc::new(Mutex::new(DspTelemetry::default()));

        // Spawn dedicated real-time DSP & protocol decoder thread
        let telemetry_clone = Arc::clone(&telemetry);
        let initial_cfg = config.clone();
        let egui_ctx = Some(cc.egui_ctx.clone());
        thread::spawn(move || {
            run_dsp_worker(
                initial_cfg,
                sample_rx,
                cmd_rx,
                msg_tx,
                telemetry_clone,
                egui_ctx,
            );
        });

        let available_devices = list_input_devices();
        let vfo_mhz_input = format!("{:.4}", (config.sdrconnect_vfo_freq_hz as f64) / 1_000_000.0);

        let mut app = Self {
            config_path,
            config,
            audio_stream: None,
            sdrconnect_stream: None,
            audio_status: String::new(),
            available_devices,
            vfo_mhz_input,
            sample_tx,
            cmd_tx,
            msg_rx,
            telemetry,
            paused: false,
            monitor_messages: VecDeque::with_capacity(1000),
            filtered_messages: VecDeque::with_capacity(500),
            dup_blocker: DuplicateBlocker::default(),
            stats: StatsTracker::default(),
            search_query: String::new(),
            sort_column: SortColumn::Arrival,
            sort_ascending: true,
            selected_message: None,
            show_audio_setup: false,
            show_options: false,
            show_filters: false,
            show_stats: false,
            editing_filter: FilterRule::default(),
            editing_filter_idx: None,
            filter_ini_path: "filters.ini".to_string(),
            status_toast: String::new(),
        };
        app.restart_audio();
        app
    }

    fn restart_audio(&mut self) {
        self.audio_stream = None;
        self.sdrconnect_stream = None;

        match self.config.input_source {
            InputSource::Soundcard => {
                let _ = self
                    .cmd_tx
                    .send(WorkerCommand::UpdateConfig(self.config.clone()));
                match start_audio_capture(
                    self.config.audio_device_name.as_deref(),
                    self.config.sample_rate,
                    self.sample_tx.clone(),
                ) {
                    Ok(stream) => {
                        if stream.actual_sample_rate != self.config.sample_rate {
                            self.config.sample_rate = stream.actual_sample_rate;
                            let _ = self
                                .cmd_tx
                                .send(WorkerCommand::UpdateConfig(self.config.clone()));
                        }
                        self.audio_status = format!(
                            "Soundcard: {} @ {} Hz",
                            stream.device_name, stream.actual_sample_rate
                        );
                        self.audio_stream = Some(stream);
                    }
                    Err(e) => {
                        self.audio_status =
                            format!("No hardware audio input ({}) — Test Generator Ready", e);
                    }
                }
            }
            InputSource::SdrConnectIq => {
                self.config.sample_rate = 48_000;
                let _ = self
                    .cmd_tx
                    .send(WorkerCommand::UpdateConfig(self.config.clone()));

                let params = SdrConnectParams {
                    host: self.config.sdrconnect_host.clone(),
                    port: self.config.sdrconnect_port,
                    target_sample_rate: self.config.sample_rate,
                    iq_gain: self.config.sdrconnect_iq_gain,
                    squelch_enabled: self.config.sdrconnect_squelch_enabled,
                    squelch_dbfs: self.config.sdrconnect_squelch_dbfs,
                    afc_enabled: self.config.sdrconnect_afc_enabled,
                    channel_bw_hz: self.config.sdrconnect_channel_bw_hz,
                    tune_on_connect: self.config.sdrconnect_tune_on_connect,
                    initial_vfo_freq_hz: self.config.sdrconnect_vfo_freq_hz,
                    use_audio_stream: self.config.sdrconnect_use_audio_stream,
                };
                let stream = start_sdrconnect_iq_capture(params, self.sample_tx.clone());
                self.audio_status = format!(
                    "SDRconnect IQ: ws://{}:{}/ ({} Hz)",
                    self.config.sdrconnect_host, self.config.sdrconnect_port, self.config.sample_rate
                );
                self.sdrconnect_stream = Some(stream);
            }
        }

        let _ = self.config.save(&self.config_path);
    }

    fn poll_decoded_messages(&mut self) {
        while let Ok(mut msg) = self.msg_rx.try_recv() {
            if self.paused {
                continue;
            }

            if self.config.hide_encrypted_alpha
                && !is_human_readable_message(&msg.message, &msg.msg_type)
            {
                self.stats.encrypted_hidden_count += 1;
                continue;
            }

            if self.config.block_duplicates
                && self.dup_blocker.is_duplicate(
                    &msg.capcode,
                    &msg.message,
                    self.config.block_duplicate_seconds,
                )
            {
                self.stats.blocked_duplicates += 1;
                continue;
            }

            let verdict = self.config.evaluate_message(&mut msg);
            match verdict {
                FilterVerdict::Rejected(_) => {
                    self.stats.rejected_count += 1;
                }
                FilterVerdict::Unmatched => {
                    self.stats.record(&msg);
                    self.config.log_message(&msg, false, None);
                    self.push_monitor(msg);
                }
                FilterVerdict::MatchedMonitorOnly(idx) => {
                    self.stats.record(&msg);
                    self.config.log_message(&msg, false, Some(idx));
                    self.push_monitor(msg);
                }
                FilterVerdict::MatchedFilterOnly(idx) => {
                    self.stats.record(&msg);
                    self.config.log_message(&msg, true, Some(idx));
                    self.push_filtered(msg);
                }
                FilterVerdict::MatchedBoth(idx) => {
                    self.stats.record(&msg);
                    self.config.log_message(&msg, true, Some(idx));
                    self.push_filtered(msg.clone());
                    self.push_monitor(msg);
                }
            }
        }
    }

    fn push_monitor(&mut self, msg: DecodedPagerMessage) {
        if self.monitor_messages.len() >= 1000 {
            self.monitor_messages.pop_front();
        }
        self.monitor_messages.push_back(msg);
    }

    fn push_filtered(&mut self, msg: DecodedPagerMessage) {
        if self.filtered_messages.len() >= 500 {
            self.filtered_messages.pop_front();
        }
        self.filtered_messages.push_back(msg);
    }
}

impl eframe::App for PdwApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if !ctx.style().visuals.dark_mode {
            ctx.set_visuals(egui::Visuals::dark());
        }
        self.poll_decoded_messages();

        // Low-CPU event-driven repaint schedule:
        // - New messages wake the UI immediately via `egui_ctx.request_repaint()` from the DSP worker.
        // - When `show_scope` is off (default), background telemetry refreshes at 1 FPS (1000 ms) instead of 20 FPS!
        // - When `show_scope` is on, the oscilloscope refreshes at a smooth 8 FPS (125 ms).
        let repaint_ms = if self.config.show_scope { 125 } else { 1000 };
        ctx.request_repaint_after(std::time::Duration::from_millis(repaint_ms));

        let telem = self
            .telemetry
            .lock()
            .map(|t| t.clone())
            .unwrap_or_default();

        // Top Toolbar & Optional Live 4-FSK Scope
        egui::TopBottomPanel::top("top_toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                let setup_btn_label = match self.config.input_source {
                    InputSource::Soundcard => "Input Setup (Soundcard)",
                    InputSource::SdrConnectIq => "Input Setup (SDRconnect IQ)",
                };
                if ui.button(setup_btn_label).clicked() {
                    self.available_devices = list_input_devices();
                    self.show_audio_setup = true;
                }
                if ui.button("Decoder Options").clicked() {
                    self.show_options = true;
                }
                if ui
                    .button(format!("Filters ({})", self.config.filters.len()))
                    .clicked()
                {
                    self.show_filters = true;
                }
                if ui.button("Statistics").clicked() {
                    self.show_stats = true;
                }

                let pause_label = if self.paused { "Resume" } else { "Pause" };
                if ui.button(pause_label).clicked() {
                    self.paused = !self.paused;
                }
                if ui.button("Clear Screen").clicked() {
                    self.monitor_messages.clear();
                    self.filtered_messages.clear();
                    self.selected_message = None;
                }

                ui.separator();
                ui.label("Search:");
                ui.add(
                    egui::TextEdit::singleline(&mut self.search_query)
                        .hint_text("Capcode / text...")
                        .desired_width(135.0),
                )
                .on_hover_text("Filter displayed rows by capcode, message text, mode, or label (click any capcode in the table to isolate it)");
                if !self.search_query.is_empty()
                    && ui
                        .small_button("×")
                        .on_hover_text("Clear search filter")
                        .clicked()
                {
                    self.search_query.clear();
                }

                ui.separator();
                if ui.button("Test POCSAG-1200").clicked() {
                    let raw_burst = synthesize_pocsag_burst(
                        self.config.sample_rate,
                        1200,
                        1_234_567,
                        4,
                        "PDW-RS 48KHZ POCSAG-1200 TEST MESSAGE OK",
                        0.02,
                    );
                    let burst = if self.config.input_source == InputSource::SdrConnectIq {
                        let iq_frame = synthesize_sdrconnect_iq_frame_from_baseband(
                            &raw_burst,
                            self.config.sample_rate,
                            0.0,
                        );
                        let mut test_demod = IqFmDemodulator::new(
                            self.config.sample_rate,
                            self.config.sample_rate,
                            self.config.sdrconnect_iq_gain,
                            false,
                            -90.0,
                            self.config.sdrconnect_afc_enabled,
                            self.config.sdrconnect_channel_bw_hz,
                        );
                        test_demod.process_binary_frame(&iq_frame).unwrap_or(raw_burst)
                    } else {
                        raw_burst
                    };
                    let _ = self.cmd_tx.send(WorkerCommand::InjectSamples(burst));
                }
                if ui.button("Test FLEX 4-FSK (Phases A+B)").clicked() {
                    let raw_burst = synthesize_flex_4fsk_burst(
                        self.config.sample_rate,
                        1_122_334,
                        "FLEX PHASE-A 4-FSK DECODE OK",
                        1_556_677,
                        "FLEX PHASE-B 4-FSK UNLOCKED!",
                        0.02,
                    );
                    let burst = if self.config.input_source == InputSource::SdrConnectIq {
                        let iq_frame = synthesize_sdrconnect_iq_frame_from_baseband(
                            &raw_burst,
                            self.config.sample_rate,
                            0.0,
                        );
                        let mut test_demod = IqFmDemodulator::new(
                            self.config.sample_rate,
                            self.config.sample_rate,
                            self.config.sdrconnect_iq_gain,
                            false,
                            -90.0,
                            self.config.sdrconnect_afc_enabled,
                            self.config.sdrconnect_channel_bw_hz,
                        );
                        test_demod.process_binary_frame(&iq_frame).unwrap_or(raw_burst)
                    } else {
                        raw_burst
                    };
                    let _ = self.cmd_tx.send(WorkerCommand::InjectSamples(burst));
                }
            });

            ui.separator();

            // Second toolbar row: Input status (Soundcard or SDRconnect IQ), Active Mode, RX-Q, and optional Scope
            ui.horizontal_wrapped(|ui| {
                if let Some(sdr) = &self.sdrconnect_stream {
                    if let Ok(st) = sdr.status.try_lock() {
                        let col = if st.connected {
                            Color32::from_rgb(90, 235, 130)
                        } else {
                            Color32::from_rgb(255, 185, 70)
                        };
                        let vfo_str = st
                            .vfo_freq_hz
                            .map(|hz| format!(" | VFO: {:.4} MHz", (hz as f64) / 1_000_000.0))
                            .unwrap_or_default();
                        let rssi_str = if st.connected {
                            let state_tag = if !st.squelch_open {
                                "SQL MUTED"
                            } else if st.carrier_detected {
                                "CARRIER"
                            } else {
                                "NOISE"
                            };
                            format!(
                                " | IQ: {:.1} dBFS ({}) | AFC: {:+.0} Hz",
                                st.iq_rssi_dbfs, state_tag, st.afc_offset_hz
                            )
                        } else {
                            String::new()
                        };
                        ui.label(
                            RichText::new(format!("{}{}{}", st.status_text, vfo_str, rssi_str))
                                .small()
                                .color(col),
                        );
                    } else {
                        ui.label(RichText::new(&self.audio_status).small());
                    }
                } else {
                    ui.label(RichText::new(&self.audio_status).small());
                }
                ui.separator();
                ui.label(
                    RichText::new(&telem.active_mode)
                        .color(Color32::from_rgb(100, 200, 255))
                        .strong(),
                );
                ui.separator();
                let q_color = if telem.rx_quality_pct >= 96.0 {
                    Color32::from_rgb(70, 230, 100)
                } else if telem.rx_quality_pct >= 88.0 {
                    Color32::from_rgb(255, 200, 60)
                } else {
                    Color32::from_rgb(255, 80, 80)
                };
                ui.label(
                    RichText::new(format!("RX-Q: {:.1}%", telem.rx_quality_pct))
                        .strong()
                        .color(q_color),
                );
                if ui
                    .small_button("Reset Q")
                    .on_hover_text("Reset RX-Quality EWMA")
                    .clicked()
                {
                    let _ = self.cmd_tx.send(WorkerCommand::ResetRxQuality);
                }
                ui.separator();
                let pol_str = if telem.invert_active { "INV" } else { "NORM" };
                ui.label(
                    RichText::new(format!(
                        "Pol: {} | DC: {:+.3} | Dev: ±{:.3} | Soft BCH: {}",
                        pol_str, telem.dc_center, telem.deviation, telem.soft_recoveries
                    ))
                    .small(),
                );
                ui.separator();
                let hide_enc_label = if self.stats.encrypted_hidden_count > 0 {
                    format!("Hide Encrypted ({})", self.stats.encrypted_hidden_count)
                } else {
                    "Hide Encrypted".to_string()
                };
                if ui
                    .checkbox(&mut self.config.hide_encrypted_alpha, hide_enc_label)
                    .on_hover_text("Hide Base64-encrypted ALPHA messages (uncheck to view raw encrypted traffic)")
                    .changed()
                {
                    let _ = self
                        .cmd_tx
                        .send(WorkerCommand::UpdateConfig(self.config.clone()));
                    let _ = self.config.save(&self.config_path);
                }
                ui.separator();
                if ui
                    .checkbox(&mut self.config.show_filter_pane, "Filter Pane")
                    .on_hover_text("Show or hide the lower Filtered Messages pane (uncheck for full-height Monitor view)")
                    .changed()
                {
                    let _ = self.config.save(&self.config_path);
                }
                ui.separator();
                if ui
                    .checkbox(&mut self.config.show_scope, "Scope")
                    .on_hover_text("Toggle live waveform oscilloscope (disabling saves CPU)")
                    .changed()
                {
                    let _ = self
                        .cmd_tx
                        .send(WorkerCommand::UpdateConfig(self.config.clone()));
                    let _ = self.config.save(&self.config_path);
                }

                if self.config.show_scope {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(220.0, 24.0), egui::Sense::hover());
                    let painter = ui.painter_at(rect);
                    painter.rect_filled(rect, 3.0, Color32::from_rgb(15, 20, 28));

                    let mid_y = rect.center().y;
                    painter.line_segment(
                        [
                            egui::pos2(rect.left(), mid_y),
                            egui::pos2(rect.right(), mid_y),
                        ],
                        Stroke::new(1.0_f32, Color32::from_rgb(70, 90, 110)),
                    );

                    if telem.scope_samples.len() > 1 {
                        let n = telem.scope_samples.len();
                        let pts: Vec<egui::Pos2> = telem
                            .scope_samples
                            .iter()
                            .enumerate()
                            .map(|(i, &s)| {
                                let x = rect.left() + (i as f32 / (n - 1) as f32) * rect.width();
                                let y = (mid_y - s * (rect.height() * 0.45))
                                    .clamp(rect.top() + 1.0, rect.bottom() - 1.0);
                                egui::pos2(x, y)
                            })
                            .collect();
                        painter.add(egui::Shape::line(
                            pts,
                            Stroke::new(1.2_f32, Color32::from_rgb(90, 240, 150)),
                        ));
                    }
                }
            });
        });

        // Optional Bottom Panel: Full-Message Word-Wrapped Inspector when a row is clicked
        if let Some(sel) = self.selected_message.clone() {
            egui::TopBottomPanel::bottom("selected_msg_inspector")
                .resizable(true)
                .default_height(78.0)
                .min_height(56.0)
                .max_height(200.0)
                .show(ctx, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.strong(format!("Selected: {}", sel.capcode));
                        ui.separator();
                        ui.label(
                            RichText::new(format!(
                                "{} {} | {} | {} | {} bps",
                                sel.date, sel.time, sel.mode, sel.msg_type, sel.bitrate
                            ))
                            .small()
                            .color(Color32::from_rgb(150, 205, 255)),
                        );
                        if let Some(lbl) = &sel.filter_label {
                            let c_idx = sel.filter_color_idx.unwrap_or(0) % LABEL_PALETTE.len();
                            ui.colored_label(LABEL_PALETTE[c_idx], format!("[{}]", lbl));
                        }
                        ui.separator();
                        if ui.small_button("Copy Message").clicked() {
                            ui.ctx().copy_text(sel.message.clone());
                        }
                        if ui.small_button("Copy Capcode").clicked() {
                            ui.ctx().copy_text(sel.capcode.clone());
                        }
                        let is_isolated = self.search_query.eq_ignore_ascii_case(&sel.capcode);
                        let iso_label = if is_isolated {
                            "Show All Capcodes"
                        } else {
                            "Isolate Capcode"
                        };
                        if ui.small_button(iso_label).clicked() {
                            if is_isolated {
                                self.search_query.clear();
                            } else {
                                self.search_query = sel.capcode.clone();
                            }
                        }
                        if ui.small_button("+ Add to Filters").clicked() {
                            self.prefill_filter_from_message(&sel);
                        }
                        if ui.small_button("Close ×").clicked() {
                            self.selected_message = None;
                        }
                    });
                    ui.separator();
                    egui::ScrollArea::vertical()
                        .id_salt("inspector_scroll")
                        .show(ui, |ui| {
                            ui.add(
                                egui::Label::new(
                                    RichText::new(&sel.message)
                                        .color(Color32::from_rgb(215, 255, 245))
                                        .monospace(),
                                )
                                .wrap(),
                            );
                        });
                });
        }

        // Main Split Client Area: Upper Monitor Pane + Optional Lower Filter Pane
        let mut table_events = TableUiEvents::default();
        egui::CentralPanel::default().show(ctx, |ui| {
            let total_h = ui.available_height();
            let upper_h = if self.config.show_filter_pane {
                (total_h * (self.config.upper_pane_percent / 100.0)).clamp(120.0, (total_h - 95.0).max(120.0))
            } else {
                total_h
            };

            let mon_indices = compute_sorted_filtered_indices(
                &self.monitor_messages,
                &self.search_query,
                self.sort_column,
                self.sort_ascending,
            );

            ui.horizontal(|ui| {
                if self.search_query.trim().is_empty() {
                    ui.heading(format!(
                        "Monitored Messages ({})",
                        self.monitor_messages.len()
                    ));
                } else {
                    ui.heading(format!(
                        "Monitored Messages ({} / {})",
                        mon_indices.len(),
                        self.monitor_messages.len()
                    ));
                }
                ui.label(
                    RichText::new("(Click column headers to sort • Click capcode to isolate • Click message to inspect)")
                        .small()
                        .color(Color32::from_rgb(145, 160, 180)),
                );
            });

            let sel_cap_time = self
                .selected_message
                .as_ref()
                .map(|m| (m.capcode.as_str(), m.time.as_str(), m.message.as_str()));

            ui.push_id("upper_monitor_pane", |ui| {
                render_message_table(
                    ui,
                    &self.monitor_messages,
                    &mon_indices,
                    (upper_h - 28.0).max(60.0),
                    &mut self.sort_column,
                    &mut self.sort_ascending,
                    sel_cap_time,
                    &mut table_events,
                );
            });

            if self.config.show_filter_pane {
                ui.separator();
                let flt_indices = compute_sorted_filtered_indices(
                    &self.filtered_messages,
                    &self.search_query,
                    self.sort_column,
                    self.sort_ascending,
                );
                if self.search_query.trim().is_empty() {
                    ui.heading(format!(
                        "Filtered Messages ({})",
                        self.filtered_messages.len()
                    ));
                } else {
                    ui.heading(format!(
                        "Filtered Messages ({} / {})",
                        flt_indices.len(),
                        self.filtered_messages.len()
                    ));
                }
                let lower_h = ui.available_height().max(65.0);
                ui.push_id("lower_filter_pane", |ui| {
                    render_message_table(
                        ui,
                        &self.filtered_messages,
                        &flt_indices,
                        lower_h,
                        &mut self.sort_column,
                        &mut self.sort_ascending,
                        sel_cap_time,
                        &mut table_events,
                    );
                });
            }
        });

        if let Some(cap) = table_events.clicked_capcode {
            if self.search_query.eq_ignore_ascii_case(&cap) {
                self.search_query.clear();
            } else {
                self.search_query = cap;
            }
        }
        if let Some(m) = table_events.selected_msg {
            self.selected_message = Some(m);
        }
        if let Some(m) = table_events.add_filter_for_msg {
            self.prefill_filter_from_message(&m);
        }

        // Dialogs
        self.render_audio_setup_dialog(ctx);
        self.render_options_dialog(ctx);
        self.render_filters_dialog(ctx);
        self.render_stats_dialog(ctx);
    }
}

fn compute_sorted_filtered_indices(
    messages: &VecDeque<DecodedPagerMessage>,
    search_query: &str,
    sort_col: SortColumn,
    sort_asc: bool,
) -> Vec<usize> {
    let q = search_query.trim().to_ascii_lowercase();
    let mut indices: Vec<usize> = if q.is_empty() {
        (0..messages.len()).collect()
    } else {
        messages
            .iter()
            .enumerate()
            .filter(|(_, m)| {
                m.capcode.to_ascii_lowercase().contains(&q)
                    || m.message.to_ascii_lowercase().contains(&q)
                    || m.mode.to_ascii_lowercase().contains(&q)
                    || m.msg_type.to_ascii_lowercase().contains(&q)
                    || m.filter_label
                        .as_ref()
                        .map(|l| l.to_ascii_lowercase().contains(&q))
                        .unwrap_or(false)
            })
            .map(|(i, _)| i)
            .collect()
    };

    match sort_col {
        SortColumn::Arrival => {
            if !sort_asc {
                indices.reverse();
            }
        }
        SortColumn::Address => {
            indices.sort_by(|&a, &b| {
                let cmp = messages[a]
                    .capcode
                    .cmp(&messages[b].capcode)
                    .then_with(|| a.cmp(&b));
                if sort_asc { cmp } else { cmp.reverse() }
            });
        }
        SortColumn::Mode => {
            indices.sort_by(|&a, &b| {
                let cmp = messages[a]
                    .mode
                    .cmp(&messages[b].mode)
                    .then_with(|| a.cmp(&b));
                if sort_asc { cmp } else { cmp.reverse() }
            });
        }
        SortColumn::MsgType => {
            indices.sort_by(|&a, &b| {
                let cmp = messages[a]
                    .msg_type
                    .cmp(&messages[b].msg_type)
                    .then_with(|| a.cmp(&b));
                if sort_asc { cmp } else { cmp.reverse() }
            });
        }
        SortColumn::Bitrate => {
            indices.sort_by(|&a, &b| {
                let cmp = messages[a]
                    .bitrate
                    .cmp(&messages[b].bitrate)
                    .then_with(|| a.cmp(&b));
                if sort_asc { cmp } else { cmp.reverse() }
            });
        }
        SortColumn::Message => {
            indices.sort_by(|&a, &b| {
                let cmp = messages[a]
                    .message
                    .cmp(&messages[b].message)
                    .then_with(|| a.cmp(&b));
                if sort_asc { cmp } else { cmp.reverse() }
            });
        }
    }

    indices
}

#[allow(clippy::too_many_arguments)]
fn render_message_table(
    ui: &mut egui::Ui,
    messages: &VecDeque<DecodedPagerMessage>,
    indices: &[usize],
    height: f32,
    sort_col: &mut SortColumn,
    sort_asc: &mut bool,
    selected_row_key: Option<(&str, &str, &str)>,
    events: &mut TableUiEvents,
) {
    let stick_bottom = *sort_col == SortColumn::Arrival && *sort_asc;
    TableBuilder::new(ui)
        .striped(true)
        .resizable(true)
        .stick_to_bottom(stick_bottom)
        .max_scroll_height(height)
        .column(Column::initial(92.0).at_least(75.0)) // Address
        .column(Column::initial(72.0).at_least(60.0)) // Time
        .column(Column::initial(70.0).at_least(60.0)) // Date
        .column(Column::initial(76.0).at_least(65.0)) // Mode
        .column(Column::initial(66.0).at_least(55.0)) // Type
        .column(Column::initial(60.0).at_least(48.0)) // Bitrate
        .column(Column::remainder().at_least(200.0))  // Message & Filter Label
        .header(21.0, |mut header| {
            let columns: [(&str, SortColumn); 7] = [
                ("Address", SortColumn::Address),
                ("Time", SortColumn::Arrival),
                ("Date", SortColumn::Arrival),
                ("Mode", SortColumn::Mode),
                ("Type", SortColumn::MsgType),
                ("Bitrate", SortColumn::Bitrate),
                ("Message", SortColumn::Message),
            ];
            for (title, col_enum) in columns {
                header.col(|ui| {
                    let is_active = *sort_col == col_enum
                        && !(title == "Date" && col_enum == SortColumn::Arrival);
                    let arrow = if is_active {
                        if *sort_asc { " ▲" } else { " ▼" }
                    } else {
                        ""
                    };
                    let btn_txt = format!("{}{}", title, arrow);
                    let resp = ui
                        .selectable_label(is_active, RichText::new(btn_txt).strong())
                        .on_hover_text(format!(
                            "Click to sort by {} (click again to toggle ascending/descending)",
                            title
                        ));
                    if resp.clicked() {
                        if *sort_col == col_enum {
                            *sort_asc = !*sort_asc;
                        } else {
                            *sort_col = col_enum;
                            *sort_asc = col_enum != SortColumn::Arrival;
                        }
                    }
                });
            }
        })
        .body(|body| {
            body.rows(19.0, indices.len(), |mut row| {
                let vis_idx = row.index();
                if let Some(&msg_idx) = indices.get(vis_idx) {
                    if let Some(m) = messages.get(msg_idx) {
                        let is_selected = selected_row_key
                            .map(|(c, t, msg)| c == m.capcode && t == m.time && msg == m.message)
                            .unwrap_or(false);
                        row.set_selected(is_selected);

                        row.col(|ui| {
                            let cap_resp = ui
                                .add(
                                    egui::Label::new(
                                        RichText::new(&m.capcode)
                                            .color(Color32::from_rgb(245, 245, 255)),
                                    )
                                    .sense(egui::Sense::click()),
                                )
                                .on_hover_text(
                                    "Click to isolate this capcode (click again to show all) • Right-click for options",
                                );
                            if cap_resp.clicked() {
                                events.clicked_capcode = Some(m.capcode.clone());
                            }
                            cap_resp.context_menu(|ui| {
                                if ui.button("Filter View by This Capcode").clicked() {
                                    events.clicked_capcode = Some(m.capcode.clone());
                                    ui.close_menu();
                                }
                                if ui.button("Add Capcode to Filter Manager...").clicked() {
                                    events.add_filter_for_msg = Some(m.clone());
                                    ui.close_menu();
                                }
                                if ui.button("Copy Capcode / Address").clicked() {
                                    ui.ctx().copy_text(m.capcode.clone());
                                    ui.close_menu();
                                }
                            });
                        });
                        row.col(|ui| {
                            ui.colored_label(Color32::from_rgb(110, 190, 255), &m.time);
                        });
                        row.col(|ui| {
                            ui.colored_label(Color32::from_rgb(110, 190, 255), &m.date);
                        });
                        row.col(|ui| {
                            ui.colored_label(Color32::from_rgb(255, 110, 110), &m.mode);
                        });
                        row.col(|ui| {
                            ui.colored_label(Color32::from_rgb(255, 180, 90), &m.msg_type);
                        });
                        row.col(|ui| {
                            ui.colored_label(
                                Color32::from_rgb(255, 110, 110),
                                format!("{}", m.bitrate),
                            );
                        });
                        row.col(|ui| {
                            ui.horizontal(|ui| {
                                let msg_color = if m.has_bit_errors {
                                    Color32::from_rgb(180, 180, 180)
                                } else if m.msg_type.contains("NUM") || m.msg_type.contains("TONE")
                                {
                                    Color32::from_rgb(255, 120, 120)
                                } else {
                                    Color32::from_rgb(120, 255, 255)
                                };
                                let resp = ui
                                    .add(
                                        egui::Label::new(
                                            RichText::new(&m.message).color(msg_color),
                                        )
                                        .sense(egui::Sense::click()),
                                    )
                                    .on_hover_text(
                                        "Click to open in bottom Message Inspector • Right-click to copy or filter",
                                    );
                                if resp.clicked() {
                                    events.selected_msg = Some(m.clone());
                                }
                                resp.context_menu(|ui| {
                                    if ui.button("Inspect Full Message").clicked() {
                                        events.selected_msg = Some(m.clone());
                                        ui.close_menu();
                                    }
                                    if ui.button("Copy Message Text").clicked() {
                                        ui.ctx().copy_text(m.message.clone());
                                        ui.close_menu();
                                    }
                                    if ui.button("Copy Capcode / Address").clicked() {
                                        ui.ctx().copy_text(m.capcode.clone());
                                        ui.close_menu();
                                    }
                                    if ui.button("Copy Full Row").clicked() {
                                        ui.ctx().copy_text(format!(
                                            "{} {} {} {} {} {} {}",
                                            m.capcode,
                                            m.time,
                                            m.date,
                                            m.mode,
                                            m.msg_type,
                                            m.bitrate,
                                            m.message
                                        ));
                                        ui.close_menu();
                                    }
                                    ui.separator();
                                    if ui.button("Filter View by This Capcode").clicked() {
                                        events.clicked_capcode = Some(m.capcode.clone());
                                        ui.close_menu();
                                    }
                                    if ui.button("Add Capcode to Filter Manager...").clicked() {
                                        events.add_filter_for_msg = Some(m.clone());
                                        ui.close_menu();
                                    }
                                });
                                if let Some(label) = &m.filter_label {
                                    let col_idx =
                                        m.filter_color_idx.unwrap_or(0) % LABEL_PALETTE.len();
                                    ui.colored_label(
                                        LABEL_PALETTE[col_idx],
                                        format!("[{}]", label),
                                    );
                                }
                                if m.soft_recovered_words > 0 {
                                    ui.colored_label(
                                        Color32::from_rgb(140, 255, 140),
                                        format!("(Soft+{})", m.soft_recovered_words),
                                    );
                                }
                            });
                        });
                    }
                }
            });
        });
}

impl PdwApp {
    fn prefill_filter_from_message(&mut self, msg: &DecodedPagerMessage) {
        self.editing_filter_idx = None;
        self.editing_filter = FilterRule {
            filter_type: if msg.mode.starts_with("POCSAG") {
                FilterType::Pocsag
            } else {
                FilterType::Flex
            },
            capcode: msg.capcode.clone(),
            label: msg.filter_label.clone().unwrap_or_default(),
            text: String::new(),
            match_exact_msg: false,
            action: FilterAction::DisplayBoth,
            label_enabled: true,
            label_color: msg.filter_color_idx.unwrap_or(0),
            sep_file_enabled: false,
            sep_filename: String::new(),
            hit_counter: 0,
            last_hit_date: String::new(),
            last_hit_time: String::new(),
        };
        self.show_filters = true;
    }

    fn render_audio_setup_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_audio_setup {
            return;
        }
        let mut open = self.show_audio_setup;
        egui::Window::new("Signal Input Setup (Soundcard / SDRconnect IQ)")
            .open(&mut open)
            .resizable(false)
            .default_width(450.0)
            .show(ctx, |ui| {
                ui.heading("Signal Input Source");
                ui.horizontal(|ui| {
                    if ui
                        .radio_value(
                            &mut self.config.input_source,
                            InputSource::Soundcard,
                            "Soundcard (Audio In)",
                        )
                        .clicked()
                    {
                        self.restart_audio();
                    }
                    if ui
                        .radio_value(
                            &mut self.config.input_source,
                            InputSource::SdrConnectIq,
                            "SDRconnect (Direct WebSocket IQ)",
                        )
                        .clicked()
                    {
                        self.restart_audio();
                    }
                });

                ui.separator();

                match self.config.input_source {
                    InputSource::Soundcard => {
                        ui.label("Select Windows / System Soundcard Input Device:");
                        let current_dev = self
                            .config
                            .audio_device_name
                            .clone()
                            .unwrap_or_else(|| "System Default".to_string());
                        egui::ComboBox::from_id_salt("audio_dev_combo")
                            .selected_text(&current_dev)
                            .width(340.0)
                            .show_ui(ui, |ui| {
                                if ui
                                    .selectable_label(
                                        self.config.audio_device_name.is_none(),
                                        "System Default",
                                    )
                                    .clicked()
                                {
                                    self.config.audio_device_name = None;
                                }
                                for dev in &self.available_devices {
                                    let selected = self.config.audio_device_name.as_deref()
                                        == Some(dev.as_str());
                                    if ui.selectable_label(selected, dev).clicked() {
                                        self.config.audio_device_name = Some(dev.clone());
                                    }
                                }
                            });

                        ui.add_space(6.0);
                        ui.label("Audio Sample Rate (48,000 Hz or 44,100 Hz):");
                        egui::ComboBox::from_id_salt("sample_rate_combo")
                            .selected_text(format!("{} Hz", self.config.sample_rate))
                            .show_ui(ui, |ui| {
                                for &sr in &SUPPORTED_SAMPLE_RATES {
                                    ui.selectable_value(
                                        &mut self.config.sample_rate,
                                        sr,
                                        format!("{} Hz", sr),
                                    );
                                }
                            });
                    }
                    InputSource::SdrConnectIq => {
                        ui.label(
                            RichText::new(
                                "Pulls raw 16-bit IQ samples directly from SDRconnect WebSocket API (iq_stream_enable / binary type 2) with a Butterworth complex filter & DC-coupled polar FM discriminator.",
                            )
                            .small()
                            .color(Color32::from_rgb(170, 215, 255)),
                        );
                        ui.add_space(4.0);

                        ui.horizontal(|ui| {
                            ui.label("SDRconnect Host:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.config.sdrconnect_host)
                                    .desired_width(140.0),
                            );
                            ui.label("Port:");
                            let mut port_u32 = self.config.sdrconnect_port as u32;
                            if ui
                                .add(
                                    egui::DragValue::new(&mut port_u32)
                                        .range(1..=65535)
                                        .speed(1.0),
                                )
                                .changed()
                            {
                                self.config.sdrconnect_port = port_u32 as u16;
                            }
                        });

                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.checkbox(
                                &mut self.config.sdrconnect_tune_on_connect,
                                "Set VFO on Connect (MHz):",
                            );
                            ui.add(
                                egui::TextEdit::singleline(&mut self.vfo_mhz_input)
                                    .desired_width(95.0),
                            );
                            if ui.button("Tune VFO Now").clicked() {
                                if let Ok(mhz) = self.vfo_mhz_input.trim().parse::<f64>() {
                                    let hz = (mhz * 1_000_000.0).round() as u64;
                                    self.config.sdrconnect_vfo_freq_hz = hz;
                                    if let Some(sdr) = &self.sdrconnect_stream {
                                        sdr.send_command(SdrConnectControlCmd::SetVfoFrequency(hz));
                                    }
                                    let _ = self.config.save(&self.config_path);
                                }
                            }
                        });

                        ui.add_space(4.0);
                        let mut demod_changed = false;
                        ui.horizontal(|ui| {
                            demod_changed |= ui
                                .add(
                                    egui::Slider::new(
                                        &mut self.config.sdrconnect_iq_gain,
                                        0.50..=2.0,
                                    )
                                    .text("IQ Discriminator Gain"),
                                )
                                .changed();
                            if ui.small_button("Reset (1.0)").clicked() {
                                self.config.sdrconnect_iq_gain = 1.0;
                                self.config.sdrconnect_channel_bw_hz = 20_000.0;
                                demod_changed = true;
                            }
                        });

                        demod_changed |= ui
                            .add(
                                egui::Slider::new(
                                    &mut self.config.sdrconnect_channel_bw_hz,
                                    12_500.0..=32_000.0,
                                )
                                .text("IQ Channel Low-Pass BW (Hz)"),
                            )
                            .on_hover_text("Keep >= 16,000 Hz so ±4.8 kHz outer 4-FSK tones pass with zero attenuation")
                            .changed();

                        ui.horizontal(|ui| {
                            demod_changed |= ui
                                .checkbox(
                                    &mut self.config.sdrconnect_squelch_enabled,
                                    "IQ RF Power Squelch",
                                )
                                .changed();
                            demod_changed |= ui
                                .add_enabled(
                                    self.config.sdrconnect_squelch_enabled,
                                    egui::Slider::new(
                                        &mut self.config.sdrconnect_squelch_dbfs,
                                        -90.0..=-20.0,
                                    )
                                    .text("Threshold (dBFS)"),
                                )
                                .changed();
                        });

                        demod_changed |= ui
                            .checkbox(
                                &mut self.config.sdrconnect_afc_enabled,
                                "Automatic Frequency Control (Carrier-Gated AFC)",
                            )
                            .changed();

                        demod_changed |= ui
                            .checkbox(
                                &mut self.config.sdrconnect_use_audio_stream,
                                "Use SDRconnect Demodulated Audio Stream (Type 1) instead of Raw IQ (Type 2)",
                            )
                            .on_hover_text("Unchecked (default): pulls raw Type-2 IQ samples and demodulates internally. Checked: pulls SDRconnect's Type-1 NFM audio stream.")
                            .changed();

                        if demod_changed {
                            if let Some(sdr) = &self.sdrconnect_stream {
                                sdr.send_command(SdrConnectControlCmd::UpdateDemodParams {
                                    iq_gain: self.config.sdrconnect_iq_gain,
                                    squelch_enabled: self.config.sdrconnect_squelch_enabled,
                                    squelch_dbfs: self.config.sdrconnect_squelch_dbfs,
                                    afc_enabled: self.config.sdrconnect_afc_enabled,
                                    channel_bw_hz: self.config.sdrconnect_channel_bw_hz,
                                    use_audio_stream: self.config.sdrconnect_use_audio_stream,
                                });
                            }
                            let _ = self.config.save(&self.config_path);
                        }

                        if let Some(sdr) = &self.sdrconnect_stream {
                            if let Ok(st) = sdr.status.try_lock() {
                                ui.separator();
                                ui.label(
                                    RichText::new(format!(
                                        "Status: {} | Frames: {} | Measured Rate: {} sps",
                                        st.status_text,
                                        st.iq_frames_received,
                                        if st.measured_iq_rate > 0 {
                                            st.measured_iq_rate
                                        } else {
                                            st.iq_sample_rate
                                        }
                                    ))
                                    .small(),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "VFO: {} | LO Center: {} | Demod: {} | VFO BW: {} | RSSI: {:.1} dBFS | SNR: {}",
                                        st.vfo_freq_hz
                                            .map(|h| format!("{:.4} MHz", (h as f64) / 1e6))
                                            .unwrap_or_else(|| "N/A".to_string()),
                                        st.center_freq_hz
                                            .map(|h| format!("{:.4} MHz", (h as f64) / 1e6))
                                            .unwrap_or_else(|| "N/A".to_string()),
                                        if st.demodulator.is_empty() {
                                            "N/A"
                                        } else {
                                            &st.demodulator
                                        },
                                        st.filter_bw_hz
                                            .map(|b| format!("{} Hz", b))
                                            .unwrap_or_else(|| "N/A".to_string()),
                                        st.iq_rssi_dbfs,
                                        st.signal_snr_db
                                            .map(|s| format!("{:.1} dB", s))
                                            .unwrap_or_else(|| "N/A".to_string()),
                                    ))
                                    .small(),
                                );
                            }
                        }
                    }
                }

                ui.separator();
                ui.label("PDW Slicer Configuration Preset (Schmitt-Trigger Thresholds):");
                egui::ComboBox::from_id_salt("audio_preset_combo")
                    .selected_text(self.config.audio_preset.label())
                    .width(340.0)
                    .show_ui(ui, |ui| {
                        for preset in AudioPreset::ALL {
                            ui.selectable_value(
                                &mut self.config.audio_preset,
                                preset,
                                preset.label(),
                            );
                        }
                    });

                ui.add_space(6.0);
                ui.checkbox(
                    &mut self.config.invert_polarity,
                    "Invert Data Polarity (Auto-detected on Sync-1 / POCSAG Sync)",
                );
                ui.add(
                    egui::Slider::new(&mut self.config.manual_center_offset, -0.25..=0.25)
                        .text("Manual DC Center Offset"),
                );

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    let btn_text = match self.config.input_source {
                        InputSource::Soundcard => "Apply & Restart Soundcard",
                        InputSource::SdrConnectIq => "Apply & Reconnect SDRconnect",
                    };
                    if ui.button(btn_text).clicked() {
                        if let Ok(mhz) = self.vfo_mhz_input.trim().parse::<f64>() {
                            self.config.sdrconnect_vfo_freq_hz = (mhz * 1_000_000.0).round() as u64;
                        }
                        self.restart_audio();
                    }
                    if ui.button("Close").clicked() {
                        self.show_audio_setup = false;
                    }
                });
            });
        self.show_audio_setup &= open;
    }

    fn render_options_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_options {
            return;
        }
        let mut open = self.show_options;
        egui::Window::new("PDW Decoder & Display Options")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.heading("POCSAG Channels");
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.config.decode_pocsag_512, "512 Baud");
                    ui.checkbox(&mut self.config.decode_pocsag_1200, "1200 Baud");
                    ui.checkbox(&mut self.config.decode_pocsag_2400, "2400 Baud");
                });
                ui.checkbox(
                    &mut self.config.pocsag_fnu_mode,
                    "Determine Alpha/Numeric strictly by Function Number (4 = Alpha)",
                );
                ui.checkbox(
                    &mut self.config.pocsag_show_both,
                    "Show ambiguous messages as both Numeric and Alpha",
                );

                ui.separator();
                ui.heading("FLEX / ReFLEX (2-FSK & 4-FSK Phases A, B, C, D)");
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.config.decode_flex_1600, "1600 bps");
                    ui.checkbox(&mut self.config.decode_flex_3200, "3200 bps");
                    ui.checkbox(&mut self.config.decode_flex_6400, "6400 bps");
                });
                ui.checkbox(
                    &mut self.config.convert_flex_si,
                    "Convert Short Instructions (Groupcalls 2029568–2029583) to Member Messages",
                );

                ui.separator();
                ui.heading("Message Display, Scope & Logging");
                ui.checkbox(&mut self.config.show_numeric, "Show Numeric Messages");
                ui.checkbox(&mut self.config.show_tone_only, "Show Tone-Only Messages");
                ui.checkbox(
                    &mut self.config.show_binary,
                    "Show Binary / Hex Messages (Encrypted / Telemetry FLEX Vector 6)",
                );
                ui.checkbox(
                    &mut self.config.hide_encrypted_alpha,
                    "Hide Non-Human-Readable / Base64-Encrypted Alpha Messages",
                );
                ui.checkbox(
                    &mut self.config.show_scope,
                    "Enable Live Waveform Oscilloscope (Uncheck to minimize CPU usage)",
                );
                ui.checkbox(
                    &mut self.config.show_filter_pane,
                    "Show Lower Filtered Messages Pane (Uncheck for Full-Height Monitor View)",
                );
                ui.checkbox(
                    &mut self.config.filter_window_only,
                    "Show Filtered Messages Only in Lower Filter Pane",
                );
                ui.checkbox(&mut self.config.block_duplicates, "Block Duplicate Messages");
                ui.add(
                    egui::Slider::new(&mut self.config.block_duplicate_seconds, 10..=600)
                        .text("Duplicate Window (s)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.config.upper_pane_percent, 20.0..=85.0)
                        .text("Upper Pane Height (%)"),
                );

                ui.checkbox(
                    &mut self.config.monitor_logfile_enabled,
                    "Enable Daily Monitor Logfile (Logfiles/YYMMDD.log)",
                );
                ui.checkbox(
                    &mut self.config.filter_logfile_enabled,
                    "Enable Daily Filter Logfile (Logfiles/YYMMDD.flt)",
                );

                ui.add_space(8.0);
                if ui.button("Save & Apply").clicked() {
                    let _ = self
                        .cmd_tx
                        .send(WorkerCommand::UpdateConfig(self.config.clone()));
                    let _ = self.config.save(&self.config_path);
                    self.show_options = false;
                }
            });
        self.show_options &= open;
    }

    fn render_filters_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_filters {
            return;
        }
        let mut open = self.show_filters;
        egui::Window::new("Filter Manager (Compatible with Legacy FILTERS.INI)")
            .open(&mut open)
            .default_width(640.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Legacy INI File:");
                    ui.text_edit_singleline(&mut self.filter_ini_path);
                    if ui.button("Import FILTERS.INI").clicked() {
                        match self
                            .config
                            .import_legacy_filters_ini(&PathBuf::from(&self.filter_ini_path))
                        {
                            Ok(n) => {
                                self.status_toast = format!("Imported {} filters", n);
                                let _ = self.config.save(&self.config_path);
                            }
                            Err(e) => self.status_toast = format!("Import error: {}", e),
                        }
                    }
                    if ui.button("Export FILTERS.INI").clicked() {
                        match self
                            .config
                            .export_legacy_filters_ini(&PathBuf::from(&self.filter_ini_path))
                        {
                            Ok(()) => self.status_toast = "Exported FILTERS.INI".to_string(),
                            Err(e) => self.status_toast = format!("Export error: {}", e),
                        }
                    }
                    if !self.status_toast.is_empty() {
                        ui.label(RichText::new(&self.status_toast).color(Color32::LIGHT_GREEN));
                    }
                });

                ui.separator();
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("new_flt_type")
                        .selected_text(format!("{:?}", self.editing_filter.filter_type))
                        .width(95.0)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.editing_filter.filter_type,
                                FilterType::Flex,
                                "FLEX Capcode",
                            );
                            ui.selectable_value(
                                &mut self.editing_filter.filter_type,
                                FilterType::Pocsag,
                                "POCSAG RIC",
                            );
                            ui.selectable_value(
                                &mut self.editing_filter.filter_type,
                                FilterType::Text,
                                "Text Only",
                            );
                        });
                    ui.label("Capcode (? = wildcard):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.editing_filter.capcode)
                            .desired_width(85.0),
                    );
                    ui.label("Label:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.editing_filter.label)
                            .desired_width(110.0),
                    );
                    ui.label("Text:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.editing_filter.text)
                            .desired_width(90.0),
                    );
                });

                ui.horizontal(|ui| {
                    ui.label("Action:");
                    egui::ComboBox::from_id_salt("new_flt_action")
                        .selected_text(match self.editing_filter.action {
                            FilterAction::DisplayBoth => "Show in Filter & Monitor",
                            FilterAction::MonitorOnly => "Monitor Pane Only",
                            FilterAction::Reject => "Reject / Block",
                        })
                        .width(155.0)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.editing_filter.action,
                                FilterAction::DisplayBoth,
                                "Show in Filter & Monitor",
                            );
                            ui.selectable_value(
                                &mut self.editing_filter.action,
                                FilterAction::MonitorOnly,
                                "Monitor Pane Only",
                            );
                            ui.selectable_value(
                                &mut self.editing_filter.action,
                                FilterAction::Reject,
                                "Reject / Block",
                            );
                        });

                    let color_names = [
                        "Light Blue",
                        "Yellow",
                        "Red",
                        "Orange",
                        "Light Green",
                        "Cyan",
                        "Pink",
                        "White",
                    ];
                    let cur_col = self.editing_filter.label_color % color_names.len();
                    ui.label("Color:");
                    egui::ComboBox::from_id_salt("new_flt_color")
                        .selected_text(
                            RichText::new(color_names[cur_col]).color(LABEL_PALETTE[cur_col]),
                        )
                        .width(95.0)
                        .show_ui(ui, |ui| {
                            for (c_idx, &c_name) in color_names.iter().enumerate() {
                                ui.selectable_value(
                                    &mut self.editing_filter.label_color,
                                    c_idx,
                                    RichText::new(c_name).color(LABEL_PALETTE[c_idx]),
                                );
                            }
                        });

                    ui.checkbox(&mut self.editing_filter.match_exact_msg, "Exact Text");

                    if let Some(edit_idx) = self.editing_filter_idx {
                        if ui
                            .button(format!("Update #{:03}", edit_idx + 1))
                            .clicked()
                        {
                            if let Some(slot) = self.config.filters.get_mut(edit_idx) {
                                *slot = self.editing_filter.clone();
                            }
                            self.editing_filter_idx = None;
                            self.editing_filter = FilterRule::default();
                            let _ = self.config.save(&self.config_path);
                        }
                        if ui.small_button("Cancel").clicked() {
                            self.editing_filter_idx = None;
                            self.editing_filter = FilterRule::default();
                        }
                    } else if ui.button("Add Filter").clicked() {
                        self.config.filters.push(self.editing_filter.clone());
                        let _ = self.config.save(&self.config_path);
                    }
                });

                ui.separator();
                let mut remove_idx: Option<usize> = None;
                let mut load_edit_idx: Option<usize> = None;
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        for (i, r) in self.config.filters.iter().enumerate() {
                            ui.horizontal(|ui| {
                                let col_idx = r.label_color % LABEL_PALETTE.len();
                                ui.colored_label(
                                    LABEL_PALETTE[col_idx],
                                    format!(
                                        "#{:03} [{:?} / {:?}] {} | \"{}\" | Text: \"{}\" | Hits: {}",
                                        i + 1,
                                        r.filter_type,
                                        r.action,
                                        r.capcode,
                                        r.label,
                                        r.text,
                                        r.hit_counter
                                    ),
                                );
                                if ui.small_button("Edit").clicked() {
                                    load_edit_idx = Some(i);
                                }
                                if ui.small_button("Delete").clicked() {
                                    remove_idx = Some(i);
                                }
                            });
                        }
                    });
                if let Some(idx) = load_edit_idx {
                    if let Some(r) = self.config.filters.get(idx).cloned() {
                        self.editing_filter = r;
                        self.editing_filter_idx = Some(idx);
                    }
                }
                if let Some(idx) = remove_idx {
                    self.config.filters.remove(idx);
                    if self.editing_filter_idx == Some(idx) {
                        self.editing_filter_idx = None;
                    }
                    let _ = self.config.save(&self.config_path);
                }
            });
        self.show_filters &= open;
    }

    fn render_stats_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_stats {
            return;
        }
        let mut open = self.show_stats;
        egui::Window::new("Decoding Statistics")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                egui::Grid::new("stats_grid").striped(true).show(ui, |ui| {
                    ui.strong("Protocol / Rate");
                    ui.strong("Numeric / Tone");
                    ui.strong("Alphanumeric");
                    ui.end_row();

                    ui.label("POCSAG-512");
                    ui.label(format!("{}", self.stats.pocsag_512_num));
                    ui.label(format!("{}", self.stats.pocsag_512_alpha));
                    ui.end_row();

                    ui.label("POCSAG-1200");
                    ui.label(format!("{}", self.stats.pocsag_1200_num));
                    ui.label(format!("{}", self.stats.pocsag_1200_alpha));
                    ui.end_row();

                    ui.label("POCSAG-2400");
                    ui.label(format!("{}", self.stats.pocsag_2400_num));
                    ui.label(format!("{}", self.stats.pocsag_2400_alpha));
                    ui.end_row();

                    ui.label("FLEX-1600");
                    ui.label(format!("{}", self.stats.flex_1600_num));
                    ui.label(format!("{}", self.stats.flex_1600_alpha));
                    ui.end_row();

                    ui.label("FLEX-3200 (2/4-FSK)");
                    ui.label(format!("{}", self.stats.flex_3200_num));
                    ui.label(format!("{}", self.stats.flex_3200_alpha));
                    ui.end_row();

                    ui.label("FLEX-6400 (4-FSK)");
                    ui.label(format!("{}", self.stats.flex_6400_num));
                    ui.label(format!("{}", self.stats.flex_6400_alpha));
                    ui.end_row();
                });
                ui.separator();
                ui.label(format!(
                    "Total Decoded: {} | Rejected by Filter: {} | Duplicates Blocked: {}",
                    self.stats.total_messages(),
                    self.stats.rejected_count,
                    self.stats.blocked_duplicates
                ));
                if ui.button("Reset Statistics").clicked() {
                    self.stats = StatsTracker::default();
                }
            });
        self.show_stats &= open;
    }
}

/// Dedicated background DSP & Decoder worker thread.
fn run_dsp_worker(
    mut cfg: AppConfig,
    sample_rx: Receiver<Vec<f32>>,
    cmd_rx: Receiver<WorkerCommand>,
    msg_tx: Sender<DecodedPagerMessage>,
    telemetry: Arc<Mutex<DspTelemetry>>,
    egui_ctx: Option<egui::Context>,
) {
    let bch = BchDecoder::new();
    let mut env = AdaptiveEnvelopeTracker::new();
    let mut qual = RxQualityTracker::new();

    let mut pocsag_atb = AtbSlicer::new(cfg.sample_rate, cfg.audio_preset, cfg.invert_polarity);
    let mut flex_atb = AtbSlicer::new(cfg.sample_rate, cfg.audio_preset, cfg.invert_polarity);
    let mut pocsag_main = PocsagChannelDecoder::new(cfg.sample_rate, 1200);
    let mut flex = FlexDecoder::new(cfg.sample_rate);

    let mut scope_ring: VecDeque<f32> = VecDeque::from(vec![0.0; 80]);
    let mut sample_counter: usize = 0;

    loop {
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                WorkerCommand::UpdateConfig(new_cfg) => {
                    if new_cfg.sample_rate != cfg.sample_rate {
                        pocsag_atb.set_sample_rate(new_cfg.sample_rate);
                        flex_atb.set_sample_rate(new_cfg.sample_rate);
                        pocsag_main.set_sample_rate(new_cfg.sample_rate);
                        flex.set_sample_rate(new_cfg.sample_rate);
                    }
                    pocsag_atb.set_preset(new_cfg.audio_preset);
                    flex_atb.set_preset(new_cfg.audio_preset);
                    if new_cfg.invert_polarity != cfg.invert_polarity {
                        pocsag_atb.set_invert(new_cfg.invert_polarity);
                        flex_atb.set_invert(new_cfg.invert_polarity);
                    }
                    env.manual_center_offset = new_cfg.manual_center_offset;
                    cfg = new_cfg;
                }
                WorkerCommand::ResetRxQuality => {
                    qual.reset();
                }
                WorkerCommand::InjectSamples(samples) => {
                    process_audio_chunk(
                        &samples,
                        &cfg,
                        &bch,
                        &mut env,
                        &mut qual,
                        &mut pocsag_atb,
                        &mut flex_atb,
                        &mut pocsag_main,
                        &mut flex,
                        &msg_tx,
                        &mut scope_ring,
                        &mut sample_counter,
                        &telemetry,
                        egui_ctx.as_ref(),
                    );
                }
            }
        }

        match sample_rx.recv_timeout(std::time::Duration::from_millis(50)) {
            Ok(chunk) => {
                process_audio_chunk(
                    &chunk,
                    &cfg,
                    &bch,
                    &mut env,
                    &mut qual,
                    &mut pocsag_atb,
                    &mut flex_atb,
                    &mut pocsag_main,
                    &mut flex,
                    &msg_tx,
                    &mut scope_ring,
                    &mut sample_counter,
                    &telemetry,
                    egui_ctx.as_ref(),
                );
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
        }
    }
}

/// Exact port of `Audio_To_Bits` (`sound_in.cpp:387–623`), running `pocsag_atb` (`Pocsag.cpp`)
/// and `flex_atb` (`Flex.cpp`) concurrently in parallel so neither protocol ever locks out the other.
#[allow(clippy::too_many_arguments)]
pub fn process_audio_chunk(
    chunk: &[f32],
    cfg: &AppConfig,
    bch: &BchDecoder,
    env: &mut AdaptiveEnvelopeTracker,
    qual: &mut RxQualityTracker,
    pocsag_atb: &mut AtbSlicer,
    flex_atb: &mut AtbSlicer,
    pocsag_main: &mut PocsagChannelDecoder,
    flex: &mut FlexDecoder,
    msg_tx: &Sender<DecodedPagerMessage>,
    scope_ring: &mut VecDeque<f32>,
    sample_counter: &mut usize,
    telemetry: &Arc<Mutex<DspTelemetry>>,
    egui_ctx: Option<&egui::Context>,
) {
    let mut out_msgs: Vec<DecodedPagerMessage> = Vec::new();
    let rate_scale = (flex_atb.sample_rate as f64) / 44_100.0;
    let crossover_samples = (78.4 * ((flex_atb.sample_rate as f64) / 3200.0)).round() as usize;
    let decode_pocsag =
        cfg.decode_pocsag_512 || cfg.decode_pocsag_1200 || cfg.decode_pocsag_2400;
    let decode_flex = cfg.decode_flex_1600 || cfg.decode_flex_3200 || cfg.decode_flex_6400;
    let show_scope = cfg.show_scope;

    let prev_counter = *sample_counter;

    for (atb_ctr_idx, &s) in chunk.iter().enumerate() {
        let atb_ctr = atb_ctr_idx as f64;
        env.update(s);
        flex.tick_sample_timer(flex_atb);

        *sample_counter = sample_counter.wrapping_add(1);
        if show_scope && (*sample_counter & 15) == 0 {
            if scope_ring.len() >= 80 {
                scope_ring.pop_front();
            }
            scope_ring.push_back(s - cfg.manual_center_offset);
        }

        // Convert float sample to PDW's scale without hard-clipping at ±127.0 when IQ gain > 1.0,
        // preserving the linear 3:1 outer-to-inner 4-FSK symbol amplitude ratio in `FlexDecoder::slice_symbol`.
        let val = ((s - cfg.manual_center_offset) * 128.0).clamp(-512.0, 512.0);

        // =========================================================================
        // 1. POCSAG `Audio_To_Bits` Pipeline (`sound_in.cpp:396–604`)
        // =========================================================================
        if decode_pocsag {
            pocsag_atb.sync_baud_rate(atb_ctr);
            pocsag_atb.n_samples += 1;

            if val < -pocsag_atb.pre_threshold && pocsag_atb.sync_bit == pocsag_atb.high_audio {
                pocsag_atb.sync_bit = pocsag_atb.low_audio;
                pocsag_atb.crossing = true;
            } else if val > pocsag_atb.pre_threshold && pocsag_atb.sync_bit == pocsag_atb.low_audio
            {
                pocsag_atb.sync_bit = pocsag_atb.high_audio;
                pocsag_atb.crossing = true;
            }

            let mut started_pocsag_preamble = false;
            if pocsag_atb.crossing && pocsag_atb.pocbit == 0 {
                pocsag_atb.crossing = false;
                let n44 = (pocsag_atb.n_samples as f64) / rate_scale;

                if n44 > 64.0 && n44 < 108.0 {
                    pocsag_atb.preamble_count[INDEX512] += 1;
                } else {
                    pocsag_atb.preamble_count[INDEX512] = 0;
                }
                if pocsag_atb.preamble_count[INDEX512] > 50 {
                    pocsag_atb.preamble_count[INDEX512] = 0;
                    if cfg.decode_pocsag_512 {
                        pocsag_atb.baud_rate = 512;
                        pocsag_atb.pocsag_baud_rate = 512;
                        pocsag_atb.n_samples = 0;
                        pocsag_atb.pocbit = 1300;
                        pocsag_atb.config_index = INDEX512;
                        pocsag_atb.clkt_hi = COURSE_CLKT_HI;
                        pocsag_atb.clkt_lo = COURSE_CLKT_LO;
                        pocsag_atb.atb_bit = pocsag_atb.sync_bit;
                        pocsag_atb.atb_len = 0;
                        started_pocsag_preamble = true;
                    }
                }

                if !started_pocsag_preamble {
                    if n44 >= 29.0 && n44 < 44.0 {
                        pocsag_atb.preamble_count[INDEX1200] += 1;
                    } else {
                        pocsag_atb.preamble_count[INDEX1200] = 0;
                    }
                    if pocsag_atb.preamble_count[INDEX1200] > 50 {
                        pocsag_atb.preamble_count[INDEX1200] = 0;
                        if cfg.decode_pocsag_1200 {
                            pocsag_atb.baud_rate = 1200;
                            pocsag_atb.pocsag_baud_rate = 1200;
                            pocsag_atb.n_samples = 0;
                            pocsag_atb.pocbit = 1250;
                            pocsag_atb.config_index = INDEX1200;
                            pocsag_atb.atb_bit = pocsag_atb.sync_bit;
                            pocsag_atb.atb_len = 0;
                            started_pocsag_preamble = true;
                        }
                    }
                }

                if !started_pocsag_preamble {
                    if n44 > 14.0 && n44 < 22.0 {
                        pocsag_atb.preamble_count[INDEX2400] += 1;
                    } else {
                        pocsag_atb.preamble_count[INDEX2400] = 0;
                    }
                    if pocsag_atb.preamble_count[INDEX2400] > 50 {
                        pocsag_atb.preamble_count[INDEX2400] = 0;
                        if cfg.decode_pocsag_2400 {
                            pocsag_atb.baud_rate = 2400;
                            pocsag_atb.pocsag_baud_rate = 2400;
                            pocsag_atb.n_samples = 0;
                            pocsag_atb.pocbit = 1250;
                            pocsag_atb.config_index = INDEX2400;
                            pocsag_atb.atb_bit = pocsag_atb.sync_bit;
                            pocsag_atb.atb_len = 0;
                            started_pocsag_preamble = true;
                        }
                    }
                }

                if !started_pocsag_preamble {
                    pocsag_atb.n_samples = 0;
                }
            }

            // Only run the POCSAG bit-clock division and sampling when `pocbit > 0`
            if !started_pocsag_preamble && pocsag_atb.pocbit > 0 {
                pocsag_atb.atb_len += 1;
                let p_center = pocsag_atb.atb_center[pocsag_atb.config_index];
                let p_thresh = pocsag_atb.atb_threshold[pocsag_atb.config_index];

                if val < (p_center - p_thresh) && pocsag_atb.atb_bit == pocsag_atb.high_audio {
                    pocsag_atb.atb_bit = pocsag_atb.low_audio;
                    let ratio = (pocsag_atb.atb_len as f64) / pocsag_atb.watch_step;
                    if (pocsag_atb.atb_len as f64) < pocsag_atb.watch_step * 2.0
                        && ratio > pocsag_atb.clkt_lo
                        && ratio < pocsag_atb.clkt_hi
                    {
                        pocsag_atb.watch_ctr = atb_ctr
                            + (pocsag_atb.watch_step * 0.5)
                            + pocsag_atb.atb_sample_offset[pocsag_atb.config_index];
                    }
                    pocsag_atb.atb_len = 0;
                } else if val > (p_center + p_thresh) && pocsag_atb.atb_bit == pocsag_atb.low_audio
                {
                    pocsag_atb.atb_bit = pocsag_atb.high_audio;
                    let ratio = (pocsag_atb.atb_len as f64) / pocsag_atb.watch_step;
                    if (pocsag_atb.atb_len as f64) < pocsag_atb.watch_step * 2.0
                        && ratio > pocsag_atb.clkt_lo
                        && ratio < pocsag_atb.clkt_hi
                    {
                        pocsag_atb.watch_ctr = atb_ctr
                            + (pocsag_atb.watch_step * 0.5)
                            + pocsag_atb.atb_sample_offset[pocsag_atb.config_index];
                    }
                    pocsag_atb.atb_len = 0;
                }

                if pocsag_atb.watch_ctr - atb_ctr < 1.0 {
                    let p_conf = ((val - p_center).abs() / 64.0).clamp(0.05, 2.0);
                    pocsag_main.frame_pdw(
                        pocsag_atb.atb_bit,
                        p_conf,
                        pocsag_atb.pocsag_baud_rate,
                        pocsag_atb,
                        bch,
                        qual,
                        cfg.pocsag_fnu_mode,
                        cfg.pocsag_show_both,
                        cfg.show_tone_only,
                        cfg.show_numeric,
                        &mut out_msgs,
                    );
                    pocsag_atb.pocbit = pocsag_atb.pocbit.saturating_sub(1);
                    if pocsag_atb.pocbit == 0 {
                        pocsag_main.frame_reset(
                            cfg.pocsag_fnu_mode,
                            cfg.pocsag_show_both,
                            cfg.show_tone_only,
                            cfg.show_numeric,
                            &mut out_msgs,
                        );
                        pocsag_atb.baud_rate = 1600;
                        pocsag_atb.config_index = INDEX1600;
                    }
                    pocsag_atb.watch_ctr += pocsag_atb.watch_step;
                }
            }
        }

        // =========================================================================
        // 2. FLEX `Audio_To_Bits` Pipeline (`sound_in.cpp:523–620` + `Flex.cpp`)
        // =========================================================================
        if decode_flex {
            flex_atb.sync_baud_rate(atb_ctr);
            flex_atb.atb_len += 1;
            let f_center = flex_atb.atb_center[flex_atb.config_index];
            let mut f_thresh = flex_atb.atb_threshold[flex_atb.config_index];
            if flex.flex_blk > 0 && flex.levels == 4 {
                f_thresh = f_thresh.min((flex.header_outer_level * 0.15).max(2.0));
            }

            if val < (f_center - f_thresh) && flex_atb.atb_bit == flex_atb.high_audio {
                flex_atb.atb_bit = flex_atb.low_audio;
                if flex_atb.baud_rate < 3200 || flex_atb.low_audio == 1 {
                    let ratio = (flex_atb.atb_len as f64) / flex_atb.watch_step;
                    if (flex_atb.atb_len as f64) < flex_atb.watch_step * 2.0
                        && ratio > flex_atb.clkt_lo
                        && ratio < flex_atb.clkt_hi
                    {
                        let target_ctr = atb_ctr
                            + (flex_atb.watch_step * 0.5)
                            + flex_atb.atb_sample_offset[flex_atb.config_index];
                        if flex.flex_blk > 0 && flex.levels == 4 {
                            // Damped PLL during 4-FSK data blocks so skewed inner-to-outer
                            // zero-crossings (+1.6 kHz <-> -4.8 kHz) never yank the sample strobe
                            let err = target_ctr - flex_atb.watch_ctr;
                            if err.abs() < flex_atb.watch_step * 0.35 {
                                flex_atb.watch_ctr += err * 0.25;
                            }
                        } else {
                            flex_atb.watch_ctr = target_ctr;
                        }
                    }
                }
                flex_atb.atb_len = 0;
            } else if val > (f_center + f_thresh) && flex_atb.atb_bit == flex_atb.low_audio {
                flex_atb.atb_bit = flex_atb.high_audio;
                if flex_atb.baud_rate < 3200 || flex_atb.high_audio == 1 {
                    let ratio = (flex_atb.atb_len as f64) / flex_atb.watch_step;
                    if (flex_atb.atb_len as f64) < flex_atb.watch_step * 2.0
                        && ratio > flex_atb.clkt_lo
                        && ratio < flex_atb.clkt_hi
                    {
                        let target_ctr = atb_ctr
                            + (flex_atb.watch_step * 0.5)
                            + flex_atb.atb_sample_offset[flex_atb.config_index];
                        if flex.flex_blk > 0 && flex.levels == 4 {
                            let err = target_ctr - flex_atb.watch_ctr;
                            if err.abs() < flex_atb.watch_step * 0.35 {
                                flex_atb.watch_ctr += err * 0.25;
                            }
                        } else {
                            flex_atb.watch_ctr = target_ctr;
                        }
                    }
                }
                flex_atb.atb_len = 0;
            }

            if flex_atb.cross_over {
                if flex_atb.skipped_sc < 39 {
                    flex.frame_flex(
                        3,
                        1.0,
                        1.0,
                        flex_atb,
                        bch,
                        qual,
                        cfg.decode_flex_1600,
                        cfg.decode_flex_3200,
                        cfg.decode_flex_6400,
                        cfg.show_tone_only,
                        cfg.show_numeric,
                        cfg.convert_flex_si,
                        &mut out_msgs,
                    );
                }
                if flex_atb.skipped_sc < crossover_samples {
                    flex_atb.skipped_sc += 1;
                    continue;
                } else {
                    flex_atb.cross_over = false;
                    flex_atb.watch_ctr = atb_ctr
                        + (flex_atb.watch_step * 0.9)
                        + flex_atb.atb_sample_offset[flex_atb.config_index];
                }
            }

            if flex_atb.watch_ctr - atb_ctr < 1.0 {
                let (gin, conf_msb, conf_lsb) = flex.slice_symbol(val, flex_atb.atb_bit);
                flex.frame_flex(
                    gin,
                    conf_msb,
                    conf_lsb,
                    flex_atb,
                    bch,
                    qual,
                    cfg.decode_flex_1600,
                    cfg.decode_flex_3200,
                    cfg.decode_flex_6400,
                    cfg.show_tone_only,
                    cfg.show_numeric,
                    cfg.convert_flex_si,
                    &mut out_msgs,
                );
                flex_atb.watch_ctr += flex_atb.watch_step;
            }
        }
    }

    if pocsag_atb.pocbit > 0 {
        pocsag_atb.watch_ctr -= chunk.len() as f64;
        if pocsag_atb.watch_ctr < -pocsag_atb.watch_step {
            pocsag_atb.watch_ctr = pocsag_atb.watch_step * 0.5;
        }
    }

    flex_atb.watch_ctr -= chunk.len() as f64;
    if flex_atb.watch_ctr < -flex_atb.watch_step {
        flex_atb.watch_ctr = flex_atb.watch_step * 0.5;
    }

    let mut emitted_any = false;
    for m in out_msgs {
        if !cfg.show_binary && m.msg_type == "BINARY" {
            continue;
        }
        let _ = msg_tx.try_send(m);
        emitted_any = true;
    }

    // Throttle telemetry mutex updates to ~10 Hz (every 4096 samples) or when a message arrives
    let should_update_telem = emitted_any || ((*sample_counter / 4096) != (prev_counter / 4096));
    if should_update_telem {
        if let Ok(mut t) = telemetry.try_lock() {
            t.rx_quality_pct = qual.quality_pct;
            t.total_codewords = qual.total_codewords;
            t.soft_recoveries = qual.soft_recoveries;
            t.dc_center = env.dc_center();
            t.deviation = env.deviation();
            t.invert_active = flex_atb.invert || pocsag_atb.invert;
            if show_scope {
                t.scope_samples.clear();
                t.scope_samples.extend(scope_ring.iter().copied());
            }
            t.active_mode = if let Some(fl) = flex.active_mode_label() {
                fl
            } else if pocsag_atb.pocbit > 0 {
                format!("POCSAG-{}", pocsag_atb.pocsag_baud_rate)
            } else {
                format!("IDLE ({} Hz)", cfg.sample_rate)
            };
        }
    }

    // Wake GUI immediately when a new message is decoded
    if emitted_any {
        if let Some(ctx) = egui_ctx {
            ctx.request_repaint();
        }
    }
}
