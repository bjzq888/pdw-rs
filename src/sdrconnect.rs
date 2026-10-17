//! Direct SDRconnect WebSocket IQ Client & DC-Coupled Quadrature FM Discriminator.
//!
//! Connects directly to a running local (or LAN) SDRconnect instance over its built-in
//! WebSocket API (`ws://127.0.0.1:5454/` by default), enables the raw IQ stream
//! (`iq_stream_enable`, Binary Frame Type `0x0002` / `0x0005` interleaved `i16` IQ samples)
//! or VFO Audio stream (`audio_stream_enable`, Binary Frame Type `0x0001` / `0x0004`),
//! and performs DC-coupled quadrature polar FM demodulation (`atan2`) into `f32` baseband
//! samples for the POCSAG and FLEX (2-FSK / 4-FSK) DSP decoders.

use crossbeam_channel::{bounded, Receiver, Sender};
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub const DEFAULT_SDRCONNECT_HOST: &str = "127.0.0.1";
pub const DEFAULT_SDRCONNECT_PORT: u16 = 5454;

pub const SDRCONNECT_BIN_AUDIO_PRIMARY: u16 = 1;
pub const SDRCONNECT_BIN_IQ_PRIMARY: u16 = 2;
pub const SDRCONNECT_BIN_SPECTRUM_PRIMARY: u16 = 3;
pub const SDRCONNECT_BIN_AUDIO_SECONDARY: u16 = 4;
pub const SDRCONNECT_BIN_IQ_SECONDARY: u16 = 5;

#[derive(Clone, Debug)]
pub struct SdrConnectStatus {
    pub connected: bool,
    pub status_text: String,
    pub vfo_freq_hz: Option<u64>,
    pub center_freq_hz: Option<u64>,
    pub iq_sample_rate: u32,
    pub measured_iq_rate: u32,
    pub device_sample_rate: Option<u32>,
    pub demodulator: String,
    pub filter_bw_hz: Option<u32>,
    pub signal_power_dbm: Option<f32>,
    pub signal_snr_db: Option<f32>,
    pub iq_rssi_dbfs: f32,
    pub squelch_open: bool,
    pub carrier_detected: bool,
    pub afc_offset_hz: f32,
    pub iq_frames_received: u64,
}

impl Default for SdrConnectStatus {
    fn default() -> Self {
        Self {
            connected: false,
            status_text: "Disconnected".to_string(),
            vfo_freq_hz: None,
            center_freq_hz: None,
            iq_sample_rate: 48_000,
            measured_iq_rate: 0,
            device_sample_rate: None,
            demodulator: String::new(),
            filter_bw_hz: None,
            signal_power_dbm: None,
            signal_snr_db: None,
            iq_rssi_dbfs: -100.0,
            squelch_open: true,
            carrier_detected: false,
            afc_offset_hz: 0.0,
            iq_frames_received: 0,
        }
    }
}

#[derive(Clone, Debug)]
pub enum SdrConnectControlCmd {
    SetVfoFrequency(u64),
    SetCenterFrequency(u64),
    SetFilterBandwidth(u32),
    UpdateDemodParams {
        iq_gain: f32,
        squelch_enabled: bool,
        squelch_dbfs: f32,
        afc_enabled: bool,
        channel_bw_hz: f32,
        use_audio_stream: bool,
    },
}

#[derive(Clone, Debug)]
pub struct SdrConnectParams {
    pub host: String,
    pub port: u16,
    pub target_sample_rate: u32,
    pub iq_gain: f32,
    pub squelch_enabled: bool,
    pub squelch_dbfs: f32,
    pub afc_enabled: bool,
    pub channel_bw_hz: f32,
    pub tune_on_connect: bool,
    pub initial_vfo_freq_hz: u64,
    pub use_audio_stream: bool,
}

pub struct SdrConnectStream {
    stop_flag: Arc<AtomicBool>,
    ctrl_tx: Sender<SdrConnectControlCmd>,
    pub status: Arc<Mutex<SdrConnectStatus>>,
    _worker: Option<JoinHandle<()>>,
}

impl SdrConnectStream {
    pub fn send_command(&self, cmd: SdrConnectControlCmd) {
        let _ = self.ctrl_tx.try_send(cmd);
    }
}

impl Drop for SdrConnectStream {
    fn drop(&mut self) {
        self.stop_flag.store(true, Ordering::SeqCst);
        if let Some(handle) = self._worker.take() {
            let _ = handle.join();
        }
    }
}

#[derive(Clone, Debug, Default)]
struct ButterworthBiquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl ButterworthBiquad {
    fn configure(&mut self, sample_rate: f32, cutoff_hz: f32) {
        let sr = sample_rate.max(8_000.0);
        let fc = cutoff_hz.clamp(1_000.0, sr * 0.46);
        let omega = 2.0 * std::f32::consts::PI * fc / sr;
        let sn = omega.sin();
        let cs = omega.cos();
        let alpha = sn / std::f32::consts::SQRT_2;

        let b0 = (1.0 - cs) * 0.5;
        let b1 = 1.0 - cs;
        let b2 = (1.0 - cs) * 0.5;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cs;
        let a2 = 1.0 - alpha;

        self.b0 = b0 / a0;
        self.b1 = b1 / a0;
        self.b2 = b2 / a0;
        self.a1 = a1 / a0;
        self.a2 = a2 / a0;
    }

    #[inline]
    fn tick(&mut self, x0: f32) -> f32 {
        let y0 = self.b0 * x0 + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x0;
        self.y2 = self.y1;
        self.y1 = y0;
        y0
    }
}

#[derive(Clone, Debug)]
pub struct IqFmDemodulator {
    pub input_rate: u32,
    pub output_rate: u32,
    pub iq_gain: f32,
    pub squelch_enabled: bool,
    pub squelch_dbfs: f32,
    pub afc_enabled: bool,
    pub channel_bw_hz: f32,

    // Complex NCO for mixing wideband Type-2 IQ (`center_freq_hz` -> `vfo_freq_hz`)
    pub nco_offset_hz: f64,
    nco_phase: f64,

    // 2nd-order Butterworth complex baseband low-pass filters for I and Q
    biquad_i: ButterworthBiquad,
    biquad_q: ButterworthBiquad,

    // Fractional decimation accumulator (when input_rate > output_rate)
    decim_phase: f64,
    accum_i: f32,
    accum_q: f32,
    accum_count: u32,

    // Previous complex sample for polar FM discriminator z[n] * conj(z[n-1])
    prev_i: f32,
    prev_q: f32,
    prev_dphi: f32,

    // Post-discriminator 2nd-order Butterworth low-pass filter at 4,200 Hz (DC-coupled)
    audio_biquad: ButterworthBiquad,

    // RF power envelope EWMA, ultrasonic FM noise detector, & squelch state
    power_ewma: f32,
    hf_noise_ewma: f32,
    pub rssi_dbfs: f32,
    pub squelch_open: bool,
    pub carrier_detected: bool,

    // Preamble-gated Automatic Frequency Control (AFC) state
    last_pos_outer: f32,
    last_neg_outer: f32,
    last_outer_sign: i8,
    outer_run_len: usize,
    preamble_alt_hits: usize,
    pub afc_offset: f32,
}

impl IqFmDemodulator {
    pub fn new(
        input_rate: u32,
        output_rate: u32,
        iq_gain: f32,
        squelch_enabled: bool,
        squelch_dbfs: f32,
        afc_enabled: bool,
        channel_bw_hz: f32,
    ) -> Self {
        let mut s = Self {
            input_rate: input_rate.max(8_000),
            output_rate: output_rate.max(8_000),
            iq_gain: iq_gain.clamp(0.25, 3.0),
            squelch_enabled,
            squelch_dbfs,
            afc_enabled,
            channel_bw_hz: channel_bw_hz.clamp(12_500.0, 36_000.0),
            nco_offset_hz: 0.0,
            nco_phase: 0.0,
            biquad_i: ButterworthBiquad::default(),
            biquad_q: ButterworthBiquad::default(),
            decim_phase: 0.0,
            accum_i: 0.0,
            accum_q: 0.0,
            accum_count: 0,
            prev_i: 0.0,
            prev_q: 0.0,
            prev_dphi: 0.0,
            audio_biquad: ButterworthBiquad::default(),
            power_ewma: 1e-7,
            hf_noise_ewma: 1.2,
            rssi_dbfs: -90.0,
            squelch_open: true,
            carrier_detected: false,
            last_pos_outer: 0.70,
            last_neg_outer: -0.70,
            last_outer_sign: 0,
            outer_run_len: 0,
            preamble_alt_hits: 0,
            afc_offset: 0.0,
        };
        s.recompute_filters();
        s
    }

    pub fn recompute_filters(&mut self) {
        let in_sr = self.input_rate.max(8_000) as f32;
        let out_sr = self.output_rate.max(8_000) as f32;
        let cutoff_hz = (self.channel_bw_hz * 0.5).clamp(6_250.0, in_sr * 0.45);
        self.biquad_i.configure(in_sr, cutoff_hz);
        self.biquad_q.configure(in_sr, cutoff_hz);
        self.audio_biquad.configure(out_sr, 4_200.0_f32.min(out_sr * 0.45));
    }

    pub fn set_rates(&mut self, input_rate: u32, output_rate: u32) {
        let in_r = input_rate.max(8_000);
        let out_r = output_rate.max(8_000);
        if self.input_rate != in_r || self.output_rate != out_r {
            self.input_rate = in_r;
            self.output_rate = out_r;
            self.recompute_filters();
        }
    }

    pub fn process_binary_frame(&mut self, frame: &[u8]) -> Option<Vec<f32>> {
        if frame.len() < 6 {
            return None;
        }
        let pkt_type = u16::from_le_bytes([frame[0], frame[1]]);
        let body = &frame[2..];

        match pkt_type {
            SDRCONNECT_BIN_IQ_PRIMARY | SDRCONNECT_BIN_IQ_SECONDARY => {
                Some(self.demodulate_iq_le_i16_bytes(body))
            }
            SDRCONNECT_BIN_AUDIO_PRIMARY | SDRCONNECT_BIN_AUDIO_SECONDARY => {
                let n_frames = body.len() / 4;
                let mut out = Vec::with_capacity(n_frames);
                let mut p_acc = 0.0f32;
                for chunk in body.chunks_exact(4) {
                    let l = (i16::from_le_bytes([chunk[0], chunk[1]]) as f32) / 32768.0;
                    let r = (i16::from_le_bytes([chunk[2], chunk[3]]) as f32) / 32768.0;
                    let mono = if l.abs() > r.abs() * 2.5 {
                        l
                    } else if r.abs() > l.abs() * 2.5 {
                        r
                    } else {
                        0.5 * (l + r)
                    };
                    p_acc += mono * mono;
                    out.push(mono * self.iq_gain);
                }
                if n_frames > 0 {
                    let mean_p = (p_acc / (n_frames as f32)).max(1e-10);
                    self.power_ewma = 0.90 * self.power_ewma + 0.10 * mean_p;
                    self.rssi_dbfs = (10.0 * self.power_ewma.log10()).clamp(-100.0, 0.0);
                    self.carrier_detected = true;
                    self.squelch_open = true;
                }
                Some(out)
            }
            _ => None,
        }
    }

    pub fn demodulate_iq_le_i16_bytes(&mut self, iq_bytes: &[u8]) -> Vec<f32> {
        let num_iq_pairs = iq_bytes.len() / 4;
        let est_out = ((num_iq_pairs as u64 * self.output_rate as u64)
            / (self.input_rate.max(1) as u64)) as usize
            + 4;
        let mut out = Vec::with_capacity(est_out);

        let in_rate_f = self.input_rate as f64;
        let out_rate_f = self.output_rate as f64;
        let freq_norm_scale =
            (out_rate_f as f32) / (2.0 * std::f32::consts::PI * 6_400.0);
        let squelch_lin = 10.0_f32.powf(self.squelch_dbfs / 10.0);

        // Apply NCO mixing when receiving wideband hardware IQ (`input_rate > 96_000`) and VFO != LO center
        let use_nco = self.input_rate > 96_000 && self.nco_offset_hz.abs() > 1.0;
        let nco_step = -2.0 * std::f64::consts::PI * self.nco_offset_hz / in_rate_f;

        for chunk in iq_bytes.chunks_exact(4) {
            let mut raw_i = (i16::from_le_bytes([chunk[0], chunk[1]]) as f32) / 32768.0;
            let mut raw_q = (i16::from_le_bytes([chunk[2], chunk[3]]) as f32) / 32768.0;

            if use_nco {
                let (s, c) = self.nco_phase.sin_cos();
                let (sf, cf) = (s as f32, c as f32);
                let mixed_i = raw_i * cf - raw_q * sf;
                let mixed_q = raw_i * sf + raw_q * cf;
                raw_i = mixed_i;
                raw_q = mixed_q;
                self.nco_phase = (self.nco_phase + nco_step).rem_euclid(2.0 * std::f64::consts::PI);
            }

            // 1. Maximally flat 2nd-order Butterworth complex baseband low-pass filter
            let filt_i = self.biquad_i.tick(raw_i);
            let filt_q = self.biquad_q.tick(raw_q);

            // 2. Decimate / resample from `input_rate` to `output_rate`
            self.accum_i += filt_i;
            self.accum_q += filt_q;
            self.accum_count += 1;
            self.decim_phase += out_rate_f;

            if self.decim_phase >= in_rate_f {
                self.decim_phase -= in_rate_f;
                let inv_cnt = 1.0 / (self.accum_count.max(1) as f32);
                let fi = self.accum_i * inv_cnt;
                let fq = self.accum_q * inv_cnt;
                self.accum_i = 0.0;
                self.accum_q = 0.0;
                self.accum_count = 0;

                let inst_power = fi * fi + fq * fq;
                self.power_ewma = 0.995 * self.power_ewma + 0.005 * inst_power;

                // 3. Quadrature Polar FM Discriminator
                let re = fi * self.prev_i + fq * self.prev_q;
                let im = fi * self.prev_q - fq * self.prev_i;
                self.prev_i = fi;
                self.prev_q = fq;

                let dphi = if inst_power > 1e-12 {
                    im.atan2(re)
                } else {
                    0.0
                };

                let phase_diff = (dphi - self.prev_dphi).abs();
                self.prev_dphi = dphi;
                self.hf_noise_ewma = 0.99 * self.hf_noise_ewma + 0.01 * phase_diff;
                let coherent_carrier = self.hf_noise_ewma < 0.75;
                self.carrier_detected = coherent_carrier;

                let power_squelched = self.squelch_enabled && self.power_ewma < squelch_lin;
                self.squelch_open = !power_squelched;

                let raw_demod = dphi * freq_norm_scale;

                // 4. Preamble-Gated Outer-Symbol AFC
                if !self.afc_enabled {
                    self.afc_offset = 0.0;
                } else if coherent_carrier && !power_squelched {
                    let cur_sign: i8 = if raw_demod > 0.50 && raw_demod < 1.25 {
                        1
                    } else if raw_demod < -0.50 && raw_demod > -1.25 {
                        -1
                    } else {
                        0
                    };

                    if cur_sign != 0 {
                        if cur_sign == self.last_outer_sign {
                            self.outer_run_len += 1;
                            if self.outer_run_len > 10 {
                                if cur_sign == 1 {
                                    self.last_pos_outer =
                                        0.90 * self.last_pos_outer + 0.10 * raw_demod;
                                } else {
                                    self.last_neg_outer =
                                        0.90 * self.last_neg_outer + 0.10 * raw_demod;
                                }
                            }
                        } else {
                            if (12..=52).contains(&self.outer_run_len) {
                                self.preamble_alt_hits = self.preamble_alt_hits.saturating_add(1);
                            } else {
                                self.preamble_alt_hits = 0;
                            }
                            self.last_outer_sign = cur_sign;
                            self.outer_run_len = 1;
                        }

                        if self.preamble_alt_hits >= 12 {
                            let span = self.last_pos_outer - self.last_neg_outer;
                            if span > 1.05 && span < 1.85 {
                                let mid = 0.5 * (self.last_pos_outer + self.last_neg_outer);
                                self.afc_offset =
                                    (0.95 * self.afc_offset + 0.05 * mid).clamp(-0.30, 0.30);
                            }
                        }
                    } else {
                        self.outer_run_len += 1;
                        if self.outer_run_len > 60 {
                            self.preamble_alt_hits = 0;
                        }
                    }
                } else {
                    self.preamble_alt_hits = 0;
                }

                let centered = (raw_demod - self.afc_offset) * self.iq_gain;

                // 5. Post-discriminator 2nd-order Butterworth low-pass filter at 4,200 Hz
                let filt_aud = self.audio_biquad.tick(centered);
                let sample = if power_squelched { 0.0 } else { filt_aud };
                out.push(sample);
            }
        }

        self.rssi_dbfs = (10.0 * self.power_ewma.max(1e-10).log10()).clamp(-100.0, 0.0);
        out
    }

    pub fn afc_offset_hz(&self) -> f32 {
        self.afc_offset * 6_400.0
    }
}

/// Snap an empirically measured incoming sample rate to the nearest standard SDR sample rate.
fn snap_measured_sample_rate(measured: u32, reported_device_sr: Option<u32>) -> u32 {
    if let Some(dev_sr) = reported_device_sr {
        if dev_sr >= 48_000 {
            let diff = measured.abs_diff(dev_sr);
            if (diff as f64) < (dev_sr as f64) * 0.12 {
                return dev_sr;
            }
        }
    }
    const STANDARD_RATES: [u32; 12] = [
        48_000, 62_500, 96_000, 125_000, 192_000, 250_000, 384_000, 500_000, 768_000, 1_000_000,
        1_536_000, 2_000_000,
    ];
    for &std_sr in &STANDARD_RATES {
        let diff = measured.abs_diff(std_sr);
        if (diff as f64) < (std_sr as f64) * 0.08 {
            return std_sr;
        }
    }
    measured.max(48_000)
}

pub fn start_sdrconnect_iq_capture(
    params: SdrConnectParams,
    sample_tx: Sender<Vec<f32>>,
) -> SdrConnectStream {
    let stop_flag = Arc::new(AtomicBool::new(false));
    let (ctrl_tx, ctrl_rx) = bounded::<SdrConnectControlCmd>(32);
    let status = Arc::new(Mutex::new(SdrConnectStatus {
        connected: false,
        status_text: format!(
            "Connecting to SDRconnect ws://{}:{}/ ...",
            params.host, params.port
        ),
        iq_sample_rate: params.target_sample_rate,
        ..Default::default()
    }));

    let stop_clone = Arc::clone(&stop_flag);
    let status_clone = Arc::clone(&status);

    let worker = thread::spawn(move || {
        sdrconnect_worker_loop(params, sample_tx, ctrl_rx, stop_clone, status_clone);
    });

    SdrConnectStream {
        stop_flag,
        ctrl_tx,
        status,
        _worker: Some(worker),
    }
}

fn sdrconnect_worker_loop(
    mut params: SdrConnectParams,
    sample_tx: Sender<Vec<f32>>,
    ctrl_rx: Receiver<SdrConnectControlCmd>,
    stop_flag: Arc<AtomicBool>,
    status: Arc<Mutex<SdrConnectStatus>>,
) {
    let mut demod = IqFmDemodulator::new(
        params.target_sample_rate,
        params.target_sample_rate,
        params.iq_gain,
        params.squelch_enabled,
        params.squelch_dbfs,
        params.afc_enabled,
        params.channel_bw_hz,
    );

    while !stop_flag.load(Ordering::Relaxed) {
        if let Ok(mut st) = status.lock() {
            st.connected = false;
            st.status_text = format!(
                "Connecting to SDRconnect ws://{}:{}/ ...",
                params.host, params.port
            );
        }

        match WsClient::connect(&params.host, params.port) {
            Ok(mut ws) => {
                if let Ok(mut st) = status.lock() {
                    st.connected = true;
                    st.status_text = format!(
                        "SDRconnect {} Active (ws://{}:{}/)",
                        if params.use_audio_stream { "Audio" } else { "IQ" },
                        params.host,
                        params.port
                    );
                }

                if params.tune_on_connect && params.initial_vfo_freq_hz > 0 {
                    let _ = ws.send_json_cmd(
                        "set_property",
                        "device_vfo_frequency",
                        &params.initial_vfo_freq_hz.to_string(),
                    );
                }

                let _ = ws.send_json_cmd("set_property", "nfm_deemphasis_enable", "false");
                let _ = ws.send_json_cmd("set_property", "audio_filter", "false");
                let _ = ws.send_json_cmd("set_property", "audio_limiters", "false");

                for prop in [
                    "device_vfo_frequency",
                    "device_center_frequency",
                    "device_sample_rate",
                    "demodulator",
                    "filter_bandwidth",
                    "signal_power",
                    "signal_snr",
                ] {
                    let _ = ws.send_json_cmd("get_property", prop, "");
                }

                if params.use_audio_stream {
                    let _ = ws.send_json_cmd("iq_stream_enable", "", "false");
                    let _ = ws.send_json_cmd("audio_stream_enable", "", "true");
                } else {
                    let _ = ws.send_json_cmd("audio_stream_enable", "", "false");
                    let _ = ws.send_json_cmd("iq_stream_enable", "", "true");
                }

                let mut last_telem_push = Instant::now();
                let mut rate_window_start = Instant::now();
                let mut rate_window_samples: u64 = 0;
                let mut iq_frames: u64 = 0;

                while !stop_flag.load(Ordering::Relaxed) {
                    while let Ok(cmd) = ctrl_rx.try_recv() {
                        match cmd {
                            SdrConnectControlCmd::SetVfoFrequency(hz) => {
                                params.initial_vfo_freq_hz = hz;
                                let need_recenter = status
                                    .lock()
                                    .ok()
                                    .and_then(|st| st.center_freq_hz)
                                    .map(|cf| cf.abs_diff(hz) > 400_000)
                                    .unwrap_or(false);
                                if need_recenter {
                                    let _ = ws.send_json_cmd(
                                        "set_property",
                                        "device_center_frequency",
                                        &hz.to_string(),
                                    );
                                }
                                let _ = ws.send_json_cmd(
                                    "set_property",
                                    "device_vfo_frequency",
                                    &hz.to_string(),
                                );
                                let _ = ws.send_json_cmd("get_property", "device_vfo_frequency", "");
                                let _ =
                                    ws.send_json_cmd("get_property", "device_center_frequency", "");
                            }
                            SdrConnectControlCmd::SetCenterFrequency(hz) => {
                                let _ = ws.send_json_cmd(
                                    "set_property",
                                    "device_center_frequency",
                                    &hz.to_string(),
                                );
                                let _ =
                                    ws.send_json_cmd("get_property", "device_center_frequency", "");
                            }
                            SdrConnectControlCmd::SetFilterBandwidth(bw) => {
                                let _ = ws.send_json_cmd(
                                    "set_property",
                                    "filter_bandwidth",
                                    &bw.to_string(),
                                );
                                let _ = ws.send_json_cmd("get_property", "filter_bandwidth", "");
                            }
                            SdrConnectControlCmd::UpdateDemodParams {
                                iq_gain,
                                squelch_enabled,
                                squelch_dbfs,
                                afc_enabled,
                                channel_bw_hz,
                                use_audio_stream,
                            } => {
                                demod.iq_gain = iq_gain.clamp(0.25, 3.0);
                                demod.squelch_enabled = squelch_enabled;
                                demod.squelch_dbfs = squelch_dbfs;
                                demod.afc_enabled = afc_enabled;
                                demod.channel_bw_hz = channel_bw_hz.clamp(12_500.0, 36_000.0);
                                demod.recompute_filters();
                                if use_audio_stream != params.use_audio_stream {
                                    params.use_audio_stream = use_audio_stream;
                                    if params.use_audio_stream {
                                        let _ = ws.send_json_cmd("iq_stream_enable", "", "false");
                                        let _ = ws.send_json_cmd("audio_stream_enable", "", "true");
                                    } else {
                                        let _ =
                                            ws.send_json_cmd("audio_stream_enable", "", "false");
                                        let _ = ws.send_json_cmd("iq_stream_enable", "", "true");
                                    }
                                }
                            }
                        }
                    }

                    match ws.read_message() {
                        Ok(Some(WsMessage::Binary(bin))) => {
                            if bin.len() >= 6 {
                                let pkt_type = u16::from_le_bytes([bin[0], bin[1]]);
                                if pkt_type == SDRCONNECT_BIN_IQ_PRIMARY
                                    || pkt_type == SDRCONNECT_BIN_IQ_SECONDARY
                                {
                                    let n_pairs = ((bin.len() - 2) / 4) as u64;
                                    rate_window_samples += n_pairs;
                                }
                            }

                            let elapsed_rate = rate_window_start.elapsed().as_secs_f64();
                            let measured_sr = if elapsed_rate >= 0.40 && rate_window_samples > 0 {
                                let raw_r =
                                    (rate_window_samples as f64 / elapsed_rate).round() as u32;
                                rate_window_start = Instant::now();
                                rate_window_samples = 0;
                                let dev_sr = status.lock().ok().and_then(|s| s.device_sample_rate);
                                let snapped = snap_measured_sample_rate(raw_r, dev_sr);
                                let out_sr = demod.output_rate;
                                demod.set_rates(snapped, out_sr);
                                Some(snapped)
                            } else {
                                None
                            };

                            if let Some(samples) = demod.process_binary_frame(&bin) {
                                iq_frames = iq_frames.wrapping_add(1);
                                if !samples.is_empty() {
                                    let _ = sample_tx.try_send(samples);
                                }

                                if last_telem_push.elapsed() >= Duration::from_millis(200) {
                                    last_telem_push = Instant::now();
                                    if let Ok(mut st) = status.try_lock() {
                                        st.iq_rssi_dbfs = demod.rssi_dbfs;
                                        st.squelch_open = demod.squelch_open;
                                        st.carrier_detected = demod.carrier_detected;
                                        st.afc_offset_hz = demod.afc_offset_hz();
                                        st.iq_frames_received = iq_frames;
                                        if let Some(msr) = measured_sr {
                                            st.measured_iq_rate = msr;
                                            st.iq_sample_rate = msr;
                                        }
                                    }
                                }
                            }
                        }
                        Ok(Some(WsMessage::Text(txt))) => {
                            handle_sdrconnect_json_event(&txt, &mut ws, &mut demod, &status);
                        }
                        Ok(None) => {}
                        Err(e) => {
                            if let Ok(mut st) = status.lock() {
                                st.connected = false;
                                st.status_text = format!("SDRconnect connection lost ({})", e);
                            }
                            break;
                        }
                    }
                }

                let _ = ws.send_json_cmd("iq_stream_enable", "", "false");
                let _ = ws.send_json_cmd("audio_stream_enable", "", "false");
                let _ = ws.send_close();
            }
            Err(e) => {
                if let Ok(mut st) = status.lock() {
                    st.connected = false;
                    st.status_text = format!(
                        "SDRconnect ws://{}:{}/ unreachable ({}) — retrying...",
                        params.host, params.port, e
                    );
                }
            }
        }

        for _ in 0..15 {
            if stop_flag.load(Ordering::Relaxed) {
                break;
            }
            while let Ok(cmd) = ctrl_rx.try_recv() {
                if let SdrConnectControlCmd::UpdateDemodParams {
                    iq_gain,
                    squelch_enabled,
                    squelch_dbfs,
                    afc_enabled,
                    channel_bw_hz,
                    use_audio_stream,
                } = cmd
                {
                    demod.iq_gain = iq_gain.clamp(0.25, 3.0);
                    demod.squelch_enabled = squelch_enabled;
                    demod.squelch_dbfs = squelch_dbfs;
                    demod.afc_enabled = afc_enabled;
                    demod.channel_bw_hz = channel_bw_hz.clamp(12_500.0, 36_000.0);
                    demod.recompute_filters();
                    params.use_audio_stream = use_audio_stream;
                }
            }
            thread::sleep(Duration::from_millis(100));
        }
    }
}

fn handle_sdrconnect_json_event(
    json_text: &str,
    ws: &mut WsClient,
    demod: &mut IqFmDemodulator,
    status: &Arc<Mutex<SdrConnectStatus>>,
) {
    let event_type = extract_json_str_field(json_text, "event_type").unwrap_or_default();
    if event_type != "get_property_response" && event_type != "property_changed" {
        return;
    }
    let prop = extract_json_str_field(json_text, "property").unwrap_or_default();
    let val = extract_json_str_field(json_text, "value").unwrap_or_default();

    let mut widen_filter_to: Option<u32> = None;

    if let Ok(mut st) = status.lock() {
        match prop.as_str() {
            "device_vfo_frequency" => {
                if let Ok(hz) = val.trim().parse::<u64>() {
                    st.vfo_freq_hz = Some(hz);
                }
            }
            "device_center_frequency" => {
                if let Ok(hz) = val.trim().parse::<u64>() {
                    st.center_freq_hz = Some(hz);
                }
            }
            "iq_sample_rate" | "audio_sample_rate" => {
                if let Ok(sr) = val.trim().parse::<u32>() {
                    if sr >= 8_000 {
                        st.iq_sample_rate = sr;
                        let out_sr = demod.output_rate;
                        demod.set_rates(sr, out_sr);
                    }
                }
            }
            "device_sample_rate" => {
                if let Ok(sr) = val.trim().parse::<f64>() {
                    let sr_u32 = sr as u32;
                    st.device_sample_rate = Some(sr_u32);
                }
            }
            "demodulator" => {
                st.demodulator = val;
            }
            "filter_bandwidth" => {
                if let Ok(bw) = val.trim().parse::<u32>() {
                    st.filter_bw_hz = Some(bw);
                    if bw > 0 && bw < 16_000 {
                        widen_filter_to = Some(16_000);
                    }
                }
            }
            "signal_power" => {
                if let Ok(p) = val.trim().parse::<f32>() {
                    st.signal_power_dbm = Some(p);
                }
            }
            "signal_snr" => {
                if let Ok(snr) = val.trim().parse::<f32>() {
                    st.signal_snr_db = Some(snr);
                }
            }
            _ => {}
        }

        if let (Some(vfo), Some(center)) = (st.vfo_freq_hz, st.center_freq_hz) {
            demod.nco_offset_hz = (vfo as f64) - (center as f64);
        }
    }

    if let Some(target_bw) = widen_filter_to {
        let _ = ws.send_json_cmd("set_property", "filter_bandwidth", &target_bw.to_string());
    }
}

fn extract_json_str_field(json: &str, key: &str) -> Option<String> {
    let pat = format!("\"{}\"", key);
    let idx = json.find(&pat)?;
    let after_key = &json[idx + pat.len()..];
    let colon_idx = after_key.find(':')?;
    let after_colon = after_key[colon_idx + 1..].trim_start();
    if let Some(stripped) = after_colon.strip_prefix('"') {
        let end_quote = stripped.find('"')?;
        Some(stripped[..end_quote].to_string())
    } else {
        let end = after_colon
            .find(|c: char| c == ',' || c.is_whitespace() || c == '\x7d')
            .unwrap_or(after_colon.len());
        Some(after_colon[..end].to_string())
    }
}

enum WsMessage {
    Text(String),
    Binary(Vec<u8>),
}

struct WsClient {
    stream: TcpStream,
    preroll: Vec<u8>,
    preroll_pos: usize,
    frag_opcode: u8,
    frag_buf: Vec<u8>,
    mask_seed: u32,
}

impl WsClient {
    fn connect(host: &str, port: u16) -> Result<Self, String> {
        let clean_host = host
            .trim()
            .trim_start_matches("ws://")
            .trim_start_matches("http://")
            .trim_end_matches('/');
        let (host_only, port_final) = if let Some((h, p_str)) = clean_host.rsplit_once(':') {
            if let Ok(p) = p_str.parse::<u16>() {
                (h, p)
            } else {
                (clean_host, port)
            }
        } else {
            (clean_host, port)
        };

        let addr_str = format!("{}:{}", host_only, port_final);
        let sock_addr: SocketAddr = addr_str
            .to_socket_addrs()
            .map_err(|e| format!("DNS/address error: {}", e))?
            .next()
            .ok_or_else(|| "Unable to resolve SDRconnect host".to_string())?;

        let mut stream = TcpStream::connect_timeout(&sock_addr, Duration::from_millis(1500))
            .map_err(|e| e.to_string())?;
        let _ = stream.set_nodelay(true);
        let _ = stream.set_read_timeout(Some(Duration::from_millis(100)));
        let _ = stream.set_write_timeout(Some(Duration::from_millis(1000)));

        let req = format!(
            "GET / HTTP/1.1\r\n\
             Host: {}:{}\r\n\
             Upgrade: websocket\r\n\
             Connection: Upgrade\r\n\
             Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
             Sec-WebSocket-Version: 13\r\n\r\n",
            host_only, port_final
        );
        stream
            .write_all(req.as_bytes())
            .map_err(|e| format!("Handshake write failed: {}", e))?;

        let mut resp_buf = Vec::with_capacity(512);
        let mut temp = [0u8; 256];
        let header_end;
        let deadline = Instant::now() + Duration::from_millis(2000);
        loop {
            if Instant::now() > deadline {
                return Err("WebSocket handshake timed out".to_string());
            }
            match stream.read(&mut temp) {
                Ok(0) => return Err("SDRconnect closed connection during handshake".to_string()),
                Ok(n) => {
                    resp_buf.extend_from_slice(&temp[..n]);
                    if let Some(pos) = resp_buf.windows(4).position(|w| w == b"\r\n\r\n") {
                        header_end = pos + 4;
                        break;
                    }
                    if resp_buf.len() > 8192 {
                        return Err("Invalid WebSocket HTTP header".to_string());
                    }
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => return Err(format!("Handshake read error: {}", e)),
            }
        }

        let header_str = String::from_utf8_lossy(&resp_buf[..header_end]);
        if !header_str.starts_with("HTTP/1.1 101") && !header_str.contains(" 101 ") {
            let first_line = header_str.lines().next().unwrap_or("Unknown HTTP response");
            return Err(format!("Handshake rejected: {}", first_line));
        }

        let preroll = resp_buf[header_end..].to_vec();
        Ok(Self {
            stream,
            preroll,
            preroll_pos: 0,
            frag_opcode: 0,
            frag_buf: Vec::new(),
            mask_seed: 0x6D2B_79F5,
        })
    }

    fn send_json_cmd(&mut self, event_type: &str, property: &str, value: &str) -> Result<(), String> {
        let json = format!(
            "{{\"event_type\":\"{}\",\"property\":\"{}\",\"value\":\"{}\"}}",
            event_type, property, value
        );
        self.send_masked_frame(0x81, json.as_bytes())
    }

    fn send_close(&mut self) -> Result<(), String> {
        self.send_masked_frame(0x88, &[])
    }

    fn send_masked_frame(&mut self, first_byte: u8, payload: &[u8]) -> Result<(), String> {
        self.mask_seed = self.mask_seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let mask = self.mask_seed.to_le_bytes();

        let mut frame = Vec::with_capacity(14 + payload.len());
        frame.push(first_byte);

        let len = payload.len();
        if len < 126 {
            frame.push(0x80 | (len as u8));
        } else if len <= 65535 {
            frame.push(0x80 | 126);
            frame.extend_from_slice(&(len as u16).to_be_bytes());
        } else {
            frame.push(0x80 | 127);
            frame.extend_from_slice(&(len as u64).to_be_bytes());
        }

        frame.extend_from_slice(&mask);
        for (i, &b) in payload.iter().enumerate() {
            frame.push(b ^ mask[i & 3]);
        }

        self.stream
            .write_all(&frame)
            .map_err(|e| format!("WebSocket write error: {}", e))
    }

    fn read_exact_bytes(&mut self, buf: &mut [u8], allow_timeout_on_first: bool) -> Result<bool, String> {
        let mut offset = 0usize;
        while offset < buf.len() {
            if self.preroll_pos < self.preroll.len() {
                let avail = self.preroll.len() - self.preroll_pos;
                let take = avail.min(buf.len() - offset);
                buf[offset..offset + take]
                    .copy_from_slice(&self.preroll[self.preroll_pos..self.preroll_pos + take]);
                self.preroll_pos += take;
                offset += take;
                if self.preroll_pos >= self.preroll.len() {
                    self.preroll.clear();
                    self.preroll_pos = 0;
                }
                continue;
            }

            match self.stream.read(&mut buf[offset..]) {
                Ok(0) => return Err("SDRconnect closed WebSocket".to_string()),
                Ok(n) => {
                    offset += n;
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    if offset == 0 && allow_timeout_on_first {
                        return Ok(false);
                    }
                    continue;
                }
                Err(e) => return Err(format!("WebSocket read error: {}", e)),
            }
        }
        Ok(true)
    }

    fn read_message(&mut self) -> Result<Option<WsMessage>, String> {
        let mut hdr = [0u8; 2];
        if !self.read_exact_bytes(&mut hdr, true)? {
            return Ok(None);
        }

        let fin = (hdr[0] & 0x80) != 0;
        let opcode = hdr[0] & 0x0f;
        let masked = (hdr[1] & 0x80) != 0;
        let mut payload_len = (hdr[1] & 0x7f) as usize;

        if payload_len == 126 {
            let mut ext = [0u8; 2];
            self.read_exact_bytes(&mut ext, false)?;
            payload_len = u16::from_be_bytes(ext) as usize;
        } else if payload_len == 127 {
            let mut ext = [0u8; 8];
            self.read_exact_bytes(&mut ext, false)?;
            payload_len = u64::from_be_bytes(ext) as usize;
        }

        if payload_len > 16 * 1024 * 1024 {
            return Err("WebSocket frame exceeds 16 MB safety limit".to_string());
        }

        let mut mask_key = [0u8; 4];
        if masked {
            self.read_exact_bytes(&mut mask_key, false)?;
        }

        let mut payload = vec![0u8; payload_len];
        if payload_len > 0 {
            self.read_exact_bytes(&mut payload, false)?;
            if masked {
                for (i, b) in payload.iter_mut().enumerate() {
                    *b ^= mask_key[i & 3];
                }
            }
        }

        match opcode {
            0x8 => Err("SDRconnect sent WebSocket Close frame".to_string()),
            0x9 => {
                let _ = self.send_masked_frame(0x8A, &payload);
                Ok(None)
            }
            0xA => Ok(None),
            0x1 | 0x2 => {
                if fin {
                    if opcode == 0x1 {
                        Ok(Some(WsMessage::Text(
                            String::from_utf8_lossy(&payload).into_owned(),
                        )))
                    } else {
                        Ok(Some(WsMessage::Binary(payload)))
                    }
                } else {
                    self.frag_opcode = opcode;
                    self.frag_buf.clear();
                    self.frag_buf.extend_from_slice(&payload);
                    Ok(None)
                }
            }
            0x0 => {
                self.frag_buf.extend_from_slice(&payload);
                if fin {
                    let completed = std::mem::take(&mut self.frag_buf);
                    let orig_op = self.frag_opcode;
                    self.frag_opcode = 0;
                    if orig_op == 0x1 {
                        Ok(Some(WsMessage::Text(
                            String::from_utf8_lossy(&completed).into_owned(),
                        )))
                    } else {
                        Ok(Some(WsMessage::Binary(completed)))
                    }
                } else {
                    Ok(None)
                }
            }
            _ => Ok(None),
        }
    }
}

pub fn synthesize_sdrconnect_iq_frame_from_baseband(
    baseband: &[f32],
    sample_rate: u32,
    carrier_offset_hz: f32,
) -> Vec<u8> {
    let mut frame = Vec::with_capacity(2 + baseband.len() * 4);
    frame.extend_from_slice(&SDRCONNECT_BIN_IQ_PRIMARY.to_le_bytes());

    let mut phase: f64 = 0.0;
    let sr = sample_rate.max(8_000) as f64;
    let two_pi = 2.0 * std::f64::consts::PI;

    for &s in baseband {
        let inst_freq_hz = -((s as f64) * 6_400.0 + (carrier_offset_hz as f64));
        phase = (phase + two_pi * inst_freq_hz / sr).rem_euclid(two_pi);

        let i_val = (phase.cos() * 22_937.0).round().clamp(-32767.0, 32767.0) as i16;
        let q_val = (phase.sin() * 22_937.0).round().clamp(-32767.0, 32767.0) as i16;

        frame.extend_from_slice(&i_val.to_le_bytes());
        frame.extend_from_slice(&q_val.to_le_bytes());
    }

    frame
}
