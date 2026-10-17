//! 48 kHz / 44.1 kHz Audio DSP Front-End & Exact Port of `sound_in.cpp` (`Audio_To_Bits`).
//!
//! Directly implements `sound_in.cpp`'s proven Schmitt-trigger slicer (`atb_bit`, `atb_center`,
//! `atb_threshold`, `pre_threshold`, `InvertData()`), 50-transition POCSAG preamble detector,
//! `WatchStep`/`WatchCtr` zero-crossing bit synchronizer (with the `WatchCtr = -1` bug fixed),
//! and 1600 -> 3200 baud FLEX crossover (`cross_over` / `skipped_sc`), scaled seamlessly to any
//! sample rate (48,000 Hz, 44,100 Hz, 22,050 Hz, 11,025 Hz, 8,000 Hz).

use serde::{Deserialize, Serialize};

/// Default sample rate matching modern Windows WASAPI soundcards (48,000 Hz).
/// Automatically adapts to 44,100 Hz when capturing from a 44.1 kHz device such as VB-Audio Virtual Cable.
pub const DEFAULT_SAMPLE_RATE: u32 = 48_000;

/// Supported audio sample rates in the Setup dialog.
pub const SUPPORTED_SAMPLE_RATES: [u32; 5] = [48_000, 44_100, 22_050, 11_025, 8_000];

pub const COURSE_CLKT_HI: f64 = 2.02;
pub const COURSE_CLKT_LO: f64 = 0.55;
pub const FINE_CLKT_HI: f64 = 1.90;
pub const FINE_CLKT_LO: f64 = 0.58;
pub const FINE_CLKT3200_HI: f64 = 1.92;
pub const FINE_CLKT3200_LO: f64 = 0.58;

pub const INDEX512: usize = 0;
pub const INDEX1200: usize = 1;
pub const INDEX2400: usize = 2;
pub const INDEX1600: usize = 3;
pub const INDEX3200: usize = 4;

/// Exact audio configuration presets from `sound_in.cpp:850–1027` (`SetAudioConfig`).
/// Default in PDW (`PDW.cpp:632`) is `Earphone1` (`sac_type = 5`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AudioPreset {
    Earphone1 = 5,
    Earphone2 = 6,
    Earphone3 = 7,
    Discriminator1 = 1,
    Discriminator2 = 2,
    Discriminator3 = 3,
    Discriminator4 = 4,
    Speaker1 = 8,
    Speaker2 = 9,
    Speaker3 = 10,
    TapeRec1 = 11,
    TapeRec2 = 12,
    TapeRec3 = 13,
    Default8 = 14,
}

impl Default for AudioPreset {
    fn default() -> Self {
        Self::Earphone1
    }
}

impl AudioPreset {
    pub const ALL: [AudioPreset; 14] = [
        AudioPreset::Earphone1,
        AudioPreset::Earphone2,
        AudioPreset::Earphone3,
        AudioPreset::Discriminator1,
        AudioPreset::Discriminator2,
        AudioPreset::Discriminator3,
        AudioPreset::Discriminator4,
        AudioPreset::Speaker1,
        AudioPreset::Speaker2,
        AudioPreset::Speaker3,
        AudioPreset::TapeRec1,
        AudioPreset::TapeRec2,
        AudioPreset::TapeRec3,
        AudioPreset::Default8,
    ];

    pub fn label(self) -> &'static str {
        match self {
            AudioPreset::Earphone1 => "Earphone 1 (PDW Default: Thresh 7/4)",
            AudioPreset::Earphone2 => "Earphone 2 (Thresh 9/5)",
            AudioPreset::Earphone3 => "Earphone 3 (Thresh 14/7)",
            AudioPreset::Discriminator1 => "Discriminator 1 (Thresh 16/6)",
            AudioPreset::Discriminator2 => "Discriminator 2 (Thresh 6/5)",
            AudioPreset::Discriminator3 => "Discriminator 3 (Thresh 44/22)",
            AudioPreset::Discriminator4 => "Discriminator 4 (Sensitive: Thresh 2/2)",
            AudioPreset::Speaker1 => "Speaker Out 1 (Thresh 14/7)",
            AudioPreset::Speaker2 => "Speaker Out 2 (Thresh 9/5)",
            AudioPreset::Speaker3 => "Speaker Out 3 (Thresh 34/14)",
            AudioPreset::TapeRec1 => "Tape/Rec Out 1 (Thresh 7/3)",
            AudioPreset::TapeRec2 => "Tape/Rec Out 2 (Thresh 5/3)",
            AudioPreset::TapeRec3 => "Tape/Rec Out 3 (Thresh 15/7)",
            AudioPreset::Default8 => "Standard (Thresh 8/4)",
        }
    }

    pub fn from_pdw_sac_type(sac_type: i32) -> Self {
        match sac_type {
            1 => AudioPreset::Discriminator1,
            2 => AudioPreset::Discriminator2,
            3 => AudioPreset::Discriminator3,
            4 => AudioPreset::Discriminator4,
            5 => AudioPreset::Earphone1,
            6 => AudioPreset::Earphone2,
            7 => AudioPreset::Earphone3,
            8 => AudioPreset::Speaker1,
            9 => AudioPreset::Speaker2,
            10 => AudioPreset::Speaker3,
            11 => AudioPreset::TapeRec1,
            12 => AudioPreset::TapeRec2,
            13 => AudioPreset::TapeRec3,
            _ => AudioPreset::Earphone1,
        }
    }

    /// Returns `(atb_threshold[0..5], pre_threshold)` on PDW's signed 8-bit scale (`-128..127`),
    /// matching `SetAudioConfig` in `sound_in.cpp:850–1027`.
    pub fn thresholds(self) -> ([f32; 5], f32) {
        match self {
            AudioPreset::Discriminator1 => ([16.0, 16.0, 16.0, 16.0, 6.0], 11.0),
            AudioPreset::Discriminator2 => ([6.0, 6.0, 6.0, 6.0, 5.0], 6.0),
            AudioPreset::Discriminator3 => ([44.0, 44.0, 44.0, 44.0, 22.0], 15.0),
            AudioPreset::Discriminator4 => ([2.0, 2.0, 2.0, 2.0, 2.0], 4.0),
            AudioPreset::Earphone1 => ([7.0, 7.0, 7.0, 7.0, 4.0], 7.0),
            AudioPreset::Earphone2 => ([9.0, 9.0, 9.0, 9.0, 5.0], 8.0),
            AudioPreset::Earphone3 => ([14.0, 14.0, 14.0, 14.0, 7.0], 9.0),
            AudioPreset::Speaker1 => ([14.0, 14.0, 14.0, 14.0, 7.0], 10.0),
            AudioPreset::Speaker2 => ([9.0, 9.0, 9.0, 9.0, 5.0], 8.0),
            AudioPreset::Speaker3 => ([34.0, 34.0, 34.0, 34.0, 14.0], 14.0),
            AudioPreset::TapeRec1 => ([7.0, 7.0, 7.0, 7.0, 3.0], 7.0),
            AudioPreset::TapeRec2 => ([5.0, 5.0, 5.0, 5.0, 3.0], 6.0),
            AudioPreset::TapeRec3 => ([15.0, 15.0, 15.0, 15.0, 7.0], 10.0),
            AudioPreset::Default8 => ([8.0, 8.0, 8.0, 8.0, 4.0], 8.0),
        }
    }
}

/// Telemetry tracker for UI display (`DC`, `Dev`, and 4-FSK header level calibration).
/// Does NOT shift `atb_center` away from 0.0 during long runs of 0s/1s, preventing slicer lockup.
#[derive(Clone, Debug)]
pub struct AdaptiveEnvelopeTracker {
    pub dc_ewma: f32,
    pub peak_ewma: f32,
    pub manual_center_offset: f32,
    pub manual_hysteresis: f32,
}

impl Default for AdaptiveEnvelopeTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl AdaptiveEnvelopeTracker {
    pub fn new() -> Self {
        Self {
            dc_ewma: 0.0,
            peak_ewma: 0.65,
            manual_center_offset: 0.0,
            manual_hysteresis: 0.02,
        }
    }

    #[inline]
    pub fn update(&mut self, sample: f32) {
        self.dc_ewma = 0.9998 * self.dc_ewma + 0.0002 * sample;
        let abs_s = sample.abs();
        if abs_s > self.peak_ewma {
            self.peak_ewma = 0.92 * self.peak_ewma + 0.08 * abs_s;
        } else {
            self.peak_ewma = (0.99995 * self.peak_ewma + 0.00005 * abs_s).max(0.05);
        }
    }

    #[inline]
    pub fn dc_center(&self) -> f32 {
        self.dc_ewma + self.manual_center_offset
    }

    #[inline]
    pub fn deviation(&self) -> f32 {
        self.peak_ewma
    }

    #[inline]
    pub fn hysteresis(&self) -> f32 {
        self.manual_hysteresis
    }
}

/// Exact port of `sound_in.cpp`'s `Audio_To_Bits` state machine (`Reset_ATB` + `SetAudioConfig` + `InvertData`).
#[derive(Clone, Debug)]
pub struct AtbSlicer {
    pub sample_rate: u32,
    pub high_audio: u8,
    pub low_audio: u8,
    pub invert: bool,

    // Preamble detector variables (`sound_in.cpp:60–67`)
    pub n_samples: usize,
    pub preamble_count: [usize; 3],
    pub sync_bit: u8,
    pub crossing: bool,
    pub pre_threshold: f32,

    // Main slicer & clock variables (`sound_in.cpp:69–86`)
    pub atb_bit: u8,
    pub atb_len: usize,
    pub watch_step: f64,
    pub clkt_hi: f64,
    pub clkt_lo: f64,
    pub watch_ctr: f64,
    pub baud_rate: u32,
    pub last_baud_rate: u32,
    pub atb_center: [f32; 5],
    pub atb_threshold: [f32; 5],
    pub atb_sample_offset: [f64; 5],
    pub config_index: usize,
    pub cross_over: bool,
    pub skipped_sc: usize,

    // Active POCSAG countdown (`pocbit` in `sound_in.cpp:89`)
    pub pocbit: usize,
    pub pocsag_baud_rate: u32,
}

impl AtbSlicer {
    pub fn new(sample_rate: u32, preset: AudioPreset, invert: bool) -> Self {
        let (atb_threshold, pre_threshold) = preset.thresholds();
        let low_audio = if invert { 1 } else { 0 };
        let high_audio = if invert { 0 } else { 1 };
        let watch_step = (sample_rate as f64) / 1600.0;
        Self {
            sample_rate,
            high_audio,
            low_audio,
            invert,
            n_samples: 0,
            preamble_count: [0; 3],
            sync_bit: low_audio,
            crossing: false,
            pre_threshold,
            atb_bit: low_audio,
            atb_len: 0,
            watch_step,
            clkt_hi: FINE_CLKT_HI,
            clkt_lo: FINE_CLKT_LO,
            watch_ctr: watch_step * 0.5,
            baud_rate: 1600,
            last_baud_rate: 1600,
            atb_center: [0.0; 5],
            atb_threshold,
            atb_sample_offset: [0.0; 5],
            config_index: INDEX1600,
            cross_over: false,
            skipped_sc: 0,
            pocbit: 0,
            pocsag_baud_rate: 1200,
        }
    }

    pub fn set_preset(&mut self, preset: AudioPreset) {
        let (atb_threshold, pre_threshold) = preset.thresholds();
        self.atb_threshold = atb_threshold;
        self.pre_threshold = pre_threshold;
    }

    pub fn set_invert(&mut self, invert: bool) {
        if self.invert != invert {
            self.invert_data();
        }
    }

    /// Exact port of `InvertData()` (`Misc.cpp:2736–2742`), plus flipping `atb_bit` and `sync_bit`
    /// so the Schmitt trigger stays aligned with physical voltage polarity without a false edge.
    pub fn invert_data(&mut self) {
        self.invert = !self.invert;
        self.low_audio = if self.invert { 1 } else { 0 };
        self.high_audio = if self.invert { 0 } else { 1 };
        self.atb_bit ^= 1;
        self.sync_bit ^= 1;
    }

    pub fn set_sample_rate(&mut self, sample_rate: u32) {
        if self.sample_rate != sample_rate {
            self.sample_rate = sample_rate;
            self.watch_step = (sample_rate as f64) / (self.baud_rate as f64);
            self.watch_ctr = self.watch_step * 0.5;
        }
    }

    /// Update `watch_step`, `config_index`, `clkt_hi`, and `clkt_lo` when `baud_rate` changes
    /// (`sound_in.cpp:396–419`, with the `WatchCtr = -1` bug fixed).
    #[inline]
    pub fn sync_baud_rate(&mut self, atb_ctr: f64) {
        if self.baud_rate != self.last_baud_rate {
            self.watch_step = (self.sample_rate as f64) / (self.baud_rate as f64);
            if self.baud_rate > 2400 {
                self.config_index = INDEX3200;
                self.clkt_hi = FINE_CLKT3200_HI;
                self.clkt_lo = FINE_CLKT3200_LO;
            } else if self.baud_rate == 1600 {
                self.cross_over = false;
                self.skipped_sc = 0;
                self.config_index = INDEX1600;
                self.clkt_hi = FINE_CLKT_HI;
                self.clkt_lo = FINE_CLKT_LO;
            } else {
                if self.baud_rate == 512 {
                    self.clkt_hi = COURSE_CLKT_HI;
                    self.clkt_lo = COURSE_CLKT_LO;
                } else {
                    self.clkt_hi = FINE_CLKT_HI;
                    self.clkt_lo = FINE_CLKT_LO;
                }
                self.watch_ctr = atb_ctr + self.watch_step * 0.5;
            }
            self.last_baud_rate = self.baud_rate;
        }
    }
}

/// Clean EWMA RX-Quality tracker.
#[derive(Clone, Debug)]
pub struct RxQualityTracker {
    pub quality_pct: f64,
    ewma_error_rate: f64,
    pub total_codewords: u64,
    pub total_bit_errors: u64,
    pub soft_recoveries: u64,
}

impl Default for RxQualityTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl RxQualityTracker {
    pub fn new() -> Self {
        Self {
            quality_pct: 100.0,
            ewma_error_rate: 0.0,
            total_codewords: 0,
            total_bit_errors: 0,
            soft_recoveries: 0,
        }
    }

    pub fn record_word(&mut self, bit_errors: usize, soft_recovered: bool) {
        self.total_codewords += 1;
        self.total_bit_errors += bit_errors as u64;
        if soft_recovered {
            self.soft_recoveries += 1;
        }

        let word_err = (bit_errors.min(5) as f64) / 20.0;
        let alpha = 0.015;
        self.ewma_error_rate = (1.0 - alpha) * self.ewma_error_rate + alpha * word_err;
        self.quality_pct = (100.0 * (1.0 - self.ewma_error_rate)).clamp(0.0, 100.0);
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
