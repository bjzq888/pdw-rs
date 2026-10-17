//! Exact Port of `Flex.cpp` (`frame_flex`, `showblock`, `showframe`) for 2-FSK & 4-FSK FLEX
//! (1600 / 3200 / 6400 bps, Phases A, B, C, D) + ReFLEX.
//!
//! Directly matches `Flex.cpp` and `sound_in.cpp` timing:
//! - `flex_bc` counts down from `89` (`88` on Sync-1 bit, `71..=40` for FIW, `39` for FIW decode
//!   and 1600->3200 `cross_over` trigger, `0` for `flex_blk = 11`).
//! - Phase A and Phase C (`gin < 2`) use the exact Schmitt-trigger latched `atb_bit` from `AtbSlicer`
//!   (100% identical to `sound_in.cpp:609`).
//! - Phase B and Phase D (`gin == 0 || gin == 3`) calibrate their 4-FSK inner/outer threshold
//!   (`0.60 * header_outer_level`) directly from the 2-FSK header (Sync-1 + FIW) of the current frame.

use crate::bch::BchDecoder;
use crate::dsp::{AtbSlicer, RxQualityTracker, INDEX1600};
use crate::pocsag::DecodedPagerMessage;

const SYNC1: u16 = 0xA6C6;
const SYNC2: u16 = 0xAAAA;
const EOT1: u16 = 0xAAAA;
const EOT2: u16 = 0xFFFF;

/// FLEX Sync-1 outer 16-bit patterns `A1` (`Flex.cpp:89–90`).
const FLEX_SYNCS: [u16; 8] = [
    0x870C, // 0: FLEX 1600 bps (1600 sps, 2-FSK, Phase A)
    0x7B18, // 1: FLEX 3200 bps (3200 sps, 2-FSK, Phases A+C)
    0xB068, // 2: FLEX 3200 bps (1600 sps, 4-FSK, Phases A+B)
    0xDEA0, // 3: FLEX 6400 bps (3200 sps, 4-FSK, Phases A+B+C+D)
    0x0000,
    0x0000,
    0x0000,
    0x4C7C, // 7: ReFLEX 6400 bps (3200 sps, 4-FSK)
];

const VTYPE_NAMES: [&str; 8] = [
    "SECURE", "INSTR", "SH/TONE", "StNUM", "SfNUM", "ALPHA", "BINARY", "NuNUM",
];

const NUMERIC_TABLE: [char; 16] = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', 'U', ' ', '-', ']', '[',
];

#[derive(Clone, Debug)]
pub struct FlexPhaseBuffer {
    pub phase_char: char,
    pub block_bits: [u8; 256],
    pub block_conf: [f32; 256],
    pub frame_words: [u32; 88],
    pub raw_codewords: [[u8; 32]; 88],
    pub raw_conf: [[f32; 32]; 88],
    pub word_errors: [u8; 88],
    pub word_soft_recovered: [bool; 88],
}

impl FlexPhaseBuffer {
    pub fn new(phase_char: char) -> Self {
        Self {
            phase_char,
            block_bits: [0; 256],
            block_conf: [1.0; 256],
            frame_words: [0x1f_ffff; 88],
            raw_codewords: [[0; 32]; 88],
            raw_conf: [[1.0; 32]; 88],
            word_errors: [0; 88],
            word_soft_recovered: [false; 88],
        }
    }

    pub fn clear_frame(&mut self) {
        self.frame_words.fill(0x1f_ffff);
        self.word_errors.fill(0);
        self.word_soft_recovered.fill(false);
    }

    /// De-interleave a 256-bit block into 8 codewords of 32 bits (`Flex.cpp:723–747`)
    /// using hard-decision BCH(31,21) + even parity so BIW, Address, and Vector words
    /// never suffer false soft-decision corruptions.
    pub fn decode_block(&mut self, blknum: usize, bch: &BchDecoder, qual: &mut RxQualityTracker) -> u32 {
        let mut last_cc = 0u32;
        for i in 0..8 {
            let mut ob = [0u8; 32];
            let mut conf = [1.0f32; 32];
            for j in 0..32 {
                let k = (j * 8) + i;
                ob[j] = self.block_bits[k];
                conf[j] = self.block_conf[k];
            }

            let word_idx = (blknum << 3) + i;
            if word_idx < 88 {
                self.raw_codewords[word_idx] = ob;
                self.raw_conf[word_idx] = conf;
            }

            let res = bch.decode_hard(&ob);
            qual.record_word(res.errors, false);

            let mut cc: u32 = 0;
            for j in 0..21 {
                cc >>= 1;
                if res.bits[j] == 0 {
                    cc ^= 0x10_0000;
                }
            }
            if !res.valid {
                cc ^= 0x40_0000;
            }

            if word_idx < 88 {
                self.frame_words[word_idx] = cc;
                self.word_errors[word_idx] = res.errors as u8;
                self.word_soft_recovered[word_idx] = false;
            }
            last_cc = cc;
        }
        last_cc
    }
}

#[derive(Clone, Debug)]
pub struct FlexDecoder {
    pub sample_rate: u32,

    // Exact `Flex.cpp` state variables
    pub flex_blk: usize,
    pub flex_bc: usize,
    pub flex_timer_samples: usize,
    pub flex_active: bool,
    pub slr: [u16; 4],
    pub ob: [u8; 32],
    pub bct: usize,
    pub hbit: usize,
    pub g_sps: u32,
    pub g_sps2: u32,
    pub levels: u8,
    pub speed_bps: u32,
    pub reflex: bool,
    pub current_cycle: u8,
    pub current_frame: u8,

    // Per-frame 2-FSK header positive & negative outer levels used to calibrate 4-FSK Phase B/D slicing
    // and remove any residual per-burst carrier frequency offset (`header_dc`)
    pub header_outer_level: f32,
    header_level_sum: f32,
    header_level_count: usize,
    header_pos_sum: f32,
    header_pos_count: usize,
    header_neg_sum: f32,
    header_neg_count: usize,

    pub phase_a: FlexPhaseBuffer,
    pub phase_b: FlexPhaseBuffer,
    pub phase_c: FlexPhaseBuffer,
    pub phase_d: FlexPhaseBuffer,

    group_frame: [i16; 16],
    group_capcodes: [Vec<u64>; 16],
}

impl FlexDecoder {
    pub fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            flex_blk: 0,
            flex_bc: 0,
            flex_timer_samples: 0,
            flex_active: false,
            slr: [0; 4],
            ob: [0; 32],
            bct: 0,
            hbit: 0,
            g_sps: 1600,
            g_sps2: 1600,
            levels: 2,
            speed_bps: 1600,
            reflex: false,
            current_cycle: 0,
            current_frame: 0,
            header_outer_level: 80.0,
            header_level_sum: 0.0,
            header_level_count: 0,
            header_pos_sum: 0.0,
            header_pos_count: 0,
            header_neg_sum: 0.0,
            header_neg_count: 0,
            phase_a: FlexPhaseBuffer::new('A'),
            phase_b: FlexPhaseBuffer::new('B'),
            phase_c: FlexPhaseBuffer::new('C'),
            phase_d: FlexPhaseBuffer::new('D'),
            group_frame: [-1; 16],
            group_capcodes: std::array::from_fn(|_| Vec::new()),
        }
    }

    pub fn set_sample_rate(&mut self, sample_rate: u32) {
        self.sample_rate = sample_rate;
    }

    pub fn is_active(&self) -> bool {
        self.flex_blk > 0 || self.flex_bc > 0 || self.flex_timer_samples > 0
    }

    pub fn active_mode_label(&self) -> Option<String> {
        if !self.is_active() {
            return None;
        }
        let prefix = if self.reflex { "ReFLEX" } else { "FLEX" };
        let phases = match (self.g_sps, self.levels) {
            (1600, 2) => "2-FSK 1600 (Phase A)",
            (1600, 4) => "4-FSK 3200 (Phases AB)",
            (3200, 2) => "2-FSK 3200 (Phases AC)",
            _ => "4-FSK 6400 (Phases ABCD)",
        };
        Some(format!(
            "{} {} [Cy:{:02} Fr:{:03}]",
            prefix, phases, self.current_cycle, self.current_frame
        ))
    }

    pub fn reset(&mut self) {
        self.flex_blk = 0;
        self.flex_bc = 0;
        self.flex_timer_samples = 0;
        self.flex_active = false;
        self.reflex = false;
        self.bct = 0;
        self.hbit = 0;
        self.g_sps = 1600;
        self.g_sps2 = 1600;
    }

    /// Tick sample-rate timer (`sound_in.cpp:271–286`: `flex_timer` 2-second timeout).
    #[inline]
    pub fn tick_sample_timer(&mut self, atb: &mut AtbSlicer) {
        if self.flex_timer_samples > 0 {
            self.flex_timer_samples -= 1;
            if self.flex_timer_samples == 0 && atb.pocbit == 0 && self.flex_blk == 0 && self.flex_bc == 0 {
                atb.baud_rate = 1600;
                atb.config_index = INDEX1600;
                self.flex_active = false;
            }
        }
    }

    /// Slice a sample `val_128` (`-128..127`) into PDW's 4-FSK symbol `gin` (`0, 1, 2, 3`)
    /// while preserving `atb.atb_bit` as the exact MSB sign bit (`gin < 2` iff `atb_bit != 0`).
    #[inline]
    pub fn slice_symbol(&mut self, val_128: f32, atb_bit: u8) -> (u8, f32, f32) {
        let abs_v = val_128.abs();
        if self.flex_blk == 0 {
            // During Sync-1, FIW, and Sync-2, FLEX transmits 2-FSK at outer deviation (`±4.8 kHz`).
            // Track positive and negative outer levels ONLY once Sync-1 is locked (`self.flex_bc > 0`),
            // so dead-air noise spikes never corrupt `header_outer_level` and any residual DC offset
            // is measured independently of FIW bit-balance!
            if self.flex_bc > 0 && abs_v > 8.0 {
                self.header_outer_level = 0.90 * self.header_outer_level + 0.10 * abs_v;
                self.header_level_sum += abs_v;
                self.header_level_count += 1;
                if val_128 > 8.0 {
                    self.header_pos_sum += val_128;
                    self.header_pos_count += 1;
                } else if val_128 < -8.0 {
                    self.header_neg_sum += val_128;
                    self.header_neg_count += 1;
                }
            }
            let gin = if atb_bit != 0 { 0 } else { 3 };
            let conf = (abs_v / self.header_outer_level.max(10.0)).clamp(0.05, 2.0);
            (gin, conf, conf)
        } else if self.levels == 4 {
            let (header_dc, outer_ref) = if self.header_pos_count >= 8 && self.header_neg_count >= 8 {
                let pos_mean = self.header_pos_sum / (self.header_pos_count as f32);
                let neg_mean = self.header_neg_sum / (self.header_neg_count as f32);
                let dc = (0.5 * (pos_mean + neg_mean)).clamp(-25.0, 25.0);
                let o_ref = (0.5 * (pos_mean - neg_mean)).max(12.0);
                (dc, o_ref)
            } else if self.header_level_count >= 16 {
                (0.0, (self.header_level_sum / (self.header_level_count as f32)).max(12.0))
            } else {
                (0.0, self.header_outer_level.max(12.0))
            };
            let adj_abs = (val_128 - header_dc).abs();
            // At 3200 sps (FLEX-6400), post-discriminator filtering slightly rounds single-symbol outer peaks,
            // so 0.56 * outer_ref centers the threshold in the 3200-sps 4-FSK eye (0.60 * outer_ref at 1600 sps).
            let thresh_ratio = if self.g_sps == 3200 { 0.56 } else { 0.60 };
            let inner_thresh = thresh_ratio * outer_ref;
            let gin = if atb_bit != 0 {
                if adj_abs >= inner_thresh { 0 } else { 1 }
            } else if adj_abs >= inner_thresh {
                3
            } else {
                2
            };
            let conf_msb = (adj_abs / outer_ref).clamp(0.05, 2.0);
            let conf_lsb = ((adj_abs - inner_thresh).abs() / (0.40 * outer_ref)).clamp(0.05, 2.0);
            (gin, conf_msb, conf_lsb)
        } else {
            let outer_ref = self.header_outer_level.max(10.0);
            let gin = if atb_bit != 0 { 0 } else { 3 };
            let conf = (abs_v / outer_ref).clamp(0.05, 2.0);
            (gin, conf, conf)
        }
    }

    /// Exact port of `frame_flex(char gin)` (`Flex.cpp:821–1237`).
    #[allow(clippy::too_many_arguments)]
    pub fn frame_flex(
        &mut self,
        gin: u8,
        conf_msb: f32,
        conf_lsb: f32,
        atb: &mut AtbSlicer,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
        decode_1600: bool,
        decode_3200: bool,
        decode_6400: bool,
        show_tone: bool,
        show_numeric: bool,
        convert_si: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        // Update 64-bit sync register (`Flex.cpp:844–851`)
        for i in 0..3 {
            self.slr[i] = (self.slr[i] << 1) | ((self.slr[i + 1] >> 15) & 1);
        }
        self.slr[3] = (self.slr[3] << 1) | if gin < 2 { 1 } else { 0 };

        if self.flex_blk == 0 {
            if self.flex_timer_samples > 0
                && (self.slr[2] ^ EOT1).count_ones() + (self.slr[3] ^ EOT2).count_ones() == 0
            {
                self.flex_timer_samples = 0;
                self.flex_active = false;
                return;
            }

            let mut nh =
                (self.slr[1] ^ SYNC1).count_ones() + (self.slr[2] ^ SYNC2).count_ones();

            // Inverted Sync-1 detection (`Flex.cpp:875–885`), guarded so we ONLY invert receive
            // polarity if `(!slr[0], !slr[3])` also matches a valid `FLEX_SYNCS` entry!
            if nh >= 31 && ((self.slr[0] ^ self.slr[3]) ^ 0xFFFF).count_ones() < 2 {
                let inv0 = !self.slr[0];
                let inv3 = !self.slr[3];
                let valid_inv_sync = FLEX_SYNCS.iter().any(|&s| {
                    s != 0 && ((inv0 ^ s).count_ones() + (inv3 ^ (!s)).count_ones()) < 2
                });
                if valid_inv_sync {
                    self.slr[0] = inv0;
                    self.slr[1] = !self.slr[1];
                    self.slr[2] = !self.slr[2];
                    self.slr[3] = inv3;
                    atb.invert_data();
                    nh = 32 - nh;
                }
            }

            if nh < 2 && ((self.slr[0] ^ self.slr[3]) ^ 0xFFFF).count_ones() < 2 {
                let mut matched_speed: Option<usize> = None;
                for (speed, &sync_word) in FLEX_SYNCS.iter().enumerate() {
                    if sync_word == 0 {
                        continue;
                    }
                    if (self.slr[0] ^ sync_word).count_ones()
                        + (self.slr[3] ^ (!sync_word)).count_ones()
                        < 2
                    {
                        matched_speed = Some(speed);
                        break;
                    }
                }

                if let Some(speed) = matched_speed {
                    let speed_bps = match speed & 0x03 {
                        0 => {
                            if !decode_1600 {
                                return;
                            }
                            1600
                        }
                        3 => {
                            if !decode_6400 {
                                return;
                            }
                            6400
                        }
                        _ => {
                            if !decode_3200 {
                                return;
                            }
                            3200
                        }
                    };

                    self.flex_bc = 89;
                    self.flex_active = true;
                    self.flex_timer_samples = (self.sample_rate as usize) * 2;
                    self.g_sps = if (speed & 0x01) != 0 { 3200 } else { 1600 };
                    self.g_sps2 = self.g_sps;
                    self.levels = if (speed & 0x02) != 0 { 4 } else { 2 };
                    self.reflex = (speed & 0x04) != 0;
                    self.speed_bps = speed_bps;
                    self.header_level_sum = 0.0;
                    self.header_level_count = 0;
                    self.header_pos_sum = 0.0;
                    self.header_pos_count = 0;
                    self.header_neg_sum = 0.0;
                    self.header_neg_count = 0;
                }
            }

            // Note:Just like `Flex.cpp:1049`, this `if self.flex_bc > 0` runs on the SAME bit
            // where `self.flex_bc = 89` was set, decrementing `flex_bc` to `88` on the Sync-1 bit!
            if self.flex_bc > 0 {
                self.flex_bc -= 1;

                if self.flex_bc < 72 && self.flex_bc > 39 {
                    self.ob[71 - self.flex_bc] = if gin < 2 { 1 } else { 0 };
                } else if self.flex_bc == 39 {
                    if self.g_sps2 == 3200 {
                        atb.cross_over = true;
                        atb.skipped_sc = 0;
                        atb.baud_rate = 3200;
                    }

                    let res = bch.decode_hard(&self.ob);
                    qual.record_word(res.errors, false);
                    if res.errors < 2 {
                        let mut hd: u8 = 0;
                        for ihd in 4..8 {
                            hd >>= 1;
                            if res.bits[ihd] == 1 {
                                hd ^= 0x08;
                            }
                        }
                        let cy = (hd & 0x0f) ^ 0x0f;

                        for ihd in 8..=14 {
                            hd >>= 1;
                            if res.bits[ihd] == 1 {
                                hd ^= 0x40;
                            }
                        }
                        let fr = (hd & 0x7f) ^ 0x7f;

                        if cy < 15 {
                            self.current_cycle = cy;
                            self.current_frame = fr;
                        }
                    }
                }

                if self.flex_bc == 0 {
                    self.flex_blk = 11;
                    self.bct = 0;
                    self.hbit = 0;
                    if self.g_sps == 1600 {
                        atb.baud_rate = 1600;
                    }
                    self.phase_a.clear_frame();
                    self.phase_b.clear_frame();
                    self.phase_c.clear_frame();
                    self.phase_d.clear_frame();
                }
            }
        } else {
            // Collect 11 interleaved data blocks (`Flex.cpp:1140–1236`)
            if self.g_sps == 1600 {
                self.phase_a.block_bits[self.bct] = if gin < 2 { 1 } else { 0 };
                self.phase_a.block_conf[self.bct] = conf_msb;

                if self.levels == 4 {
                    self.phase_b.block_bits[self.bct] =
                        if gin == 0 || gin == 3 { 1 } else { 0 };
                    self.phase_b.block_conf[self.bct] = conf_lsb;
                }
                self.bct += 1;
            } else if self.hbit == 0 {
                self.phase_a.block_bits[self.bct] = if gin < 2 { 1 } else { 0 };
                self.phase_a.block_conf[self.bct] = conf_msb;

                if self.levels == 4 {
                    self.phase_b.block_bits[self.bct] =
                        if gin == 0 || gin == 3 { 1 } else { 0 };
                    self.phase_b.block_conf[self.bct] = conf_lsb;
                }
                self.hbit = 1;
            } else {
                self.phase_c.block_bits[self.bct] = if gin < 2 { 1 } else { 0 };
                self.phase_c.block_conf[self.bct] = conf_msb;

                if self.levels == 4 {
                    self.phase_d.block_bits[self.bct] =
                        if gin == 0 || gin == 3 { 1 } else { 0 };
                    self.phase_d.block_conf[self.bct] = conf_lsb;
                }
                self.hbit = 0;
                self.bct += 1;
            }

            if self.bct == 256 {
                self.bct = 0;
                let blknum = 11 - self.flex_blk;

                self.phase_a.decode_block(blknum, bch, qual);
                if self.levels == 4 {
                    self.phase_b.decode_block(blknum, bch, qual);
                }
                if self.g_sps == 3200 {
                    self.phase_c.decode_block(blknum, bch, qual);
                    if self.levels == 4 {
                        self.phase_d.decode_block(blknum, bch, qual);
                    }
                }

                if blknum == 10 && !self.reflex {
                    Self::parse_phase_frame(
                        &self.phase_a,
                        &mut self.group_frame,
                        &mut self.group_capcodes,
                        self.current_frame,
                        self.speed_bps,
                        bch,
                        qual,
                        show_tone,
                        show_numeric,
                        convert_si,
                        out,
                    );

                    if self.levels == 4 {
                        Self::parse_phase_frame(
                            &self.phase_b,
                            &mut self.group_frame,
                            &mut self.group_capcodes,
                            self.current_frame,
                            self.speed_bps,
                            bch,
                            qual,
                            show_tone,
                            show_numeric,
                            convert_si,
                            out,
                        );
                    }
                    if self.g_sps == 3200 {
                        Self::parse_phase_frame(
                            &self.phase_c,
                            &mut self.group_frame,
                            &mut self.group_capcodes,
                            self.current_frame,
                            self.speed_bps,
                            bch,
                            qual,
                            show_tone,
                            show_numeric,
                            convert_si,
                            out,
                        );
                        if self.levels == 4 {
                            Self::parse_phase_frame(
                                &self.phase_d,
                                &mut self.group_frame,
                                &mut self.group_capcodes,
                                self.current_frame,
                                self.speed_bps,
                                bch,
                                qual,
                                show_tone,
                                show_numeric,
                                convert_si,
                                out,
                            );
                        }
                    }
                }

                self.flex_blk -= 1;

                if self.flex_blk == 0 {
                    atb.baud_rate = 1600;
                    self.flex_active = false;
                }
            }
        }
    }

    /// Try 5-bit Chase-2 soft-decision recovery on a FLEX BIW (`frame[0]`) or Vector word (`frame[vb]`),
    /// enforcing the 4-bit hardware checksum (`Self::xsumchk`) inside the Chase-2 search loop.
    fn try_soft_xsum_word(
        phase: &FlexPhaseBuffer,
        k: usize,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
    ) -> Option<u32> {
        if k >= 88 {
            return None;
        }
        let soft = bch.decode_soft_filtered(
            &phase.raw_codewords[k],
            &phase.raw_conf[k],
            |data_bits| {
                let cc = Self::bits_to_flex_word(data_bits);
                Self::xsumchk(cc)
            },
        );
        if !soft.valid {
            return None;
        }
        let data_arr: [u8; 21] = soft.bits[0..21].try_into().ok()?;
        let cc = Self::bits_to_flex_word(&data_arr);
        if Self::xsumchk(cc) {
            if soft.soft_recovered {
                qual.soft_recoveries += 1;
            }
            Some(cc)
        } else {
            None
        }
    }

    pub fn xsumchk(l: u32) -> bool {
        if l > 0x3f_ffff {
            return false;
        }
        let xs = (l & 0x0f)
            + ((l >> 4) & 0x0f)
            + ((l >> 8) & 0x0f)
            + ((l >> 12) & 0x0f)
            + ((l >> 16) & 0x0f)
            + ((l >> 20) & 0x01);
        (xs & 0x0f) == 0x0f
    }

    #[inline]
    fn bits_to_flex_word(data_bits: &[u8; 21]) -> u32 {
        let mut cc: u32 = 0;
        for j in 0..21 {
            cc >>= 1;
            if data_bits[j] == 0 {
                cc ^= 0x10_0000;
            }
        }
        cc
    }

    #[inline]
    fn is_valid_flex_alpha_char(c: u8) -> bool {
        c == 0x03 || c == b'\n' || c == b'\r' || (32..=126).contains(&c)
    }

    #[inline]
    fn is_clean_alpha_word(cc: u32, skip_c0: bool) -> bool {
        if cc > 0x3f_ffff {
            return false;
        }
        let c0 = (cc & 0x7f) as u8;
        let c1 = ((cc >> 7) & 0x7f) as u8;
        let c2 = ((cc >> 14) & 0x7f) as u8;
        (skip_c0 || Self::is_valid_flex_alpha_char(c0))
            && Self::is_valid_flex_alpha_char(c1)
            && Self::is_valid_flex_alpha_char(c2)
    }

    /// Try 5-bit Chase-2 soft-decision recovery on a corrupted FLEX header/control word (`k < 88`).
    fn try_soft_raw_word(
        phase: &FlexPhaseBuffer,
        k: usize,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
    ) -> Option<u32> {
        if k >= 88 {
            return None;
        }
        let soft = bch.decode_soft(&phase.raw_codewords[k], &phase.raw_conf[k]);
        if !soft.valid {
            return None;
        }
        if soft.soft_recovered {
            qual.soft_recoveries += 1;
        }
        Some(Self::bits_to_flex_word(&soft.bits[0..21].try_into().ok()?))
    }

    /// Try guarded 5-bit Chase-2 soft-decision recovery on an Alpha message word (`k >= w1`).
    /// Filters candidate codewords inside the Chase-2 search loop so all extracted 7-bit character
    /// slots (`c1`, `c2`, and `c0` when `!skip_c0`) are guaranteed printable ASCII (`32..=126` or `0x03`).
    fn try_soft_alpha_word(
        phase: &FlexPhaseBuffer,
        k: usize,
        skip_c0: bool,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
    ) -> Option<u32> {
        if k >= 88 {
            return None;
        }
        let soft = bch.decode_soft_filtered(
            &phase.raw_codewords[k],
            &phase.raw_conf[k],
            |data_bits| {
                let cc = Self::bits_to_flex_word(data_bits);
                Self::is_clean_alpha_word(cc, skip_c0)
            },
        );
        if !soft.valid {
            return None;
        }
        let data_arr: [u8; 21] = soft.bits[0..21].try_into().ok()?;
        let cc = Self::bits_to_flex_word(&data_arr);
        if Self::is_clean_alpha_word(cc, skip_c0) {
            if soft.soft_recovered {
                qual.soft_recoveries += 1;
            }
            Some(cc)
        } else {
            None
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn parse_phase_frame_for_test(
        phase_buf: &FlexPhaseBuffer,
        group_frame: &mut [i16; 16],
        group_capcodes: &mut [Vec<u64>; 16],
        current_frame: u8,
        speed_bps: u32,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
        show_tone: bool,
        show_numeric: bool,
        convert_si: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        Self::parse_phase_frame(
            phase_buf,
            group_frame,
            group_capcodes,
            current_frame,
            speed_bps,
            bch,
            qual,
            show_tone,
            show_numeric,
            convert_si,
            out,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn parse_phase_frame(
        phase_buf: &FlexPhaseBuffer,
        group_frame: &mut [i16; 16],
        group_capcodes: &mut [Vec<u64>; 16],
        current_frame: u8,
        speed_bps: u32,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
        show_tone: bool,
        show_numeric: bool,
        convert_si: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        let mut frame = phase_buf.frame_words;
        if !Self::xsumchk(frame[0]) {
            if let Some(rec_biw) = Self::try_soft_xsum_word(phase_buf, 0, bch, qual) {
                frame[0] = rec_biw;
            } else {
                qual.record_word(1, false);
                return;
            }
        }
        qual.record_word(0, false);

        let vsa = ((frame[0] >> 10) & 0x3f) as usize;
        let asa = (((frame[0] >> 8) & 0x03) + 1) as usize;
        if vsa <= asa || vsa >= 88 {
            return;
        }

        // Pre-pass: Soft-recover any corrupted Address words (`asa..vsa`) or Vector words (`vsa..2*vsa-asa`)
        for idx in asa..vsa {
            if frame[idx] > 0x3f_ffff {
                if let Some(rec_addr) = Self::try_soft_raw_word(phase_buf, idx, bch, qual) {
                    frame[idx] = rec_addr;
                }
            }
            let vb = vsa + idx - asa;
            if vb < 88 && !Self::xsumchk(frame[vb]) {
                if let Some(rec_vec) = Self::try_soft_xsum_word(phase_buf, vb, bch, qual) {
                    frame[vb] = rec_vec;
                }
            }
        }

        let now = chrono::Local::now();
        let time_str = now.format("%H:%M:%S").to_string();
        let date_str = now.format("%d-%m-%y").to_string();
        let mode_str = format!("FLEX-{}", phase_buf.phase_char);

        // Pass 1: Register all `MODE_SHORT_INSTRUCTION` (`vt == 1`) groupcall assignments first,
        // so group messages on Priority Addresses (`2029568..2029583`) at `j = asa` in the same frame
        // always have their member capcodes ready.
        let mut j = asa;
        while j < vsa {
            let cc2 = frame[j] & 0x1f_ffff;
            let is_long =
                cc2 < 0x00_8001 || (cc2 > 0x1e_0000 && cc2 < 0x1f_0001) || cc2 > 0x1f_7ffe;
            let vb = vsa + j - asa;
            if vb >= 88 || !Self::xsumchk(frame[vb]) {
                j += 1;
                continue;
            }
            if is_long && (j + 1 >= vsa || vb + 1 >= 88) {
                j += 1;
                continue;
            }
            let vt = ((frame[vb] >> 4) & 0x07) as usize;
            if vt == 1 {
                let (capcode_num, _, _) =
                    Self::decode_capcode(frame[j], if is_long { frame[j + 1] } else { 0 }, is_long);
                if !(2_029_568..=2_029_583).contains(&capcode_num) {
                    let assigned_frame = ((frame[vb] >> 10) & 0x7f) as i16;
                    let temp_addr = ((frame[vb] >> 17) & 0x7f) as usize;
                    if temp_addr < 16 {
                        if group_frame[temp_addr] != assigned_frame {
                            group_capcodes[temp_addr].clear();
                        }
                        group_frame[temp_addr] = assigned_frame;
                        let cap_u64 = capcode_num as u64;
                        if !group_capcodes[temp_addr].contains(&cap_u64) {
                            group_capcodes[temp_addr].push(cap_u64);
                        }
                    }
                }
            }
            j += if is_long { 2 } else { 1 };
        }

        // Pass 2: Decode and emit all messages (`Flex.cpp:411–710`)
        let mut j = asa;
        while j < vsa {
            let cc2 = frame[j] & 0x1f_ffff;
            let is_long =
                cc2 < 0x00_8001 || (cc2 > 0x1e_0000 && cc2 < 0x1f_0001) || cc2 > 0x1f_7ffe;

            let vb = vsa + j - asa;
            // Match `Flex.cpp:423–425`: if `xsumchk(frame[vb])` fails, increment `j` by 1 (`j++`), NOT 2!
            if vb >= 88 || !Self::xsumchk(frame[vb]) {
                qual.record_word(1, false);
                j += 1;
                continue;
            }
            if is_long && (j + 1 >= vsa || vb + 1 >= 88) {
                j += 1;
                continue;
            }
            qual.record_word(0, false);

            let vt = ((frame[vb] >> 4) & 0x07) as usize;
            let (capcode_num, capcode_str, addr_err) =
                Self::decode_capcode(frame[j], if is_long { frame[j + 1] } else { 0 }, is_long);

            match vt {
                5 | 0 => {
                    // MODE_ALPHA (5) or MODE_SECURE (0) (`Flex.cpp:435–506`)
                    let mut w1 = ((frame[vb] >> 7) & 0x7f) as usize;
                    let len = ((frame[vb] >> 14) & 0x7f) as usize;
                    if w1 >= 88 || len == 0 {
                        j += if is_long { 2 } else { 1 };
                        continue;
                    }
                    let mut w2 = (w1 + len - 1).min(87);

                    let mut soft_cnt = 0usize;
                    let fragment_num = if !is_long {
                        let mut hdr_w = frame[w1];
                        if hdr_w > 0x3f_ffff {
                            if let Some(rec_hdr) = Self::try_soft_raw_word(phase_buf, w1, bch, qual) {
                                hdr_w = rec_hdr;
                                soft_cnt += 1;
                            }
                        }
                        let f = if hdr_w <= 0x3f_ffff {
                            ((hdr_w >> 11) & 0x03) as u8
                        } else {
                            3
                        };
                        w1 += 1;
                        f
                    } else {
                        let mut hdr_w = frame[vb + 1];
                        if hdr_w > 0x3f_ffff {
                            if let Some(rec_hdr) = Self::try_soft_raw_word(phase_buf, vb + 1, bch, qual) {
                                hdr_w = rec_hdr;
                                soft_cnt += 1;
                            }
                        }
                        // Note: `w2` is already initialized as `(w1 + len - 1).min(87)` (inclusive).
                        // For 9-digit long addresses (`is_long`), the header word lives in `frame[vb + 1]`
                        // rather than `frame[w1]`, so all `len` words `w1..=w2` are character words!
                        let f = if hdr_w <= 0x3f_ffff {
                            ((hdr_w >> 11) & 0x03) as u8
                        } else {
                            3
                        };
                        f
                    };

                    let mut msg_text = String::new();
                    let mut has_err = addr_err;

                    for k in w1..=w2 {
                        if k >= 88 {
                            break;
                        }
                        let skip_c0 = k == w1 && fragment_num == 3;
                        let mut word_val = frame[k];
                        let suspicious_2err = word_val <= 0x3f_ffff
                            && phase_buf.word_errors[k] >= 2
                            && !Self::is_clean_alpha_word(word_val, skip_c0);

                        if word_val > 0x3f_ffff || suspicious_2err {
                            if let Some(recovered) =
                                Self::try_soft_alpha_word(phase_buf, k, skip_c0, bch, qual)
                            {
                                if word_val > 0x3f_ffff || recovered != word_val {
                                    soft_cnt += 1;
                                }
                                word_val = recovered;
                            } else if word_val > 0x3f_ffff {
                                has_err = true;
                            }
                        }

                        // If the codeword is still uncorrectable after 5-bit Chase-2 soft BCH,
                        // emit clean '?' placeholders instead of unpacking 21 random corrupted bits.
                        if word_val > 0x3f_ffff {
                            if !skip_c0 {
                                msg_text.push('?');
                            }
                            msg_text.push('?');
                            msg_text.push('?');
                            continue;
                        }

                        // Valid codeword: unpack 7-bit ASCII characters and stop immediately on ETX (0x03)
                        let push_flex_char = |s: &mut String, c: u8| -> bool {
                            if c == 0x03 {
                                return true; // ETX (End of Text)
                            }
                            if c == 0 {
                                // NULL padding
                            } else if c == b'\n' || c == b'\r' {
                                s.push(' ');
                            } else if (32..=126).contains(&c) {
                                s.push(c as char);
                            } else if c != 23 && c != 4 {
                                s.push('?');
                            }
                            false
                        };

                        if !skip_c0 {
                            let c0 = (word_val & 0x7f) as u8;
                            if push_flex_char(&mut msg_text, c0) {
                                break;
                            }
                        }
                        let c1 = ((word_val >> 7) & 0x7f) as u8;
                        if push_flex_char(&mut msg_text, c1) {
                            break;
                        }

                        let c2 = ((word_val >> 14) & 0x7f) as u8;
                        if push_flex_char(&mut msg_text, c2) {
                            break;
                        }
                    }

                    if convert_si && (2_029_568..=2_029_583).contains(&capcode_num) {
                        let gbit = (capcode_num - 2_029_568) as usize;
                        if group_frame[gbit] == current_frame as i16 {
                            let mut members = group_capcodes[gbit].clone();
                            members.sort_unstable();
                            for member in members {
                                out.push(DecodedPagerMessage {
                                    capcode: format!("{:07}", member),
                                    time: time_str.clone(),
                                    date: date_str.clone(),
                                    mode: mode_str.clone(),
                                    msg_type: "GROUP".to_string(),
                                    bitrate: speed_bps,
                                    message: msg_text.clone(),
                                    has_bit_errors: has_err,
                                    soft_recovered_words: soft_cnt,
                                    filter_label: None,
                                    filter_color_idx: None,
                                });
                            }
                            group_capcodes[gbit].clear();
                            group_frame[gbit] = -1;
                        }
                    }

                    out.push(DecodedPagerMessage {
                        capcode: capcode_str,
                        time: time_str.clone(),
                        date: date_str.clone(),
                        mode: mode_str.clone(),
                        msg_type: VTYPE_NAMES[vt].to_string(),
                        bitrate: speed_bps,
                        message: msg_text,
                        has_bit_errors: has_err,
                        soft_recovered_words: soft_cnt,
                        filter_label: None,
                        filter_color_idx: None,
                    });
                }
                1 => {
                    // MODE_SHORT_INSTRUCTION (`Flex.cpp:508–540`)
                    if !(2_029_568..=2_029_583).contains(&capcode_num) && !convert_si {
                        let assigned_frame = ((frame[vb] >> 10) & 0x7f) as i16;
                        let temp_addr = ((frame[vb] >> 17) & 0x7f) as usize;
                        out.push(DecodedPagerMessage {
                            capcode: capcode_str,
                            time: time_str.clone(),
                            date: date_str.clone(),
                            mode: mode_str.clone(),
                            msg_type: "INSTR".to_string(),
                            bitrate: speed_bps,
                            message: format!(
                                "TEMPORARY ADDRESS: {:07} -> FRAME {:03}",
                                2_029_568 + temp_addr,
                                assigned_frame
                            ),
                            has_bit_errors: addr_err,
                            soft_recovered_words: 0,
                            filter_label: None,
                            filter_color_idx: None,
                        });
                    }
                }
                3 | 4 | 7 => {
                    // MODE_STNUM, MODE_SFNUM, MODE_NUNUM (`Flex.cpp:542–598`, with off-by-one fixed)
                    if show_numeric {
                        let w1 = ((frame[vb] >> 7) & 0x7f) as usize;
                        let w2 = (w1 + (((frame[vb] >> 14) & 0x07) as usize)).min(87);
                        let mut num_words: Vec<usize> = Vec::with_capacity(10);
                        if is_long {
                            num_words.push(vb + 1);
                        }
                        for k in w1..=w2 {
                            if k < 88 {
                                num_words.push(k);
                            }
                        }

                        let mut m: usize = if vt == 7 { 14 } else { 6 };
                        let mut c_nibble: usize = 0;
                        let mut msg_text = String::new();
                        let mut has_err = addr_err;
                        let mut soft_cnt = 0usize;

                        for &w_idx in &num_words {
                            let mut cc = frame[w_idx];
                            if cc > 0x3f_ffff {
                                if let Some(rec) = Self::try_soft_raw_word(phase_buf, w_idx, bch, qual) {
                                    cc = rec;
                                    soft_cnt += 1;
                                } else {
                                    has_err = true;
                                }
                            }
                            for _ in 0..21 {
                                c_nibble >>= 1;
                                if (cc & 0x01) != 0 {
                                    c_nibble ^= 0x08;
                                }
                                cc >>= 1;
                                m -= 1;
                                if m == 0 {
                                    msg_text.push(NUMERIC_TABLE[c_nibble & 0x0f]);
                                    m = 4;
                                }
                            }
                        }

                        out.push(DecodedPagerMessage {
                            capcode: capcode_str,
                            time: time_str.clone(),
                            date: date_str.clone(),
                            mode: mode_str.clone(),
                            msg_type: VTYPE_NAMES[vt].to_string(),
                            bitrate: speed_bps,
                            message: msg_text.trim_end().to_string(),
                            has_bit_errors: has_err,
                            soft_recovered_words: soft_cnt,
                            filter_label: None,
                            filter_color_idx: None,
                        });
                    }
                }
                2 => {
                    // MODE_SH_TONE (`Flex.cpp:600–642`)
                    let tt = (frame[vb] >> 7) & 0x03;
                    if (show_tone && tt != 0) || (show_numeric && tt == 0) {
                        let msg_text = if tt != 0 {
                            "TONE-ONLY".to_string()
                        } else {
                            let mut s = String::new();
                            for shift in (9..=17).step_by(4) {
                                let idx = ((frame[vb] >> shift) & 0x0f) as usize;
                                s.push(NUMERIC_TABLE[idx]);
                            }
                            if is_long {
                                for shift in (0..=16).step_by(4) {
                                    let idx = ((frame[vb + 1] >> shift) & 0x0f) as usize;
                                    s.push(NUMERIC_TABLE[idx]);
                                }
                            }
                            s
                        };
                        out.push(DecodedPagerMessage {
                            capcode: capcode_str,
                            time: time_str.clone(),
                            date: date_str.clone(),
                            mode: mode_str.clone(),
                            msg_type: "SH/TONE".to_string(),
                            bitrate: speed_bps,
                            message: msg_text,
                            has_bit_errors: addr_err,
                            soft_recovered_words: 0,
                            filter_label: None,
                            filter_color_idx: None,
                        });
                    }
                }
                6 => {
                    // MODE_BINARY (`Flex.cpp:644–695`)
                    let mut w1 = ((frame[vb] >> 7) & 0x7f) as usize;
                    let len = ((frame[vb] >> 14) & 0x7f) as usize;
                    if w1 < 88 && len > 0 {
                        let w2 = (w1 + len - 1).min(87);
                        if !is_long {
                            let frag = (frame[w1] >> 13) & 0x03;
                            w1 += if frag == 3 { 2 } else { 1 };
                        } else {
                            let frag = (frame[vb + 1] >> 13) & 0x03;
                            if frag == 3 {
                                w1 += 1;
                            }
                        }
                        let mut hex_str = String::new();
                        let mut n = 0usize;
                        let mut m_nib = 0u8;
                        for k in w1..=w2 {
                            if k >= 88 {
                                break;
                            }
                            let mut cc3 = frame[k];
                            for _ in 0..21 {
                                m_nib >>= 1;
                                if (cc3 & 1) != 0 {
                                    m_nib ^= 0x08;
                                }
                                cc3 >>= 1;
                                n += 1;
                                if n == 4 {
                                    let ch = if m_nib < 10 {
                                        (b'0' + m_nib) as char
                                    } else {
                                        (b'A' + (m_nib - 10)) as char
                                    };
                                    hex_str.push(ch);
                                    n = 0;
                                    m_nib = 0;
                                }
                            }
                        }
                        out.push(DecodedPagerMessage {
                            capcode: capcode_str,
                            time: time_str.clone(),
                            date: date_str.clone(),
                            mode: mode_str.clone(),
                            msg_type: "BINARY".to_string(),
                            bitrate: speed_bps,
                            message: hex_str,
                            has_bit_errors: addr_err,
                            soft_recovered_words: 0,
                            filter_label: None,
                            filter_color_idx: None,
                        });
                    }
                }
                _ => {}
            }

            j += if is_long { 2 } else { 1 };
        }
    }

    fn decode_capcode(l1: u32, l2: u32, is_long: bool) -> (i64, String, bool) {
        let capcode: i64 = if !is_long {
            ((l1 & 0x1f_ffff) as i64) - 32_768
        } else {
            let high = (((l2 & 0x1f_ffff) ^ 0x1f_ffff) as i64) << 15;
            high + 2_068_480 + ((l1 & 0x1f_ffff) as i64)
        };

        let bad = l1 > 0x3f_ffff || (is_long && l2 > 0x3f_ffff) || capcode < 0;
        let text = if bad {
            if is_long {
                "?????????".to_string()
            } else {
                "???????".to_string()
            }
        } else if is_long {
            format!("{:09}", capcode)
        } else {
            format!("{:07}", capcode)
        };

        (capcode.max(0), text, bad)
    }
}
