//! Exact Port of `Pocsag.cpp` (`POCSAG::frame`, `process_word`, `GetMessageType`, `show_message`)
//! with 5-Bit Chase-2 Soft-Decision BCH Recovery.
//!
//! Fixes over original C++ PDW:
//! 1. `pocsag.frame('X')` bug (`sound_in.cpp:600`): `frame_reset()` properly flushes any pending
//!    message at end-of-carrier and resets `synced = false`, `shift_reg = 0`.
//! 2. `GetMessageType()` `(20 * wordc) % 6` bug: prevents 2-word and 5-word Alphanumeric messages
//!    from being misclassified as Numeric.
//! 3. Strict 50-transition preamble gating (`pocbit` in `sound_in.cpp:454–515`) so noise and FLEX
//!    signals never trigger false POCSAG syncs or random polarity flips.
//! 4. 5-bit Chase-2 soft-decision BCH recovery on marginal POCSAG message codewords, and clean `'?'`
//!    substitution when a codeword remains uncorrectable instead of unpacking 20 corrupted bits.

use crate::bch::BchDecoder;
use crate::dsp::{AtbSlicer, RxQualityTracker};

pub const POCSAG_SYNC: u32 = 0x7CD2_15D8;
pub const POCSAG_IDLE: u32 = 0x7A89_C197;
const NUMERIC_TABLE: [char; 16] = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', 'U', ' ', '-', ']', '[',
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PocsagMsgType {
    ToneOnly,
    Numeric,
    Alpha,
    Both,
}

#[derive(Clone, Debug)]
pub struct DecodedPagerMessage {
    pub capcode: String,
    pub time: String,
    pub date: String,
    pub mode: String,
    pub msg_type: String,
    pub bitrate: u32,
    pub message: String,
    pub has_bit_errors: bool,
    pub soft_recovered_words: usize,
    pub filter_label: Option<String>,
    pub filter_color_idx: Option<usize>,
}

#[derive(Clone, Debug)]
pub struct PocsagChannelDecoder {
    pub sample_rate: u32,
    pub baud_rate: u32,

    shift_reg: u32,
    synced: bool,
    word_number: usize,
    bit_count: usize,
    word_bits: [u8; 32],
    word_conf: [f32; 32],

    // Active message accumulation state (persists across multi-batch sync words!)
    has_address: bool,
    pocaddr: u32,
    addr_corrupted: bool,
    function_num: u8,
    wordc: usize,
    alpha_chars: Vec<u8>,
    alpha_err: Vec<bool>,
    num_chars: Vec<char>,
    num_err: Vec<bool>,
    sr_char: u32,
    srca: usize,
    srcn: usize,
    rest_alpha_bits: Vec<u8>,
    soft_words_in_msg: usize,
}

impl PocsagChannelDecoder {
    pub fn new(sample_rate: u32, baud_rate: u32) -> Self {
        Self {
            sample_rate,
            baud_rate,
            shift_reg: 0,
            synced: false,
            word_number: 0,
            bit_count: 0,
            word_bits: [0; 32],
            word_conf: [1.0; 32],
            has_address: false,
            pocaddr: 0,
            addr_corrupted: false,
            function_num: 1,
            wordc: 0,
            alpha_chars: Vec::new(),
            alpha_err: Vec::new(),
            num_chars: Vec::new(),
            num_err: Vec::new(),
            sr_char: 0,
            srca: 0,
            srcn: 0,
            rest_alpha_bits: Vec::new(),
            soft_words_in_msg: 0,
        }
    }

    pub fn set_sample_rate(&mut self, sample_rate: u32) {
        self.sample_rate = sample_rate;
    }

    pub fn is_active(&self) -> bool {
        self.synced
    }

    /// Reset called when `pocbit == 0` (`pocsag.frame(-1)` in `Pocsag.cpp:94–105`).
    /// Flushes any pending message before resetting sync and message state.
    pub fn frame_reset(
        &mut self,
        use_fnu: bool,
        show_both: bool,
        show_tone: bool,
        show_numeric: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        self.synced = false;
        if self.has_address {
            self.flush_pending(use_fnu, show_both, show_tone, show_numeric, out);
        }
        self.reset_message_state();
        self.shift_reg = 0;
        self.bit_count = 0;
        self.word_number = 0;
    }

    /// Port of `POCSAG::frame(int bit)` (`Pocsag.cpp:76–164`) with analog bit confidence `conf`
    /// for 5-bit Chase-2 soft-decision BCH recovery.
    #[allow(clippy::too_many_arguments)]
    pub fn frame_pdw(
        &mut self,
        bit: u8,
        conf: f32,
        baud_rate: u32,
        atb: &mut AtbSlicer,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
        use_fnu: bool,
        show_both: bool,
        show_tone: bool,
        show_numeric: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        self.baud_rate = baud_rate;
        self.shift_reg = (self.shift_reg << 1) | ((bit & 1) as u32);

        if !self.synced {
            let nh = (self.shift_reg ^ POCSAG_SYNC).count_ones();
            if nh < 5 {
                self.synced = true;
                self.word_number = 0;
                self.bit_count = 0;
            } else if nh >= 31 {
                // 31 or 32 mismatched bits inside an active 50-crossing POCSAG preamble -> inverted polarity
                atb.invert_data();
                self.shift_reg = !self.shift_reg;
                self.synced = true;
                self.word_number = 0;
                self.bit_count = 0;
            }
            return;
        }

        // Collect 32-bit codeword and per-bit analog confidence inside a 16-word POCSAG batch
        self.word_bits[self.bit_count] = bit & 1;
        self.word_conf[self.bit_count] = conf;
        self.bit_count += 1;

        if self.bit_count == 32 {
            self.bit_count = 0;
            let nh = (self.shift_reg ^ POCSAG_IDLE).count_ones();
            if nh < 5 {
                atb.pocbit = 170;
                qual.record_word(nh as usize, false);
                if self.has_address {
                    self.flush_pending(use_fnu, show_both, show_tone, show_numeric, out);
                }
                self.reset_message_state();
            } else {
                let valid_cw = self.process_codeword(
                    self.word_number,
                    bch,
                    qual,
                    use_fnu,
                    show_both,
                    show_tone,
                    show_numeric,
                    out,
                );
                if valid_cw {
                    atb.pocbit = 170;
                }
            }

            self.word_number += 1;
            if self.word_number == 16 {
                self.synced = false;
            }
        }
    }

    /// Returns `true` if the codeword is valid (`Pocsag.cpp:172` refreshes `pocbit = 170`).
    #[allow(clippy::too_many_arguments)]
    fn process_codeword(
        &mut self,
        fn2: usize,
        bch: &BchDecoder,
        qual: &mut RxQualityTracker,
        use_fnu: bool,
        show_both: bool,
        show_tone: bool,
        show_numeric: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) -> bool {
        let mut res = bch.decode_hard(&self.word_bits);

        // If hard BCH fails while accumulating an active message, attempt 5-bit Chase-2 soft recovery
        // constrained to message codewords (`data_bits[0] == 1`).
        if !res.valid && self.has_address {
            let soft = bch.decode_soft_filtered(&self.word_bits, &self.word_conf, |d| d[0] == 1);
            if soft.valid {
                res = soft;
                self.soft_words_in_msg += 1;
            }
        }

        qual.record_word(res.errors, res.soft_recovered);

        let ob = &res.bits;
        let uncorrectable = !res.valid;

        if ob[0] == 1 || (uncorrectable && self.has_address) {
            // Message codeword (MSB == 1, `Pocsag.cpp:174–220`)
            if !self.has_address {
                return res.errors < 2;
            }
            for i in 1..=20 {
                self.sr_char >>= 1;
                if !uncorrectable && ob[i] == 1 {
                    self.sr_char += 0x40;
                }

                self.srca += 1;
                if self.srca == 7 {
                    let ch = if uncorrectable {
                        b'?'
                    } else {
                        (self.sr_char & 0x7f) as u8
                    };
                    self.alpha_chars.push(ch);
                    self.alpha_err.push(uncorrectable);
                    self.srca = 0;
                }

                if self.wordc < 7 {
                    self.srcn += 1;
                    if self.srcn == 4 {
                        let ch = if uncorrectable {
                            '?'
                        } else {
                            let du = ((self.sr_char >> 3) & 0x0f) as usize;
                            NUMERIC_TABLE[du]
                        };
                        self.num_chars.push(ch);
                        self.num_err.push(uncorrectable);
                        self.srcn = 0;
                    }
                }
            }
            self.wordc += 1;

            // Match `Pocsag.cpp:216–219` (`strncpy` stops at the first `0` byte!):
            let restbits = (20 * self.wordc) % 6;
            let startbit = 21 - restbits;
            self.rest_alpha_bits.clear();
            if !uncorrectable {
                for &b in &ob[startbit..21] {
                    if b == 0 {
                        break;
                    }
                    self.rest_alpha_bits.push(b);
                }
            }
        } else {
            // Address codeword (MSB == 0, `Pocsag.cpp:221–256`)
            if self.has_address {
                self.flush_pending(use_fnu, show_both, show_tone, show_numeric, out);
            }
            self.reset_message_state();

            // Do not start a new address on an uncorrectable random noise word
            if uncorrectable {
                return false;
            }

            let mut addr: u32 = 0;
            for i in (1..=18).rev() {
                addr >>= 1;
                if ob[i] == 1 {
                    addr ^= 0x10_0000;
                }
            }
            addr += ((fn2 >> 1) & 0x07) as u32;

            self.pocaddr = addr & 0x1f_ffff;
            self.addr_corrupted = uncorrectable;
            self.function_num = ((ob[19] << 1) | ob[20]) + 1;
            self.has_address = true;
        }

        res.valid && res.errors <= 2
    }

    pub fn flush_pending(
        &mut self,
        use_fnu: bool,
        show_both: bool,
        show_tone: bool,
        show_numeric: bool,
        out: &mut Vec<DecodedPagerMessage>,
    ) {
        if !self.has_address {
            return;
        }

        while self.num_chars.last() == Some(&' ') {
            self.num_chars.pop();
            self.num_err.pop();
        }
        while let Some(&last) = self.alpha_chars.last() {
            if matches!(last, 0 | 3 | 4 | 10 | 13 | 23 | 25) {
                self.alpha_chars.pop();
                self.alpha_err.pop();
            } else {
                break;
            }
        }

        let msg_type = if self.wordc == 0 {
            if self.addr_corrupted {
                self.reset_message_state();
                return;
            }
            PocsagMsgType::ToneOnly
        } else {
            match self.classify_message_type(use_fnu, show_both) {
                Some(t) => t,
                None => {
                    self.reset_message_state();
                    return;
                }
            }
        };

        let now = chrono::Local::now();
        let time_str = now.format("%H:%M:%S").to_string();
        let date_str = now.format("%d-%m-%y").to_string();
        let capcode_str = if self.addr_corrupted {
            "???????".to_string()
        } else {
            format!("{:07}", self.pocaddr)
        };
        let mode_str = format!("POCSAG-{}", self.function_num);

        if matches!(msg_type, PocsagMsgType::Alpha | PocsagMsgType::Both) {
            let mut text = String::with_capacity(self.alpha_chars.len());
            for &c in &self.alpha_chars {
                if c == 0x03 {
                    break; // Stop immediately on ETX
                } else if c == 0 {
                    // Skip NUL padding
                } else if c == b'\n' || c == b'\r' {
                    text.push(' ');
                } else if (32..=126).contains(&c) {
                    text.push(c as char);
                } else {
                    text.push('?');
                }
            }
            if !text.is_empty() {
                let has_err = self.addr_corrupted || self.alpha_err.iter().any(|&e| e);
                out.push(DecodedPagerMessage {
                    capcode: capcode_str.clone(),
                    time: time_str.clone(),
                    date: date_str.clone(),
                    mode: mode_str.clone(),
                    msg_type: "ALPHA".to_string(),
                    bitrate: self.baud_rate,
                    message: text,
                    has_bit_errors: has_err,
                    soft_recovered_words: self.soft_words_in_msg,
                    filter_label: None,
                    filter_color_idx: None,
                });
            }
        }

        if matches!(
            msg_type,
            PocsagMsgType::ToneOnly | PocsagMsgType::Numeric | PocsagMsgType::Both
        ) {
            if msg_type == PocsagMsgType::ToneOnly && show_tone {
                out.push(DecodedPagerMessage {
                    capcode: capcode_str,
                    time: time_str,
                    date: date_str,
                    mode: mode_str,
                    msg_type: "NUMERIC".to_string(),
                    bitrate: self.baud_rate,
                    message: "TONE ONLY".to_string(),
                    has_bit_errors: self.addr_corrupted,
                    soft_recovered_words: self.soft_words_in_msg,
                    filter_label: None,
                    filter_color_idx: None,
                });
            } else if msg_type != PocsagMsgType::ToneOnly && show_numeric {
                let text: String = self.num_chars.iter().collect();
                let has_err = self.addr_corrupted || self.num_err.iter().any(|&e| e);
                out.push(DecodedPagerMessage {
                    capcode: capcode_str,
                    time: time_str,
                    date: date_str,
                    mode: mode_str,
                    msg_type: "NUMERIC".to_string(),
                    bitrate: self.baud_rate,
                    message: text,
                    has_bit_errors: has_err,
                    soft_recovered_words: self.soft_words_in_msg,
                    filter_label: None,
                    filter_color_idx: None,
                });
            }
        }

        self.reset_message_state();
    }

    fn classify_message_type(&self, use_fnu: bool, show_both: bool) -> Option<PocsagMsgType> {
        if self.wordc >= 7 {
            return Some(PocsagMsgType::Alpha);
        }
        if use_fnu {
            return Some(if self.function_num == 4 {
                PocsagMsgType::Alpha
            } else {
                PocsagMsgType::Numeric
            });
        }

        let nnum = self.num_chars.len();
        let nalp = self.alpha_chars.len();

        let mut bad_num: i32 = 0;
        let mut bad_counts = [1i32; 6];
        for (i, &ch) in self.num_chars.iter().enumerate() {
            match ch {
                '[' => {
                    bad_num += 20 * bad_counts[0];
                    bad_counts[0] += 1;
                }
                ']' => {
                    bad_num += 20 * bad_counts[1];
                    bad_counts[1] += 1;
                }
                'U' => {
                    if i > 0 {
                        bad_num += 50 * bad_counts[2];
                    }
                    bad_counts[2] += 1;
                }
                '*' => {
                    bad_num += 200 * bad_counts[3];
                    bad_counts[3] += 1;
                }
                '-' => {
                    bad_num += 10 * bad_counts[4];
                    bad_counts[4] += 1;
                }
                ' ' => {
                    let allowed_space = (i == 1 && self.num_chars[0].is_ascii_digit() && self.num_chars[0] > '0')
                        || (i == 2
                            && self.num_chars[0] > '0'
                            && self.num_chars[0] <= '3'
                            && self.num_chars[1].is_ascii_digit());
                    if allowed_space {
                        bad_counts[5] += 1;
                    } else {
                        bad_num += 10 * bad_counts[5];
                        bad_counts[5] += 1;
                    }
                }
                _ => {}
            }
        }
        let num_score = if nnum > 0 {
            (100 - (bad_num / (nnum as i32))).max(0)
        } else {
            0
        };

        let mut bad_alp: i32 = 0;
        if nalp > 0 {
            let n = nalp as i32;
            for &c in &self.alpha_chars {
                if (123..127).contains(&c) || c == 96 {
                    bad_alp += 100 / n;
                } else if (91..96).contains(&c) || c == 10 {
                    bad_alp += 50 / n;
                } else if (58..65).contains(&c) || (35..45).contains(&c) {
                    bad_alp += 50 / n;
                } else if (33..35).contains(&c) {
                    bad_alp += 10 / n;
                } else if c < 32 || c >= 127 {
                    bad_alp += 1000 / n;
                }
            }
        }
        let alp_score = if nalp > 0 {
            (100 - (bad_alp / (nalp as i32))).max(0)
        } else {
            0
        };

        if self.wordc < 7 && self.rest_alpha_bits.contains(&1) && num_score >= 50 && alp_score < 95 {
            return Some(PocsagMsgType::Numeric);
        }

        if num_score < 50 && alp_score < 50 {
            return None;
        }
        if num_score == alp_score && show_both {
            return Some(PocsagMsgType::Both);
        }
        if bad_num == 0 || num_score > alp_score {
            Some(PocsagMsgType::Numeric)
        } else if bad_alp == 0 || alp_score > num_score {
            Some(PocsagMsgType::Alpha)
        } else {
            None
        }
    }

    fn reset_message_state(&mut self) {
        self.has_address = false;
        self.pocaddr = 0;
        self.addr_corrupted = false;
        self.wordc = 0;
        self.alpha_chars.clear();
        self.alpha_err.clear();
        self.num_chars.clear();
        self.num_err.clear();
        self.sr_char = 0;
        self.srca = 0;
        self.srcn = 0;
        self.rest_alpha_bits.clear();
        self.soft_words_in_msg = 0;
    }
}
