//! Lock-Free Soundcard Capture Engine (`cpal`) & 48 kHz / 44.1 kHz Synthetic Test Signal Generator.
//!
//! Eliminates `sound_in.cpp`'s `buffers_ready` thread race and `WaveHeader` leak by streaming
//! `f32` audio chunks over a lock-free bounded channel to the dedicated DSP worker thread.

use crate::bch::BchDecoder;
use crate::flex::FlexDecoder;
use crate::pocsag::{POCSAG_IDLE, POCSAG_SYNC};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use crossbeam_channel::Sender;

pub struct AudioCaptureStream {
    _stream: cpal::Stream,
    pub device_name: String,
    pub actual_sample_rate: u32,
}

pub fn list_input_devices() -> Vec<String> {
    let host = cpal::default_host();
    let mut names = Vec::new();
    if let Ok(devices) = host.input_devices() {
        for dev in devices {
            if let Ok(name) = dev.name() {
                names.push(name);
            }
        }
    }
    names
}

#[inline]
fn downmix_frame_f32(frame: &[f32]) -> f32 {
    if frame.len() == 1 {
        frame[0]
    } else if frame.len() >= 2 {
        let l = frame[0];
        let r = frame[1];
        if l.abs() > r.abs() * 2.5 {
            l
        } else if r.abs() > l.abs() * 2.5 {
            r
        } else {
            0.5 * (l + r)
        }
    } else {
        0.0
    }
}

pub fn start_audio_capture(
    preferred_device: Option<&str>,
    preferred_sample_rate: u32,
    sample_tx: Sender<Vec<f32>>,
) -> Result<AudioCaptureStream, String> {
    let host = cpal::default_host();
    let device = if let Some(target_name) = preferred_device {
        host.input_devices()
            .map_err(|e| e.to_string())?
            .find(|d| d.name().map(|n| n == target_name).unwrap_or(false))
            .or_else(|| host.default_input_device())
            .ok_or_else(|| "No audio input device available".to_string())?
    } else {
        host.default_input_device()
            .ok_or_else(|| "No default audio input device available".to_string())?
    };

    let device_name = device.name().unwrap_or_else(|_| "Default Input".to_string());

    let default_cfg = device
        .default_input_config()
        .map_err(|e| format!("Failed to query input config: {}", e))?;
    let channels = default_cfg.channels() as usize;
    let sample_format = default_cfg.sample_format();
    let native_rate = default_cfg.sample_rate().0;

    let supported_desired = device
        .supported_input_configs()
        .ok()
        .and_then(|mut cfgs| {
            cfgs.find(|c| {
                c.channels() == default_cfg.channels()
                    && c.min_sample_rate().0 <= preferred_sample_rate
                    && preferred_sample_rate <= c.max_sample_rate().0
            })
        });

    let candidate_rates: Vec<u32> = if supported_desired.is_some() && preferred_sample_rate != native_rate {
        vec![preferred_sample_rate, native_rate]
    } else {
        vec![native_rate, preferred_sample_rate]
    };

    let mut last_err = String::from("Unable to open audio stream");
    for &rate in &candidate_rates {
        let stream_cfg = cpal::StreamConfig {
            channels: default_cfg.channels(),
            sample_rate: cpal::SampleRate(rate),
            buffer_size: cpal::BufferSize::Default,
        };
        let tx = sample_tx.clone();
        let err_fn = |err| {
            eprintln!("Audio capture stream error: {}", err);
        };

        let stream_res = match sample_format {
            cpal::SampleFormat::F32 => device.build_input_stream(
                &stream_cfg,
                move |data: &[f32], _: &cpal::InputCallbackInfo| {
                    let ch = channels.max(1);
                    let mut mono = Vec::with_capacity(data.len() / ch);
                    for frame in data.chunks(ch) {
                        mono.push(downmix_frame_f32(frame));
                    }
                    let _ = tx.try_send(mono);
                },
                err_fn,
                None,
            ),
            cpal::SampleFormat::I16 => device.build_input_stream(
                &stream_cfg,
                move |data: &[i16], _: &cpal::InputCallbackInfo| {
                    let ch = channels.max(1);
                    let mut mono = Vec::with_capacity(data.len() / ch);
                    for frame in data.chunks(ch) {
                        let f: [f32; 2] = [
                            (frame[0] as f32) / 32768.0,
                            (frame.get(1).copied().unwrap_or(frame[0]) as f32) / 32768.0,
                        ];
                        mono.push(downmix_frame_f32(&f[..frame.len().min(2)]));
                    }
                    let _ = tx.try_send(mono);
                },
                err_fn,
                None,
            ),
            cpal::SampleFormat::U8 => device.build_input_stream(
                &stream_cfg,
                move |data: &[u8], _: &cpal::InputCallbackInfo| {
                    let ch = channels.max(1);
                    let mut mono = Vec::with_capacity(data.len() / ch);
                    for frame in data.chunks(ch) {
                        let f: [f32; 2] = [
                            ((frame[0] as f32) - 128.0) / 128.0,
                            ((frame.get(1).copied().unwrap_or(frame[0]) as f32) - 128.0) / 128.0,
                        ];
                        mono.push(downmix_frame_f32(&f[..frame.len().min(2)]));
                    }
                    let _ = tx.try_send(mono);
                },
                err_fn,
                None,
            ),
            other => return Err(format!("Unsupported sample format: {:?}", other)),
        };

        match stream_res {
            Ok(stream) => {
                stream.play().map_err(|e| e.to_string())?;
                return Ok(AudioCaptureStream {
                    _stream: stream,
                    device_name,
                    actual_sample_rate: rate,
                });
            }
            Err(e) => {
                last_err = e.to_string();
            }
        }
    }

    Err(last_err)
}

/// Synthesize a 48 kHz / 44.1 kHz POCSAG paging burst with optional noise for built-in testing.
pub fn synthesize_pocsag_burst(
    sample_rate: u32,
    baud_rate: u32,
    capcode: u32,
    function_num: u8,
    text: &str,
    noise_level: f32,
) -> Vec<f32> {
    let bch = BchDecoder::new();
    let mut bits = Vec::new();

    // 1. 576 bits of alternating 101010... preamble
    for i in 0..576 {
        bits.push(if i % 2 == 0 { 1u8 } else { 0u8 });
    }

    let push_u32 = |v: &mut Vec<u8>, w: u32| {
        for i in (0..32).rev() {
            v.push(((w >> i) & 1) as u8);
        }
    };

    // 2. Frame Sync Codeword (`0x7CD215D8`)
    push_u32(&mut bits, POCSAG_SYNC);

    let target_frame = (capcode & 0x07) as usize;
    let target_word_start = target_frame * 2;

    // Pack 7-bit ASCII characters (LSB first per character) into 20-bit message payloads
    let mut msg_bits = Vec::new();
    for &byte in text.as_bytes() {
        let c = byte & 0x7f;
        for b in 0..7 {
            msg_bits.push((c >> b) & 1);
        }
    }
    while msg_bits.len() % 20 != 0 {
        msg_bits.push(0);
    }

    let mut codewords: Vec<[u8; 32]> = Vec::new();

    let mut addr_data = [0u8; 21];
    addr_data[0] = 0;
    let addr_18 = (capcode >> 3) & 0x3_ffff;
    for i in 1..=18 {
        addr_data[i] = ((addr_18 >> (18 - i)) & 1) as u8;
    }
    let fn_bits = function_num.saturating_sub(1) & 0x03;
    addr_data[19] = (fn_bits >> 1) & 1;
    addr_data[20] = fn_bits & 1;
    codewords.push(bch.encode(&addr_data));

    for chunk in msg_bits.chunks(20) {
        let mut m_data = [0u8; 21];
        m_data[0] = 1;
        for (i, &b) in chunk.iter().enumerate() {
            m_data[1 + i] = b;
        }
        codewords.push(bch.encode(&m_data));
    }

    let mut idle_bits = [0u8; 32];
    for i in 0..32 {
        idle_bits[i] = ((POCSAG_IDLE >> (31 - i)) & 1) as u8;
    }

    let mut word_pos = 0usize;
    for _ in 0..target_word_start {
        bits.extend_from_slice(&idle_bits);
        word_pos += 1;
    }

    for cw in &codewords {
        if word_pos == 16 {
            push_u32(&mut bits, POCSAG_SYNC);
            word_pos = 0;
        }
        bits.extend_from_slice(cw);
        word_pos += 1;
    }

    while word_pos < 16 {
        bits.extend_from_slice(&idle_bits);
        word_pos += 1;
    }

    let sps = (sample_rate as f64) / (baud_rate as f64);
    let total_samples = ((bits.len() as f64) * sps).ceil() as usize + 200;
    let mut samples = Vec::with_capacity(total_samples);
    let mut lfsr: u32 = 0xACE1_1234;

    for s_idx in 0..total_samples {
        let bit_idx = ((s_idx as f64) / sps).floor() as usize;
        let b = if bit_idx < bits.len() { bits[bit_idx] } else { 0 };
        let clean = if b == 1 { 0.65f32 } else { -0.65f32 };
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        let rand_norm = ((lfsr & 0xFFFF) as f32 / 32768.0) - 1.0;
        samples.push(clean + noise_level * rand_norm);
    }

    samples
}

/// Build an 88-word FLEX phase frame with 1 Alphanumeric message (`Flex.cpp:435–496`).
pub fn build_flex_alpha_phase_words(capcode: u32, msg: &str) -> [u32; 88] {
    let mut words = [0x1f_ffffu32; 88];
    // BIW at word 0: asa = 1 (end block = 0), vsa = 2
    let mut biw: u32 = (2 << 10) | (0 << 8);
    biw = add_flex_xsum(biw);
    words[0] = biw;

    // Short address at word 1: `capcode + 32768`
    words[1] = (capcode + 32_768) & 0x1f_ffff;

    // In `Flex.cpp:449–496`:
    // - `words[3]` (`w1` initial) is the message header word (`iFragmentNumber = 3` in bits 11..12); `w1++` advances `w1` to `4`.
    // - `words[4]` (`k == w1`) has signature/flags in bits 0..6 (skipped when `iFragmentNumber == 3`),
    //   and the first 2 characters (`bytes[0]` and `bytes[1]`) in bits 7..13 and 14..20!
    // - `words[5..]` (`k > w1`) pack 3 characters per 21-bit word!
    let bytes = msg.as_bytes();
    let rem_len = bytes.len().saturating_sub(2);
    let extra_char_words = (rem_len + 2) / 3;
    let total_words = 2 + extra_char_words; // words[3] (header) + words[4] (first 2 chars) + words[5..]

    // Vector word at word 2: vt = 5 (ALPHA), w1 = 3, len = total_words
    let mut vec_w: u32 = (5 << 4) | (3 << 7) | ((total_words as u32) << 14);
    vec_w = add_flex_xsum(vec_w);
    words[2] = vec_w;

    // Word 3: Header word with `iFragmentNumber = 3` (bits 11..=12)
    words[3] = 3 << 11;

    // Word 4: Signature in bits 0..6 + first 2 characters in bits 7..13 and 14..20
    let first_c1 = bytes.first().copied().unwrap_or(0x03) as u32;
    let first_c2 = bytes.get(1).copied().unwrap_or(0x03) as u32;
    words[4] = 0x7f | ((first_c1 & 0x7f) << 7) | ((first_c2 & 0x7f) << 14);

    let mut b_idx = 2usize;
    for w_offset in 0..extra_char_words {
        let ch0 = bytes.get(b_idx).copied().unwrap_or(0x03) as u32;
        let ch1 = bytes.get(b_idx + 1).copied().unwrap_or(0x03) as u32;
        let ch2 = bytes.get(b_idx + 2).copied().unwrap_or(0x03) as u32;
        words[5 + w_offset] = (ch0 & 0x7f) | ((ch1 & 0x7f) << 7) | ((ch2 & 0x7f) << 14);
        b_idx += 3;
    }

    words
}

/// Synthesize a 48 kHz / 44.1 kHz 4-Level FSK FLEX-3200 (1600 sps, Phases A+B) frame
/// carrying independent messages on Phase A and Phase B.
pub fn synthesize_flex_4fsk_burst(
    sample_rate: u32,
    capcode_a: u32,
    text_a: &str,
    capcode_b: u32,
    text_b: &str,
    noise_level: f32,
) -> Vec<f32> {
    let bch = BchDecoder::new();
    let mut symbols_1600: Vec<u8> = Vec::new();

    // 1. 64 symbols of alternating 2-level 1600-baud preamble (0, 3, 0, 3...)
    for i in 0..64 {
        symbols_1600.push(if i % 2 == 0 { 0 } else { 3 });
    }

    let push_word_2fsk = |v: &mut Vec<u8>, w: u16| {
        for i in (0..16).rev() {
            let bit = (w >> i) & 1;
            v.push(if bit == 1 { 0 } else { 3 });
        }
    };

    // 2. Sync-1 for FLEX-3200 4-FSK (1600 sps, 4-level, Phases A + B): A1 = 0xB068
    let a1: u16 = 0xB068;
    push_word_2fsk(&mut symbols_1600, a1);
    push_word_2fsk(&mut symbols_1600, 0xA6C6);
    push_word_2fsk(&mut symbols_1600, 0xAAAA);
    push_word_2fsk(&mut symbols_1600, !a1);

    // 16-bit frame marker before FIW (`flex_bc` 87..=72)
    push_word_2fsk(&mut symbols_1600, 0xAAAA);

    // 32-bit Frame Info Word (FIW) at `flex_bc` 71..=40 (Cycle = 3, Frame = 42)
    let mut fiw_data = [0u8; 21];
    let cy_inv = (3u8 ^ 0x0f) & 0x0f;
    for ihd in 4..8 {
        fiw_data[ihd] = (cy_inv >> (ihd - 4)) & 1;
    }
    let fr_inv = (42u8 ^ 0x7f) & 0x7f;
    for ihd in 8..=14 {
        fiw_data[ihd] = (fr_inv >> (ihd - 8)) & 1;
    }
    let fiw_cw = bch.encode(&fiw_data);
    for &b in &fiw_cw {
        symbols_1600.push(if b == 1 { 0 } else { 3 });
    }

    // 40-symbol Sync-2 sequence at 1600 sps (`flex_bc` 39..=0)
    for i in 0..40 {
        symbols_1600.push(if i % 2 == 0 { 0 } else { 3 });
    }

    let words_a = build_flex_alpha_phase_words(capcode_a, text_a);
    let words_b = build_flex_alpha_phase_words(capcode_b, text_b);

    for blk in 0..11 {
        let mut cws_a = [[0u8; 32]; 8];
        let mut cws_b = [[0u8; 32]; 8];
        for i in 0..8 {
            let wa = words_a[blk * 8 + i];
            let wb = words_b[blk * 8 + i];
            let mut da = [0u8; 21];
            let mut db = [0u8; 21];
            for j in 0..21 {
                da[j] = if ((wa >> j) & 1) == 1 { 0 } else { 1 };
                db[j] = if ((wb >> j) & 1) == 1 { 0 } else { 1 };
            }
            cws_a[i] = bch.encode(&da);
            cws_b[i] = bch.encode(&db);
        }

        for j in 0..32 {
            for i in 0..8 {
                let bit_a = cws_a[i][j];
                let bit_b = cws_b[i][j];
                let gin = match (bit_a, bit_b) {
                    (1, 1) => 0,
                    (1, 0) => 1,
                    (0, 0) => 2,
                    _ => 3,
                };
                symbols_1600.push(gin);
            }
        }
    }

    let sps = (sample_rate as f64) / 1600.0;
    let total_samples = ((symbols_1600.len() as f64) * sps).ceil() as usize + 200;
    let mut samples = Vec::with_capacity(total_samples);
    let mut lfsr: u32 = 0xCAFE_BEEF;

    for s_idx in 0..total_samples {
        let sym_idx = ((s_idx as f64) / sps).floor() as usize;
        let gin = if sym_idx < symbols_1600.len() {
            symbols_1600[sym_idx]
        } else {
            2
        };
        let level = match gin {
            0 => 0.75f32,
            1 => 0.25f32,
            2 => -0.25f32,
            _ => -0.75f32,
        };
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        let rand_norm = ((lfsr & 0xFFFF) as f32 / 32768.0) - 1.0;
        samples.push(level + noise_level * rand_norm);
    }

    samples
}

/// Synthesize a 3200-sps FLEX-6400 (4-FSK, Phases A+B+C+D) or FLEX-3200 (2-FSK, Phases A+C) burst
/// to verify 1600 -> 3200 baud `cross_over` synchronization at 44.1 kHz and 48 kHz.
pub fn synthesize_flex_6400_4fsk_burst(
    sample_rate: u32,
    cap_a: u32,
    txt_a: &str,
    cap_b: u32,
    txt_b: &str,
    cap_c: u32,
    txt_c: &str,
    cap_d: u32,
    txt_d: &str,
    noise_level: f32,
) -> Vec<f32> {
    let bch = BchDecoder::new();
    // Express the entire frame in 3200-sps half-bit units (where each 1600-bps header bit = 2 units)
    let mut symbols_3200: Vec<u8> = Vec::new();

    let push_1600_bit = |v: &mut Vec<u8>, gin: u8| {
        v.push(gin);
        v.push(gin);
    };
    let push_word_1600 = |v: &mut Vec<u8>, w: u16| {
        for i in (0..16).rev() {
            let bit = (w >> i) & 1;
            let gin = if bit == 1 { 0 } else { 3 };
            v.push(gin);
            v.push(gin);
        }
    };

    // 1. 64 bits of 1600-bps preamble
    for i in 0..64 {
        push_1600_bit(&mut symbols_3200, if i % 2 == 0 { 0 } else { 3 });
    }

    // 2. Sync-1 for FLEX-6400 (3200 sps, 4-FSK, Phases A+B+C+D): A1 = 0xDEA0
    let a1: u16 = 0xDEA0;
    push_word_1600(&mut symbols_3200, a1);
    push_word_1600(&mut symbols_3200, 0xA6C6);
    push_word_1600(&mut symbols_3200, 0xAAAA);
    push_word_1600(&mut symbols_3200, !a1);

    // 3. 16-bit marker + 32-bit FIW at 1600 bps (48 bits = 96 3200-sps units)
    push_word_1600(&mut symbols_3200, 0xAAAA);
    let mut fiw_data = [0u8; 21];
    let cy_inv = (6u8 ^ 0x0f) & 0x0f;
    for ihd in 4..8 {
        fiw_data[ihd] = (cy_inv >> (ihd - 4)) & 1;
    }
    let fr_inv = (49u8 ^ 0x7f) & 0x7f;
    for ihd in 8..=14 {
        fiw_data[ihd] = (fr_inv >> (ihd - 8)) & 1;
    }
    let fiw_cw = bch.encode(&fiw_data);
    for &b in &fiw_cw {
        push_1600_bit(&mut symbols_3200, if b == 1 { 0 } else { 3 });
    }

    // 4. Sync-2: 40 1600-baud bit-periods = 80 symbols at 3200 sps (ending with 3200-baud alternating 0, 3, 0, 3...)
    for i in 0..80 {
        symbols_3200.push(if i % 2 == 0 { 0 } else { 3 });
    }

    // 5. 11 data blocks at 3200 sps (interleaving Phase A/B on hbit=0 and Phase C/D on hbit=1)
    let words_a = build_flex_alpha_phase_words(cap_a, txt_a);
    let words_b = build_flex_alpha_phase_words(cap_b, txt_b);
    let words_c = build_flex_alpha_phase_words(cap_c, txt_c);
    let words_d = build_flex_alpha_phase_words(cap_d, txt_d);

    let encode_block_words = |words: &[u32; 88], blk: usize| -> [[u8; 32]; 8] {
        let mut cws = [[0u8; 32]; 8];
        for i in 0..8 {
            let w = words[blk * 8 + i];
            let mut d = [0u8; 21];
            for j in 0..21 {
                d[j] = if ((w >> j) & 1) == 1 { 0 } else { 1 };
            }
            cws[i] = bch.encode(&d);
        }
        cws
    };

    for blk in 0..11 {
        let cws_a = encode_block_words(&words_a, blk);
        let cws_b = encode_block_words(&words_b, blk);
        let cws_c = encode_block_words(&words_c, blk);
        let cws_d = encode_block_words(&words_d, blk);

        for j in 0..32 {
            for i in 0..8 {
                let gin_ab = match (cws_a[i][j], cws_b[i][j]) {
                    (1, 1) => 0,
                    (1, 0) => 1,
                    (0, 0) => 2,
                    _ => 3,
                };
                let gin_cd = match (cws_c[i][j], cws_d[i][j]) {
                    (1, 1) => 0,
                    (1, 0) => 1,
                    (0, 0) => 2,
                    _ => 3,
                };
                symbols_3200.push(gin_ab);
                symbols_3200.push(gin_cd);
            }
        }
    }

    let sps = (sample_rate as f64) / 3200.0;
    let total_samples = ((symbols_3200.len() as f64) * sps).ceil() as usize + 200;
    let mut samples = Vec::with_capacity(total_samples);
    let mut lfsr: u32 = 0x1357_9BDF;

    for s_idx in 0..total_samples {
        let sym_idx = ((s_idx as f64) / sps).floor() as usize;
        let gin = if sym_idx < symbols_3200.len() {
            symbols_3200[sym_idx]
        } else {
            2
        };
        let level = match gin {
            0 => 0.75f32,
            1 => 0.25f32,
            2 => -0.25f32,
            _ => -0.75f32,
        };
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        let rand_norm = ((lfsr & 0xFFFF) as f32 / 32768.0) - 1.0;
        samples.push(level + noise_level * rand_norm);
    }

    samples
}

fn add_flex_xsum(word_without_xsum: u32) -> u32 {
    let w = word_without_xsum & 0x1f_fff0;
    let sum = ((w >> 4) & 0x0f)
        + ((w >> 8) & 0x0f)
        + ((w >> 12) & 0x0f)
        + ((w >> 16) & 0x0f)
        + ((w >> 20) & 0x01);
    let xsum = (0x0f - (sum & 0x0f)) & 0x0f;
    let candidate = w | xsum;
    debug_assert!(FlexDecoder::xsumchk(candidate));
    candidate
}
