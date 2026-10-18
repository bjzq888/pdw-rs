//! `pdw-rs`: Modern Rust POCSAG & FLEX Paging Decoder with Parallel Multi-Frequency SDRconnect IQ,
//! Multi-Fragment FLEX Reassembly, Capcode Directory Catalog & Built-In Live Web Server.

pub mod audio;
pub mod bch;
pub mod catalog;
pub mod config;
pub mod dsp;
pub mod flex;
pub mod gui;
pub mod pocsag;
pub mod sdrconnect;
pub mod web;

use gui::PdwApp;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("PDW-RS v0.2.7 — POCSAG & FLEX (2/4-FSK) Decoder (Parallel Multi-Freq IQ, Capcode Catalog & Live Web Server)")
            .with_inner_size([1160.0, 720.0])
            .with_min_inner_size([760.0, 440.0]),
        ..Default::default()
    };

    eframe::run_native(
        "pdw-rs",
        native_options,
        Box::new(|cc| Ok(Box::new(PdwApp::new(cc)))),
    )
}

#[cfg(test)]
mod tests {
    use crate::audio::{
        build_flex_alpha_phase_words_frag, synthesize_flex_4fsk_burst,
        synthesize_flex_6400_4fsk_burst, synthesize_pocsag_burst,
    };
    use crate::bch::BchDecoder;
    use crate::catalog::CapcodeCatalogDb;
    use crate::config::{AppConfig, DEFAULT_PRIMARY_FREQ_HZ, DEFAULT_SECONDARY_FREQ_HZ};
    use crate::dsp::{AdaptiveEnvelopeTracker, AtbSlicer, RxQualityTracker};
    use crate::flex::{FlexDecoder, FlexPhaseBuffer, PendingFlexFragment};
    use crate::gui::{process_audio_chunk, DspTelemetry};
    use crate::pocsag::{DecodedPagerMessage, PocsagChannelDecoder};
    use crate::sdrconnect::{
        synthesize_multi_channel_sdrconnect_iq_frame, MultiChannelIqDemodulator,
    };
    use crossbeam_channel::bounded;
    use std::collections::VecDeque;
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex};

    fn run_stream_in_wasapi_chunks(sample_rate: u32, samples: &[f32]) -> Vec<DecodedPagerMessage> {
        let mut cfg = AppConfig::default();
        cfg.sample_rate = sample_rate;
        let bch = BchDecoder::new();
        let mut env = AdaptiveEnvelopeTracker::new();
        let mut qual = RxQualityTracker::new();
        let mut pocsag_atb = AtbSlicer::new(cfg.sample_rate, cfg.audio_preset, cfg.invert_polarity);
        let mut flex_atb = AtbSlicer::new(cfg.sample_rate, cfg.audio_preset, cfg.invert_polarity);
        let mut pocsag_main = PocsagChannelDecoder::new(cfg.sample_rate, 1200);
        let mut flex = FlexDecoder::new(cfg.sample_rate);
        let (msg_tx, msg_rx) = bounded::<DecodedPagerMessage>(256);
        let mut scope_ring = VecDeque::from(vec![0.0; 80]);
        let mut sample_counter = 0usize;
        let telemetry = Arc::new(Mutex::new(DspTelemetry::default()));

        for chunk in samples.chunks(441) {
            process_audio_chunk(
                chunk,
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
                None,
            );
        }

        let mut decoded = Vec::new();
        while let Ok(m) = msg_rx.try_recv() {
            decoded.push(m);
        }
        decoded
    }

    #[test]
    fn test_pocsag_512_1200_2400_and_inverted_polarity_at_44100_and_48000() {
        for &sr in &[44_100u32, 48_000u32] {
            let mut stream = Vec::new();
            for i in 0..2000 {
                stream.push(if i < 1000 { -0.974 } else { 0.974 });
            }
            stream.extend(synthesize_pocsag_burst(sr, 512, 111_222, 4, "HELLO", 0.03));
            stream.extend(vec![0.0; (sr / 4) as usize]);

            let mut inv_1200 = synthesize_pocsag_burst(
                sr,
                1200,
                1_234_567,
                4,
                "INVERTED POCSAG 1200 LIVE TEST OK",
                0.03,
            );
            for s in &mut inv_1200 {
                *s = -*s;
            }
            stream.extend(inv_1200);
            stream.extend(vec![0.0; (sr / 4) as usize]);

            stream.extend(synthesize_pocsag_burst(
                sr,
                2400,
                765_432,
                4,
                "HIGH SPEED POCSAG 2400 DECODE OK",
                0.03,
            ));

            let msgs = run_stream_in_wasapi_chunks(sr, &stream);
            assert_eq!(msgs.len(), 3, "Failed at {} Hz: got {:?}", sr, msgs);
            assert_eq!(msgs[0].capcode, "0111222");
            assert_eq!(msgs[0].msg_type, "ALPHA");
            assert_eq!(msgs[0].message, "HELLO");

            assert_eq!(msgs[1].capcode, "1234567");
            assert_eq!(msgs[1].message, "INVERTED POCSAG 1200 LIVE TEST OK");

            assert_eq!(msgs[2].capcode, "0765432");
            assert_eq!(msgs[2].message, "HIGH SPEED POCSAG 2400 DECODE OK");
        }
    }

    #[test]
    fn test_flex_4fsk_3200_and_6400_all_phases_after_974_noise_at_44100_and_48000() {
        for &sr in &[44_100u32, 48_000u32] {
            let mut stream = Vec::new();
            for i in 0..4000 {
                stream.push(if (i / 30) % 2 == 0 { 0.974 } else { -0.974 });
            }

            stream.extend(synthesize_flex_4fsk_burst(
                sr,
                1_122_334,
                "FLEX PHASE-A 4-FSK DECODE OK",
                1_556_677,
                "FLEX PHASE-B 4-FSK UNLOCKED!",
                0.02,
            ));
            stream.extend(vec![-0.75; (sr / 5) as usize]);

            stream.extend(synthesize_flex_6400_4fsk_burst(
                sr,
                1_000_001,
                "PHASE A 6400 OK",
                1_000_002,
                "PHASE B 6400 OK",
                1_000_003,
                "PHASE C 6400 OK",
                1_000_004,
                "PHASE D 6400 OK",
                0.02,
            ));

            let mut inv_flex = synthesize_flex_4fsk_burst(
                sr,
                1_233_445,
                "INVERTED FLEX A OK",
                1_677_889,
                "INVERTED FLEX B OK",
                0.02,
            );
            for s in &mut inv_flex {
                *s = -*s;
            }
            stream.extend(inv_flex);

            let msgs = run_stream_in_wasapi_chunks(sr, &stream);
            assert_eq!(msgs.len(), 8, "Failed at {} Hz: got {:?}", sr, msgs);
            assert_eq!(msgs[0].capcode, "1122334");
            assert_eq!(msgs[0].message, "FLEX PHASE-A 4-FSK DECODE OK");
            assert_eq!(msgs[1].capcode, "1556677");
            assert_eq!(msgs[1].message, "FLEX PHASE-B 4-FSK UNLOCKED!");

            assert_eq!(msgs[2].capcode, "1000001");
            assert_eq!(msgs[2].message, "PHASE A 6400 OK");
            assert_eq!(msgs[3].capcode, "1000002");
            assert_eq!(msgs[3].message, "PHASE B 6400 OK");
            assert_eq!(msgs[4].capcode, "1000003");
            assert_eq!(msgs[4].message, "PHASE C 6400 OK");
            assert_eq!(msgs[5].capcode, "1000004");
            assert_eq!(msgs[5].message, "PHASE D 6400 OK");

            assert_eq!(msgs[6].capcode, "1233445");
            assert_eq!(msgs[6].message, "INVERTED FLEX A OK");
            assert_eq!(msgs[7].capcode, "1677889");
            assert_eq!(msgs[7].message, "INVERTED FLEX B OK");
        }
    }

    #[test]
    fn test_encrypted_base64_alpha_filter_preserves_human_messages() {
        use crate::config::is_human_readable_message;

        let encrypted = [
            "FoMKTDczTUY0DZoK5NoqKgipCzmL55H16DzYnUw33swJSx0pxfaDYSFHiq27ymLo8bJ",
            "LK/xtiwTG1kR//QJarZNHgrzDXffQc1acKrYGGXUA4SFA5AuZV21Qp+NfJuEX/2rC9/prD",
            "2vVJeM6xvYpelY1umn8k83/SsOm0hvLQzbkovHSVoAxPw5c0zCPQl2gqiLWiely0LEW77",
            "t2ucQOeWpkowj5S2gidWoysx5y+J/jDBLYh89qgWeKn5sKhKgbwBP7a9+CO5v/x+3CEZ",
            "ru97xOivQ1h/U/9m7JgTkNRXxwS+asJN",
            "2nRwfzKtEwiMMxyngax0caFbKhQywCM6fLQAVMJEcZZvyHw3ftBAA",
            "WrA1tmyBUeRZB//sUKQ4U8UGNY0FR9/J41xI4A6Rgq7SVFZ6ErX+GTGWuQ+Hkf/175X",
        ];
        for msg in encrypted {
            assert!(
                !is_human_readable_message(msg, "ALPHA"),
                "Should filter encrypted Base64 message: {}",
                msg
            );
        }

        let human = [
            "From: <noreply@rush.edu> Message: T4-09 is ready to clean.",
            "HELLO",
            "FLEX PHASE-A 4-FSK DECODE OK",
            "FLEX PHASE-B 4-FSK UNLOCKED!",
            "CODE BLUE MICU BED 4 - CALL X4502",
        ];
        for msg in human {
            assert!(
                is_human_readable_message(msg, "ALPHA"),
                "Should preserve human-readable message: {}",
                msg
            );
        }
    }

    #[test]
    fn test_sdrconnect_type2_iq_demodulation_pocsag_and_flex_4fsk_including_high_gain() {
        use crate::sdrconnect::{
            synthesize_sdrconnect_iq_frame_from_baseband, IqFmDemodulator,
        };

        for &gain in &[1.0f32, 3.0f32] {
            let sr = 48_000u32;
            let mut baseband = Vec::new();
            baseband.extend(vec![0.0; (sr / 10) as usize]);
            baseband.extend(synthesize_pocsag_burst(
                sr,
                1200,
                1_234_567,
                4,
                "SDRCONNECT DIRECT IQ POCSAG 1200 OK",
                0.01,
            ));
            baseband.extend(vec![0.0; (sr / 5) as usize]);
            baseband.extend(synthesize_flex_4fsk_burst(
                sr,
                1_122_334,
                "SDRCONNECT IQ FLEX PHASE A OK",
                1_556_677,
                "SDRCONNECT IQ FLEX PHASE B OK",
                0.01,
            ));
            baseband.extend(vec![0.0; (sr / 5) as usize]);
            baseband.extend(synthesize_flex_6400_4fsk_burst(
                sr,
                1_000_001,
                "IQ 6400 A OK",
                1_000_002,
                "IQ 6400 B OK",
                1_000_003,
                "IQ 6400 C OK",
                1_000_004,
                "IQ 6400 D OK",
                0.01,
            ));
            baseband.extend(vec![0.0; (sr / 10) as usize]);

            let type2_frame = synthesize_sdrconnect_iq_frame_from_baseband(&baseband, sr, 180.0);

            let mut iq_demod = IqFmDemodulator::new(sr, sr, gain, false, -85.0, true, 20_000.0);
            let demod_audio = iq_demod
                .process_binary_frame(&type2_frame)
                .expect("Type-2 binary IQ frame should produce demodulated baseband audio");

            let msgs = run_stream_in_wasapi_chunks(sr, &demod_audio);
            assert_eq!(
                msgs.len(),
                7,
                "Expected 7 decoded messages at gain={}, got {:?}",
                gain,
                msgs
            );
            assert_eq!(msgs[0].capcode, "1234567");
            assert_eq!(msgs[0].message, "SDRCONNECT DIRECT IQ POCSAG 1200 OK");
            assert_eq!(msgs[1].capcode, "1122334");
            assert_eq!(msgs[1].message, "SDRCONNECT IQ FLEX PHASE A OK");
            assert_eq!(msgs[2].capcode, "1556677");
            assert_eq!(msgs[2].message, "SDRCONNECT IQ FLEX PHASE B OK");
            assert_eq!(msgs[3].message, "IQ 6400 A OK");
            assert_eq!(msgs[4].message, "IQ 6400 B OK");
            assert_eq!(msgs[5].message, "IQ 6400 C OK");
            assert_eq!(msgs[6].message, "IQ 6400 D OK");
        }
    }

    #[test]
    fn test_parallel_multi_frequency_decoding_929_61250_and_929_73862_mhz_and_2mhz_limit() {
        let cfg = AppConfig::default();
        let enabled = cfg.enabled_frequencies_hz();
        assert_eq!(
            enabled,
            vec![DEFAULT_PRIMARY_FREQ_HZ, DEFAULT_SECONDARY_FREQ_HZ],
            "Default config must monitor 929.61250 MHz and 929.73862 MHz in parallel"
        );
        assert!(
            cfg.validate_frequency_within_2mhz(930_500_000, None).is_ok(),
            "Frequency within 2 MHz span should be accepted"
        );
        assert!(
            cfg.validate_frequency_within_2mhz(932_000_000, None).is_err(),
            "Frequency > 2 MHz away from 929.61250 MHz should be rejected"
        );

        // Synthesize a 384 kHz wideband IQ stream containing simultaneous paging bursts on:
        // - Channel 1: 929.61250 MHz (offset 0 Hz from 929.61250 MHz reference)
        // - Channel 2: 929.73862 MHz (offset +126,120 Hz from 929.61250 MHz reference)
        let baseband_sr = 48_000u32;
        let iq_sr = 384_000u32;
        let mut ch1_baseband = vec![0.0f32; 2400];
        ch1_baseband.extend(synthesize_pocsag_burst(
            baseband_sr,
            1200,
            1_234_567,
            4,
            "CH1 929.61250 MHZ PARALLEL DECODE OK",
            0.005,
        ));
        ch1_baseband.extend(vec![0.0f32; 2400]);

        let mut ch2_baseband = vec![0.0f32; 2400];
        ch2_baseband.extend(synthesize_flex_4fsk_burst(
            baseband_sr,
            1_122_334,
            "CH2 929.73862 MHZ FLEX A OK",
            1_556_677,
            "CH2 929.73862 MHZ FLEX B OK",
            0.005,
        ));
        ch2_baseband.extend(vec![0.0f32; 2400]);

        let offset_ch2 = (DEFAULT_SECONDARY_FREQ_HZ - DEFAULT_PRIMARY_FREQ_HZ) as f64; // +126,120 Hz
        let composite_iq = synthesize_multi_channel_sdrconnect_iq_frame(
            &[(0.0, &ch1_baseband), (offset_ch2, &ch2_baseband)],
            iq_sr,
            baseband_sr,
        );

        let mut multi_demod = MultiChannelIqDemodulator::new(
            &[DEFAULT_PRIMARY_FREQ_HZ, DEFAULT_SECONDARY_FREQ_HZ],
            DEFAULT_PRIMARY_FREQ_HZ,
            iq_sr,
            baseband_sr,
            1.0,
            false,
            -85.0,
            true,
            20_000.0,
        );

        let chunks = multi_demod.process_binary_frame(&composite_iq);
        assert_eq!(chunks.len(), 2);

        let msgs_ch1 = run_stream_in_wasapi_chunks(baseband_sr, &chunks[0].samples);
        let msgs_ch2 = run_stream_in_wasapi_chunks(baseband_sr, &chunks[1].samples);

        assert_eq!(msgs_ch1.len(), 1);
        assert_eq!(msgs_ch1[0].capcode, "1234567");
        assert_eq!(msgs_ch1[0].message, "CH1 929.61250 MHZ PARALLEL DECODE OK");

        assert_eq!(msgs_ch2.len(), 2);
        assert_eq!(msgs_ch2[0].capcode, "1122334");
        assert_eq!(msgs_ch2[0].message, "CH2 929.73862 MHZ FLEX A OK");
        assert_eq!(msgs_ch2[1].capcode, "1556677");
        assert_eq!(msgs_ch2[1].message, "CH2 929.73862 MHZ FLEX B OK");
    }

    #[test]
    fn test_multi_fragment_flex_reassembly_and_capcode_catalog_db() {
        let bch = BchDecoder::new();
        let mut qual = RxQualityTracker::new();
        let mut group_frame = [-1i16; 16];
        let mut group_capcodes: [Vec<u64>; 16] = std::array::from_fn(|_| Vec::new());
        let mut pending: Vec<PendingFlexFragment> = Vec::new();
        let mut out: Vec<DecodedPagerMessage> = Vec::new();

        // Build Frame 1: Fragment 1 (fragment_num = 3, continued = true)
        let words_f1 = build_flex_alpha_phase_words_frag(
            1_445_566,
            "PART 1 OF LONG HOSPITAL DISPATCH - ",
            3,
            true,
        );
        let mut phase_f1 = FlexPhaseBuffer::new('A');
        phase_f1.frame_words = words_f1;

        FlexDecoder::parse_phase_frame_with_pending(
            &phase_f1,
            &mut group_frame,
            &mut group_capcodes,
            &mut pending,
            10,
            3200,
            &bch,
            &mut qual,
            true,
            true,
            true,
            &mut out,
        );
        assert!(
            out.is_empty(),
            "First fragment with continued=true should be buffered, not emitted prematurely"
        );
        assert_eq!(pending.len(), 1);

        // Build Frame 2: Fragment 2 (fragment_num = 0, continued = false)
        let words_f2 = build_flex_alpha_phase_words_frag(
            1_445_566,
            "PART 2 COMPLETED REASSEMBLY OK",
            0,
            false,
        );
        let mut phase_f2 = FlexPhaseBuffer::new('A');
        phase_f2.frame_words = words_f2;

        FlexDecoder::parse_phase_frame_with_pending(
            &phase_f2,
            &mut group_frame,
            &mut group_capcodes,
            &mut pending,
            11,
            3200,
            &bch,
            &mut qual,
            true,
            true,
            true,
            &mut out,
        );
        assert!(pending.is_empty());
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].capcode, "1445566");
        assert_eq!(
            out[0].message,
            "PART 1 OF LONG HOSPITAL DISPATCH - PART 2 COMPLETED REASSEMBLY OK"
        );
        assert_eq!(out[0].fragment_info.as_deref(), Some("2 frags"));

        // Verify CapcodeCatalogDb tracking, groupcall recording, deletion, and clear
        let tmp_db_path = PathBuf::from("test_unit_capcode_db.dat");
        let _ = std::fs::remove_file(&tmp_db_path);
        let mut db = CapcodeCatalogDb::new(tmp_db_path.clone());
        let mut m = out[0].clone();
        m.freq_hz = Some(DEFAULT_SECONDARY_FREQ_HZ);
        m.group_address = Some("2029568".to_string());
        db.record_message(&m);
        db.save_to_disk().expect("Should save catalog DB to disk");

        let mut db_reloaded = CapcodeCatalogDb::load_or_default(&tmp_db_path);
        assert_eq!(db_reloaded.total_capcodes(), 1);
        let prof = db_reloaded.profiles.get("1445566").expect("Profile must exist");
        assert_eq!(prof.message_count, 1);
        assert_eq!(prof.seen_bitrates, vec![3200]);
        assert_eq!(prof.frequencies_hz, vec![DEFAULT_SECONDARY_FREQ_HZ]);
        assert_eq!(prof.flex_group_addresses, vec!["2029568".to_string()]);
        assert_eq!(prof.messages.len(), 1);

        assert!(db_reloaded.remove_capcode("1445566"));
        assert_eq!(db_reloaded.total_capcodes(), 0);
        let _ = std::fs::remove_file(&tmp_db_path);
    }

    #[test]
    fn test_pdw_holdover_fixes_long_capcode_biw_soft_recovery_and_numeric_bounds() {
        use crate::dsp::RxQualityTracker;
        use crate::flex::{FlexDecoder, FlexPhaseBuffer};

        let bch = BchDecoder::new();
        let mut qual = RxQualityTracker::new();
        let mut phase = FlexPhaseBuffer::new('A');

        let encode_flex_word = |phase: &mut FlexPhaseBuffer,
                                word_idx: usize,
                                w: u32,
                                err_bits: &[usize]| {
            let mut d = [0u8; 21];
            for j in 0..21 {
                d[j] = if ((w >> j) & 1) != 0 { 0 } else { 1 };
            }
            let mut cw = bch.encode(&d);
            let mut conf = [1.0f32; 32];
            for &eb in err_bits {
                cw[eb] ^= 1;
                conf[eb] = 0.08;
            }
            phase.raw_codewords[word_idx] = cw;
            phase.raw_conf[word_idx] = conf;
            let hard = bch.decode_hard(&cw);
            let mut cc: u32 = 0;
            for j in 0..21 {
                cc >>= 1;
                if hard.bits[j] == 0 {
                    cc ^= 0x10_0000;
                }
            }
            if !hard.valid {
                cc ^= 0x40_0000;
            }
            phase.frame_words[word_idx] = cc;
            phase.word_errors[word_idx] = hard.errors as u8;
        };

        let add_xsum = |w_no_xsum: u32| -> u32 {
            let w = w_no_xsum & 0x1f_fff0;
            let sum = ((w >> 4) & 0x0f)
                + ((w >> 8) & 0x0f)
                + ((w >> 12) & 0x0f)
                + ((w >> 16) & 0x0f)
                + ((w >> 20) & 0x01);
            let xsum = (0x0f - (sum & 0x0f)) & 0x0f;
            w | xsum
        };

        let biw = add_xsum((4 << 10) | (0 << 8));
        encode_flex_word(&mut phase, 0, biw, &[2, 9, 17]);
        assert!(!FlexDecoder::xsumchk(phase.frame_words[0]), "Hard BCH should fail on 3-bit BIW error");

        encode_flex_word(&mut phase, 1, 100, &[1, 8, 15]);
        encode_flex_word(&mut phase, 2, 0x1f_fffe, &[]);

        let vec_long = add_xsum((5 << 4) | (7 << 7) | (2 << 14));
        encode_flex_word(&mut phase, 4, vec_long, &[3, 11, 19]);
        encode_flex_word(&mut phase, 5, 3 << 11, &[]);

        let w7 = 0x7f | ((b'L' as u32) << 7) | ((b'O' as u32) << 14);
        let w8 = (b'N' as u32) | ((b'G' as u32) << 7) | ((b'9' as u32) << 14);
        encode_flex_word(&mut phase, 7, w7, &[]);
        encode_flex_word(&mut phase, 8, w8, &[]);

        encode_flex_word(&mut phase, 3, 1_234_567 + 32_768, &[]);
        let vec_num = add_xsum((3 << 4) | (9 << 7) | (0 << 14));
        encode_flex_word(&mut phase, 6, vec_num, &[]);
        let w9: u32 = (1 << 2) | (2 << 6) | (3 << 10) | (4 << 14) | (0x0C << 18);
        encode_flex_word(&mut phase, 9, w9, &[]);

        let mut group_frame = [-1i16; 16];
        let mut group_capcodes: [Vec<u64>; 16] = std::array::from_fn(|_| Vec::new());
        let mut out = Vec::new();
        FlexDecoder::parse_phase_frame_for_test(
            &phase,
            &mut group_frame,
            &mut group_capcodes,
            0,
            6400,
            &bch,
            &mut qual,
            true,
            true,
            true,
            &mut out,
        );

        assert_eq!(out.len(), 2, "Expected both Long-Address Alpha and Short-Address Numeric messages, got {:?}", out);
        assert_eq!(out[0].capcode, "002101348");
        assert_eq!(out[0].message, "LONG9");
        assert_eq!(out[1].capcode, "1234567");
        assert_eq!(out[1].msg_type, "StNUM");
        assert_eq!(out[1].message, "1234");
    }
}
