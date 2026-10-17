# PDW-RS: Modern 48 kHz POCSAG & FLEX (2/4-FSK) Paging Decoder in Rust

`pdw-rs` is a high-performance, multi-threaded Rust reimplementation of **PDW (Paging Decoder for Windows)** designed for modern 64-bit systems, Software Defined Radios (SDRconnect, SDR++, SDR#), and 48,000 Hz / 44,100 Hz audio pipelines.

All legacy 16/32-bit Windows hardware dependencies (`COMPRT.VXD`, `slicer.sys`, Win16/Win32 `waveIn` polling timers, and RS-232 serial bit-banging) have been replaced with a lock-free `cpal` audio engine, sample-rate-adaptive DSP state machines, full 4-level FSK slicing across all four FLEX phases (A, B, C, and D), and an event-driven dark-mode `egui` desktop interface that idles at **under 1% CPU usage**.

## Supported Protocols & Modulation Matrix

`pdw-rs` runs POCSAG and FLEX demodulators concurrently on the same audio stream without locking into a single baud rate:

| Protocol | Speed (bps) | Symbol Rate (sps) | Modulation | Active Phases | Samples / Symbol @ 48 kHz | Samples / Symbol @ 44.1 kHz |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **POCSAG-512** | 512 bps | 512 sps | 2-FSK | Single Channel | 93.75 | 86.13 |
| **POCSAG-1200** | 1200 bps | 1200 sps | 2-FSK | Single Channel | 40.00 | 36.75 |
| **POCSAG-2400** | 2400 bps | 2400 sps | 2-FSK | Single Channel | 20.00 | 18.38 |
| **FLEX-1600** | 1600 bps | 1600 sps | 2-FSK | Phase A | 30.00 | 27.56 |
| **FLEX-3200 (2-FSK)** | 3200 bps | 3200 sps | 2-FSK | Phases A + C | 15.00 | 13.78 |
| **FLEX-3200 (4-FSK)** | 3200 bps | 1600 sps | 4-FSK | Phases A + B | 30.00 | 27.56 |
| **FLEX-6400 (4-FSK)** | 6400 bps | 3200 sps | 4-FSK | Phases A + B + C + D | 15.00 | 13.78 |

## Architectural & DSP Improvements Over Legacy C++ PDW

### 1. Lock-Free Audio Capture & Zero Buffer Leaks (`src/audio.rs`)
In the original C++ PDW (`sound_in.cpp:267–342`), soundcard capture relied on a 100 ms Win32 `WM_TIMER` on the main UI thread polling an unsynchronized `buffers_ready` counter. Under heavy UI load or after several hours of continuous operation, `WaveHeader` descriptors could be skipped or leaked, eventually starving the decoder of audio until the application was restarted.

`pdw-rs` isolates capture, DSP, and UI rendering into three separate threads:
- **Real-Time Audio Thread (`cpal`)**: Captures native `f32`, `i16`, or `u8` PCM audio via Windows WASAPI, DirectSound, ALSA, or CoreAudio and pushes mono-downmixed sample buffers over a bounded lock-free `crossbeam-channel`.
- **Direct WASAPI `[Loopback]` Capture**: In addition to standard recording inputs (such as `CABLE Output`), `pdw-rs` can capture directly from any Windows playback endpoint (`Speakers [Loopback]`, `CABLE Input [Loopback]`), eliminating the need for third-party virtual audio cables if desired.
- **Dedicated DSP Worker Thread**: Processes sample chunks deterministically without blocking on disk I/O or GUI rendering.

### 2. True 4-Level FSK Slicing Unlocking FLEX Phases B & D (`src/flex.rs`)
Original PDW only supported 4-level FSK when using an external hardware 4-level serial data slicer connected to a physical RS-232 COM port. In soundcard mode (`sound_in.cpp:609`), original PDW hardcoded:

```c
frame_flex(atb_bit ? 0 : 3);
```

Because symbols `0` (`11_2`) and `3` (`00_2`) both map their least-significant bit to `1` in PDW's Gray-code mapping, original PDW fed a constant stream of `1`s into FLEX **Phase B** and **Phase D** on soundcard input—silently discarding 50% of all FLEX-3200 (4-FSK) and FLEX-6400 (4-FSK) traffic.

`pdw-rs` unlocks full 4-FSK soundcard decoding:
- Every FLEX frame begins with a 1600-baud **2-FSK** header (`Sync-1`, 16-bit marker, 32-bit `FIW`, and `Sync-2`) transmitted at the outer deviation levels ($\pm 4.8\text{ kHz}$).
- `FlexDecoder::slice_symbol()` measures the outer symbol magnitude during the 2-FSK header of each frame and sets the inner/outer 4-FSK decision boundary at $0.60 \times \text{outer\_ref}$ (halfway between the $\pm 1.6\text{ kHz}$ inner symbol level and $\pm 4.8\text{ kHz}$ outer symbol level).
- During the 11 interleaved data blocks, `pdw-rs` slices all four Gray-coded symbols (`0, 1, 2, 3`) and decodes **Phases A, B, C, and D** simultaneously.

### 3. Exact 48 kHz / 44.1 kHz Clock Recovery & `cross_over` Timing (`src/dsp.rs` & `src/gui.rs`)
- **Cross-Chunk `WatchCtr` Continuity**: In `sound_in.cpp:622`, `WatchCtr = -1` was reset at the end of every soundcard buffer, introducing a 1-bit symbol-clock phase slip at every audio buffer boundary. `pdw-rs` preserves fractional `watch_ctr` phase continuously (`watch_ctr -= chunk.len() as f64`) across arbitrary WASAPI buffer sizes.
- **Exact Bit-49 (`flex_bc == 39`) `1600 -> 3200` Baud Crossover**: For 3200-symbol/sec frames (`FLEX-3200 2-FSK` and `FLEX-6400 4-FSK`), `pdw-rs` triggers the 1600-to-3200 baud rate switch at `flex_bc == 39` (right after the 32-bit Frame Info Word) and advances through the 78.4-sample scaled `cross_over` window so Block 0 is sampled at the exact center of the first 3200-baud symbol (`0.9 * watch_step`), preventing Phase A/C swap.
- **64-Sample Block-Max Automatic Gain Control (AGC)**: Tracks the peak signal envelope over 64-sample (1.33 ms = 2-symbol) blocks while idle, normalizing quiet virtual-cable inputs within the first 20 bits of the 64-bit preamble, and **locks the gain multiplier constant** (`flex_blk > 0 || flex_bc > 0 || pocbit > 0`) the instant a frame syncs so 4-FSK inner/outer amplitude ratios remain 100% linear throughout the frame.

### 4. Fixed BCH(31,21) Parity & Guarded Chase-2 Soft-Decision FEC (`src/bch.rs`)
- **Hard-Decision BCH(31,21) + Even Parity Fix**: In original PDW's `Misc.cpp:ecd()` (`line 2387`), the parity check logic (`if (errors == 1) parity ^= 1`) inverted overall parity only when `errors == 1`. As a result, all **210 combinations** of `(1 data-bit error + 1 ECC parity-bit error)` failed the overall parity check and were falsely rejected as uncorrectable (`errors = 3`). `pdw-rs` computes the syndromes and checks overall 32-bit even parity on the corrected codeword directly, correcting all $\binom{31}{1} + \binom{31}{2} = 496$ single- and double-bit error patterns in $O(1)$ table lookups.
- **Guarded Chase-2 Soft-Decision Recovery**: For uncorrectable alphanumeric data words, `pdw-rs` records per-bit analog slicing confidence and tests single- and double-bit flips among the 4 least-confident bits in the 32-bit codeword, accepting a soft recovery only when the resulting codeword passes BCH(31,21) + even parity **and** all three unpacked 7-bit characters are valid printable ASCII (`32..=126` or `ETX`).

### 5. POCSAG Framing & Message Classification Fixes (`src/pocsag.rs`)
- **End-of-Burst Pending Message Flush (`frame_reset`)**: In `sound_in.cpp:600`, original PDW called `pocsag.frame('X')` (ASCII `88`) when `pocbit` counted down to `0`, whereas `Pocsag.cpp:94` checked `if (bit == -1)`. Because `'X' != -1`, original PDW failed to flush the final message of a POCSAG transmission until a subsequent burst arrived. `pdw-rs` explicitly calls `frame_reset()` as soon as `pocbit == 0`.
- **Fixed Alphanumeric vs. Numeric Heuristic**: Original PDW's `GetMessageType()` (`Pocsag.cpp:365`) penalized candidate alphanumeric messages whenever `(20 * wordc) % 6 != 0`, which caused valid 2-word and 5-word alphanumeric messages (such as `"HELLO"`) to be misclassified as garbage numeric strings. `pdw-rs` inspects printable ASCII vs. `ETX`/`EOT` termination without the modulo-6 penalty.
- **50-Transition Preamble Gate**: Retains `sound_in.cpp`'s 50-transition preamble duration filter (`pocbit`) for 512, 1200, and 2400 baud so random FM discriminator noise and FLEX frames never trigger false POCSAG syncs or spurious polarity flips.

### 6. Low-CPU Dark-Mode GUI & Ciphertext Filtering (`src/gui.rs` & `src/config.rs`)
- **Sub-1% Idle CPU Usage**: Uses event-driven GUI repainting (`ctx.request_repaint()`) that wakes the UI thread immediately when a message is decoded and otherwise refreshes background telemetry at 1 FPS when the oscilloscope is turned off.
- **Toggleable Live Oscilloscope**: A `Scope` checkbox on the top toolbar lets you enable the 10 FPS live waveform monitor while tuning your SDR and disable it for `< 1%` CPU 24/7 background logging.
- **Encrypted / Non-Human-Readable Message Filter**: Optionally hides raw `BINARY` hex dumps (FLEX Vector Type 6) and Base64-encoded ciphertext (`Hide Encrypted` checkbox on the toolbar) while preserving all human-readable dispatches, URLs, and email-to-page alerts.
- **Legacy `FILTERS.INI` & Logfile Compatibility**: Imports and exports original PDW `FILTERS.INI` files and writes daily `Logfiles/YYMMDD.log` and `Logfiles/YYMMDD.flt` files in standard PDW column format.

## SDR & Audio Setup Guide (SDRconnect / SDR++ / SDR#)

When feeding `pdw-rs` from an SDR application (such as **SDRconnect**, **SDR++**, or **SDR#**) via **VB-Audio Virtual Cable** or **WASAPI Loopback**, use the following receiver settings for 95–99.9% decode rates:

1. **Demodulator Mode & Bandwidth**:
   - Set mode to **NFM** (Narrowband FM).
   - Set filter bandwidth to **12.5 kHz** or **15.0 kHz** (standard FLEX/POCSAG signals use $\pm 4.8\text{ kHz}$ deviation, requiring ~12.5–16 kHz Carson bandwidth).
2. **Disable De-Emphasis & Audio Filtering**:
   - **Turn OFF De-emphasis** in your SDR software. An FM de-emphasis filter integrates flat-topped FSK square waves into low-amplitude triangle waves and destroys 1600/3200-baud symbol transitions.
   - **Turn OFF Audio Noise Reduction (NR)** and **Voice High-Pass/Low-Pass Filters** (4-FSK FLEX requires clean baseband response down to near-DC).
   - **Turn OFF Squelch** (or set squelch very low so the opening 64-bit preamble of a burst is never clipped).
3. **RF Gain**:
   - Adjust your SDR's **RF Gain** so paging bursts rise at least 25–40 dB above the noise floor (achieving full FM quieting). When `Scope` is checked in `pdw-rs`, a healthy paging burst should appear as a crisp, flat-topped square/4-level wave.
4. **Sample Rate Matching (`48000 Hz` or `44100 Hz`)**:
   - In Windows **Sound Control Panel (`mmsys.cpl`)**, ensure both **Playback $\rightarrow$ `CABLE Input` $\rightarrow$ Properties $\rightarrow$ Advanced** and **Recording $\rightarrow$ `CABLE Output` $\rightarrow$ Properties $\rightarrow$ Advanced** are set to the same sample rate (preferably **`16 bit, 48000 Hz (DVD Quality)`**).
   - Alternatively, in `pdw-rs` $\rightarrow$ **`Audio Setup`**, select any **`[Loopback]`** output device to tap directly into your SDR's output stream via Windows WASAPI Loopback without configuring a virtual recording cable.

## Building & Running

### Prerequisites
- [Rust toolchain](https://www.google.com/search?q=https%3A%2F%2Frustup.rs%2F) (`rustc` / `cargo` 1.75+)

### Run the Test Suite
The repository includes an end-to-end synthetic signal test suite that verifies POCSAG (512, 1200, 2400 baud), FLEX-3200 4-FSK (Phases A+B), FLEX-6400 4-FSK (Phases A+B+C+D across the 1600-to-3200 baud crossover), automatic polarity inversion (`Pol: INV` / `Pol: NORM`), low-amplitude 2-bar AGC recovery, and ciphertext filtering at both **44,100 Hz** and **48,000 Hz**:

```bash
cargo test --release
```

### Build & Launch the Application

```bash
cargo run --release
```

On first launch, you can immediately verify the decoder pipeline without an SDR attached by clicking **`Test POCSAG-1200`** or **`Test FLEX 4-FSK (Phases A+B)`** on the top toolbar.

## Repository Structure

| File | Description |
| :--- | :--- |
| `src/main.rs` | Application entry point and end-to-end 44.1 kHz / 48 kHz synthetic burst integration tests |
| `src/audio.rs` | Lock-free `cpal` soundcard & WASAPI `[Loopback]` capture engine, plus POCSAG & 4-FSK FLEX test signal synthesizers |
| `src/dsp.rs` | 48 kHz / 44.1 kHz `AtbSlicer` (`Audio_To_Bits` port), 14 PDW threshold presets, 64-sample block AGC, and `RxQualityTracker` |
| `src/bch.rs` | Algebraic `BCH(31,21)` + even parity hard-decision decoder and ASCII-guarded Chase-2 soft-decision FEC |
| `src/pocsag.rs` | Multi-rate POCSAG (512 / 1200 / 2400 baud) framing, sync correlator, and alphanumeric/numeric unpacker |
| `src/flex.rs` | Full 2-FSK & 4-FSK FLEX (1600 / 3200 / 6400 bps, Phases A, B, C, D) and ReFLEX state machine |
| `src/config.rs` | `pdw_config.toml` persistence, filter matching engine, duplicate blocker, ciphertext detector, daily logger, and `FILTERS.INI` importer/exporter |
| `src/gui.rs` | Low-CPU dark-mode `eframe`/`egui` desktop interface, dual Monitor/Filter tables, live oscilloscope, and setup dialogs |

## License

Released under the **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**, in keeping with the open-source license of the original PDW project.
