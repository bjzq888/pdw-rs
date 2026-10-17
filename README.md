# PDW-RS v0.2.6: Modern 48 kHz POCSAG & FLEX (2/4-FSK) Paging Decoder in Rust

`pdw-rs` is a high-performance, multi-threaded Rust reimplementation of **PDW (Paging Decoder for Windows)** designed for modern 64-bit systems, Software Defined Radios (**SDRconnect**, **SDR++**, **SDR#**), 48,000 Hz / 44,100 Hz soundcard pipelines, and live browser-based monitoring.

All legacy 16/32-bit Windows hardware dependencies (`COMPRT.VXD`, `slicer.sys`, Win16/Win32 `waveIn` polling timers, and RS-232 serial bit-banging) have been replaced with a lock-free `cpal` audio engine, a direct **SDRconnect WebSocket IQ client and quadrature FM discriminator**, sample-rate-adaptive DSP state machines, full 4-level FSK slicing across all four FLEX phases (A, B, C, and D), an event-driven dark-mode `egui` desktop interface that idles at **under 1% CPU usage**, and a **built-in HTTP + Server-Sent Events (SSE) live web server** (`port 8008` by default) with local browser caching and incremental delta synchronization.

## What's New in v0.2.6

1. **Built-In Live HTTP + SSE Web Server (`src/web.rs`)**:
   - Serves a real-time, dark-mode web dashboard on **port `8008`** by default (`http://localhost:8008/` and `0.0.0.0:8008` for LAN access) using pure Rust standard-library networking (`std::net::TcpListener`) and `crossbeam-channel` with zero external web-crate bloat.
   - **Runtime Toggle & Port Configuration**: Enable or disable the web server and change its listening port (`1..=65535`) on the fly from either the main toolbar (`Web` checkbox + `:8008` port control + clickable URL copy badge) or the **PDW Decoder & Display Options** dialog (`web_server_enabled`, `web_server_port`, and `web_server_bind_addr` in `pdw_config.toml`). Toggling off or changing the port immediately wakes the listener via loopback connect and cleanly rebinds without restarting `pdw-rs`.
2. **Local Browser Caching & Delta Bootstrap Sync**:
   - **Instant `localStorage` Hydration (`< 5 ms`)**: The web client caches decoded messages in `localStorage` (`pdw_rs_msg_cache_v1`) alongside the server's `session_id` and highest sequence number `last_seq` (`pdw_rs_meta_v1`), rendering your message history immediately on page load before network round-trips finish.
   - **Configurable Browser Cache Limit**: Choose between `Off`, `250`, `500`, `1,000` (default), `2,500`, or `5,000` cached messages directly in the web toolbar, complete with automatic `QuotaExceededError` LRU trimming and a one-click **Clear Cache** button.
   - **Delta-Only Network Sync (`/api/bootstrap?session_id=<sid>&since_seq=<seq>`)**: On page refresh or reconnect, the browser requests only messages with `seq > last_seq` (`delta_mode: true`), eliminating redundant transfers of already-cached dispatches.
   - **Sub-Millisecond SSE Push (`/api/events`)**: Decoded POCSAG and FLEX pages and 2 Hz throttled receiver telemetry (`RX-Q`, active mode, polarity, DC/deviation, and SDRconnect VFO/RSSI/AFC) stream live over a persistent `text/event-stream` connection into `requestAnimationFrame`-batched DOM updates.
3. **Full Web & Desktop UI Parity**:
   - Both the desktop GUI (`src/gui.rs`) and web dashboard (`src/web.rs`) provide **Split View (Monitor + Filtered)**, **Monitor Only**, and **Filtered Only** modes, clickable column sorting (`▲` / `▼`), live substring search, one-click capcode isolation, a bottom word-wrapped **Message Inspector** drawer, CSV export, and remote **Test POCSAG-1200** / **Test FLEX 4-FSK** signal injection buttons.

## Supported Protocols & Modulation Matrix

`pdw-rs` runs POCSAG and FLEX demodulators concurrently on the same baseband stream without locking into a single baud rate:

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

### 1. Dual Input Engine: Lock-Free Soundcard (`src/audio.rs`) & Direct SDRconnect IQ (`src/sdrconnect.rs`)
In the original C++ PDW (`sound_in.cpp:267–342`), soundcard capture relied on a 100 ms Win32 `WM_TIMER` on the main UI thread polling an unsynchronized `buffers_ready` counter. Under heavy UI load or after several hours of continuous operation, `WaveHeader` descriptors could be skipped or leaked, eventually starving the decoder of audio until the application was restarted.

`pdw-rs` isolates capture, DSP, web streaming, and UI rendering into dedicated threads:
- **Real-Time Soundcard Thread (`cpal`)**: Captures native `f32`, `i16`, or `u8` PCM audio via Windows WASAPI, DirectSound, ALSA, or CoreAudio and pushes mono-downmixed sample buffers over a bounded lock-free `crossbeam-channel`.
- **Direct SDRconnect WebSocket IQ Client (`src/sdrconnect.rs`)**: Connects directly to SDRconnect's built-in WebSocket server (`ws://127.0.0.1:5454/` by default), enables the VFO IQ stream (`iq_stream_enable`, Binary Frame Type `0x0002` / `0x0005` interleaved `i16` IQ samples) or Audio stream (Type `0x0001`), automatically disables NFM de-emphasis and audio limiters, and performs complex NCO mixing, 2nd-order Butterworth complex channel filtering (`12.5–32 kHz`), preamble-gated outer-symbol AFC, and DC-coupled polar FM discrimination (`atan2`) followed by a 4,200 Hz Butterworth post-discriminator low-pass filter.
- **Dedicated DSP Worker Thread**: Processes sample chunks deterministically without blocking on disk I/O, network sockets, or GUI rendering.

### 2. True 4-Level FSK Slicing Unlocking FLEX Phases B & D (`src/flex.rs`)
Original PDW only supported 4-level FSK when using an external hardware 4-level serial data slicer connected to a physical RS-232 COM port. In soundcard mode (`sound_in.cpp:609`), original PDW hardcoded:

```c
frame_flex(atb_bit ? 0 : 3);
```

Because symbols `0` (`11_2`) and `3` (`00_2`) both map their least-significant bit to `1` in PDW's Gray-code mapping, original PDW fed a constant stream of `1`s into FLEX **Phase B** and **Phase D** on soundcard input—silently discarding 50% of all FLEX-3200 (4-FSK) and FLEX-6400 (4-FSK) traffic.

`pdw-rs` unlocks full 4-FSK soundcard and IQ decoding:
- Every FLEX frame begins with a 1600-baud **2-FSK** header (`Sync-1`, 16-bit marker, 32-bit `FIW`, and `Sync-2`) transmitted at the outer deviation levels ($\pm 4.8\text{ kHz}$).
- `FlexDecoder::slice_symbol()` measures the positive and negative outer symbol levels independently during the sincronized 2-FSK header of each frame, removes any residual per-burst carrier DC offset, and sets the inner/outer 4-FSK decision boundary at $0.60 \times \text{outer\_ref}$ at 1600 sps ($0.56 \times \text{outer\_ref}$ at 3200 sps to compensate for post-discriminator roll-off).
- During the 11 interleaved data blocks, `pdw-rs` slices all four Gray-coded symbols (`0, 1, 2, 3`) and decodes **Phases A, B, C, and D** simultaneously.

### 3. Exact 48 kHz / 44.1 kHz Clock Recovery & `cross_over` Timing (`src/dsp.rs` & `src/gui.rs`)
- **Cross-Chunk `WatchCtr` Continuity**: In `sound_in.cpp:622`, `WatchCtr = -1` was reset at the end of every soundcard buffer, introducing a 1-bit symbol-clock phase slip at every audio buffer boundary. `pdw-rs` preserves fractional `watch_ctr` phase continuously (`watch_ctr -= chunk.len() as f64`) across arbitrary WASAPI and IQ buffer sizes.
- **Exact Bit-49 (`flex_bc == 39`) `1600 -> 3200` Baud Crossover**: For 3200-symbol/sec frames (`FLEX-3200 2-FSK` and `FLEX-6400 4-FSK`), `pdw-rs` triggers the 1600-to-3200 baud rate switch at `flex_bc == 39` (right after the 32-bit Frame Info Word) and advances through the 78.4-sample scaled `cross_over` window so Block 0 is sampled at the exact center of the first 3200-baud symbol (`0.9 * watch_step`), preventing Phase A/C swap.
- **Damped PLL in 4-FSK Data Blocks**: Uses a damped phase error update (`0.25 * err` within `0.35 * watch_step`) during multi-level 4-FSK payloads so asymmetric inner-to-outer zero-crossings ($+1.6\text{ kHz} \leftrightarrow -4.8\text{ kHz}$) never skew the sampling instant away from the center of the eye diagram.

### 4. Fixed BCH(31,21) Parity & 5-Bit Chase-2 Soft-Decision FEC (`src/bch.rs`)
- **Hard-Decision BCH(31,21) + Even Parity Fix**: In original PDW's `Misc.cpp:ecd()` (`line 2387`), the parity check logic (`if (errors == 1) parity ^= 1`) inverted overall parity only when `errors == 1`. As a result, all **210 combinations** of `(1 data-bit error + 1 ECC parity-bit error)` failed the overall parity check and were falsely rejected as uncorrectable (`errors = 3`). `pdw-rs` computes the syndromes and checks overall 32-bit even parity on the corrected codeword directly, correcting all $\binom{31}{1} + \binom{31}{2} = 496$ single- and double-bit error patterns in $O(1)$ table lookups.
- **5-Weak-Bit Chase-2 Soft-Decision Recovery**: Using per-bit analog slicing confidence (`decode_soft_filtered`), `pdw-rs` searches the $2^5 - 1 = 31$ flip patterns of the 5 least-confident bits in corrupted 32-bit codewords. It recovers up to 3- and 4-bit errors across FLEX Block Information Words (`BIW`), Vector words (validated against the 4-bit hardware checksum `xsumchk`), Address words, Alphanumeric data words (validated for printable ASCII `32..=126` or `ETX`), and POCSAG message codewords.

### 5. POCSAG & FLEX Framing Fixes (`src/pocsag.rs` & `src/flex.rs`)
- **End-of-Burst Pending Message Flush (`frame_reset`)**: In `sound_in.cpp:600`, original PDW called `pocsag.frame('X')` (ASCII `88`) when `pocbit` counted down to `0`, whereas `Pocsag.cpp:94` checked `if (bit == -1)`. Because `'X' != -1`, original PDW failed to flush the final message of a POCSAG transmission until a subsequent burst arrived. `pdw-rs` explicitly calls `frame_reset()` as soon as `pocbit == 0`.
- **9-Digit Long FLEX Capcodes & Numeric Bounds**: Fixes original PDW's long-address `w2` truncation and StNUM/SfNUM/NuNUM off-by-one trailing idle word leak, and stops character unpacking immediately on `ETX` (`0x03`).
- **50-Transition Preamble Gate**: Retains `sound_in.cpp`'s 50-transition preamble duration filter (`pocbit`) for 512, 1200, and 2400 baud so random FM discriminator noise and FLEX frames never trigger false POCSAG syncs or spurious polarity flips.

### 6. Low-CPU Dark-Mode GUI, Live Web Monitor & Ciphertext Filtering (`src/gui.rs`, `src/web.rs`, `src/config.rs`)
- **Sub-1% Idle CPU Usage**: Uses event-driven GUI repainting (`ctx.request_repaint()`) that wakes the UI thread immediately when a message is decoded and otherwise refreshes background telemetry at 1 FPS when the oscilloscope is turned off.
- **Built-In Live Web Server (`http://localhost:8008/`)**: Watch live decoded traffic from any local or LAN browser with instant `localStorage` caching, `since_seq` delta synchronization, live SSE push, and interactive search/sorting.
- **Encrypted / Non-Human-Readable Message Filter**: Optionally hides raw `BINARY` hex dumps (FLEX Vector Type 6) and Base64-encoded ciphertext (`Hide Encrypted` checkbox on the toolbar) while preserving all human-readable dispatches, URLs, and email-to-page alerts.
- **Legacy `FILTERS.INI` & Logfile Compatibility**: Supports adding, editing in-place (`Edit` / `Update #N`), and deleting filter rules, imports/exports original PDW `FILTERS.INI` files, and writes daily `Logfiles/YYMMDD.log` and `Logfiles/YYMMDD.flt` files in standard PDW column format.

## SDR & Audio Setup Guide (SDRconnect / SDR++ / SDR#)

### Option A: Direct SDRconnect WebSocket IQ Stream (Recommended for SDRconnect)
1. In SDRconnect, ensure the WebSocket server is enabled (default port `5454`) and tune your receiver to the target paging frequency.
2. In `pdw-rs`, click **`Input Setup`**, select **`SDRconnect (Direct WebSocket IQ)`**, verify Host `127.0.0.1` and Port `5454`, and click **`Apply & Reconnect SDRconnect`**.
3. `pdw-rs` automatically streams raw 16-bit IQ samples over WebSocket, disables NFM de-emphasis, and demodulates 2-FSK and 4-FSK signals internally with carrier-gated AFC.

### Option B: Soundcard / Virtual Audio Cable (SDR++ / SDR# / Discriminator Tap)
1. **Demodulator Mode & Bandwidth**:
   - Set mode to **NFM** (Narrowband FM) with filter bandwidth between **12.5 kHz** and **16.0 kHz**.
2. **Disable De-Emphasis & Audio Filtering**:
   - **Turn OFF De-emphasis**, **Audio Noise Reduction (NR)**, and **Voice High-Pass/Low-Pass Filters** (4-FSK FLEX requires clean baseband response down to near-DC).
   - **Turn OFF Squelch** (or set squelch very low so the opening 64-bit preamble of a burst is never clipped).
3. **Sample Rate Matching (`48000 Hz` or `44100 Hz`)**:
   - In Windows **Sound Control Panel (`mmsys.cpl`)**, ensure both **Playback $\rightarrow$ `CABLE Input`** and **Recording $\rightarrow$ `CABLE Output`** are set to **`16 bit, 48000 Hz (DVD Quality)`**.

## Live Web Server Configuration (`port 8008`)

By default, `pdw-rs` starts its live web server on `http://0.0.0.0:8008/` (`http://localhost:8008/`):
- **Quick Toolbar Controls**: Use the **`Web`** checkbox and **`:8008`** port selector in the second toolbar row to toggle the server or change ports at runtime, or click the green `http://localhost:8008/` button to copy the URL.
- **Decoder Options Dialog**: Open **`Decoder Options`** $\rightarrow$ **`Live Web Server & Browser Caching`** to toggle the server, reset to port `8008`, or switch between **`0.0.0.0 (LAN & Localhost)`** and **`127.0.0.1 (Localhost Only)`**.
- **`pdw_config.toml` Fields**:
  ```toml
  web_server_enabled = true
  web_server_port = 8008
  web_server_bind_addr = "0.0.0.0"
  ```

## Building & Running

### Prerequisites
- [Rust toolchain](https://www.google.com/search?q=https%3A%2F%2Frustup.rs%2F) (`rustc` / `cargo` 1.75+)

### Run the Test Suite
The repository includes an end-to-end synthetic signal and web server test suite that verifies POCSAG (512, 1200, 2400 baud), FLEX-3200 4-FSK (Phases A+B), FLEX-6400 4-FSK (Phases A+B+C+D across the 1600-to-3200 baud crossover), SDRconnect Type-2 IQ demodulation, 5-bit Chase-2 soft-decision BCH recovery, ciphertext filtering, and web server `since_seq` delta bootstrap synchronization:

```bash
cargo test --release
```

### Build & Launch the Application

```bash
cargo run --release
```

On first launch, you can immediately verify both the desktop decoder and the live web interface at `http://localhost:8008/` without an SDR attached by clicking **`Test POCSAG-1200`** or **`Test FLEX 4-FSK (Phases A+B)`**.

## Repository Structure

| File | Description |
| :--- | :--- |
| `src/main.rs` | Application entry point and end-to-end 44.1 kHz / 48 kHz synthetic burst & IQ integration tests |
| `src/audio.rs` | Lock-free `cpal` soundcard capture engine, plus POCSAG & 4-FSK FLEX test signal synthesizers |
| `src/sdrconnect.rs` | Direct SDRconnect WebSocket IQ/Audio client, Butterworth complex channel filter, carrier-gated AFC, and polar FM discriminator |
| `src/dsp.rs` | 48 kHz / 44.1 kHz `AtbSlicer` (`Audio_To_Bits` port), 14 PDW threshold presets, envelope tracker, and `RxQualityTracker` |
| `src/bch.rs` | Algebraic `BCH(31,21)` + even parity hard-decision decoder and 5-weak-bit Chase-2 soft-decision FEC |
| `src/pocsag.rs` | Multi-rate POCSAG (512 / 1200 / 2400 baud) framing, sync correlator, and alphanumeric/numeric unpacker |
| `src/flex.rs` | Full 2-FSK & 4-FSK FLEX (1600 / 3200 / 6400 bps, Phases A, B, C, D) and ReFLEX state machine |
| `src/config.rs` | `pdw_config.toml` persistence (including web server settings), filter matching engine, duplicate blocker, ciphertext detector, daily logger, and `FILTERS.INI` importer/exporter |
| `src/gui.rs` | Low-CPU dark-mode `eframe`/`egui` desktop interface, sortable Monitor/Filter tables, message inspector, web server controls, and setup dialogs |
| `src/web.rs` | Built-in HTTP + SSE live web server (`port 8008` default), incremental `since_seq` delta bootstrap endpoint, and single-page browser monitor with `localStorage` caching |

## License

Released under the **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**, in keeping with the open-source license of the original PDW project.
