# PDW-RS v0.2.7: Modern 48 kHz POCSAG & FLEX (2/4-FSK) Paging Decoder in Rust

`pdw-rs` is a high-performance, multi-threaded Rust reimplementation of **PDW (Paging Decoder for Windows)** designed for modern 64-bit systems, Software Defined Radios (**SDRconnect**, **SDR++**, **SDR#**), 48,000 Hz / 44,100 Hz soundcard pipelines, and live desktop and mobile browser monitoring.

All legacy 16/32-bit Windows hardware dependencies (`COMPRT.VXD`, `slicer.sys`, Win16/Win32 `waveIn` polling timers, and RS-232 serial bit-banging) have been replaced with a lock-free `cpal` audio engine, a direct **SDRconnect WebSocket IQ client with parallel multi-frequency NCO downconversion**, sample-rate-adaptive DSP state machines, full 4-level FSK slicing across all four FLEX phases (A, B, C, and D), multi-fragment FLEX message reassembly, a persistent **Capcode Directory & Message Catalog Database**, an event-driven dark-mode `egui` desktop interface that idles at **under 1% CPU usage**, and a **built-in HTTP + Server-Sent Events (SSE) live web server** (`port 8008` by default) with local browser caching, incremental delta synchronization, and a responsive mobile dispatch card layout.

## What's New in v0.2.7

1. **Parallel Multi-Frequency SDRconnect Monitoring (`<= 2.0 MHz` Span)**:
   - Monitors multiple paging frequencies in parallel within your receiver's **2.0 MHz** bandwidth, defaulting to **`929.61250 MHz`** (`929_612_500 Hz`) and **`929.73862 MHz`** (`929_738_620 Hz`).
   - Each enabled frequency runs its own isolated NCO mixer (`MultiChannelIqDemodulator`), Butterworth channel filter, polar FM discriminator, and `ChannelDecoderPipeline` (`AtbSlicer`, `PocsagChannelDecoder`, and `FlexDecoder`), stamping every decoded message with its exact RF frequency (`Freq (MHz)` column and card pill in both the desktop and web interfaces).
   - Automatic **2.000 MHz Bandwidth Validation**: Both the desktop **Input Setup** dialog and the web **Parallel Freqs** drawer allow adding, toggling, labeling, and removing frequencies while enforcing that all simultaneously enabled channels remain within `2.000 MHz` of each other.
2. **Multi-Fragment FLEX Message Reassembly (`src/flex.rs`)**:
   - Reassembles long multi-frame FLEX alphanumeric (`ALPHA`) and `SECURE` dispatches across consecutive frames using the FLEX continued flag (bit 10) and 2-bit fragment counter (`bits 11..12`), tagging reassembled dispatches with `[N frags]` while automatically flushing partial fragments if a continuation frame is lost over the air.
3. **Capcode Directory Tracker & Message Catalog Database (`src/catalog.rs`)**:
   - Automatically profiles every valid capcode observed over the air in a fast local database file (`pdw_capcode_db.dat`) while keeping classic flat daily logfiles (`Logfiles/YYMMDD.log` and `Logfiles/YYMMDD.flt`) completely intact.
   - Records **first appearance**, **last appearance**, **total message count**, **observed baud rates**, **modes**, **RF frequencies**, **user-editable alias/label**, **assigned FLEX groupcall temporary addresses (`2029568–2029583`)**, and the **chronological message history** associated with each capcode.
   - Click or tap any message or capcode to inspect its directory profile in the bottom inspector, or open **`Capcode Catalog (N)`** on the top toolbar (in both the desktop GUI and the live web interface) to search capcodes, browse all messages organized by capcode, edit aliases, delete individual capcode entries, or clear the entire database.
4. **Responsive Mobile Dispatch Card Interface (`src/web.rs`)**:
   - Automatically switches to a multi-line wrapped **Dispatch Card Feed** on mobile viewports (`<= 820px`, or when toggled manually via `View: Auto / Cards / Table`) so full message text, capcodes, RF frequencies, and badges are immediately readable on phones without horizontal scrolling.
   - Includes touch-safe DOM update guards during high-speed `FLEX-6400` bursts, full-row and full-card tap delegation, a collapsible **`RF & Tools ▾`** mobile header drawer, and a slide-up **Capcode Directory & Message Inspector** bottom sheet.

## What Was Added in v0.2.6

1. **Built-In Live HTTP + SSE Web Server (`src/web.rs`)**:
   - Serves a real-time, dark-mode web dashboard on **port `8008`** by default (`http://localhost:8008/` and `0.0.0.0:8008` for LAN/Tailscale access) using pure Rust standard-library networking (`std::net::TcpListener`) and `crossbeam-channel` with zero external web-crate bloat.
   - **Runtime Toggle & Port Configuration**: Enable or disable the web server and change its listening port (`1..=65535`) on the fly from either the main toolbar (`Web` checkbox + `:8008` port control + clickable URL copy badge) or the **PDW Decoder & Display Options** dialog (`web_server_enabled`, `web_server_port`, and `web_server_bind_addr` in `pdw_config.toml`).
2. **Local Browser Caching & Delta Bootstrap Sync**:
   - **Instant `localStorage` Hydration (`< 5 ms`)**: Caches decoded messages in `localStorage` (`pdw_rs_msg_cache_v1`) alongside the server's `session_id` and highest sequence number `last_seq` (`pdw_rs_meta_v1`), rendering your message history immediately on page load before network round-trips finish.
   - **Delta-Only Network Sync (`/api/bootstrap?session_id=<sid>&since_seq=<seq>`)**: On page refresh or reconnect, the browser requests only messages with `seq > last_seq` (`delta_mode: true`).

## Supported Protocols & Modulation Matrix

`pdw-rs` runs POCSAG and FLEX demodulators concurrently on each monitored channel without locking into a single baud rate:

| Protocol | Speed (bps) | Symbol Rate (sps) | Modulation | Active Phases | Samples / Symbol @ 48 kHz | Samples / Symbol @ 44.1 kHz |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **POCSAG-512** | 512 bps | 512 sps | 2-FSK | Single Channel | 93.75 | 86.13 |
| **POCSAG-1200** | 1200 bps | 1200 sps | 2-FSK | Single Channel | 40.00 | 36.75 |
| **POCSAG-2400** | 2400 bps | 2400 sps | 2-FSK | Single Channel | 20.00 | 18.38 |
| **FLEX-1600** | 1600 bps | 1600 sps | 2-FSK | Phase A | 30.00 | 27.56 |
| **FLEX-3200 (2-FSK)** | 3200 bps | 3200 sps | 2-FSK | Phases A + C | 15.00 | 13.78 |
| **FLEX-3200 (4-FSK)** | 3200 bps | 1600 sps | 4-FSK | Phases A + B | 30.00 | 27.56 |
| **FLEX-6400 (4-FSK)** | 6400 bps | 3200 sps | 4-FSK | Phases A + B + C + D | 15.00 | 13.78 |

## Building & Running

### Prerequisites
- [Rust toolchain](https://www.google.com/search?q=https%3A%2F%2Frustup.rs%2F) (`rustc` / `cargo` 1.75+)

### Run the Test Suite
The repository includes an end-to-end synthetic signal and web server test suite that verifies POCSAG (512, 1200, 2400 baud), FLEX-3200 4-FSK (Phases A+B), FLEX-6400 4-FSK (Phases A+B+C+D across the 1600-to-3200 baud crossover), parallel multi-frequency SDRconnect IQ decoding (`929.61250 MHz` + `929.73862 MHz`), multi-fragment FLEX reassembly, Capcode Catalog persistence (`pdw_capcode_db.dat`), 5-bit Chase-2 soft-decision BCH recovery, and web server `since_seq` delta bootstrap synchronization:

```bash
cargo test --release
```

### Build & Launch the Application

```bash
cargo run --release
```

## Repository Structure

| File | Description |
| :--- | :--- |
| `src/main.rs` | Application entry point and end-to-end synthetic burst, multi-frequency IQ, fragment reassembly, and catalog integration tests |
| `src/audio.rs` | Lock-free `cpal` soundcard capture engine, plus POCSAG & 4-FSK FLEX test signal synthesizers |
| `src/sdrconnect.rs` | Direct SDRconnect WebSocket IQ/Audio client, `MultiChannelIqDemodulator` parallel NCO DDC bank, and polar FM discriminator |
| `src/dsp.rs` | 48 kHz / 44.1 kHz `AtbSlicer` (`Audio_To_Bits` port), 14 PDW threshold presets, envelope tracker, and `RxQualityTracker` |
| `src/bch.rs` | Algebraic `BCH(31,21)` + even parity hard-decision decoder and 5-weak-bit Chase-2 soft-decision FEC |
| `src/pocsag.rs` | Multi-rate POCSAG (512 / 1200 / 2400 baud) framing, sync correlator, and alphanumeric/numeric unpacker |
| `src/flex.rs` | Full 2-FSK & 4-FSK FLEX (1600 / 3200 / 6400 bps, Phases A, B, C, D), multi-fragment reassembler, and groupcall tracker |
| `src/catalog.rs` | Persistent Capcode Directory Tracker & Message Catalog Database (`pdw_capcode_db.dat`) with individual deletion and full clear |
| `src/config.rs` | `pdw_config.toml` persistence (parallel frequencies, `<= 2.0 MHz` span validation, web server settings), filter engine, and daily logger |
| `src/gui.rs` | Low-CPU dark-mode `eframe`/`egui` desktop interface, parallel frequency manager, Capcode Catalog window, and setup dialogs |
| `src/web.rs` | Built-in HTTP + SSE live web server (`port 8008` default), responsive mobile card UI, Capcode Catalog & Frequency APIs, and `localStorage` caching |

## License

Released under the **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**, in keeping with the open-source license of the original PDW project.