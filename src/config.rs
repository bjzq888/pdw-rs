//! Configuration, Parallel Multi-Frequency List (`<= 2.0 MHz` Span), Filter Matching Engine,
//! Duplicate Blocker, Statistics, and Legacy `PDW.ini` / `FILTERS.INI` Importer/Exporter.

use crate::dsp::{AudioPreset, DEFAULT_SAMPLE_RATE};
use crate::pocsag::DecodedPagerMessage;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::Path;

/// Maximum allowed frequency spread (2.0 MHz) across simultaneously enabled SDR frequencies.
pub const MAX_PARALLEL_SPAN_HZ: u64 = 2_000_000;

/// Default primary paging frequency: 929.61250 MHz (929,612.50 kHz).
pub const DEFAULT_PRIMARY_FREQ_HZ: u64 = 929_612_500;

/// Default secondary paging frequency: 929.73862 MHz (929,738.62 kHz).
pub const DEFAULT_SECONDARY_FREQ_HZ: u64 = 929_738_620;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MonitoredFrequency {
    pub enabled: bool,
    pub freq_hz: u64,
    pub label: String,
}

impl MonitoredFrequency {
    pub fn mhz_str(&self) -> String {
        format!("{:.5}", (self.freq_hz as f64) / 1_000_000.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InputSource {
    Soundcard,
    SdrConnectIq,
}

impl Default for InputSource {
    fn default() -> Self {
        Self::Soundcard
    }
}

impl InputSource {
    pub fn label(self) -> &'static str {
        match self {
            InputSource::Soundcard => "Soundcard (System / Virtual Audio Cable)",
            InputSource::SdrConnectIq => "SDRconnect (Direct WebSocket IQ Stream)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterType {
    Flex = 1,
    Pocsag = 2,
    Text = 3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterAction {
    DisplayBoth = 0,
    Reject = 1,
    MonitorOnly = 2,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FilterRule {
    pub filter_type: FilterType,
    pub capcode: String,
    pub label: String,
    pub text: String,
    pub match_exact_msg: bool,
    pub action: FilterAction,
    pub label_enabled: bool,
    pub label_color: usize,
    pub sep_file_enabled: bool,
    pub sep_filename: String,
    pub hit_counter: u32,
    pub last_hit_date: String,
    pub last_hit_time: String,
}

impl Default for FilterRule {
    fn default() -> Self {
        Self {
            filter_type: FilterType::Flex,
            capcode: String::new(),
            label: String::new(),
            text: String::new(),
            match_exact_msg: false,
            action: FilterAction::DisplayBoth,
            label_enabled: true,
            label_color: 0,
            sep_file_enabled: false,
            sep_filename: String::new(),
            hit_counter: 0,
            last_hit_date: String::new(),
            last_hit_time: String::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct AppConfig {
    // Input source selection: Soundcard vs. Direct SDRconnect WebSocket IQ
    pub input_source: InputSource,

    // Soundcard settings
    pub audio_device_name: Option<String>,
    pub sample_rate: u32,

    // SDRconnect WebSocket IQ settings & Parallel Multi-Frequency List
    pub sdrconnect_host: String,
    pub sdrconnect_port: u16,
    pub sdrconnect_iq_gain: f32,
    pub sdrconnect_squelch_enabled: bool,
    pub sdrconnect_squelch_dbfs: f32,
    pub sdrconnect_afc_enabled: bool,
    pub sdrconnect_channel_bw_hz: f32,
    pub sdrconnect_tune_on_connect: bool,
    pub sdrconnect_vfo_freq_hz: u64,
    pub sdrconnect_use_audio_stream: bool,
    pub monitored_frequencies: Vec<MonitoredFrequency>,

    pub audio_preset: AudioPreset,
    pub invert_polarity: bool,
    pub auto_dc_tracking: bool,
    pub manual_center_offset: f32,
    pub manual_hysteresis: f32,

    // Decoder toggles
    pub decode_pocsag_512: bool,
    pub decode_pocsag_1200: bool,
    pub decode_pocsag_2400: bool,
    pub pocsag_fnu_mode: bool,
    pub pocsag_show_both: bool,

    pub decode_flex_1600: bool,
    pub decode_flex_3200: bool,
    pub decode_flex_6400: bool,
    pub convert_flex_si: bool,

    pub show_tone_only: bool,
    pub show_numeric: bool,
    pub show_binary: bool,
    pub hide_encrypted_alpha: bool,
    pub show_scope: bool,
    pub show_filter_pane: bool,
    pub filter_window_only: bool,
    pub block_duplicates: bool,
    pub block_duplicate_seconds: u64,

    // Logging
    pub monitor_logfile_enabled: bool,
    pub filter_logfile_enabled: bool,
    pub log_dir: String,

    // UI split ratio (percentage for upper monitor pane)
    pub upper_pane_percent: f32,

    // Built-in Live Web Server settings (default: enabled on port 8008)
    pub web_server_enabled: bool,
    pub web_server_port: u16,
    pub web_server_bind_addr: String,

    // Filter rules
    pub filters: Vec<FilterRule>,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            input_source: InputSource::Soundcard,
            audio_device_name: None,
            sample_rate: DEFAULT_SAMPLE_RATE,
            sdrconnect_host: crate::sdrconnect::DEFAULT_SDRCONNECT_HOST.to_string(),
            sdrconnect_port: crate::sdrconnect::DEFAULT_SDRCONNECT_PORT,
            sdrconnect_iq_gain: 1.0,
            sdrconnect_squelch_enabled: false,
            sdrconnect_squelch_dbfs: -75.0,
            sdrconnect_afc_enabled: true,
            sdrconnect_channel_bw_hz: 20_000.0,
            sdrconnect_tune_on_connect: false,
            sdrconnect_vfo_freq_hz: DEFAULT_PRIMARY_FREQ_HZ,
            sdrconnect_use_audio_stream: false,
            monitored_frequencies: default_monitored_frequencies(DEFAULT_PRIMARY_FREQ_HZ),
            audio_preset: AudioPreset::Earphone1,
            invert_polarity: false,
            auto_dc_tracking: true,
            manual_center_offset: 0.0,
            manual_hysteresis: 0.02,
            decode_pocsag_512: true,
            decode_pocsag_1200: true,
            decode_pocsag_2400: true,
            pocsag_fnu_mode: false,
            pocsag_show_both: false,
            decode_flex_1600: true,
            decode_flex_3200: true,
            decode_flex_6400: true,
            convert_flex_si: true,
            show_tone_only: true,
            show_numeric: true,
            show_binary: false,
            hide_encrypted_alpha: true,
            show_scope: false,
            show_filter_pane: true,
            filter_window_only: false,
            block_duplicates: false,
            block_duplicate_seconds: 120,
            monitor_logfile_enabled: false,
            filter_logfile_enabled: false,
            log_dir: "Logfiles".to_string(),
            upper_pane_percent: 65.0,
            web_server_enabled: true,
            web_server_port: crate::web::DEFAULT_WEB_PORT,
            web_server_bind_addr: crate::web::DEFAULT_WEB_BIND_ADDR.to_string(),
            filters: Vec::new(),
        }
    }
}

pub fn default_monitored_frequencies(primary_hz: u64) -> Vec<MonitoredFrequency> {
    let f1 = if primary_hz > 0 {
        primary_hz
    } else {
        DEFAULT_PRIMARY_FREQ_HZ
    };
    let mut list = vec![MonitoredFrequency {
        enabled: true,
        freq_hz: f1,
        label: format!("Channel 1 ({:.5} MHz)", (f1 as f64) / 1_000_000.0),
    }];
    if f1 != DEFAULT_SECONDARY_FREQ_HZ
        && f1.abs_diff(DEFAULT_SECONDARY_FREQ_HZ) <= MAX_PARALLEL_SPAN_HZ
    {
        list.push(MonitoredFrequency {
            enabled: true,
            freq_hz: DEFAULT_SECONDARY_FREQ_HZ,
            label: "Channel 2 (929.73862 MHz)".to_string(),
        });
    }
    list
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterVerdict {
    Unmatched,
    MatchedBoth(usize),
    MatchedFilterOnly(usize),
    MatchedMonitorOnly(usize),
    Rejected(usize),
}

impl AppConfig {
    pub fn load_or_default(path: &Path) -> Self {
        if let Ok(contents) = fs::read_to_string(path) {
            if let Ok(mut cfg) = toml::from_str::<AppConfig>(&contents) {
                if cfg.sdrconnect_channel_bw_hz < 12_500.0 {
                    cfg.sdrconnect_channel_bw_hz = 20_000.0;
                }
                if cfg.sdrconnect_iq_gain > 2.0 || cfg.sdrconnect_iq_gain < 0.25 {
                    cfg.sdrconnect_iq_gain = 1.0;
                }
                if cfg.web_server_port == 0 {
                    cfg.web_server_port = crate::web::DEFAULT_WEB_PORT;
                }
                if cfg.web_server_bind_addr.trim().is_empty() {
                    cfg.web_server_bind_addr = crate::web::DEFAULT_WEB_BIND_ADDR.to_string();
                }
                if cfg.monitored_frequencies.is_empty() {
                    cfg.monitored_frequencies =
                        default_monitored_frequencies(cfg.sdrconnect_vfo_freq_hz);
                } else if cfg.monitored_frequencies.len() == 1
                    && cfg.monitored_frequencies[0].freq_hz == DEFAULT_PRIMARY_FREQ_HZ
                {
                    cfg.monitored_frequencies.push(MonitoredFrequency {
                        enabled: true,
                        freq_hz: DEFAULT_SECONDARY_FREQ_HZ,
                        label: "Channel 2 (929.73862 MHz)".to_string(),
                    });
                }
                cfg.enforce_2mhz_span_limit();
                return cfg;
            }
        }
        let mut cfg = Self::default();
        for ini_name in ["PDW.ini", "pdw.ini", "PDW.INI"] {
            let p = Path::new(ini_name);
            if p.exists() {
                cfg.import_legacy_pdw_ini(p);
                break;
            }
        }
        cfg
    }

    /// Returns all currently enabled frequencies in Hz (always non-empty; falls back to `sdrconnect_vfo_freq_hz`).
    pub fn enabled_frequencies_hz(&self) -> Vec<u64> {
        let mut freqs: Vec<u64> = self
            .monitored_frequencies
            .iter()
            .filter(|f| f.enabled && f.freq_hz > 0)
            .map(|f| f.freq_hz)
            .collect();
        if freqs.is_empty() {
            let fallback = if self.sdrconnect_vfo_freq_hz > 0 {
                self.sdrconnect_vfo_freq_hz
            } else {
                DEFAULT_PRIMARY_FREQ_HZ
            };
            freqs.push(fallback);
        }
        freqs
    }

    /// Validates whether enabling or adding `candidate_hz` stays within the 2.0 MHz receiver bandwidth
    /// relative to all other enabled frequencies. Returns `Ok(new_span_hz)` or an explanation `Err`.
    pub fn validate_frequency_within_2mhz(
        &self,
        candidate_hz: u64,
        ignore_idx: Option<usize>,
    ) -> Result<u64, String> {
        if candidate_hz < 100_000 {
            return Err("Frequency must be >= 0.100 MHz".to_string());
        }
        let mut min_hz = candidate_hz;
        let mut max_hz = candidate_hz;
        for (i, item) in self.monitored_frequencies.iter().enumerate() {
            if Some(i) == ignore_idx || !item.enabled || item.freq_hz == 0 {
                continue;
            }
            if item.freq_hz < min_hz {
                min_hz = item.freq_hz;
            }
            if item.freq_hz > max_hz {
                max_hz = item.freq_hz;
            }
        }
        let span = max_hz - min_hz;
        if span > MAX_PARALLEL_SPAN_HZ {
            Err(format!(
                "{:.5} MHz is {:.3} MHz away from enabled channels (max parallel receiver bandwidth is 2.000 MHz).",
                (candidate_hz as f64) / 1_000_000.0,
                (span as f64) / 1_000_000.0
            ))
        } else {
            Ok(span)
        }
    }

    /// Current span in Hz across all enabled frequencies (`max_hz - min_hz`), and the midpoint LO frequency.
    pub fn active_span_and_midpoint_hz(&self) -> (u64, u64) {
        let enabled = self.enabled_frequencies_hz();
        let min_hz = enabled.iter().copied().min().unwrap_or(DEFAULT_PRIMARY_FREQ_HZ);
        let max_hz = enabled.iter().copied().max().unwrap_or(DEFAULT_PRIMARY_FREQ_HZ);
        (max_hz - min_hz, (min_hz + max_hz) / 2)
    }

    /// Ensure any loaded configuration does not violate the 2.0 MHz parallel span limit.
    pub fn enforce_2mhz_span_limit(&mut self) {
        let mut min_hz: Option<u64> = None;
        let mut max_hz: Option<u64> = None;
        for item in &mut self.monitored_frequencies {
            if !item.enabled || item.freq_hz == 0 {
                continue;
            }
            let new_min = min_hz.map(|m| m.min(item.freq_hz)).unwrap_or(item.freq_hz);
            let new_max = max_hz.map(|m| m.max(item.freq_hz)).unwrap_or(item.freq_hz);
            if new_max - new_min > MAX_PARALLEL_SPAN_HZ {
                item.enabled = false;
            } else {
                min_hz = Some(new_min);
                max_hz = Some(new_max);
            }
        }
        if let Some(first_en) = self.monitored_frequencies.iter().find(|f| f.enabled) {
            self.sdrconnect_vfo_freq_hz = first_en.freq_hz;
        }
    }

    pub fn import_legacy_pdw_ini(&mut self, path: &Path) {
        if let Ok(content) = fs::read_to_string(path) {
            for line in content.lines() {
                if let Some((k, v)) = line.trim().split_once('=') {
                    let key = k.trim();
                    let val = v.trim();
                    match key {
                        "InvertData" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.invert_polarity = n != 0;
                            }
                        }
                        "AudioConfiguration" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.audio_preset = AudioPreset::from_pdw_sac_type(n);
                            }
                        }
                        "ShowTone" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.show_tone_only = n != 0;
                            }
                        }
                        "ShowNumeric" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.show_numeric = n != 0;
                            }
                        }
                        "ShowBinary" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.show_binary = n != 0;
                            }
                        }
                        "ConvertSI" => {
                            if let Ok(n) = val.parse::<i32>() {
                                self.convert_flex_si = n != 0;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Ok(serialized) = toml::to_string_pretty(self) {
            fs::write(path, serialized)?;
        }
        Ok(())
    }

    /// Evaluate a decoded message against all configured filters (`Check_4_Filtermatch` in `Misc.cpp`).
    pub fn evaluate_message(&mut self, msg: &mut DecodedPagerMessage) -> FilterVerdict {
        let is_flex = msg.mode.starts_with("FLEX") || msg.mode.starts_with("ReFLEX");
        let is_pocsag = msg.mode.starts_with("POCSAG");

        for (idx, rule) in self.filters.iter_mut().enumerate() {
            let addr_matched = match rule.filter_type {
                FilterType::Flex if is_flex => wildcard_match(&msg.capcode, &rule.capcode),
                FilterType::Pocsag if is_pocsag => {
                    if let Some((cap_pat, fn_pat)) = rule.capcode.split_once('-') {
                        wildcard_match(&msg.capcode, cap_pat) && msg.mode.ends_with(fn_pat)
                    } else {
                        wildcard_match(&msg.capcode, &rule.capcode)
                    }
                }
                FilterType::Text => true,
                _ => false,
            };

            if !addr_matched {
                continue;
            }

            let text_required = rule.filter_type == FilterType::Text || !rule.text.is_empty();
            let text_matched = if !text_required {
                true
            } else {
                match_filter_text(&msg.message, &rule.text, rule.match_exact_msg)
            };

            if text_matched {
                rule.hit_counter = rule.hit_counter.saturating_add(1);
                rule.last_hit_date = msg.date.clone();
                rule.last_hit_time = msg.time.clone();

                if rule.label_enabled && !rule.label.is_empty() {
                    msg.filter_label = Some(expand_filter_label(&rule.label, &msg.capcode));
                    msg.filter_color_idx = Some(rule.label_color);
                }

                return match rule.action {
                    FilterAction::Reject => FilterVerdict::Rejected(idx),
                    FilterAction::MonitorOnly => FilterVerdict::MatchedMonitorOnly(idx),
                    FilterAction::DisplayBoth => {
                        if self.filter_window_only {
                            FilterVerdict::MatchedFilterOnly(idx)
                        } else {
                            FilterVerdict::MatchedBoth(idx)
                        }
                    }
                };
            }
        }

        FilterVerdict::Unmatched
    }

    /// Import legacy PDW `FILTERS.INI` file (`ReadFilters` in `PDW.cpp:9686`).
    pub fn import_legacy_filters_ini(&mut self, path: &Path) -> std::io::Result<usize> {
        let content = fs::read_to_string(path)?;
        let mut count = 0usize;
        for line in content.lines() {
            let trimmed = line.trim();
            if !trimmed.starts_with("Filter") || trimmed.starts_with("FilterCount") {
                continue;
            }
            if let Some((_, rhs)) = trimmed.split_once('=') {
                if let Some(rule) = parse_legacy_filter_line(rhs) {
                    self.filters.push(rule);
                    count += 1;
                }
            }
        }
        Ok(count)
    }

    /// Export filters to legacy PDW `FILTERS.INI` format (`WriteFilters` in `PDW.cpp:10133`).
    pub fn export_legacy_filters_ini(&self, path: &Path) -> std::io::Result<()> {
        let mut f = fs::File::create(path)?;
        writeln!(f, "[Filter]\n\nFilterCount={}\n", self.filters.len())?;
        for (i, r) in self.filters.iter().enumerate() {
            let ftype = r.filter_type as u8;
            let reject_flag = match r.action {
                FilterAction::Reject => 1,
                FilterAction::MonitorOnly => 2,
                FilterAction::DisplayBoth => 0,
            };
            let label_col = if r.label_enabled { r.label_color + 1 } else { 0 };
            let mut sep_flags = 0u8;
            if r.sep_file_enabled {
                sep_flags |= 0x01;
            }
            if r.match_exact_msg {
                sep_flags |= 0x10;
            }
            writeln!(
                f,
                "Filter{}={},\"{}\",\"{}\",\"{}\",{},0,0,{},{},\"{}\",{},{},{}",
                i + 1,
                ftype,
                r.capcode,
                r.label,
                r.text,
                reject_flag,
                label_col,
                sep_flags,
                r.sep_filename,
                r.hit_counter,
                r.last_hit_date,
                r.last_hit_time
            )?;
        }
        Ok(())
    }

    pub fn log_message(&self, msg: &DecodedPagerMessage, is_filtered: bool, filter_idx: Option<usize>) {
        if !self.monitor_logfile_enabled && !self.filter_logfile_enabled {
            return;
        }
        let _ = fs::create_dir_all(&self.log_dir);
        let date_tag = chrono::Local::now().format("%y%m%d").to_string();
        let label_suffix = msg
            .filter_label
            .as_ref()
            .map(|l| format!(" - {} -", l))
            .unwrap_or_default();
        let line = format!(
            "{} {} {} {} {} {} {}{}\n",
            msg.capcode,
            msg.time,
            msg.date,
            msg.mode,
            msg.msg_type,
            msg.bitrate,
            msg.message,
            label_suffix
        );

        if self.monitor_logfile_enabled {
            let p = Path::new(&self.log_dir).join(format!("{}.log", date_tag));
            if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(p) {
                let _ = f.write_all(line.as_bytes());
            }
        }
        if is_filtered && self.filter_logfile_enabled {
            let p = Path::new(&self.log_dir).join(format!("{}.flt", date_tag));
            if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(p) {
                let _ = f.write_all(line.as_bytes());
            }
        }
        if let Some(idx) = filter_idx {
            if let Some(rule) = self.filters.get(idx) {
                if rule.sep_file_enabled && !rule.sep_filename.is_empty() {
                    let p = Path::new(&self.log_dir).join(&rule.sep_filename);
                    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(p) {
                        let _ = f.write_all(line.as_bytes());
                    }
                }
            }
        }
    }
}

fn wildcard_match(text: &str, pattern: &str) -> bool {
    if pattern.is_empty() {
        return false;
    }
    let t_bytes = text.as_bytes();
    let p_bytes = pattern.as_bytes();
    if t_bytes.len() != p_bytes.len() {
        return false;
    }
    t_bytes
        .iter()
        .zip(p_bytes.iter())
        .all(|(&tc, &pc)| pc == b'?' || tc.eq_ignore_ascii_case(&pc))
}

fn match_filter_text(msg: &str, pat: &str, exact: bool) -> bool {
    if pat.is_empty() {
        return true;
    }
    let msg_low = msg.to_lowercase();
    let pat_low = pat.to_lowercase();
    if exact {
        msg_low == pat_low
    } else if let Some(prefix) = pat_low.strip_prefix('^') {
        msg_low.starts_with(prefix)
    } else if pat_low.contains('&') {
        let mut search_pos = 0usize;
        for term in pat_low.split('&') {
            if term.is_empty() {
                continue;
            }
            if let Some(rel) = msg_low[search_pos..].find(term) {
                search_pos += rel + term.len();
            } else {
                return false;
            }
        }
        true
    } else {
        msg_low.contains(&pat_low)
    }
}

fn expand_filter_label(label: &str, capcode: &str) -> String {
    let cap_chars: Vec<char> = capcode.chars().collect();
    let mut out = String::with_capacity(label.len());
    let mut chars = label.chars().peekable();
    while let Some(ch) = chars.next() {
        if ch == '%' {
            while let Some(&next_ch) = chars.peek() {
                if let Some(digit) = next_ch.to_digit(10) {
                    if digit >= 1 && (digit as usize) <= cap_chars.len() {
                        out.push(cap_chars[(digit as usize) - 1]);
                        chars.next();
                        continue;
                    }
                }
                break;
            }
        } else {
            out.push(ch);
        }
    }
    out
}

fn parse_legacy_filter_line(rhs: &str) -> Option<FilterRule> {
    let mut parts = Vec::new();
    let mut current = String::new();
    let mut in_quotes = false;
    for ch in rhs.chars() {
        match ch {
            '"' => in_quotes = !in_quotes,
            ',' if !in_quotes => {
                parts.push(current.clone());
                current.clear();
            }
            _ => current.push(ch),
        }
    }
    parts.push(current);
    if parts.len() < 10 {
        return None;
    }

    let filter_type = match parts[0].trim() {
        "1" => FilterType::Flex,
        "2" => FilterType::Pocsag,
        "3" => FilterType::Text,
        _ => return None,
    };
    let reject_val: u8 = parts[4].trim().parse().unwrap_or(0);
    let action = if (reject_val & 1) != 0 {
        FilterAction::Reject
    } else if (reject_val & 2) != 0 {
        FilterAction::MonitorOnly
    } else {
        FilterAction::DisplayBoth
    };
    let label_col_raw: usize = parts[7].trim().parse().unwrap_or(0);
    let sep_raw: u8 = parts[8].trim().parse().unwrap_or(0);

    Some(FilterRule {
        filter_type,
        capcode: parts[1].clone(),
        label: parts[2].clone(),
        text: parts[3].clone(),
        match_exact_msg: (sep_raw & 0x10) != 0,
        action,
        label_enabled: label_col_raw > 0,
        label_color: label_col_raw.saturating_sub(1),
        sep_file_enabled: (sep_raw & 0x01) != 0,
        sep_filename: parts[9].clone(),
        hit_counter: parts.get(10).and_then(|s| s.trim().parse().ok()).unwrap_or(0),
        last_hit_date: parts.get(11).cloned().unwrap_or_default(),
        last_hit_time: parts.get(12).cloned().unwrap_or_default(),
    })
}

/// Duplicate message blocker (`BlockChecker` in `Misc.cpp:1141`).
#[derive(Clone, Debug, Default)]
pub struct DuplicateBlocker {
    history: VecDeque<(String, String, i64)>,
}

impl DuplicateBlocker {
    pub fn is_duplicate(&mut self, capcode: &str, message: &str, window_secs: u64) -> bool {
        if message.is_empty() || message == "TONE ONLY" || message == "TONE-ONLY" {
            return false;
        }
        let now = chrono::Local::now().timestamp();
        let cutoff = now - (window_secs as i64);

        while let Some(&(_, _, ts)) = self.history.front() {
            if ts < cutoff {
                self.history.pop_front();
            } else {
                break;
            }
        }

        if self
            .history
            .iter()
            .any(|(c, m, _)| c == capcode && m == message)
        {
            return true;
        }

        if self.history.len() >= 1000 {
            self.history.pop_front();
        }
        self.history
            .push_back((capcode.to_string(), message.to_string(), now));
        false
    }
}

/// Hourly and Daily message/character statistics (`hourly_stat` / `daily_stat` in `decode.cpp`).
#[derive(Clone, Debug, Default)]
pub struct StatsTracker {
    pub pocsag_512_alpha: u64,
    pub pocsag_512_num: u64,
    pub pocsag_1200_alpha: u64,
    pub pocsag_1200_num: u64,
    pub pocsag_2400_alpha: u64,
    pub pocsag_2400_num: u64,
    pub flex_1600_alpha: u64,
    pub flex_1600_num: u64,
    pub flex_3200_alpha: u64,
    pub flex_3200_num: u64,
    pub flex_6400_alpha: u64,
    pub flex_6400_num: u64,
    pub rejected_count: u64,
    pub blocked_duplicates: u64,
    pub encrypted_hidden_count: u64,
}

impl StatsTracker {
    pub fn record(&mut self, msg: &DecodedPagerMessage) {
        let is_alpha = msg.msg_type.contains("ALPHA") || msg.msg_type.contains("GROUP");
        if msg.mode.starts_with("POCSAG") {
            match msg.bitrate {
                512 => {
                    if is_alpha {
                        self.pocsag_512_alpha += 1;
                    } else {
                        self.pocsag_512_num += 1;
                    }
                }
                1200 => {
                    if is_alpha {
                        self.pocsag_1200_alpha += 1;
                    } else {
                        self.pocsag_1200_num += 1;
                    }
                }
                _ => {
                    if is_alpha {
                        self.pocsag_2400_alpha += 1;
                    } else {
                        self.pocsag_2400_num += 1;
                    }
                }
            }
        } else {
            match msg.bitrate {
                1600 => {
                    if is_alpha {
                        self.flex_1600_alpha += 1;
                    } else {
                        self.flex_1600_num += 1;
                    }
                }
                3200 => {
                    if is_alpha {
                        self.flex_3200_alpha += 1;
                    } else {
                        self.flex_3200_num += 1;
                    }
                }
                _ => {
                    if is_alpha {
                        self.flex_6400_alpha += 1;
                    } else {
                        self.flex_6400_num += 1;
                    }
                }
            }
        }
    }

    pub fn total_messages(&self) -> u64 {
        self.pocsag_512_alpha
            + self.pocsag_512_num
            + self.pocsag_1200_alpha
            + self.pocsag_1200_num
            + self.pocsag_2400_alpha
            + self.pocsag_2400_num
            + self.flex_1600_alpha
            + self.flex_1600_num
            + self.flex_3200_alpha
            + self.flex_3200_num
            + self.flex_6400_alpha
            + self.flex_6400_num
    }
}

/// Detects whether an Alphanumeric (`ALPHA` / `GROUP` / `SECURE`) message is human-readable text
/// vs. Base64-encoded ciphertext (such as Spok/USA Mobility T5/T52 encrypted pages) or unprintable gibberish.
pub fn is_human_readable_message(msg: &str, msg_type: &str) -> bool {
    if !matches!(msg_type, "ALPHA" | "GROUP" | "SECURE") {
        return true;
    }
    let s = msg.trim();
    if s.is_empty() {
        return false;
    }

    let char_count = s.chars().count();
    let weird_count = s
        .chars()
        .filter(|&c| matches!(c, '~' | '|' | '^' | '`' | '\\' | '{' | '}' | '$' | '%'))
        .count();
    if char_count >= 10 && (weird_count * 100 / char_count) >= 45 {
        return false;
    }

    let mut bad_chars = 0usize;
    let mut total_non_space = 0usize;
    for tok in s.split_whitespace() {
        let t_len = tok.len();
        total_non_space += t_len;
        if is_encrypted_or_gibberish_token(tok) {
            bad_chars += t_len;
        }
    }

    if total_non_space > 0 && (bad_chars * 100 / total_non_space) >= 50 {
        return false;
    }

    true
}

fn is_encrypted_or_gibberish_token(tok: &str) -> bool {
    let t_len = tok.len();
    if t_len < 14 {
        return false;
    }

    let low = tok.to_ascii_lowercase();
    if low.contains('@')
        || low.contains("http:")
        || low.contains("https:")
        || low.contains("www.")
        || low.contains(".com")
        || low.contains(".edu")
        || low.contains(".org")
        || low.contains(".net")
        || low.contains(".gov")
    {
        return false;
    }

    let bytes = tok.as_bytes();
    let mut upper = 0usize;
    let mut lower = 0usize;
    let mut digits = 0usize;
    let mut b64_sym = 0usize;
    let mut vowels = 0usize;

    for &b in bytes {
        if b.is_ascii_uppercase() {
            upper += 1;
        } else if b.is_ascii_lowercase() {
            lower += 1;
        } else if b.is_ascii_digit() {
            digits += 1;
        } else if matches!(b, b'+' | b'/' | b'=') {
            b64_sym += 1;
        }
        if matches!(b.to_ascii_lowercase(), b'a' | b'e' | b'i' | b'o' | b'u' | b'y') {
            vowels += 1;
        }
    }

    let other_sym = t_len.saturating_sub(upper + lower + digits + b64_sym);

    let mut suspicious_trans = 0usize;
    for pair in bytes.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let a_up = a.is_ascii_uppercase();
        let a_lo = a.is_ascii_lowercase();
        let a_dg = a.is_ascii_digit();
        let b_up = b.is_ascii_uppercase();
        let b_lo = b.is_ascii_lowercase();
        let b_dg = b.is_ascii_digit();

        if (a_lo && b_up)
            || ((a_up || a_lo) && b_dg)
            || (a_dg && (b_up || b_lo))
            || matches!(a, b'+' | b'/' | b'=')
            || matches!(b, b'+' | b'/' | b'=')
        {
            suspicious_trans += 1;
        }
    }

    if upper > 0 && lower > 0 && suspicious_trans >= 4 {
        return true;
    }
    if (upper + lower) > 0 && digits > 0 && suspicious_trans >= 5 {
        return true;
    }
    if t_len >= 24 && other_sym == 0 && upper > 0 && lower > 0 {
        return true;
    }
    if t_len >= 20 && (upper + lower) >= 16 && vowels * 10 < t_len {
        return true;
    }
    if other_sym * 2 >= t_len {
        return true;
    }

    false
}
