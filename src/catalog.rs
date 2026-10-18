//! Capcode Directory Tracker & Local Message Catalog Database (`catalog.rs`).
//!
//! Tracks every valid capcode observed over the air alongside its associated messages,
//! persisting to a dedicated local database file (`pdw_capcode_db.dat`) without affecting
//! the classic flat daily logfiles (`Logfiles/YYMMDD.log` and `Logfiles/YYMMDD.flt`).
//!
//! Recorded per-capcode intelligence:
//! - First appearance (`first_seen_date`, `first_seen_time`) and last appearance (`last_seen_date`, `last_seen_time`)
//! - Total message count (`message_count`)
//! - Last & all observed baud rates (`last_bitrate`, `seen_bitrates`) and protocol modes (`modes_seen`)
//! - Observed RF carrier frequencies (`frequencies_hz`)
//! - Assigned FLEX groupcall temporary addresses (`2029568`–`2029583` in `flex_group_addresses`)
//! - Optional user-editable alias / label (`alias`)
//! - Chronological message history (`messages`) associated with the capcode
//!
//! Supports removing individual capcode entries or clearing the entire database from both
//! the desktop GUI and the live web interface.

use crate::pocsag::DecodedPagerMessage;
use crate::web::escape_json_str;
use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

pub const DEFAULT_CATALOG_DB_FILENAME: &str = "pdw_capcode_db.dat";
pub const MAX_MESSAGES_PER_CAPCODE: usize = 500;

#[derive(Clone, Debug)]
pub struct CapcodeProfile {
    pub capcode: String,
    pub alias: String,
    pub first_seen_date: String,
    pub first_seen_time: String,
    pub last_seen_date: String,
    pub last_seen_time: String,
    pub message_count: u64,
    pub last_bitrate: u32,
    pub seen_bitrates: Vec<u32>,
    pub modes_seen: Vec<String>,
    pub frequencies_hz: Vec<u64>,
    pub flex_group_addresses: Vec<String>,
    pub messages: VecDeque<DecodedPagerMessage>,
}

impl CapcodeProfile {
    pub fn new(capcode: &str, date: &str, time: &str, bitrate: u32) -> Self {
        Self {
            capcode: capcode.to_string(),
            alias: String::new(),
            first_seen_date: date.to_string(),
            first_seen_time: time.to_string(),
            last_seen_date: date.to_string(),
            last_seen_time: time.to_string(),
            message_count: 0,
            last_bitrate: bitrate,
            seen_bitrates: Vec::new(),
            modes_seen: Vec::new(),
            frequencies_hz: Vec::new(),
            flex_group_addresses: Vec::new(),
            messages: VecDeque::new(),
        }
    }

    pub fn bitrates_display(&self) -> String {
        if self.seen_bitrates.is_empty() {
            if self.last_bitrate > 0 {
                format!("{}", self.last_bitrate)
            } else {
                "-".to_string()
            }
        } else {
            self.seen_bitrates
                .iter()
                .map(|b| b.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        }
    }

    pub fn modes_display(&self) -> String {
        if self.modes_seen.is_empty() {
            "-".to_string()
        } else {
            self.modes_seen.join(", ")
        }
    }

    pub fn frequencies_display(&self) -> String {
        if self.frequencies_hz.is_empty() {
            "Audio".to_string()
        } else {
            self.frequencies_hz
                .iter()
                .map(|&hz| format!("{:.5} MHz", (hz as f64) / 1_000_000.0))
                .collect::<Vec<_>>()
                .join(", ")
        }
    }

    pub fn groups_display(&self) -> String {
        if self.flex_group_addresses.is_empty() {
            "-".to_string()
        } else {
            self.flex_group_addresses.join(", ")
        }
    }

    pub fn summary_to_json(&self) -> String {
        let rates_json = self
            .seen_bitrates
            .iter()
            .map(|r| r.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let modes_json = self
            .modes_seen
            .iter()
            .map(|m| format!("\"{}\"", escape_json_str(m)))
            .collect::<Vec<_>>()
            .join(",");
        let freqs_json = self
            .frequencies_hz
            .iter()
            .map(|f| f.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let groups_json = self
            .flex_group_addresses
            .iter()
            .map(|g| format!("\"{}\"", escape_json_str(g)))
            .collect::<Vec<_>>()
            .join(",");
        let latest_msg = self
            .messages
            .back()
            .map(|m| m.message.as_str())
            .unwrap_or("");

        format!(
            "{{\"capcode\":\"{}\",\"alias\":\"{}\",\"first_seen_date\":\"{}\",\"first_seen_time\":\"{}\",\"last_seen_date\":\"{}\",\"last_seen_time\":\"{}\",\"message_count\":{},\"last_bitrate\":{},\"seen_bitrates\":[{}],\"modes_seen\":[{}],\"frequencies_hz\":[{}],\"flex_group_addresses\":[{}],\"stored_messages\":{},\"latest_message\":\"{}\"}}",
            escape_json_str(&self.capcode),
            escape_json_str(&self.alias),
            escape_json_str(&self.first_seen_date),
            escape_json_str(&self.first_seen_time),
            escape_json_str(&self.last_seen_date),
            escape_json_str(&self.last_seen_time),
            self.message_count,
            self.last_bitrate,
            rates_json,
            modes_json,
            freqs_json,
            groups_json,
            self.messages.len(),
            escape_json_str(latest_msg)
        )
    }

    pub fn detail_to_json(&self) -> String {
        let mut msgs_json = String::with_capacity(self.messages.len() * 160 + 32);
        msgs_json.push('[');
        for (idx, m) in self.messages.iter().rev().enumerate() {
            if idx > 0 {
                msgs_json.push(',');
            }
            let freq_str = m.freq_hz.map(|f| f.to_string()).unwrap_or_else(|| "null".to_string());
            let grp_str = m
                .group_address
                .as_ref()
                .map(|g| format!("\"{}\"", escape_json_str(g)))
                .unwrap_or_else(|| "null".to_string());
            let frag_str = m
                .fragment_info
                .as_ref()
                .map(|f| format!("\"{}\"", escape_json_str(f)))
                .unwrap_or_else(|| "null".to_string());
            let lbl_str = m
                .filter_label
                .as_ref()
                .map(|l| format!("\"{}\"", escape_json_str(l)))
                .unwrap_or_else(|| "null".to_string());
            msgs_json.push_str(&format!(
                "{{\"capcode\":\"{}\",\"time\":\"{}\",\"date\":\"{}\",\"mode\":\"{}\",\"msg_type\":\"{}\",\"bitrate\":{},\"message\":\"{}\",\"has_bit_errors\":{},\"soft_recovered_words\":{},\"filter_label\":{},\"freq_hz\":{},\"group_address\":{},\"fragment_info\":{}}}",
                escape_json_str(&m.capcode),
                escape_json_str(&m.time),
                escape_json_str(&m.date),
                escape_json_str(&m.mode),
                escape_json_str(&m.msg_type),
                m.bitrate,
                escape_json_str(&m.message),
                m.has_bit_errors,
                m.soft_recovered_words,
                lbl_str,
                freq_str,
                grp_str,
                frag_str
            ));
        }
        msgs_json.push(']');

        format!(
            "{{\"profile\":{},\"messages\":{}}}",
            self.summary_to_json(),
            msgs_json
        )
    }
}

#[derive(Clone, Debug)]
pub struct CapcodeCatalogDb {
    pub db_path: PathBuf,
    pub profiles: BTreeMap<String, CapcodeProfile>,
    pub dirty: bool,
}

impl Default for CapcodeCatalogDb {
    fn default() -> Self {
        Self::new(PathBuf::from(DEFAULT_CATALOG_DB_FILENAME))
    }
}

impl CapcodeCatalogDb {
    pub fn new(db_path: PathBuf) -> Self {
        Self {
            db_path,
            profiles: BTreeMap::new(),
            dirty: false,
        }
    }

    pub fn load_or_default(path: &Path) -> Self {
        let mut db = Self::new(path.to_path_buf());
        let _ = db.load_from_disk();
        db
    }

    pub fn is_valid_capcode(capcode: &str) -> bool {
        let trimmed = capcode.trim();
        !trimmed.is_empty() && !trimmed.contains('?')
    }

    /// Record a decoded message into the capcode directory and append it to the capcode's message history.
    pub fn record_message(&mut self, msg: &DecodedPagerMessage) {
        if !Self::is_valid_capcode(&msg.capcode) {
            return;
        }

        let profile = self
            .profiles
            .entry(msg.capcode.clone())
            .or_insert_with(|| CapcodeProfile::new(&msg.capcode, &msg.date, &msg.time, msg.bitrate));

        if profile.first_seen_date.is_empty() {
            profile.first_seen_date = msg.date.clone();
            profile.first_seen_time = msg.time.clone();
        }
        profile.last_seen_date = msg.date.clone();
        profile.last_seen_time = msg.time.clone();
        profile.message_count = profile.message_count.saturating_add(1);

        if msg.bitrate > 0 {
            profile.last_bitrate = msg.bitrate;
            if !profile.seen_bitrates.contains(&msg.bitrate) {
                profile.seen_bitrates.push(msg.bitrate);
                profile.seen_bitrates.sort_unstable();
            }
        }

        if !msg.mode.is_empty() && !profile.modes_seen.contains(&msg.mode) {
            profile.modes_seen.push(msg.mode.clone());
        }

        if let Some(freq) = msg.freq_hz {
            if freq > 0 && !profile.frequencies_hz.contains(&freq) {
                profile.frequencies_hz.push(freq);
                profile.frequencies_hz.sort_unstable();
            }
        }

        if let Some(grp) = &msg.group_address {
            for token in grp.split(',').map(|s| s.trim()).filter(|s| !s.is_empty()) {
                let clean = token.trim_start_matches("MEMBERS:").trim();
                if !clean.is_empty() && !profile.flex_group_addresses.iter().any(|g| g == clean) {
                    profile.flex_group_addresses.push(clean.to_string());
                    profile.flex_group_addresses.sort();
                }
            }
        }

        if profile.alias.is_empty() {
            if let Some(lbl) = &msg.filter_label {
                if !lbl.trim().is_empty() {
                    profile.alias = lbl.trim().to_string();
                }
            }
        }

        if profile.messages.len() >= MAX_MESSAGES_PER_CAPCODE {
            profile.messages.pop_front();
        }
        profile.messages.push_back(msg.clone());
        self.dirty = true;
    }

    /// Record a FLEX short-instruction groupcall assignment (`member_capcode` -> `group_capcode` `2029568..2029583`).
    pub fn record_group_assignment(
        &mut self,
        member_capcode: &str,
        group_capcode: &str,
        date: &str,
        time: &str,
        bitrate: u32,
        freq_hz: Option<u64>,
    ) {
        if !Self::is_valid_capcode(member_capcode) || !Self::is_valid_capcode(group_capcode) {
            return;
        }
        let profile = self
            .profiles
            .entry(member_capcode.to_string())
            .or_insert_with(|| CapcodeProfile::new(member_capcode, date, time, bitrate));
        if !profile
            .flex_group_addresses
            .iter()
            .any(|g| g == group_capcode)
        {
            profile.flex_group_addresses.push(group_capcode.to_string());
            profile.flex_group_addresses.sort();
            self.dirty = true;
        }
        if let Some(f) = freq_hz {
            if f > 0 && !profile.frequencies_hz.contains(&f) {
                profile.frequencies_hz.push(f);
                profile.frequencies_hz.sort_unstable();
                self.dirty = true;
            }
        }
    }

    pub fn set_alias(&mut self, capcode: &str, alias: &str) -> bool {
        if let Some(p) = self.profiles.get_mut(capcode) {
            p.alias = alias.trim().to_string();
            self.dirty = true;
            let _ = self.save_to_disk();
            true
        } else {
            false
        }
    }

    pub fn remove_capcode(&mut self, capcode: &str) -> bool {
        let removed = self.profiles.remove(capcode).is_some();
        if removed {
            self.dirty = true;
            let _ = self.save_to_disk();
        }
        removed
    }

    pub fn clear_all(&mut self) {
        self.profiles.clear();
        self.dirty = false;
        let _ = self.save_to_disk();
    }

    pub fn flush_if_dirty(&mut self) {
        if self.dirty {
            let _ = self.save_to_disk();
        }
    }

    pub fn total_capcodes(&self) -> usize {
        self.profiles.len()
    }

    pub fn total_indexed_messages(&self) -> usize {
        self.profiles.values().map(|p| p.messages.len()).sum()
    }

    pub fn all_summaries_to_json(&self) -> String {
        let mut out = String::with_capacity(self.profiles.len() * 220 + 64);
        out.push_str("{\"total_capcodes\":");
        out.push_str(&self.profiles.len().to_string());
        out.push_str(",\"profiles\":[");
        for (i, p) in self.profiles.values().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&p.summary_to_json());
        }
        out.push_str("]}");
        out
    }

    pub fn capcode_detail_to_json(&self, capcode: &str) -> String {
        if let Some(p) = self.profiles.get(capcode) {
            p.detail_to_json()
        } else {
            "{\"profile\":null,\"messages\":[]}".to_string()
        }
    }

    pub fn save_to_disk(&mut self) -> std::io::Result<()> {
        let tmp_path = self.db_path.with_extension("dat.tmp");
        {
            let mut f = fs::File::create(&tmp_path)?;
            writeln!(
                f,
                "# PDW-RS Capcode Directory & Message Catalog Database v1"
            )?;
            for p in self.profiles.values() {
                let rates_csv = p
                    .seen_bitrates
                    .iter()
                    .map(|b| b.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                let modes_csv = p
                    .modes_seen
                    .iter()
                    .map(|m| escape_field(m))
                    .collect::<Vec<_>>()
                    .join(",");
                let freqs_csv = p
                    .frequencies_hz
                    .iter()
                    .map(|hz| hz.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                let groups_csv = p
                    .flex_group_addresses
                    .iter()
                    .map(|g| escape_field(g))
                    .collect::<Vec<_>>()
                    .join(",");

                writeln!(
                    f,
                    "P\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    escape_field(&p.capcode),
                    escape_field(&p.alias),
                    escape_field(&p.first_seen_date),
                    escape_field(&p.first_seen_time),
                    escape_field(&p.last_seen_date),
                    escape_field(&p.last_seen_time),
                    p.message_count,
                    p.last_bitrate,
                    rates_csv,
                    modes_csv,
                    freqs_csv,
                    groups_csv
                )?;

                for m in &p.messages {
                    let err_flag = if m.has_bit_errors { 1 } else { 0 };
                    let flt_lbl = m.filter_label.as_deref().unwrap_or("");
                    let flt_col = m
                        .filter_color_idx
                        .map(|c| c.to_string())
                        .unwrap_or_default();
                    let freq_str = m.freq_hz.map(|hz| hz.to_string()).unwrap_or_default();
                    let grp_str = m.group_address.as_deref().unwrap_or("");
                    let frag_str = m.fragment_info.as_deref().unwrap_or("");

                    writeln!(
                        f,
                        "M\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                        escape_field(&m.capcode),
                        escape_field(&m.time),
                        escape_field(&m.date),
                        escape_field(&m.mode),
                        escape_field(&m.msg_type),
                        m.bitrate,
                        err_flag,
                        m.soft_recovered_words,
                        escape_field(flt_lbl),
                        flt_col,
                        freq_str,
                        escape_field(grp_str),
                        escape_field(frag_str),
                        escape_field(&m.message)
                    )?;
                }
            }
            f.flush()?;
        }
        fs::rename(&tmp_path, &self.db_path)?;
        self.dirty = false;
        Ok(())
    }

    pub fn load_from_disk(&mut self) -> std::io::Result<()> {
        if !self.db_path.exists() {
            return Ok(());
        }
        let file = fs::File::open(&self.db_path)?;
        let reader = BufReader::new(file);
        self.profiles.clear();

        for line_res in reader.lines() {
            let line = line_res?;
            let trimmed = line.trim_end_matches(&['\r', '\n'][..]);
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            let parts: Vec<&str> = trimmed.split('\t').collect();
            if parts.is_empty() {
                continue;
            }
            match parts[0] {
                "P" if parts.len() >= 13 => {
                    let capcode = unescape_field(parts[1]);
                    if !Self::is_valid_capcode(&capcode) {
                        continue;
                    }
                    let alias = unescape_field(parts[2]);
                    let first_seen_date = unescape_field(parts[3]);
                    let first_seen_time = unescape_field(parts[4]);
                    let last_seen_date = unescape_field(parts[5]);
                    let last_seen_time = unescape_field(parts[6]);
                    let message_count: u64 = parts[7].parse().unwrap_or(0);
                    let last_bitrate: u32 = parts[8].parse().unwrap_or(0);
                    let seen_bitrates: Vec<u32> = parts[9]
                        .split(',')
                        .filter_map(|s| s.trim().parse::<u32>().ok())
                        .collect();
                    let modes_seen: Vec<String> = parts[10]
                        .split(',')
                        .map(|s| unescape_field(s.trim()))
                        .filter(|s| !s.is_empty())
                        .collect();
                    let frequencies_hz: Vec<u64> = parts[11]
                        .split(',')
                        .filter_map(|s| s.trim().parse::<u64>().ok())
                        .collect();
                    let flex_group_addresses: Vec<String> = parts[12]
                        .split(',')
                        .map(|s| unescape_field(s.trim()))
                        .filter(|s| !s.is_empty())
                        .collect();

                    self.profiles.insert(
                        capcode.clone(),
                        CapcodeProfile {
                            capcode,
                            alias,
                            first_seen_date,
                            first_seen_time,
                            last_seen_date,
                            last_seen_time,
                            message_count,
                            last_bitrate,
                            seen_bitrates,
                            modes_seen,
                            frequencies_hz,
                            flex_group_addresses,
                            messages: VecDeque::new(),
                        },
                    );
                }
                "M" if parts.len() >= 15 => {
                    let capcode = unescape_field(parts[1]);
                    if let Some(p) = self.profiles.get_mut(&capcode) {
                        let flt_lbl = unescape_field(parts[9]);
                        let grp_str = unescape_field(parts[12]);
                        let frag_str = unescape_field(parts[13]);
                        let msg = DecodedPagerMessage {
                            capcode: capcode.clone(),
                            time: unescape_field(parts[2]),
                            date: unescape_field(parts[3]),
                            mode: unescape_field(parts[4]),
                            msg_type: unescape_field(parts[5]),
                            bitrate: parts[6].parse().unwrap_or(0),
                            has_bit_errors: parts[7] == "1",
                            soft_recovered_words: parts[8].parse().unwrap_or(0),
                            filter_label: if flt_lbl.is_empty() {
                                None
                            } else {
                                Some(flt_lbl)
                            },
                            filter_color_idx: parts[10].parse().ok(),
                            freq_hz: parts[11].parse().ok(),
                            group_address: if grp_str.is_empty() {
                                None
                            } else {
                                Some(grp_str)
                            },
                            fragment_info: if frag_str.is_empty() {
                                None
                            } else {
                                Some(frag_str)
                            },
                            message: unescape_field(parts[14]),
                        };
                        if p.messages.len() < MAX_MESSAGES_PER_CAPCODE {
                            p.messages.push_back(msg);
                        }
                    }
                }
                _ => {}
            }
        }
        self.dirty = false;
        Ok(())
    }
}

fn escape_field(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for ch in s.chars() {
        match ch {
            '\\' => out.push_str("\\\\"),
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            ',' => out.push_str("\\c"),
            c => out.push(c),
        }
    }
    out
}

fn unescape_field(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(ch) = chars.next() {
        if ch == '\\' {
            match chars.next() {
                Some('t') => out.push('\t'),
                Some('n') => out.push('\n'),
                Some('r') => out.push('\r'),
                Some('c') => out.push(','),
                Some('\\') => out.push('\\'),
                Some(other) => {
                    out.push('\\');
                    out.push(other);
                }
                None => out.push('\\'),
            }
        } else {
            out.push(ch);
        }
    }
    out
}
