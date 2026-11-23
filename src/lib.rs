// SPDX-License-Identifier: Apache-2.0
//! Electrical repair of a placed design: repeaters inserted along each net's Steiner tree where a
//! wire is too long, a driver sees too much capacitance or fans out too far, or a load's transition
//! is too slow; a driver resized where its own transition is.
//!
//! The modules follow the repair's call sequence:
//! - [`preamble`]: what the repair reads about the libraries before any net — the buffers, their
//!   target slews and each cell's target load, the slew shape factor.
//! - [`repair_design`]: the driver loop, a thin sequencer of the stages in their order.
//! - [`repair_timing`]: `repair_timing -setup` up to its first progress row.
//! - [`repair_setup`]: the setup repair's legacy pass loop (SizeUp and Unbuffer moves), to its
//!   summary.
//! - [`unbuffer`]: UnbufferMove — the guards and the slack estimate a buffer removal must pass.
//! - [`swap_pins`]: SwapPinsMove — the symmetric inputs and the fastest of them.
//! - [`rebuffer`]: BufferMove — the net rebuilt as a buffer tree, by slack then by area.
//! - [`clone`]: CloneMove — a high-fanout driver duplicated for half its loads.
//! - [`timing`]: what the repair asks the timer — slew, capacitance and fanout checks per scene, the
//!   forward pass, the driver order.
//! - [`driver_slew`]: a driver's own slew violation — the size that fits, else the load cap that
//!   would.
//! - [`fanout`]: fanout repair's load regions.
//! - [`buffered_net`]: the net's Steiner tree as the repair walks it.
//! - [`walk`]: the walk from the loads up, and the repeater it inserts.
//! - [`sizing`]: equivalent cells and which of them fits a load.
//! - [`design`]: what the repair asks of the design it edits.
//! - [`order`]: sorts whose order is a value.
//! - [`trace`]: the call-sequence trace, one line per decision.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_rsz::…` paths resolve here.
extern crate self as vyges_rsz;

/// `VYGES_RSZ_STAGES`: a stage boundary on stderr — `VYGSTAGE|name|epoch ms|VmRSS kB|VmHWM kB`,
/// the high-water mark since the previous boundary (it is reset at each one) — for the top-down
/// stage table (time and memory per reference stage). Off: one read of a flag set once.
pub fn stage(name: &str) {
    if !prof::enabled() {
        return;
    }
    let ms = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_millis());
    let st = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    let kb = |k: &str| st.lines().find(|l| l.starts_with(k)).and_then(|l| l.split_whitespace().nth(1)).unwrap_or("0").to_string();
    eprintln!("VYGSTAGE|{name}|{ms}|{}|{}", kb("VmRSS:"), kb("VmHWM:"));
    prof::dump(name);
    let _ = std::fs::write("/proc/self/clear_refs", "5");
}

/// `VYGES_RSZ_STAGES` work counters: per site, calls, wall ns and items, cumulative, printed at
/// each [`stage`] boundary (`VYGCNT|stage|site|calls|ms|items`). Off, a site costs one read of a
/// flag the first call sets (no clock read, no atomic add).
pub mod prof {
    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
    use std::sync::OnceLock;
    /// Whether `VYGES_RSZ_STAGES` is set, read once.
    pub fn enabled() -> bool {
        static ON: OnceLock<bool> = OnceLock::new();
        *ON.get_or_init(|| std::env::var_os("VYGES_RSZ_STAGES").is_some())
    }
    pub const NAMES: [&str; 9] = ["refresh_touched", "refresh_pins_read", "timer_graphs", "find_vertex_delay_scenes", "ensure_wire_parasitic", "insert_buffer_ffi", "swap_master", "make_repeater", "timer_invalid_pins"];
    static CALLS: [AtomicU64; 9] = [const { AtomicU64::new(0) }; 9];
    static NS: [AtomicU64; 9] = [const { AtomicU64::new(0) }; 9];
    static ITEMS: [AtomicU64; 9] = [const { AtomicU64::new(0) }; 9];
    /// A timed call of site `i`; nothing when the counters are off.
    pub struct Span(Option<(usize, std::time::Instant)>);
    impl Span {
        pub fn new(i: usize) -> Span {
            if !enabled() {
                return Span(None);
            }
            CALLS[i].fetch_add(1, Relaxed);
            Span(Some((i, std::time::Instant::now())))
        }
    }
    impl Drop for Span {
        fn drop(&mut self) {
            if let Some((i, t)) = self.0 {
                NS[i].fetch_add(t.elapsed().as_nanos() as u64, Relaxed);
            }
        }
    }
    pub fn items(i: usize, n: u64) {
        if enabled() {
            ITEMS[i].fetch_add(n, Relaxed);
        }
    }
    pub fn dump(stage: &str) {
        for i in 0..NAMES.len() {
            let c = CALLS[i].load(Relaxed);
            let it = ITEMS[i].load(Relaxed);
            if c > 0 || it > 0 {
                eprintln!("VYGCNT|{stage}|{}|{c}|{:.1}|{it}", NAMES[i], NS[i].load(Relaxed) as f64 / 1e6);
            }
        }
    }
}

pub mod buffer_ports;
pub mod buffered_net;
pub mod clone;
pub mod delay_estimator;
pub mod design;
pub mod driver_slew;
#[cfg(feature = "cli")]
pub mod env;
pub mod fanout;
#[cfg(feature = "cli")]
pub mod job;
pub mod max_wire_length;
#[cfg(feature = "cli")]
pub mod odb_design;
pub mod move_tracker;
pub mod order;
pub mod path_group;
pub mod preamble;
pub mod rebuffer;
pub mod remove_buffers;
pub mod repair_design;
pub mod repair_hold;
pub mod repair_setup;
pub mod repair_timing;
pub mod scene_parasitics;
pub mod sequential_timing;
pub mod sizing;
pub mod swap_pins;
pub mod timing;
pub mod trace;
pub mod unbuffer;
pub mod virtual_cts;
pub mod walk;

/// Why a run stopped short of a result.
#[derive(Debug, Clone, PartialEq)]
pub enum Stop {
    /// The input is outside what is modelled. Never a pass.
    Refused { code: &'static str, msg: String },
    /// The reference would stop here too (its own error).
    Error { code: &'static str, msg: String },
}

impl Stop {
    pub fn refused(code: &'static str, msg: String) -> Stop {
        Stop::Refused { code, msg }
    }
    pub fn error(code: &'static str, msg: String) -> Stop {
        Stop::Error { code, msg }
    }
    pub fn code(&self) -> &'static str {
        match self {
            Stop::Refused { code, .. } | Stop::Error { code, .. } => code,
        }
    }
    pub fn message(&self) -> &str {
        match self {
            Stop::Refused { msg, .. } | Stop::Error { msg, .. } => msg,
        }
    }
}

/// The `vyges-rsz` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
