// SPDX-License-Identifier: Apache-2.0
//! `repair_timing -hold`: hold repair by buffer insertion.
//!
//! Stages, in the reference's call order (`repairHold`):
//! - [`find_hold_buffer`] — the buffer the repair inserts (`findHoldBuffer` over
//!   `filterHoldBuffers`);
//! - [`find_hold_violations`] — the endpoints whose min slack is below the hold margin;
//! - [`print_progress`] — the progress table.
//!
//! The passes (`repairHoldPass`, `repairEndHold`, `makeHoldDelay`) run on the setup repair's
//! incremental timer and journal: `repair_setup::repair_hold`.

use std::collections::{BTreeMap, BTreeSet};

use vyges_sta::liberty::Cell;

use crate::preamble::{get_buffer_list, Libs, Master};
use crate::rebuffer::{gate_delays, port_cap, std_min};
use crate::repair_timing::{delay_as_string, total_negative_slack, worst_slack, Point};
use crate::Stop;

const INF: f32 = 1e30;

/// `print_interval_`: a progress row every this many passes (with `-verbose`).
const PRINT_INTERVAL: i64 = 10;

/// What the hold repair reads besides the timer.
pub struct HoldCtx<'a> {
    pub libs: &'a Libs,
    pub masters: &'a BTreeMap<String, Master>,
    /// Each site's height (`dbSite::getHeight`), by name.
    pub site_heights: &'a BTreeMap<String, i32>,
    pub dont_use: &'a BTreeSet<String>,
    /// `tgt_slews_` of the hold preamble (`findTargetLoads` over its buffers).
    pub tgt_slews: [f32; 2],
    pub time_scale: f32,
    /// `set_debug_level` by (tool, group).
    pub debug: &'a BTreeMap<(String, String), i64>,
}

/// `rsz::repair_hold`'s arguments. The margins are `double` seconds, as the command passes them
/// (`time_ui_sta`), and stay `double`: every comparison against them is in `double`.
#[derive(Debug, Clone, PartialEq)]
pub struct HoldArgs {
    pub setup_margin: f64,
    pub hold_margin: f64,
    pub allow_setup_violations: bool,
    /// `-max_buffer_percent` as a fraction (`float`).
    pub max_buffer_percent: f32,
    pub max_passes: i64,
    pub max_iterations: i64,
    pub verbose: bool,
    /// `-max_utilization` as a fraction, when given.
    pub max_utilization: Option<f64>,
}

/// An endpoint as the hold repair reads it: its min (hold) and max (setup) slacks, and whether
/// its pin is a clock pin (`Sta::isClock`).
#[derive(Debug, Clone, PartialEq)]
pub struct HoldEnd {
    pub pin: String,
    pub hold_slack: f32,
    pub setup_slack: f32,
    pub is_clock: bool,
}

/// What the hold repair logged: `lines` (the report) and `trace` (report and debug, in order).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HoldOutcome {
    pub lines: Vec<String>,
    pub trace: Vec<String>,
    pub inserted: i64,
    pub resized: i64,
    /// The endpoints with hold violations when the repair began.
    pub violating: usize,
    /// Where the modelling stops, when it does.
    pub stopped: Option<String>,
    /// The reference's error that ends the call (`RSZ-0050`, `RSZ-0060`), when one does.
    pub error: Option<String>,
}

impl HoldOutcome {
    fn report(&mut self, line: String) {
        self.trace.push(line.clone());
        self.lines.push(line);
    }
}

fn debug(ctx: &HoldCtx<'_>, out: &mut HoldOutcome, group: &str, level: i64, line: String) {
    if ctx.debug.get(&("RSZ".to_string(), group.to_string())).is_some_and(|&l| l >= level) {
        out.trace.push(format!("[DEBUG RSZ-{group}] {line}"));
    }
}

/// `isDelayCell`: a name containing `DEL`, `DLY` or `dlygate`.
fn is_delay_cell(name: &str) -> bool {
    !name.is_empty() && (name.contains("DEL") || name.contains("DLY") || name.contains("dlygate"))
}

/// `filterHoldBuffers`: `getBufferList` with clock buffers kept, then the first of four ever
/// looser matches that keeps any buffer — site, VT and footprint; VT and footprint; footprint;
/// none. The site is the shortest among the list's (`cells_by_site`, a map keyed by the site
/// object: a tie on the shortest height is ordered by pointer — refused); the VT the least leaky
/// category (one category here; several refused); the footprint matches when no footprint in the list is a delay
/// cell's, else when the buffer's is. Each kept buffer is logged as it is added.
fn filter_hold_buffers<'l>(ctx: &HoldCtx<'l>, out: &mut HoldOutcome) -> Result<Vec<&'l Cell>, Stop> {
    let list = get_buffer_list(ctx.libs, ctx.masters, ctx.dont_use, false)?;
    let height = |s: &str| ctx.site_heights.get(s).copied().unwrap_or(i32::MAX);
    let best_height = list.by_site.keys().map(|s| height(s)).min().unwrap_or(i32::MAX);
    let shortest: Vec<&String> = list.by_site.keys().filter(|s| height(s) == best_height).collect();
    if shortest.len() > 1 {
        return Err(Stop::refused("RSZ-SITES", format!("hold buffer sites of equal height ({}): the first is the reference's pointer order, not modelled", shortest.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(" "))));
    }
    let best_site = shortest.first().map(|s| s.as_str());
    // "Pick the least leaky VT": `sorted_vt_categories[0]` by average cell leakage — with one
    // category, that one; several would need the leakage, not read here.
    let best_vt = match list.categories.as_slice() {
        [] => None,
        [only] => Some(only.index),
        many => return Err(Stop::refused("RSZ-VT", format!("{} VT categories among the hold buffers: the least leaky (average cell leakage) is not modelled", many.len()))),
    };
    let lib_has_footprints = list.by_footprint.keys().any(|f| is_delay_cell(f));
    for (match_site, match_vt, match_footprint) in [(true, true, true), (false, true, true), (false, false, true), (false, false, false)] {
        let mut kept = Vec::new();
        for &b in &list.cells {
            let site_matches = !match_site || ctx.masters.get(&b.name).map(|m| m.site.as_str()) == best_site;
            let vt_matches = !match_vt || best_vt.is_none_or(|i| list.vt[&b.name].index == i);
            let footprint_matches = !match_footprint || !lib_has_footprints || is_delay_cell(&b.footprint);
            if site_matches && vt_matches && footprint_matches {
                debug(ctx, out, "resizer", 1, format!("{} added to hold buffer", b.name));
                kept.push(b);
            }
        }
        if !kept.is_empty() {
            return Ok(kept);
        }
    }
    Err(Stop::error("RSZ-0167", "No suitable hold buffers have been found".into()))
}

/// `bufferHoldDelay`: the buffer driving its own input capacitance at the target slews — per
/// transition the least over every scene (`bufferHoldDelays`: the scene port's capacitance, the
/// scene's arcs) of the largest arc delay; then the smaller of rise and fall (`std::min`).
fn buffer_hold_delay(ctx: &HoldCtx<'_>, buffer: &Cell) -> f32 {
    let Some((input, output)) = buffer.buffer_ports() else { return INF };
    let mut delays = [INF; 2];
    for k in 0..ctx.libs.scene_count() {
        let sc = ctx.libs.scene_cell(k, &buffer.name).unwrap_or(buffer);
        let load_cap = port_cap(sc, &input.name);
        let (gd, _) = gate_delays(sc, &output.name, load_cap, ctx.tgt_slews);
        delays = [std_min(delays[0], gd[0]), std_min(delays[1], gd[1])];
    }
    std_min(delays[0], delays[1])
}

/// `findHoldBuffer`: each hold buffer with an area scored `bufferHoldDelay / area` (`float`),
/// sorted ascending (libc++ `std::sort`); the best is the highest score, then — walking down —
/// any buffer within 0.95 of it with a smaller area replaces it.
///
/// ⚠️ In the reference the best and the "highest metric" are both REFERENCES to the last
/// element, so a replacement also moves the threshold: later buffers are compared with 0.95 of
/// the REPLACEMENT's score and area. Kept as such.
pub fn find_hold_buffer(ctx: &HoldCtx<'_>, out: &mut HoldOutcome) -> Result<Option<String>, Stop> {
    let hold_buffers = filter_hold_buffers(ctx, out)?;
    let mut buffers: Vec<(f32, &Cell)> = Vec::new();
    for b in hold_buffers {
        let area = b.area;
        if area != 0.0 {
            buffers.push((buffer_hold_delay(ctx, b) / area, b));
        }
    }
    crate::order::libcxx_sort_by(&mut buffers, |l, r| l.0 < r.0).map_err(|h| Stop::refused("RSZ-ORDER", format!("{} hold buffers reach the sort's heap fallback, which is not modelled", h.len)))?;
    let Some(&last) = buffers.last() else { return Ok(None) };
    let mut best = last;
    let margin: f32 = 0.95;
    for &cand in buffers.iter().rev().skip(1) {
        if cand.0 >= margin * best.0 && cand.1.area < best.1.area {
            best = cand;
        }
    }
    Ok(Some(best.1.name.clone()))
}

/// `findHoldViolations`: in endpoint order, each non-clock endpoint whose hold slack is below the
/// margin (`float < double`, compared in `double`), logged with its setup slack; the worst is the
/// `std::min` of their slacks from `INF`.
pub fn find_hold_violations(ctx: &HoldCtx<'_>, ends: &[HoldEnd], hold_margin: f64, out: &mut HoldOutcome) -> (f32, Vec<usize>) {
    let mut worst = INF;
    let mut viol = Vec::new();
    debug(ctx, out, "repair_hold", 3, "Hold violations".into());
    for (i, e) in ends.iter().enumerate() {
        if !e.is_clock && f64::from(e.hold_slack) < hold_margin {
            debug(ctx, out, "repair_hold", 3, format!(" {} hold_slack={} setup_slack={}", e.pin, delay_as_string(e.hold_slack, 3, ctx.time_scale), delay_as_string(e.setup_slack, 3, ctx.time_scale)));
            worst = std_min(e.hold_slack, worst);
            viol.push(i);
        }
    }
    (worst, viol)
}

/// The counters a progress row shows.
pub struct Progress {
    pub resized: i64,
    pub inserted: i64,
    pub cloned: i64,
    pub initial_area: f64,
    pub design_area: f64,
}

/// `printProgress(iteration, force, end)`: the header before row 0; a row every
/// [`PRINT_INTERVAL`] passes, or when forced, or at the end (`final`) — the hold WNS and its
/// endpoint, the hold TNS, the area growth; a closing rule at the end.
pub fn print_progress(ctx: &HoldCtx<'_>, ends: &[HoldEnd], p: &Progress, iteration: i64, force: bool, end: bool, out: &mut HoldOutcome) {
    if iteration == 0 {
        out.report("Iteration | Resized | Buffers | Cloned Gates |   Area   |   WNS   |   TNS   | Endpoint".into());
        out.report("-".repeat(86));
    }
    if iteration % PRINT_INTERVAL == 0 || force || end {
        let points: Vec<Point> = ends.iter().map(|e| Point { pin: e.pin.clone(), slack: e.hold_slack }).collect();
        let (wns, at) = worst_slack(&points);
        let tns = total_negative_slack(&points);
        let field = if end { "final".to_string() } else { iteration.to_string() };
        let growth = (p.design_area - p.initial_area) / p.initial_area * 1e2;
        let worst_name = at.map(|a| a.pin.clone()).unwrap_or_default();
        out.report(format!(
            "{:>9} | {:>7} | {:>7} | {:>12} | {:>+7.1}% | {:>7} | {:>7} | {}",
            field,
            p.resized,
            p.inserted,
            p.cloned,
            growth,
            delay_as_string(wns, 3, ctx.time_scale),
            delay_as_string(tns, 3, ctx.time_scale),
            worst_name
        ));
    }
    if end {
        out.report("-".repeat(86));
    }
}
