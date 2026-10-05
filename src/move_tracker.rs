// SPDX-License-Identifier: Apache-2.0
//! `MoveTracker` at level 1 (`set_debug_level RSZ move_tracker 1`): the slack distributions the
//! setup repair is reported against, and the final analysis reports.
//!
//! At level 1 no move is tracked: `MoveCommitter::trackMoveAttempt` and
//! `trackViolatorWithTimingInfo` wait for level 2, so no pin is ever visited, no move event is
//! recorded and no endpoint profile is collected — the phase profilers and the success, failure
//! and missed-opportunity reports print their empty branches. What level 1 does keep:
//! - `captureInitialSlackDistribution` (the preamble, after RSZ-0099): each non-clock driver pin
//!   whose RISE slack over every scene is negative, and each endpoint whose slack is negative, in
//!   ns ([`capture`]);
//! - `inDbITermDestroy` / `inDbITermCreate`: an entry moves to a backup when its terminal is
//!   destroyed and back when a terminal is created at the same address. Terminals are odb table
//!   slots and every create notifies (a journal undo recreates through `dbInst::create`), so at
//!   the end an entry is live exactly when a terminal with its odb id exists — a slot reused by
//!   another instance's terminal revives the entry with the new pin ([`Initial::split`]);
//! - `trackCriticalPins`, at the end: every non-clock driver pin with a negative RISE slack.
//!
//! Not modelled (refused by the caller): level 2 and above (the moves tracked), and the reports
//! over violating endpoints left at the end, which enumerate each endpoint's k worst paths.

/// The debug group.
pub const GROUP: &str = "move_tracker";

/// `printSlackDistribution`'s `target_num_bins`, and every histogram's `target_bar_width`.
const TARGET_NUM_BINS: usize = 10;
const TARGET_BAR_WIDTH: usize = 50;

/// A slack in seconds, in ns as the tracker keeps it: the `float` times the `double` 1e9,
/// narrowed back to `float`.
pub fn ns(slack: f32) -> f32 {
    (f64::from(slack) * 1e9) as f32
}

/// What `captureInitialSlackDistribution` found.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Initial {
    /// Every driver pin the instance iterator yielded, clock pins included.
    pub driver_pins: usize,
    /// `initial_pin_slack_`: each kept pin's odb terminal id (`NetInfo::pin_id`) and slack, ns.
    pub pins: Vec<(u64, f32)>,
    /// Every endpoint.
    pub endpoints: usize,
    /// `initial_endpoint_slack_`.
    pub endpoint_slacks: Vec<(u64, f32)>,
}

/// One pin as the capture reads it: its terminal id, whether it is a driver, whether it is on a
/// clock network, and its RISE slack over every scene (s).
#[derive(Debug, Clone, Copy)]
pub struct DriverPin {
    pub id: u64,
    pub driver: bool,
    pub clock: bool,
    pub rise_slack: f32,
}

/// The design's pins as the tracker reads them at one moment.
#[derive(Debug, Clone, Default)]
pub struct View {
    /// Each leaf instance's signal pins, in the instance × pin iterator order.
    pub pins: Vec<DriverPin>,
    /// Each endpoint's terminal id and slack (s).
    pub ends: Vec<(u64, f32)>,
    /// Each terminal on a net by id: `[RISE slack, slack over both transitions]` (s).
    pub slacks: std::collections::HashMap<u64, [f32; 2]>,
}

/// `captureInitialSlackDistribution`: `pins` in the leaf instance × pin iterator order,
/// `endpoints` each endpoint's id and slack (s).
pub fn capture(pins: &[DriverPin], endpoints: &[(u64, f32)]) -> Initial {
    let mut out = Initial::default();
    for p in pins.iter().filter(|p| p.driver) {
        out.driver_pins += 1;
        if p.clock {
            continue;
        }
        let slack_ns = ns(p.rise_slack);
        if slack_ns < 0.0 {
            out.pins.push((p.id, slack_ns));
        }
    }
    for &(id, slack) in endpoints {
        out.endpoints += 1;
        let slack_ns = ns(slack);
        if slack_ns < 0.0 {
            out.endpoint_slacks.push((id, slack_ns));
        }
    }
    out
}

/// The two lines the capture prints.
pub fn capture_lines(c: &Initial) -> Vec<String> {
    vec![
        debug(&format!("Scanned {} driver pins, found {} with negative slack", c.driver_pins, c.pins.len())),
        debug(&format!("Scanned {} endpoints, found {} with negative slack", c.endpoints, c.endpoint_slacks.len())),
    ]
}

/// The entries live at the end (each with its initial slack and its terminal id) and the count
/// moved to the backup: `live(id)` — a terminal with that odb id exists now.
pub fn split(entries: &[(u64, f32)], live: impl Fn(u64) -> bool) -> (Vec<(u64, f32)>, usize) {
    let (kept, gone): (Vec<_>, Vec<_>) = entries.iter().copied().partition(|&(id, _)| live(id));
    (kept, gone.len())
}

/// A debug line of the group.
pub fn debug(msg: &str) -> String {
    format!("[DEBUG RSZ-{GROUP}] {msg}")
}

/// `bin = i + 1` for the last edge `slack >= edge`, scanning up and stopping at the first edge
/// above it.
fn bin_of(edges: &[f32], slack: f32) -> usize {
    let mut bin = 0;
    for (i, &e) in edges.iter().enumerate() {
        if slack >= e {
            bin = i + 1;
        } else {
            break;
        }
    }
    bin
}

/// `histogramBar`.
fn bar(count: usize, per_hash: usize) -> String {
    let len = count / per_hash;
    if len > 0 {
        format!(" {}", "#".repeat(len))
    } else {
        String::new()
    }
}

/// The integral scale: about 50 characters at the largest count, at least 1.
fn per_hash(max_count: usize) -> usize {
    max_count.div_ceil(TARGET_BAR_WIDTH).max(1)
}

/// `std::min(a, b)`: `b < a ? b : a` — on equal values (0.0 and -0.0) the FIRST argument.
fn std_min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}

/// `std::max(a, b)`: `a < b ? b : a`.
fn std_max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// `std::fixed << setprecision(3)` of a `float` (promoted, exactly, to `double`).
fn fixed3(v: f32) -> String {
    format!("{:.3}", f64::from(v))
}

fn histogram(out: &mut Vec<String>, title: &str, labels: &[String], counts: &[usize], per_hash: usize) {
    out.push(debug(title));
    out.push(debug(&format!("{:<18} | {:>6} | {}", "Slack (ns)", "Count", "Distribution")));
    for (label, &count) in labels.iter().zip(counts) {
        out.push(debug(&format!("{label:<18} | {count:>6} |{}", bar(count, per_hash))));
    }
}

/// About 10 bins: 9 edges at `min + i × (range / 10)`, all in `float`; or the one edge `min` when
/// the range is empty.
fn bin_edges(min_ns: f32, max_ns: f32) -> Vec<f32> {
    let mut edges = Vec::new();
    if max_ns > min_ns {
        let width = (max_ns - min_ns) / TARGET_NUM_BINS as f32;
        for i in 1..TARGET_NUM_BINS {
            edges.push(min_ns + i as f32 * width);
        }
    } else {
        edges.push(min_ns);
    }
    edges
}

/// `printSlackDistribution`: the live pins' (initial ns, slack now in s) and how many were
/// destroyed; the live endpoints' (initial ns, slack now in s — `None` with no load vertex) and
/// how many were destroyed.
pub fn slack_distribution(title: &str, pins: &[(f32, f32)], pins_destroyed: usize, endpoints: &[(f32, Option<f32>)], endpoints_destroyed: usize) -> Vec<String> {
    let mut out = Vec::new();
    if pins.is_empty() {
        out.push(debug(&format!("{title}: No initial slack data captured")));
        return out;
    }
    out.push(debug(&format!("{title}:")));
    // The range over the initial and the present slacks, the top capped at 0.
    let mut min_ns = f32::MAX;
    let mut max_ns = f32::MIN;
    for &(initial, post) in pins {
        min_ns = std_min(initial, min_ns);
        max_ns = std_max(initial, max_ns);
        let post_ns = ns(post);
        min_ns = std_min(post_ns, min_ns);
        max_ns = std_max(post_ns, max_ns);
    }
    max_ns = std_min(max_ns, 0.0);
    out.push(debug(&format!("Slack range: min={:.3} ns, max={:.3} ns", f64::from(min_ns), f64::from(max_ns))));
    let edges = bin_edges(min_ns, max_ns);
    let mut pre = vec![0usize; edges.len() + 1];
    let mut post = vec![0usize; edges.len() + 1];
    for &(initial, now) in pins {
        pre[bin_of(&edges, initial)] += 1;
        post[bin_of(&edges, ns(now))] += 1;
    }
    let max_count = pre.iter().chain(&post).copied().max().unwrap_or(0);
    let gates_per_hash = per_hash(max_count);
    // The labels: below the first edge, each [edge, next), and the last bin always `>= 0.000`
    // (it holds everything from the last edge up).
    let mut labels = vec![format!("< {}", fixed3(edges[0]))];
    for w in edges.windows(2) {
        labels.push(format!("[{},{})", fixed3(w[0]), fixed3(w[1])));
    }
    labels.push(format!(">= {}", fixed3(0.0)));
    out.push(debug(""));
    histogram(&mut out, "Pre-Optimization Gate Slack Distribution:", &labels, &pre, gates_per_hash);
    out.push(debug(""));
    histogram(&mut out, "Post-Optimization Gate Slack Distribution:", &labels, &post, gates_per_hash);
    out.push(debug(""));
    out.push(debug(&format!(
        "Summary: {} driver pins tracked ({} destroyed), max bin count: {} (# = {} {})",
        pre.iter().sum::<usize>(),
        pins_destroyed,
        max_count,
        gates_per_hash,
        if gates_per_hash == 1 { "gate" } else { "gates" }
    )));
    // The endpoints on the same bins.
    if !endpoints.is_empty() {
        out.push(debug(""));
        out.push(debug("=== Endpoint Slack Distribution ==="));
        let mut pre = vec![0usize; edges.len() + 1];
        let mut post = vec![0usize; edges.len() + 1];
        for &(initial, now) in endpoints {
            pre[bin_of(&edges, initial)] += 1;
            if let Some(now) = now {
                post[bin_of(&edges, ns(now))] += 1;
            }
        }
        let max_count = pre.iter().chain(&post).copied().max().unwrap_or(0);
        let endpoints_per_hash = per_hash(max_count);
        out.push(debug(""));
        histogram(&mut out, "Pre-Optimization Endpoint Slack Distribution:", &labels, &pre, endpoints_per_hash);
        out.push(debug(""));
        histogram(&mut out, "Post-Optimization Endpoint Slack Distribution:", &labels, &post, endpoints_per_hash);
        out.push(debug(""));
        out.push(debug(&format!(
            "Summary: {} endpoints tracked ({} destroyed), max bin count: {} (# = {} {})",
            pre.iter().sum::<usize>(),
            endpoints_destroyed,
            max_count,
            endpoints_per_hash,
            if endpoints_per_hash == 1 { "endpoint" } else { "endpoints" }
        )));
    }
    out
}

/// `printEndpointSummary` with no endpoint profile (always, at level 1).
pub fn endpoint_summary(title: &str) -> String {
    debug(&format!("{title}: No endpoint statistics collected"))
}

/// `MoveCommitter::printTrackerFinalReports` with no violating endpoint and no critical pin left:
/// the distributions, then every report's empty branch. `distribution` is
/// [`slack_distribution`]'s lines.
pub fn final_reports_all_met(distribution: Vec<String>) -> Vec<String> {
    let mut out = vec!["[INFO RSZ-0211] ".to_string(), "[INFO RSZ-0212] === Optimization Analysis Reports ===".to_string()];
    out.extend(distribution);
    // printTopBinEndpoints
    out.push(debug("Most Critical Endpoints After Optimization:"));
    out.push(debug("No violating endpoints after optimization (all meet timing!)"));
    // printCriticalEndpointPathHistogram
    out.push(debug(""));
    out.push(debug("=== Critical Endpoint Path Distribution ==="));
    out.push(debug("No violating endpoints found."));
    // printSuccessReport / printFailureReport: no move event at level 1.
    out.push(debug("Successful Optimizations Report: No successful optimizations"));
    out.push(debug("Unsuccessful Optimizations Report: No rejected optimizations"));
    // printMissedOpportunitiesReport: nothing visited, no critical pin.
    out.push(debug("Missed Opportunities Report:"));
    out.push(debug("Category 1: All visited pins with negative slack had moves attempted (good!)"));
    out.push(debug("Category 2: All critical pins were visited (good!)"));
    out.push(debug("Summary: 0 critical pins identified, 0 visited, 0 had moves attempted"));
    out.push(debug("  Missed opportunities: 0 visited but no moves, 0 never visited"));
    out.push("[INFO RSZ-0213] ".to_string());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pin(id: u64, driver: bool, clock: bool, rise_slack: f32) -> DriverPin {
        DriverPin { id, driver, clock, rise_slack }
    }

    /// Rule (`captureInitialSlackDistribution`): every driver pin is COUNTED, clock pins included,
    /// but a clock pin is never kept; a pin is kept when its slack in ns (`float × 1e9` in
    /// `double`, narrowed) is below 0.
    #[test]
    fn capture_counts_clock_drivers_but_keeps_none() {
        let c = capture(&[pin(2, true, true, -1e-9), pin(4, false, false, -1e-9), pin(6, true, false, -1e-10), pin(8, true, false, 0.0)], &[(10, -1e-12), (11, 1e-12)]);
        assert_eq!(c.driver_pins, 3);
        assert_eq!(c.pins, vec![(6, ns(-1e-10))]);
        assert_eq!(c.endpoints, 2);
        assert_eq!(c.endpoint_slacks.len(), 1);
        assert_eq!(capture_lines(&c)[0], "[DEBUG RSZ-move_tracker] Scanned 3 driver pins, found 1 with negative slack");
    }

    /// Rule (`inDbITermDestroy`/`Create`): an entry whose terminal is gone at the end counts as
    /// destroyed; one whose slot holds a terminal is live.
    #[test]
    fn split_by_live_terminal() {
        let (kept, gone) = split(&[(2, -1.0), (4, -2.0), (6, -3.0)], |id| id != 4);
        assert_eq!(kept, vec![(2, -1.0), (6, -3.0)]);
        assert_eq!(gone, 1);
    }

    /// Rule (`printSlackDistribution`): 9 edges at `min + i × (range / 10)` in float; the last
    /// label reads `>= 0.000` though the bin holds everything from the last edge up; the
    /// endpoints share the pins' bins. Values: the reference's `report_move_tracker`, raw
    /// (`rsz-tracker-patch.py`): 2 live pins of 6, 4 live endpoints; edges bit for bit.
    #[test]
    fn distribution_bins_and_labels() {
        let f = f32::from_bits;
        let min = f(0xbeaa9870);
        let want: Vec<f32> = [0xbe998932, 0xbe8879f3, 0xbe6ed56a, 0xbe4cb6ed, 0xbe2a9870, 0xbe0879f4, 0xbdccb6ee, 0xbd8879f4, 0xbd0879f8].map(f).to_vec();
        assert_eq!(bin_edges(min, 0.0), want);
        let pins = [(min, f(0x2d5caa10)), (min, f(0x2d5caa00))];
        let eps = [(f(0xbeaa986f), Some(f(0x2d5caa00))), (f(0xbc245d9f), Some(f(0x2e80f046))), (f(0xbcf337fa), Some(f(0x2e25f580))), (f(0xbd3f40da), Some(f(0x2db35c88)))];
        let l = slack_distribution("Pin Slack Distribution", &pins, 4, &eps, 0);
        let s: Vec<&str> = l.iter().map(|x| x.trim_start_matches("[DEBUG RSZ-move_tracker] ")).collect();
        assert_eq!(s[1], "Slack range: min=-0.333 ns, max=0.000 ns");
        assert_eq!(s[5], "< -0.300           |      2 | ##");
        assert_eq!(s[6], "[-0.300,-0.267)    |      0 |");
        assert_eq!(s[14], ">= 0.000           |      0 |");
        assert!(s.contains(&"Summary: 2 driver pins tracked (4 destroyed), max bin count: 2 (# = 1 gate)"));
        assert!(s.contains(&"Summary: 4 endpoints tracked (0 destroyed), max bin count: 4 (# = 1 endpoint)"));
        // −0.010 and −0.018 ns sit at or above the last edge (−0.0333): the `>= 0.000` bin.
        let pre_ep = s.iter().position(|x| *x == "Pre-Optimization Endpoint Slack Distribution:").unwrap();
        assert_eq!(s[pre_ep + 11], ">= 0.000           |      2 | ##");
        assert_eq!(s[pre_ep + 10], "[-0.067,-0.033)    |      1 | #");
        assert_eq!(s[pre_ep + 2], "< -0.300           |      1 | #");
    }

    /// Rule: every pin destroyed — the report stops at its title.
    #[test]
    fn distribution_with_no_live_pin() {
        assert_eq!(slack_distribution("T", &[], 3, &[(-1.0, Some(0.0))], 0), vec!["[DEBUG RSZ-move_tracker] T: No initial slack data captured".to_string()]);
    }

    /// Rule: one value only — one edge at it, two bins.
    #[test]
    fn distribution_degenerate_range() {
        let l = slack_distribution("T", &[(0.0, 0.0)], 0, &[], 0);
        assert!(l.contains(&debug(&format!("{:<18} | {:>6} |", "< 0.000", 0))));
        assert!(l.contains(&debug(&format!("{:<18} | {:>6} | #", ">= 0.000", 1))));
    }

    #[test]
    fn bar_scale_is_integral() {
        assert_eq!(per_hash(0), 1);
        assert_eq!(per_hash(50), 1);
        assert_eq!(per_hash(51), 2);
        assert_eq!(bar(3, 2), " #");
        assert_eq!(bar(1, 2), "");
    }
}
