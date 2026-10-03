// SPDX-License-Identifier: Apache-2.0
//! `repair_timing -setup`: what the command decides and prints BEFORE it moves anything — the move
//! sequence (RSZ-0100), the violation summary (RSZ-0094 / RSZ-0099, or RSZ-0098), and the first row
//! of the progress table.
//!
//! One function per stage, in the reference's call order (`prepareForPhasePipeline`, then
//! `printProgress(0)`):
//! - [`Args::parse`] — the command's flags, with the reference's defaults;
//! - [`move_sequence`] — `buildMainMoveSequence`;
//! - [`collect_violating`] — `collectViolatingEndpoints` / `collectViolatingStartpoints`;
//! - [`preamble`] — the summary lines;
//! - [`row0`] — the progress header and row 0.
//!
//! The repair itself — the pass loop and its moves — is [`crate::repair_setup`]'s; where its
//! moves are not modelled, the caller refuses after these lines.

use vyges_sta::fuzzy;

/// The timer's initial slack (`MinMax::min()->initValue()`), as the reference spells INF.
pub const SLACK_INIT: f32 = 1e30;

/// `repair_timing`'s flags, as `repair_setup` receives them.
#[derive(Debug, Clone, PartialEq)]
pub struct Args {
    pub setup: bool,
    pub hold: bool,
    /// `-setup_margin`, user time units.
    pub setup_margin: f64,
    /// `-hold_margin`, user time units.
    pub hold_margin: f64,
    /// `-max_buffer_percent` (20 by default), percent.
    pub max_buffer_percent: f64,
    /// `-repair_tns` as a fraction (1.0 by default).
    pub repair_tns_end_percent: f64,
    /// `-sequence`, parsed (`parseMoveSequence`); empty for the default sequence.
    pub sequence: Vec<Move>,
    pub phases: Option<String>,
    pub recover_power: bool,
    pub skip_pin_swap: bool,
    pub skip_gate_cloning: bool,
    pub skip_size_down_fanout: bool,
    pub skip_buffering: bool,
    pub skip_buffer_removal: bool,
    pub skip_vt_swap: bool,
    /// `-max_passes` (10000), `-max_iterations` (−1: no limit), `-max_repairs_per_pass` (1).
    pub max_passes: i64,
    pub max_iterations: i64,
    pub max_repairs_per_pass: i64,
    /// `-max_utilization`, when given.
    pub max_utilization: Option<String>,
    pub verbose: bool,
    pub skip_last_gasp: bool,
    pub skip_crit_vt_swap: bool,
    pub match_cell_footprint: bool,
    pub allow_setup_violations: bool,
}

/// `MoveType`, by the name `moveName` prints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Move {
    Buffer,
    Unbuffer,
    SwapPins,
    SizeUp,
    SizeDownFanout,
    Clone,
    SplitLoad,
    SizeUpMatch,
    VtSwap,
    Reroute,
}

impl Move {
    /// `moveName`.
    pub fn name(self) -> &'static str {
        match self {
            Move::Buffer => "BufferMove",
            Move::Clone => "CloneMove",
            Move::SizeUp => "SizeUpMove",
            Move::SizeUpMatch => "SizeUpMatchMove",
            Move::SizeDownFanout => "SizeDownFanoutMove",
            Move::SwapPins => "SwapPinsMove",
            Move::VtSwap => "VtSwapMove",
            Move::Unbuffer => "UnbufferMove",
            Move::SplitLoad => "SplitLoadMove",
            Move::Reroute => "RerouteMove",
        }
    }

    /// `moveTypeFromString`, case-insensitively.
    fn from_word(w: &str) -> Result<Move, String> {
        Ok(match w.to_ascii_lowercase().as_str() {
            "buffer" => Move::Buffer,
            "unbuffer" => Move::Unbuffer,
            "swap" => Move::SwapPins,
            "sizeup" => Move::SizeUp,
            "size_down_fanout" | "size_down" => Move::SizeDownFanout,
            "clone" => Move::Clone,
            "split" => Move::SplitLoad,
            "sizeup_match" => Move::SizeUpMatch,
            "vt_swap" => Move::VtSwap,
            "reroute" => Move::Reroute,
            _ => return Err(format!("Invalid move type: {w}")),
        })
    }
}

/// `parseMoveSequence`: commas or blanks between words; `size` is SizeUp then SizeDownFanout.
pub fn parse_move_sequence(sequence: &str) -> Result<Vec<Move>, String> {
    let mut out = Vec::new();
    for w in sequence.replace(',', " ").split_whitespace() {
        if w.eq_ignore_ascii_case("size") {
            out.push(Move::SizeUp);
            out.push(Move::SizeDownFanout);
            continue;
        }
        out.push(Move::from_word(w)?);
    }
    Ok(out)
}

impl Args {
    /// `proc repair_timing`: neither `-setup` nor `-hold` means both.
    pub fn parse(args: &[String]) -> Result<Args, String> {
        let mut a = Args {
            setup: false,
            hold: false,
            setup_margin: 0.0,
            hold_margin: 0.0,
            max_buffer_percent: 20.0,
            repair_tns_end_percent: 1.0,
            sequence: Vec::new(),
            phases: None,
            recover_power: false,
            skip_pin_swap: false,
            skip_gate_cloning: false,
            skip_size_down_fanout: false,
            skip_buffering: false,
            skip_buffer_removal: false,
            skip_vt_swap: false,
            max_passes: 10000,
            max_iterations: -1,
            max_repairs_per_pass: 1,
            max_utilization: None,
            verbose: false,
            skip_last_gasp: false,
            skip_crit_vt_swap: false,
            match_cell_footprint: false,
            allow_setup_violations: false,
        };
        let mut i = 0;
        while i < args.len() {
            let f = args[i].as_str();
            let value = |i: usize| args.get(i + 1).cloned().ok_or_else(|| format!("repair_timing {f} needs a value"));
            match f {
                "-setup" => a.setup = true,
                "-hold" => a.hold = true,
                "-setup_margin" => {
                    a.setup_margin = value(i)?.parse().map_err(|_| "repair_timing -setup_margin: not a number".to_string())?;
                    i += 1;
                }
                "-repair_tns" => {
                    let v: f64 = value(i)?.parse().map_err(|_| "repair_timing -repair_tns: not a number".to_string())?;
                    a.repair_tns_end_percent = v / 100.0;
                    i += 1;
                }
                "-sequence" => {
                    a.sequence = parse_move_sequence(&value(i)?)?;
                    i += 1;
                }
                "-phases" | "-policy" | "-policies" => {
                    a.phases = Some(value(i)?);
                    i += 1;
                }
                "-recover_power" => {
                    a.recover_power = true;
                    i += 1;
                }
                "-max_passes" | "-max_iterations" | "-max_repairs_per_pass" => {
                    let v: i64 = value(i)?.parse().map_err(|_| format!("repair_timing {f}: not an integer"))?;
                    match f {
                        "-max_passes" => a.max_passes = v,
                        "-max_iterations" => a.max_iterations = v,
                        _ => a.max_repairs_per_pass = v,
                    }
                    i += 1;
                }
                "-max_utilization" => {
                    a.max_utilization = Some(value(i)?);
                    i += 1;
                }
                "-hold_margin" => {
                    a.hold_margin = value(i)?.parse().map_err(|_| "repair_timing -hold_margin: not a number".to_string())?;
                    i += 1;
                }
                "-max_buffer_percent" => {
                    let v: f64 = value(i)?.parse().map_err(|_| "repair_timing -max_buffer_percent: not a number".to_string())?;
                    if !(0.0..=100.0).contains(&v) {
                        return Err("repair_timing -max_buffer_percent: must be between 0 and 100".into());
                    }
                    a.max_buffer_percent = v;
                    i += 1;
                }
                // Read by the library selection only: accepted with its value.
                "-libraries" => i += 1,
                "-skip_pin_swap" => a.skip_pin_swap = true,
                "-skip_gate_cloning" => a.skip_gate_cloning = true,
                "-skip_size_down" => a.skip_size_down_fanout = true,
                "-skip_buffering" => a.skip_buffering = true,
                "-skip_buffer_removal" => a.skip_buffer_removal = true,
                "-skip_vt_swap" => a.skip_vt_swap = true,
                "-skip_last_gasp" => a.skip_last_gasp = true,
                "-skip_crit_vt_swap" => a.skip_crit_vt_swap = true,
                "-allow_setup_violations" => a.allow_setup_violations = true,
                "-match_cell_footprint" => a.match_cell_footprint = true,
                "-verbose" => a.verbose = true,
                other => return Err(format!("repair_timing {other}: not modelled")),
            }
            i += 1;
        }
        if !a.setup && !a.hold {
            a.setup = true;
            a.hold = true;
        }
        Ok(a)
    }
}

/// `buildMainMoveSequence`. `has_vt_swap_cells`: more than one VT category among the libraries.
pub fn move_sequence(a: &Args, has_vt_swap_cells: bool) -> Vec<Move> {
    let mut seq = Vec::new();
    let mut push = |enabled: bool, m: Move| {
        if enabled {
            seq.push(m);
        }
    };
    if !a.sequence.is_empty() {
        for &m in &a.sequence {
            match m {
                Move::Buffer => push(!a.skip_buffering, m),
                Move::Unbuffer => push(!a.skip_buffer_removal, m),
                Move::SwapPins => push(!a.skip_pin_swap, m),
                Move::SizeUp | Move::SizeUpMatch | Move::Reroute => push(true, m),
                Move::SizeDownFanout => push(!a.skip_size_down_fanout, m),
                Move::Clone => push(!a.skip_gate_cloning, m),
                Move::SplitLoad => push(!a.skip_buffering, m),
                Move::VtSwap => push(!a.skip_vt_swap && has_vt_swap_cells, m),
            }
        }
    } else {
        push(!a.skip_buffer_removal, Move::Unbuffer);
        push(!a.skip_vt_swap && has_vt_swap_cells, Move::VtSwap);
        push(true, Move::SizeUp);
        push(!a.skip_pin_swap, Move::SwapPins);
        push(!a.skip_buffering, Move::Buffer);
        push(!a.skip_gate_cloning, Move::Clone);
        push(!a.skip_buffering, Move::SplitLoad);
    }
    seq
}

/// A timing point and its slack (seconds).
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    pub pin: String,
    pub slack: f32,
}

/// `collectViolatingEndpoints` / `collectViolatingStartpoints`: every point whose slack is fuzzily
/// below the margin, stable-sorted by slack — equal slacks keep the timer's order.
pub fn collect_violating(points: &[Point], margin: f32) -> Vec<Point> {
    let mut v: Vec<Point> = points.iter().filter(|p| fuzzy::less(p.slack, margin)).cloned().collect();
    v.sort_by(|a, b| a.slack.partial_cmp(&b.slack).unwrap_or(std::cmp::Ordering::Equal));
    v
}

/// The summary `prepareForPhasePipeline` logs after the move sequence: RSZ-0098 when nothing
/// violates, else RSZ-0094 and RSZ-0099 with `max(int(N × repair_tns), 1)` endpoints to repair,
/// and RSZ-0221 for a custom phase list.
pub fn preamble(seq: &[Move], violating: usize, repair_tns_end_percent: f64, phases: Option<&str>) -> Vec<String> {
    let mut lines = vec![format!(
        "[INFO RSZ-0100] Repair move sequence: {}",
        seq.iter().map(|m| format!("{} ", m.name())).collect::<String>()
    )];
    if violating == 0 {
        lines.push("[INFO RSZ-0098] No setup violations found".to_string());
        return lines;
    }
    lines.push(format!("[INFO RSZ-0094] Found {violating} endpoints with setup violations."));
    let max_end_repairs = ((violating as f64 * repair_tns_end_percent) as i64).max(1);
    lines.push(format!(
        "[INFO RSZ-0099] Repairing {max_end_repairs} out of {violating} ({:.2}%) violating endpoints...",
        repair_tns_end_percent * 100.0
    ));
    // `reportCustomPhaseSetup`: the phase list as the user gave it.
    if let Some(p) = phases.filter(|p| !p.is_empty()) {
        lines.push(format!("[INFO RSZ-0221] Using custom phase sequence: {p}"));
    }
    lines
}

/// `Unit::asString(float value, digits)`: `INF` / `-INF` when `|value| ≥ INF × .1` (`float`
/// against a `double` product, compared in `double`); else `value / scale` in `float`, an
/// absolute value under 1e-6 printed as 0 (no `-0.000`), `%.<digits>f`.
pub fn unit_as_string(value: f32, scale: f32, digits: usize) -> String {
    if f64::from(value.abs()) >= f64::from(1e30f32) * 0.1 {
        return if value > 0.0 { "INF".into() } else { "-INF".into() };
    }
    let mut scaled = value / scale;
    if scaled.abs() < 1e-6 {
        scaled = 0.0;
    }
    format!("{:.*}", digits, f64::from(scaled))
}

/// `delayAsString(value, digits)`: the time unit's [`unit_as_string`].
pub fn delay_as_string(value: f32, digits: usize, time_scale: f32) -> String {
    unit_as_string(value, time_scale, digits)
}

/// A float as `fmt`'s `{}` prints it: the shortest digits that read back to the same float, in
/// fixed notation when the decimal exponent is in [-4, 7) (a float's `digits10 + 1`), else as
/// `d.ddde±XX` with at least two exponent digits.
pub fn fmt_float(v: f32) -> String {
    if v.is_nan() {
        return "nan".into();
    }
    if v.is_infinite() {
        return if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let sci = format!("{v:e}");
    let (mantissa, exp) = sci.split_once('e').expect("exponent form");
    let exp: i32 = exp.parse().expect("an exponent");
    if (-4..7).contains(&exp) {
        format!("{v}")
    } else {
        format!("{mantissa}e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
    }
}

/// `Search::worstSlack`: over the endpoints in the timer's order, the first strictly worse.
pub fn worst_slack(endpoints: &[Point]) -> (f32, Option<&Point>) {
    let mut worst = SLACK_INIT;
    let mut at = None;
    for p in endpoints {
        if !fuzzy::equal(p.slack, SLACK_INIT) && fuzzy::less(p.slack, worst) {
            worst = p.slack;
            at = Some(p);
        }
    }
    (worst, at)
}

/// `Search::totalNegativeSlack`: every endpoint's slack below zero, summed in double.
pub fn total_negative_slack(endpoints: &[Point]) -> f32 {
    let mut tns = 0.0f64;
    for p in endpoints {
        if fuzzy::less(p.slack, 0.0) {
            tns += f64::from(p.slack);
        }
    }
    tns as f32
}

/// `getOverallStartpointTns(false)`: each violating startpoint's own slack below zero, in float.
pub fn startpoint_tns(violating_startpoints: &[Point]) -> f32 {
    let mut tns = 0.0f32;
    for p in violating_startpoints {
        if p.slack < 0.0 {
            tns += p.slack;
        }
    }
    tns
}

/// The progress table's three header lines (`printProgress(0, …)`).
pub fn progress_header() -> Vec<String> {
    vec![
        "   Iter   | Removed | Resized | Inserted | Cloned |  Pin  |   Area   |    WNS   |   StTNS    |   EnTNS    |  Viol  |  Worst  ".to_string(),
        "          | Buffers |  Gates  | Buffers  |  Gates | Swaps |          |          |            |            | Endpts | St/EnPt ".to_string(),
        "-".repeat(126),
    ]
}

/// One progress row's values: the move totals by column, the area growth (%), the slacks.
#[derive(Debug, Clone, PartialEq)]
pub struct Row<'a> {
    pub iter: &'a str,
    pub removed: i64,
    pub resized: i64,
    pub inserted: i64,
    pub cloned: i64,
    pub swaps: i64,
    pub area_growth_percent: f64,
    pub wns: f32,
    pub st_tns: f32,
    pub en_tns: f32,
    pub viol: usize,
    pub worst: &'a str,
}

/// A progress row as `printProgress` / `printFinalProgress` format it.
pub fn progress_row(r: &Row<'_>, time_scale: f32) -> String {
    format!(
        "{: >9} | {: >7} | {: >7} | {: >8} | {: >6} | {: >5} | {: >+7.1}% | {: >8} | {: >10} | {: >10} | {: >6} | {}",
        r.iter,
        r.removed,
        r.resized,
        r.inserted,
        r.cloned,
        r.swaps,
        r.area_growth_percent,
        delay_as_string(r.wns, 3, time_scale),
        delay_as_string(r.st_tns, 1, time_scale),
        delay_as_string(r.en_tns, 1, time_scale),
        r.viol,
        r.worst
    )
}

/// The progress header and row 0 (`printProgress(0, …, '*')`): nothing moved yet, area unchanged.
pub fn row0(endpoints: &[Point], violating_endpoints: usize, violating_startpoints: &[Point], time_scale: f32) -> Vec<String> {
    let (wns, worst) = worst_slack(endpoints);
    let mut lines = progress_header();
    lines.push(progress_row(
        &Row {
            iter: "0*",
            removed: 0,
            resized: 0,
            inserted: 0,
            cloned: 0,
            swaps: 0,
            area_growth_percent: 0.0,
            wns,
            st_tns: startpoint_tns(violating_startpoints),
            en_tns: total_negative_slack(endpoints),
            viol: violating_endpoints,
            worst: worst.map(|p| p.pin.as_str()).unwrap_or(""),
        },
        time_scale,
    ));
    lines
}

/// The search's endpoints (`Sta::endpoints`, in vertex order) and startpoints (input ports and
/// register outputs that are not clock pins, `walkStartpoints`), each with its slack.
///
/// An endpoint (`Search::isEndpoint`): a vertex with fanin that has timing checks, carries an
/// output delay, or has no fanout. ⚠️ One WITH fanout times its slack through the path ends
/// downstream (`wnsSlacks`), which is not modelled: refused.
pub fn timing_points(g: &vyges_sta::graph::Graph<'_>, search: &vyges_sta::search::Search<'_, '_>, ssdc: &vyges_sta::sdc::Sdc, libs: &crate::preamble::Libs, clocks: &std::collections::BTreeSet<usize>) -> Result<(Vec<Point>, Vec<Point>), String> {
    use vyges_sta::graph::EdgeKind;
    use vyges_sta::liberty::Role;
    let role = |e: usize| match g.edges[e].kind {
        EdgeKind::Gate { set } => {
            let vx = &g.vertices[g.edges[e].to];
            Some(g.libs[vx.lib.expect("an instance pin")].cells[vx.cell.as_deref().expect("its cell")].arc_sets[set].role)
        }
        EdgeKind::Wire => None,
    };
    let is_check = |e: usize| matches!(role(e), Some(Role::Setup | Role::Hold | Role::Recovery | Role::Removal));
    let mut ends = Vec::new();
    let mut starts = Vec::new();
    for (v, vx) in g.vertices.iter().enumerate() {
        let fanin = g.in_edges[v].iter().any(|&e| !is_check(e));
        let fanout = g.out_edges[v].iter().any(|&e| !is_check(e));
        let checks = g.in_edges[v].iter().any(|&e| is_check(e));
        let port = vx.lib.is_none();
        let constrained = port && ssdc.output_delays.iter().any(|d| d.port == vx.name);
        if fanin && (checks || constrained || !fanout) {
            if fanout {
                return Err(format!("endpoint {} has fanout: its slack through the path ends downstream is not modelled", vx.name));
            }
            ends.push(Point { pin: vx.name.clone(), slack: search.vertex_slack(v) });
        }
        if clocks.contains(&v) || !vx.is_driver {
            continue;
        }
        let register = vx.cell.as_deref().and_then(|c| libs.link_cell(c)).is_some_and(|c| !c.sequentials.is_empty() || c.has_seq_bank);
        if port || register {
            starts.push(Point { pin: vx.name.clone(), slack: search.vertex_slack(v) });
        }
    }
    Ok((ends, starts))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(pin: &str, slack: f32) -> Point {
        Point { pin: pin.into(), slack }
    }

    /// Rules (proc repair_timing, buildMainMoveSequence): neither flag means both; the default
    /// sequence drops what a -skip names; an explicit -sequence keeps its own order.
    #[test]
    fn the_move_sequence_follows_the_flags() {
        let a = Args::parse(&["-setup".into(), "-skip_pin_swap".into(), "-skip_gate_cloning".into()]).unwrap();
        assert!(a.setup && !a.hold);
        let names: Vec<&str> = move_sequence(&a, false).iter().map(|m| m.name()).collect();
        assert_eq!(names, ["UnbufferMove", "SizeUpMove", "BufferMove", "SplitLoadMove"]);
        let both = Args::parse(&[]).unwrap();
        assert!(both.setup && both.hold);
        let s = Args::parse(&["-sequence".into(), "size_down_fanout sizeup".into()]).unwrap();
        assert_eq!(move_sequence(&s, false), [Move::SizeDownFanout, Move::SizeUp]);
        assert_eq!(parse_move_sequence("size").unwrap(), [Move::SizeUp, Move::SizeDownFanout]);
    }

    /// Rule (Unit::asString): INF past 1e29, and a value under 1e-6 in the unit prints as 0 —
    /// never `-0.000`.
    #[test]
    fn a_delay_prints_inf_and_never_negative_zero() {
        assert_eq!(delay_as_string(1e30, 3, 1e-9), "INF");
        assert_eq!(delay_as_string(-1e30, 3, 1e-9), "-INF");
        assert_eq!(delay_as_string(-1e-16, 3, 1e-9), "0.000");
        assert_eq!(delay_as_string(-0.0004e-9, 3, 1e-9), "-0.000");
        assert_eq!(delay_as_string(-0.088e-9, 3, 1e-9), "-0.088");
    }

    /// Rule (fmt `{}` on a float): shortest round-trip digits; fixed for exponents -4..6, else
    /// exponent form with a sign and two digits. Values from the reference's debug lines.
    #[test]
    fn floats_print_as_fmt_does() {
        assert_eq!(fmt_float(1.3929026e-10), "1.3929026e-10");
        assert_eq!(fmt_float(6.073e-14), "6.073e-14");
        assert_eq!(fmt_float(10.0), "10");
        assert_eq!(fmt_float(0.0001), "0.0001");
        assert_eq!(fmt_float(0.00001), "1e-05");
        assert_eq!(fmt_float(-2.5e-11), "-2.5e-11");
        assert_eq!(fmt_float(1.0e7), "1e+07");
        assert_eq!(fmt_float(0.0), "0");
    }

    /// Rule (prepareForPhasePipeline): RSZ-0099 repairs max(int(N × pct), 1) endpoints.
    #[test]
    fn the_summary_names_what_will_be_repaired() {
        let l = preamble(&[Move::SizeUp], 4, 1.0, None);
        assert_eq!(l[0], "[INFO RSZ-0100] Repair move sequence: SizeUpMove ");
        assert_eq!(l[2], "[INFO RSZ-0099] Repairing 4 out of 4 (100.00%) violating endpoints...");
        assert_eq!(preamble(&[], 3, 0.1, None)[2], "[INFO RSZ-0099] Repairing 1 out of 3 (10.00%) violating endpoints...");
        assert_eq!(preamble(&[], 0, 1.0, Some("LEGACY"))[1], "[INFO RSZ-0098] No setup violations found");
        // RSZ-0221 only after a violation summary (`prepareForPhasePipeline` returns before it).
        assert_eq!(preamble(&[], 2, 1.0, Some("LEGACY"))[3], "[INFO RSZ-0221] Using custom phase sequence: LEGACY");
        assert_eq!(preamble(&[], 0, 1.0, Some("LEGACY")).len(), 2);
    }

    /// Rules (worstSlack, collectViolating*): the FIRST strictly worse endpoint wins a tie; the
    /// violators keep the timer's order among equal slacks.
    #[test]
    fn ties_keep_the_timers_order() {
        let ends = [p("a/D", -1e-10), p("b/D", -3e-10), p("c/D", -3e-10), p("d/D", 1e-10)];
        assert_eq!(worst_slack(&ends).1.unwrap().pin, "b/D");
        let v = collect_violating(&ends, 0.0);
        assert_eq!(v.iter().map(|x| x.pin.as_str()).collect::<Vec<_>>(), ["b/D", "c/D", "a/D"]);
        let row = row0(&ends, 3, &[], 1e-9);
        assert_eq!(row[3], "       0* |       0 |       0 |        0 |      0 |     0 |    +0.0% |   -0.300 |        0.0 |       -0.7 |      3 | b/D");
    }
}
