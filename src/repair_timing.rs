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
//! The repair moves themselves are not modelled: with violations to repair, the caller refuses
//! after these lines.

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
}

/// `MoveType`, by the name `moveName` prints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
                // Read by the repair moves only, which are not modelled: accepted with their value.
                "-max_passes" | "-max_iterations" | "-max_repairs_per_pass" | "-max_utilization" | "-max_buffer_percent" | "-hold_margin" | "-libraries" => i += 1,
                "-skip_pin_swap" => a.skip_pin_swap = true,
                "-skip_gate_cloning" => a.skip_gate_cloning = true,
                "-skip_size_down" => a.skip_size_down_fanout = true,
                "-skip_buffering" => a.skip_buffering = true,
                "-skip_buffer_removal" => a.skip_buffer_removal = true,
                "-skip_vt_swap" => a.skip_vt_swap = true,
                "-skip_last_gasp" | "-skip_crit_vt_swap" | "-allow_setup_violations" | "-match_cell_footprint" | "-verbose" => {}
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
/// violates, else RSZ-0094 and RSZ-0099 with `max(int(N × repair_tns), 1)` endpoints to repair.
pub fn preamble(seq: &[Move], violating: usize, repair_tns_end_percent: f64) -> Vec<String> {
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
    lines
}

/// `delayAsString(value, digits)`: the time in the user unit, `%.<digits>f`.
pub fn delay_as_string(value: f32, digits: usize, time_scale: f32) -> String {
    format!("{:.*}", digits, f64::from(value / time_scale))
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

/// The progress header and row 0 (`printProgress(0, …, '*')`): nothing moved yet, area unchanged.
pub fn row0(endpoints: &[Point], violating_endpoints: usize, violating_startpoints: &[Point], time_scale: f32) -> Vec<String> {
    let (wns, worst) = worst_slack(endpoints);
    let st_tns = startpoint_tns(violating_startpoints);
    let en_tns = total_negative_slack(endpoints);
    vec![
        "   Iter   | Removed | Resized | Inserted | Cloned |  Pin  |   Area   |    WNS   |   StTNS    |   EnTNS    |  Viol  |  Worst  ".to_string(),
        "          | Buffers |  Gates  | Buffers  |  Gates | Swaps |          |          |            |            | Endpts | St/EnPt ".to_string(),
        "-".repeat(126),
        format!(
            "{: >9} | {: >7} | {: >7} | {: >8} | {: >6} | {: >5} | {: >+7.1}% | {: >8} | {: >10} | {: >10} | {: >6} | {}",
            "0*",
            0,
            0,
            0,
            0,
            0,
            0.0f64,
            delay_as_string(wns, 3, time_scale),
            delay_as_string(st_tns, 1, time_scale),
            delay_as_string(en_tns, 1, time_scale),
            violating_endpoints,
            worst.map(|p| p.pin.as_str()).unwrap_or("")
        ),
    ]
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

    /// Rule (prepareForPhasePipeline): RSZ-0099 repairs max(int(N × pct), 1) endpoints.
    #[test]
    fn the_summary_names_what_will_be_repaired() {
        let l = preamble(&[Move::SizeUp], 4, 1.0);
        assert_eq!(l[0], "[INFO RSZ-0100] Repair move sequence: SizeUpMove ");
        assert_eq!(l[2], "[INFO RSZ-0099] Repairing 4 out of 4 (100.00%) violating endpoints...");
        assert_eq!(preamble(&[], 3, 0.1)[2], "[INFO RSZ-0099] Repairing 1 out of 3 (10.00%) violating endpoints...");
        assert_eq!(preamble(&[], 0, 1.0)[1], "[INFO RSZ-0098] No setup violations found");
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
