// SPDX-License-Identifier: Apache-2.0
//! `repair_timing -path_group`: the reference's `PathGroupFilter` — which group a name means
//! ([`crate::repair_timing::resolve_path_group`]), the group it makes when the constraints have
//! none of that name ([`make_path_group`]), and what the repair reads under it: the group's path
//! ends instead of every endpoint's worst path.
//!
//! Stages, in the reference's order: `resolvePathGroupName` (the Tcl, before the command runs) ->
//! `makePathGroup` -> the repair's collector (`groupEndpointSlacks`, `groupWorst`,
//! `groupSlack`, `findWorstSlackPath`, `startpointInGroup`).

use vyges_sta::graph::Graph;
use vyges_sta::netlist::{Netlist, PortDir};
use vyges_sta::sdc::PathDelay;

/// `groupStartpointKind` / `groupEndpointKind` of a group name: whether its paths start at a
/// primary input (else at a register), and end at a primary output (else at a register).
/// `gated_clock`: `None` (no group path can name its ends).
pub fn group_sides(name: &str) -> Option<(bool, bool)> {
    match name {
        "reg2reg" => Some((false, false)),
        "in2reg" => Some((true, false)),
        "reg2out" => Some((false, true)),
        "in2out" => Some((true, true)),
        _ => None,
    }
}

/// `groupStartpointKind == kPrimaryInput` of a group name: whether its paths start at a primary
/// input, at a register, or anywhere (`kAny`: `gated_clock`, whose `startpointInGroup` is true).
pub fn group_starts_at_input(name: &str) -> Option<bool> {
    group_sides(name).map(|(from_inputs, _)| from_inputs)
}

/// `staPathGroupName`: the timer's name of a group — its own, but `gated_clock` is the timer's
/// built-in [`vyges_sta::path_group::GATED_CLK_GROUP`].
pub fn sta_group_name(name: &str) -> &str {
    if name == "gated_clock" {
        vyges_sta::path_group::GATED_CLK_GROUP
    } else {
        name
    }
}

/// `primaryPins`: the top ports on one side — every input that is no clock's source (`all_inputs
/// -no_clocks`), or every output — inout ports on both.
fn primary_pins(nl: &Netlist, inputs: bool, clock_sources: &[String]) -> Vec<String> {
    nl.ports
        .iter()
        .filter(|(name, dir)| match dir {
            PortDir::Input => inputs && !clock_sources.contains(name),
            PortDir::Output => !inputs,
            PortDir::Inout => !inputs || !clock_sources.contains(name),
        })
        .map(|(name, _)| name.clone())
        .collect()
}

/// `makePathGroup`: `group_path -name <name> -from <from> -to <to>` with the sides the name
/// means — primary inputs (no clocks) or the registers' clock pins, to primary outputs or the
/// registers' data pins ([`vyges_sta::path_group::register_pins`]) — and its RSZ-0228 line. No
/// group (and no line) when a side has nothing: the repair is still restricted, and finds
/// nothing. `None` for `gated_clock` too (its ends are the timer's own group).
pub fn make_path_group(g: &Graph<'_>, nl: &Netlist, name: &str, clock_sources: &[String]) -> Option<(PathDelay, String)> {
    let (from_inputs, to_outputs) = group_sides(name)?;
    let names = |vs: Vec<usize>| -> Vec<String> { vs.into_iter().map(|v| g.vertices[v].name.clone()).collect() };
    let from = if from_inputs { primary_pins(nl, true, clock_sources) } else { names(vyges_sta::path_group::register_pins(g, false)) };
    let to = if to_outputs { primary_pins(nl, false, clock_sources) } else { names(vyges_sta::path_group::register_pins(g, true)) };
    if from.is_empty() || to.is_empty() {
        return None;
    }
    let pd = PathDelay {
        from_pins: from,
        from_clock: false,
        to_pins: to,
        to_clock: false,
        min_max: vyges_sta::liberty::MAX,
        ignore_clk_latency: false,
        break_path: false,
        delay: 0.0,
        false_path: false,
        multicycle: None,
        thrus: Vec::new(),
        group: Some(name.to_string()),
    };
    Some((pd, format!("[INFO RSZ-0228] Created path group '{name}'.")))
}

/// `PathGroupFilter::groupEndpointSlacks(max, INF)` on one scene: the group's path ends, one per
/// endpoint, in the group's order (slack, then pin name: `PathEnd::cmp`), as points whose slack
/// is the group's (`groupSlack`), not the endpoint's.
pub fn group_ends(g: &Graph<'_>, s: &vyges_sta::search::Search<'_, '_>, name: &str) -> Vec<crate::repair_timing::Point> {
    s.group_path_ends(sta_group_name(name), vyges_sta::liberty::MAX, usize::MAX, -vyges_sta::search::INF_SLACK, vyges_sta::search::INF_SLACK)
        .into_iter()
        .map(|e| crate::repair_timing::Point { pin: g.vertices[e.vertex].name.clone(), slack: e.slack, wns_slack: e.slack })
        .collect()
}

/// A progress row's numbers under a group (`getWns`, `getTns(false)`, `getWorstPin(false)` with
/// `restrictedToPathGroup`), over `violating` — the group's ends fuzzily under the margin
/// (`refreshGroupSlacks`), in the group's order — and `ends`, all of the group's:
/// - WNS: the least violating slack; none violating, the group's worst end (`groupWorst`), else 0;
/// - TNS: the violating slacks strictly under zero, summed in float in that order;
/// - Worst: the first violating end of least slack (strict `<`), else the group's worst end.
pub fn group_row(violating: &[crate::repair_timing::Point], ends: &[crate::repair_timing::Point]) -> (f32, f32, Option<String>) {
    let mut wns = f32::MAX;
    let mut worst: Option<&str> = None;
    for p in violating {
        if p.slack < wns {
            wns = p.slack;
            worst = Some(&p.pin);
        }
    }
    let mut tns = 0.0f32;
    for p in violating {
        if p.slack < 0.0 {
            tns += p.slack;
        }
    }
    match worst {
        Some(w) => (wns, tns, Some(w.to_string())),
        None => (ends.first().map_or(0.0, |p| p.slack), tns, ends.first().map(|p| p.pin.clone())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::repair_timing::Point;

    fn p(pin: &str, slack: f32) -> Point {
        Point { pin: pin.into(), slack, wns_slack: slack }
    }

    /// Rules (`getOverallEndpointWns`, `getOverallEndpointTns(false)`, `getWorstPin(false)` under
    /// `restrictedToPathGroup`): WNS is the least violating group slack, TNS the violating slacks
    /// strictly below zero (a margin end at +0.1 counts as violating, not toward TNS), Worst the
    /// first end of least slack; with none violating both fall back to the group's worst end
    /// (`groupWorst`), and an empty group reads 0 and no pin.
    #[test]
    fn a_group_rows_numbers_are_the_groups() {
        let ends = vec![p("a", -2.0), p("b", -2.0), p("c", 0.1), p("d", 3.0)];
        let violating = vec![ends[0].clone(), ends[1].clone(), ends[2].clone()];
        assert_eq!(group_row(&violating, &ends), (-2.0, -4.0, Some("a".to_string())));
        assert_eq!(group_row(&[], &ends[3..]), (3.0, 0.0, Some("d".to_string())));
        assert_eq!(group_row(&[], &[]), (0.0, 0.0, None));
    }

    /// Rules (`groupStartpointKind` / `groupEndpointKind`): `<start>2<end>`, `in` a primary input,
    /// `reg` a register, `out` a primary output; `gated_clock` names no sides.
    #[test]
    fn a_group_names_its_sides() {
        assert_eq!(group_sides("reg2reg"), Some((false, false)));
        assert_eq!(group_sides("in2reg"), Some((true, false)));
        assert_eq!(group_sides("reg2out"), Some((false, true)));
        assert_eq!(group_sides("in2out"), Some((true, true)));
        assert_eq!(group_sides("gated_clock"), None);
        // `staPathGroupName` / `groupStartpointKind`: gated_clock is the timer's "gated clock"
        // group, and its paths start anywhere.
        assert_eq!(sta_group_name("gated_clock"), "gated clock");
        assert_eq!(sta_group_name("reg2reg"), "reg2reg");
        assert_eq!(group_starts_at_input("gated_clock"), None);
        assert_eq!(group_starts_at_input("in2reg"), Some(true));
    }

    /// Rule (`primaryPins`: `all_inputs -no_clocks` / `all_outputs`): an input that is a clock's
    /// source is no start; an inout is on both sides.
    #[test]
    fn the_primary_pins_leave_out_the_clocks() {
        let nl = Netlist { insts: Vec::new(), ports: vec![("clk".into(), PortDir::Input), ("a".into(), PortDir::Input), ("y".into(), PortDir::Output), ("io".into(), PortDir::Inout)], nets: Vec::new() };
        let clocks = vec!["clk".to_string()];
        assert_eq!(primary_pins(&nl, true, &clocks), ["a", "io"]);
        assert_eq!(primary_pins(&nl, false, &clocks), ["y", "io"]);
    }
}
