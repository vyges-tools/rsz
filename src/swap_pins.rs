// SPDX-License-Identifier: Apache-2.0
//! SwapPinsMove: the path's input pin traded for a logically equivalent one that drives the output
//! faster.
//!
//! One function per stage, named after the reference's and in its call order:
//! - [`equiv_cell_pins`] — `SwapPinsGenerator::equivCellPins`: the inputs the cell's every output
//!   function is symmetric in with the path's input ([`is_port_equiv`], a truth-table check);
//! - [`find_swap_pin_candidate`] — `Resizer::findSwapPinCandidate`: the current input's delay to the
//!   driver port and each equivalent input's, at the instance's input slews; the fastest that
//!   beats the current one;
//! - [`select_swap_port`] — `selectSwapPort`: both, and whether the swap is a different port.
//!
//! The sequencer ([`crate::repair_setup`]) resolves the driver, applies the dont_touch and
//! already-swapped guards, keeps the candidate when it is strictly faster (`estimate().legal`) and
//! commits it (`SwapPinsCandidate::apply` → `Resizer::swapPins`).

use std::collections::HashMap;

use vyges_sta::func_expr::FuncExpr;
use vyges_sta::liberty::{Cell, Direction, Model, Port};

use crate::timing::INF;

/// `PortDirection` as the reference's reader sets it: an output with a `three_state` is tristate.
fn is_output(p: &Port) -> bool {
    p.direction == Direction::Output && !p.is_any_tristate()
}

fn is_input(p: &Port) -> bool {
    p.direction == Direction::Input
}

/// `SwapPinsGenerator::equivCellPins(cell, input_port)`: none for a sequential or isolation cell,
/// or one with a port that is neither input nor output (power and ground pins aside); none with
/// no output or fewer than two inputs; else, in port order, each other input that every output
/// with a function is symmetric in together with `input_port`. `Err` when a function names
/// something other than an input (the reference would read past its truth table).
pub fn equiv_cell_pins(cell: &Cell, input_port: &str) -> Result<Vec<String>, String> {
    if !cell.seqs.is_empty() || cell.has_seq_bank || cell.is_isolation_cell {
        return Ok(Vec::new());
    }
    let (mut outputs, mut inputs) = (0, 0);
    for p in &cell.ports {
        if is_output(p) {
            outputs += 1;
        } else if is_input(p) {
            inputs += 1;
        } else {
            return Ok(Vec::new());
        }
    }
    if outputs < 1 || inputs < 2 {
        return Ok(Vec::new());
    }
    let mut ports = Vec::new();
    for candidate in cell.ports.iter().filter(|p| is_input(p)) {
        let mut is_equivalent: Option<bool> = None;
        for out in &cell.ports {
            let Some(function) = out.function.as_deref() else { continue };
            if !is_output(out) || input_port == candidate.name {
                continue;
            }
            let expr = FuncExpr::parse(function).map_err(|e| format!("{}/{}: {e}", cell.name, out.name))?;
            let r = is_port_equiv(&expr, cell, input_port, &candidate.name)?;
            is_equivalent = Some(is_equivalent.map_or(r, |e| e && r));
        }
        if is_equivalent == Some(true) && !ports.contains(&candidate.name) {
            ports.push(candidate.name.clone());
        }
    }
    // Sorted by port id: the cell's port order, which the loop already keeps.
    Ok(ports)
}

/// `isPortEqiv`: the function's truth table over every input is unchanged when ports `a` and `b`
/// trade values. More than 16 inputs: not equivalent. (The reference assigns the table's bits to
/// the inputs in a hash map's order; a swap's effect does not depend on that assignment.)
pub fn is_port_equiv(expr: &FuncExpr, cell: &Cell, a: &str, b: &str) -> Result<bool, String> {
    let inputs: Vec<&str> = cell.ports.iter().filter(|p| is_input(p)).map(|p| p.name.as_str()).collect();
    if inputs.len() > 16 {
        return Ok(false);
    }
    let table = 1usize << inputs.len();
    let bit = |name: &str| inputs.iter().position(|&n| n == name);
    let (Some(ia), Some(ib)) = (bit(a), bit(b)) else { return Ok(false) };
    for i in 0..table {
        let value = |k: usize| (i >> k) & 1 == 1;
        let plain = simulate(expr, &|n| bit(n).map(value))?;
        let swapped = simulate(expr, &|n| {
            bit(n).map(|k| if k == ia { value(ib) } else if k == ib { value(ia) } else { value(k) })
        })?;
        if plain != swapped {
            return Ok(false);
        }
    }
    Ok(true)
}

/// `simulateExpr` at one row of the table.
fn simulate(expr: &FuncExpr, port: &dyn Fn(&str) -> Option<bool>) -> Result<bool, String> {
    Ok(match expr {
        FuncExpr::Not(e) => !simulate(e, port)?,
        FuncExpr::And(l, r) => simulate(l, port)? && simulate(r, port)?,
        FuncExpr::Or(l, r) => simulate(l, port)? || simulate(r, port)?,
        FuncExpr::Xor(l, r) => simulate(l, port)? ^ simulate(r, port)?,
        FuncExpr::One => true,
        FuncExpr::Zero => false,
        FuncExpr::Port(n) => port(n).ok_or_else(|| format!("function reads {n}, not an input: not modelled"))?,
    })
}

/// `Resizer::findSwapPinCandidate(input_port, drvr_port, equiv_ports, load_cap, …)`: over the
/// non-check arcs into the driver port, each at its input transition's slew (the instance's
/// annotated input slew, else the target slew): the current input's largest delay is the base;
/// any other port keeps its FIRST arc's delay (a later arc from it writes the current input's
/// entry instead, which nothing reads — kept as the reference has it). Then, in the equivalent
/// ports' order, a port that is an input, not the current input or the driver, and faster than
/// the best so far becomes the swap. Returns the swap (the current input when none), the base
/// delay (0 when no arc) and the swap's delay (0 when no arc).
pub fn find_swap_pin_candidate(cell: &Cell, input_port: &str, drvr_port: &str, equiv_ports: &[String], load_cap: f32, in_slew: &dyn Fn(&str, usize) -> f32) -> (String, f32, f32) {
    let mut port_delays: HashMap<&str, f32> = HashMap::new();
    let mut base_delay = -INF;
    for set in cell.arc_sets.iter().filter(|s| s.to == drvr_port && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            let Model::Gate(m) = &arc.model else { continue };
            let (gate_delay, _) = m.gate_delay(in_slew(&set.from, arc.from_rf), load_cap);
            let port = set.from.as_str();
            if port == input_port {
                base_delay = base_delay.max(gate_delay);
            } else if !port_delays.contains_key(port) {
                port_delays.insert(port, gate_delay);
            } else {
                let d = port_delays[port].max(gate_delay);
                port_delays.insert(input_port, d);
            }
        }
    }
    let reference_delay = base_delay;
    let mut swap_port = input_port.to_string();
    for port in equiv_ports {
        let is_in = cell.port(port).is_some_and(is_input);
        let Some(&port_delay) = port_delays.get(port.as_str()) else { continue };
        if !is_in || port == input_port || port == drvr_port {
            continue;
        }
        if port_delay < base_delay {
            swap_port = port.clone();
            base_delay = port_delay;
        }
    }
    let base_out = if reference_delay != -INF { reference_delay } else { 0.0 };
    let swap_out = if base_delay != -INF { base_delay } else { 0.0 };
    (swap_port, base_out, swap_out)
}

/// `selectSwapPort`: the equivalent inputs, then the fastest; `None` without an equivalent input
/// or when the fastest is the current input. Returns (swap port, current delay, swap delay).
pub fn select_swap_port(cell: &Cell, drvr_port: &str, input_port: &str, load_cap: f32, in_slew: &dyn Fn(&str, usize) -> f32) -> Result<Option<(String, f32, f32)>, String> {
    let equiv_ports = equiv_cell_pins(cell, input_port)?;
    if equiv_ports.is_empty() {
        return Ok(None);
    }
    let (swap, current, swap_delay) = find_swap_pin_candidate(cell, input_port, drvr_port, &equiv_ports, load_cap, in_slew);
    Ok((swap != input_port).then_some((swap, current, swap_delay)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty::Library;
    use vyges_sta::liberty_parse::parse as lparse;

    fn cell(body: &str) -> Cell {
        let text = format!(
            r#"library (l) {{ lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
              index_1 ("0, 1") ; index_2 ("0, 1") ; }} cell (c) {{ {body} }} }}"#
        );
        Library::read(&lparse(&text).unwrap()).unwrap().cells["c"].clone()
    }

    fn arc(from: &str, d: f32) -> String {
        format!(
            r#"timing () {{ related_pin : "{from}" ; timing_sense : positive_unate ;
                cell_rise (t) {{ values ("{d}, {d}", "{d}, {d}") ; }} cell_fall (t) {{ values ("{d}, {d}", "{d}, {d}") ; }}
                rise_transition (t) {{ values ("0, 0", "0, 0") ; }} fall_transition (t) {{ values ("0, 0", "0, 0") ; }} }}"#
        )
    }

    // Rule (equivCellPins / isPortEqiv): an input is a swap partner when every output function is
    // unchanged by trading the two; an AOI's inputs pair only within their AND term.
    #[test]
    fn swap_partners_are_the_symmetric_inputs() {
        let and3 = cell(r#"pin (A) { direction : input ; } pin (B) { direction : input ; } pin (C) { direction : input ; }
            pin (Z) { direction : output ; function : "A*B*C" ; }"#);
        assert_eq!(equiv_cell_pins(&and3, "A").unwrap(), vec!["B", "C"]);
        let aoi = cell(r#"pin (A1) { direction : input ; } pin (A2) { direction : input ; } pin (B) { direction : input ; }
            pin (ZN) { direction : output ; function : "!((A1*A2)+B)" ; }"#);
        assert_eq!(equiv_cell_pins(&aoi, "A1").unwrap(), vec!["A2"]);
        assert!(equiv_cell_pins(&aoi, "B").unwrap().is_empty());
        let one_in = cell(r#"pin (A) { direction : input ; } pin (Z) { direction : output ; function : "!A" ; }"#);
        assert!(equiv_cell_pins(&one_in, "A").unwrap().is_empty(), "fewer than two inputs");
    }

    // Rule (findSwapPinCandidate): the swap is the fastest equivalent input strictly faster than
    // the current one, the equivalent ports taken in order (a tie keeps the earlier).
    #[test]
    fn the_fastest_strictly_faster_input_wins() {
        let c = cell(&format!(
            r#"pin (A) {{ direction : input ; }} pin (B) {{ direction : input ; }} pin (C) {{ direction : input ; }}
               pin (Z) {{ direction : output ; function : "A*B*C" ; {} {} {} }}"#,
            arc("A", 0.3),
            arc("B", 0.1),
            arc("C", 0.1)
        ));
        let slew = |_: &str, _: usize| 0.0;
        let (swap, cur, sw) = find_swap_pin_candidate(&c, "A", "Z", &["B".into(), "C".into()], 0.0, &slew);
        assert_eq!(swap, "B", "C ties B and comes later");
        assert!(cur > sw);
        assert_eq!(select_swap_port(&c, "Z", "B", 0.0, &slew).unwrap(), None, "nothing beats B");
    }
}
