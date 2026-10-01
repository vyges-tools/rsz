// SPDX-License-Identifier: Apache-2.0
//! A driver's own slew violation: `RepairDesign::repairDriverSlew` (the smallest size of the
//! driver that fits the limit, else the one that violates least) and, when the violation stays,
//! `RepairDesign::findSlewLoadCap` (the load the driver may see, which then caps the buffered net).

use std::cmp::Ordering;

use vyges_sta::graph::Graph;
use vyges_sta::liberty::{Cell, Model, Role, MAX};

use crate::order;
use crate::sizing::Sizing;
use crate::timing::{self, Limits};
use crate::trace::{g9, Trace};
use crate::Stop;

/// What `repairDriverSlew` reads about the driver's instance besides the graph.
pub struct Driver<'a> {
    /// The instance, its current cell, and the driver pin's port name.
    pub inst: &'a str,
    pub cell: &'a str,
    pub port: &'a str,
    /// `dontTouch(inst)`, and `isLogicStdCell(inst)`: a master of type CORE exactly.
    pub dont_touch: bool,
    pub logic_std_cell: bool,
}

/// `RepairDesign::checkDriverArcSlew(corner, inst, arc, load_cap, limit, violation)`: the arc's
/// slew at the lumped load from the slew on the instance's own input pin (an ideal clock's slew —
/// 0, no `set_clock_transition` is modelled — for a register clock-to-Q arc from an ideal clock
/// pin); over the limit, the violation is the larger excess.
#[allow(clippy::too_many_arguments)]
fn check_driver_arc_slew(g: &Graph<'_>, inst: &str, from: &str, role: Role, arc: &vyges_sta::liberty::Arc, load_cap: f32, limit: f32, clocks: &std::collections::BTreeSet<usize>, violation: &mut f32) {
    let Model::Gate(model) = &arc.model else { return };
    let pin = format!("{inst}/{from}");
    let Some(v) = g.vertices.iter().position(|x| x.name == pin && !x.is_driver) else { return };
    let in_slew = if role == Role::RegClkToQ && clocks.contains(&v) { 0.0 } else { g.slew[v][arc.from_rf][MAX] };
    let (_, arc_slew) = model.gate_delay(in_slew, load_cap);
    if arc_slew > limit {
        *violation = (arc_slew - limit).max(*violation);
    }
}

/// `RepairDesign::repairDriverSlew(corner, drvr_pin)`, from the load cap on: every swappable size
/// of the driver's cell, in `getSwappableCells` order, scored by its worst slew excess over its own
/// port's margined limit (0 with no limit); `std::ranges::sort` — two non-violating sizes by
/// liberty area, otherwise by violation — and the first. Returns the size to swap to, or `None`
/// (a top-level port, dont_touch, not a CORE master, no swappable size, or the cell it has).
///
/// At the violating `corner` (`g` is its timer): each size's limit is its CORNER port's
/// (`findSlewLimit(port, corner)`); its arcs and their models are the LINK size's (`timingArcSets()`
/// on the link cell — not the corner's models); the input slews are the corner's.
#[allow(clippy::too_many_arguments)]
pub fn repair_driver_slew(g: &Graph<'_>, corner: usize, d: &Driver<'_>, load_cap: f32, sizing: &Sizing<'_>, limits: &Limits, slew_margin: f64, clocks: &std::collections::BTreeSet<usize>, trace: &mut Trace) -> Result<Option<String>, Stop> {
    if d.dont_touch || !d.logic_std_cell {
        return Ok(None);
    }
    let equiv_cells = sizing.swappable_cells(d.cell)?;
    if equiv_cells.is_empty() {
        return Ok(None);
    }
    let mut sizes: Vec<(f32, &str, f32)> = Vec::new();
    for size_cell in &equiv_cells {
        let c = sizing.libs.link_cell(size_cell).expect("a link cell");
        let lib = sizing.libs.scene_library(corner, size_cell).expect("its library");
        let mut violation = 0.0f32;
        let port = sizing.libs.scene_cell(corner, size_cell).and_then(|sc| sc.port(d.port));
        let (limit, limit_exists) = match port {
            Some(p) => timing::find_slew_limit(lib, p.direction, p.max_transition, limits),
            None => (0.0, false),
        };
        if limit_exists {
            let limit_w_margin = (f64::from(limit) * (1.0 - slew_margin / 100.0)) as f32;
            // Not checks, not tristate enable/disable, not clock-tree path arcs (those roles carry
            // no arcs here).
            for set in c.arc_sets.iter().filter(|s| !s.role.is_timing_check()) {
                for arc in &set.arcs {
                    check_driver_arc_slew(g, d.inst, &set.from, set.role, arc, load_cap, limit_w_margin, clocks, &mut violation);
                }
            }
        }
        trace.push(format!("rdsc|{size_cell}|{limit_exists}|{}|{}|{}", g9(f64::from(if limit_exists { limit } else { 0.0 })), g9(f64::from(violation)), g9(f64::from(c.area))));
        sizes.push((violation, size_cell, c.area));
    }
    order::std_sort_by(&mut sizes, |a, b| {
        if a.0 == 0.0 && b.0 == 0.0 {
            a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal)
        } else {
            a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal)
        }
    })
    .map_err(|t| Stop::refused("RSZ-ORDER", format!("repairDriverSlew: {} sizes with a tie the reference's introsort orders by its partitioning", t.len)))?;
    let selected = sizes[0].1;
    trace.push(format!("rdsel|{selected}"));
    Ok((selected != d.cell).then(|| selected.to_string()))
}

/// `Resizer::gateDelays(drvr_port, load_cap, …)` as `gateSlewDiff` reads it: over the arcs into
/// the port (no checks), each at the target slew of its input transition, the larger slew per
/// output transition (from −INF); then the larger of rise and fall.
fn gate_slew(cell: &Cell, port: &str, load_cap: f32, tgt_slews: [f32; 2]) -> f32 {
    let mut slews = [-timing::INF; 2];
    for set in cell.arc_sets.iter().filter(|s| s.to == port && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (_, slew) = m.gate_delay(tgt_slews[arc.from_rf], load_cap);
                slews[arc.to_rf] = slews[arc.to_rf].max(slew);
            }
        }
    }
    slews[0].max(slews[1])
}

/// `RepairDesign::findSlewLoadCap(drvr_port, slew, corner)`: with no drive resistance (the LINK
/// port's), INF; otherwise a bisection in double between 0 and `2 · slew / R` (doubled while the
/// slew still fits) to within 1 %, on `gateSlewDiff` — the load passed to the timer as float, the
/// arcs' models the CORNER's (`scene_cell`) — returning the lower bound.
pub fn find_slew_load_cap(link: &Cell, scene_cell: &Cell, port: &str, slew: f64, tgt_slews: [f32; 2], trace: &mut Trace) -> f64 {
    let cell = scene_cell;
    let drvr_res = f64::from(link.drive_resistance(port));
    if drvr_res == 0.0 {
        return f64::from(timing::INF);
    }
    let diff = |cap: f64| f64::from(gate_slew(cell, port, cap as f32, tgt_slews)) - slew;
    let mut cap1 = 0.0f64;
    let mut cap2 = slew / drvr_res * 2.0;
    let tol = 0.01;
    let mut diff1 = diff(cap2);
    trace.push(format!("slc0|{}|{}|{}|{}", g9(drvr_res), g17(slew), g17(cap2), g17(diff1)));
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol {
        trace.push(format!("slc|{}|{}|{}", g17(cap1), g17(cap2), g17(diff1)));
        if diff1 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff1 = diff(cap2);
        } else {
            let cap3 = (cap1 + cap2) / 2.0;
            let diff2 = diff(cap3);
            if diff2 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff1 = diff2;
            }
        }
    }
    cap1
}

fn g17(x: f64) -> String {
    crate::trace::g(x, 17)
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty::Library;
    use vyges_sta::liberty_parse::parse as lparse;

    fn cell(r: f32) -> Library {
        let text = format!(
            r#"library (t) {{ time_unit : "1ns" ; capacitive_load_unit (1, ff) ;
              lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ; index_1 ("0, 1") ; index_2 ("0, 10") ; }}
              cell (D) {{ pin (A) {{ direction : input ; }} pin (Z) {{ direction : output ; function : "A" ;
                timing () {{ related_pin : "A" ; timing_sense : positive_unate ;
                  cell_rise (t) {{ values ("0, {r}", "0, {r}") ; }} rise_transition (t) {{ values ("0, {r}", "0, {r}") ; }}
                  cell_fall (t) {{ values ("0, {r}", "0, {r}") ; }} fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }} }}"#
        );
        Library::read(&lparse(&text).unwrap()).unwrap()
    }

    // Rule (findSlewLoadCap): the bisection keeps the LOWER bound, and stops within 1 % — on a
    // linear slew of r·C/10 (ns per fF) the answer is just under the load that meets the slew.
    #[test]
    fn the_slew_load_cap_is_the_lower_bound_within_one_percent() {
        let lib = cell(1.0);
        let c = &lib.cells["D"];
        // Slew 0.1 ns per fF; a 0.5 ns limit ⇒ 5 fF.
        let cap = find_slew_load_cap(c, c, "Z", 0.5e-9, [0.0, 0.0], &mut Trace::default());
        assert!(cap <= 5e-15 && cap > 5e-15 * 0.99, "{cap}");
    }

    // Rule (findSlewLoadCap): no drive resistance ⇒ INF, no search.
    #[test]
    fn no_drive_resistance_is_no_limit() {
        let lib = cell(0.0);
        let mut t = Trace::default();
        assert_eq!(find_slew_load_cap(&lib.cells["D"], &lib.cells["D"], "Z", 0.5e-9, [0.0, 0.0], &mut t), f64::from(timing::INF));
        assert!(t.lines.is_empty());
    }
}
