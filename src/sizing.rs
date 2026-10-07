// SPDX-License-Identifier: Apache-2.0
//! Which cells a cell may be swapped for, and which of them fits a load: `makeEquivCells`,
//! `getSwappableCells`, `bufferDelay`, `findTargetCell`.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_sta::func_expr::FuncExpr;
use vyges_sta::liberty::{Cell, Direction, Model};

use crate::preamble::{Libs, Master};
use crate::timing::INF;
use crate::Stop;

/// `equiv_cells_`: each cell's class, its members by decreasing drive resistance. A cell whose
/// equivalence this crate cannot decide is in `undecided` — asking for its class is refused.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EquivCells {
    pub classes: Vec<Vec<String>>,
    pub class_of: HashMap<String, usize>,
    pub undecided: BTreeSet<String>,
}

/// A port's direction as `LibertyPort::equiv` compares it: an output with a `three_state`
/// function is the reference's TRISTATE direction.
fn dir_key(direction: Direction, three_state: bool) -> &'static str {
    match direction {
        Direction::Input => "input",
        Direction::Output if three_state => "tristate",
        Direction::Output => "output",
        Direction::Tristate => "tristate",
        Direction::Bidirect => "bidirect",
        Direction::Internal => "internal",
        Direction::Unknown => "unknown",
    }
}

fn parse(f: Option<&String>) -> Result<Option<FuncExpr>, String> {
    f.map(|s| FuncExpr::parse(s)).transpose()
}

/// `sta::equivCells(c1, c2)`: same ports (count, and by name the same direction and power/ground
/// type — `pg_pin`s count), the same functions and tristate enables (structurally), the same
/// sequentials and statetables, and — only when the cell has no function — the same timing arcs.
/// `None` where this crate cannot decide: `ff_bank`/`latch_bank`, statetables, a function-less
/// cell (arc comparison), or a function it cannot parse.
pub fn equiv_cells(c1: &Cell, c2: &Cell) -> Option<bool> {
    if c1.ports.len() + c1.pg_pins.len() != c2.ports.len() + c2.pg_pins.len() {
        return Some(false);
    }
    for p1 in &c1.ports {
        let Some(p2) = c2.port(&p1.name) else { return Some(false) };
        if dir_key(p1.direction, p1.three_state.is_some()) != dir_key(p2.direction, p2.three_state.is_some()) {
            return Some(false);
        }
    }
    for (name, ty) in &c1.pg_pins {
        if !c2.pg_pins.iter().any(|(n, t)| n == name && t == ty) {
            return Some(false);
        }
    }
    for p1 in &c1.ports {
        let p2 = c2.port(&p1.name).expect("checked above");
        let (f1, f2) = (parse(p1.function.as_ref()).ok()?, parse(p2.function.as_ref()).ok()?);
        let (t1, t2) = (parse(p1.three_state.as_ref()).ok()?, parse(p2.three_state.as_ref()).ok()?);
        if !(FuncExpr::equiv(f1.as_ref(), f2.as_ref()) && FuncExpr::equiv(t1.as_ref(), t2.as_ref())) {
            return Some(false);
        }
    }
    if c1.has_seq_bank || c2.has_seq_bank || c1.has_statetable || c2.has_statetable {
        return None;
    }
    if !equiv_cell_sequentials(c1, c2)? {
        return Some(false);
    }
    if !c1.ports.iter().any(|p| p.function.is_some()) {
        return None;
    }
    Some(true)
}

/// `equivCellSequentials`: pairwise in order, the same kind (register or latch), clock, data,
/// clear and preset (structurally), output and inverted output ports (by name: both are the
/// cell's internal ports), and clear/preset output values — and the same count. `None` where an
/// expression does not parse.
fn equiv_cell_sequentials(c1: &Cell, c2: &Cell) -> Option<bool> {
    if c1.seqs.len() != c2.seqs.len() {
        return Some(false);
    }
    for (a, b) in c1.seqs.iter().zip(&c2.seqs) {
        let same = |x: Option<&String>, y: Option<&String>| -> Option<bool> { Some(FuncExpr::equiv(parse(x).ok()?.as_ref(), parse(y).ok()?.as_ref())) };
        if !(a.is_register == b.is_register
            && same(a.clock.as_ref(), b.clock.as_ref())?
            && same(a.data.as_ref(), b.data.as_ref())?
            && a.output == b.output
            && a.output_inv == b.output_inv
            && same(a.clear.as_ref(), b.clear.as_ref())?
            && same(a.preset.as_ref(), b.preset.as_ref())?
            && a.clear_preset_var1 == b.clear_preset_var1
            && a.clear_preset_var2 == b.clear_preset_var2)
        {
            return Some(false);
        }
    }
    Some(true)
}

/// `Resizer::cellDriveResistance`: the first OUTPUT port's drive resistance (0 without one).
pub fn cell_drive_resistance(cell: &Cell) -> f32 {
    cell.ports.iter().find(|p| p.direction == Direction::Output).map_or(0.0, |p| cell.drive_resistance(&p.name))
}

/// `Resizer::makeEquivCells`: every link cell, in library then name order, joins the first class
/// whose FIRST member it is equivalent to, else starts one; classes of one are dropped; each class
/// is stable-sorted by decreasing [`cell_drive_resistance`].
pub fn make_equiv_cells(libs: &Libs) -> EquivCells {
    let mut out = EquivCells::default();
    let mut classes: Vec<Vec<&Cell>> = Vec::new();
    for (li, lib) in libs.libs.iter().enumerate() {
        for cell in lib.cells.values() {
            if !libs.is_link_cell(li, &cell.name) {
                continue;
            }
            let mut placed = false;
            let mut undecided = false;
            for class in classes.iter_mut() {
                match equiv_cells(class[0], cell) {
                    Some(true) => {
                        class.push(cell);
                        placed = true;
                        break;
                    }
                    Some(false) => {}
                    None => {
                        // Neither this cell's class nor that one's membership is known: asking
                        // for either is refused (the class's members would be missing a cell).
                        undecided = true;
                        out.undecided.extend(class.iter().map(|c| c.name.clone()));
                        break;
                    }
                }
            }
            if undecided {
                out.undecided.insert(cell.name.clone());
            } else if !placed {
                classes.push(vec![cell]);
            }
        }
    }
    for mut members in classes.into_iter().filter(|m| m.len() >= 2) {
        members.sort_by(|a, b| cell_drive_resistance(b).partial_cmp(&cell_drive_resistance(a)).expect("a number"));
        let k = out.classes.len();
        for c in &members {
            out.class_of.insert(c.name.clone(), k);
        }
        out.classes.push(members.iter().map(|c| c.name.clone()).collect());
    }
    out
}

/// The sizing inputs a resize reads.
pub struct Sizing<'a> {
    pub libs: &'a Libs,
    pub masters: &'a BTreeMap<String, Master>,
    pub dont_use: &'a BTreeSet<String>,
    pub equiv: &'a EquivCells,
    pub target_loads: &'a BTreeMap<String, f32>,
    pub tgt_slews: [f32; 2],
    /// `tgt_slew_corner_`: the scene buffer delays and slews are timed at.
    pub tgt_scene: usize,
    /// The block's sizing restrictions (`initBlock`).
    pub limits: SizingLimits,
}

/// `sizing_area_limit_` and `sizing_leakage_limit_`'s defaults: one sizing move may not grow a
/// cell's area or its leakage more than fourfold (`initBlock` writes both to the block).
pub const SIZING_AREA_LIMIT: f64 = 4.0;
pub const SIZING_LEAKAGE_LIMIT: f64 = 4.0;

/// `Resizer::initBlock`'s sizing restrictions: the block's `limit_sizing_area` /
/// `limit_sizing_leakage` double properties (`set_opt_config` writes them), else the defaults;
/// `keep_sizing_site` (a bool property, false when absent).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SizingLimits {
    pub area: Option<f64>,
    pub leakage: Option<f64>,
    pub keep_site: bool,
    /// `sizing_keep_vt_` (`keep_sizing_vt`): the source's VT category only.
    pub keep_vt: bool,
}

impl Default for SizingLimits {
    fn default() -> SizingLimits {
        SizingLimits { area: Some(SIZING_AREA_LIMIT), leakage: Some(SIZING_LEAKAGE_LIMIT), keep_site: false, keep_vt: false }
    }
}

/// A master's IMPLANT obstruction layers as a set — what `cellVTType` hashes, so two masters share
/// a VT index exactly when their sets are equal, whatever order the categories were numbered in.
fn implant_set(m: &Master) -> Vec<String> {
    let mut l = m.implant_obs.clone();
    l.sort();
    l.dedup();
    l
}

/// `Resizer::cellLeakage`: `cell_leakage_power`, else the mean of the `leakage_power` groups
/// (summed in float, divided by their count), else none.
pub fn cell_leakage(cell: &Cell) -> Option<f32> {
    if let Some(l) = cell.leakage_power {
        return Some(l);
    }
    if cell.leakage_powers.is_empty() {
        return None;
    }
    let mut total = 0.0f32;
    for l in &cell.leakage_powers {
        total += l;
    }
    Some(total / cell.leakage_powers.len() as f32)
}

impl Sizing<'_> {
    /// `Resizer::getSwappableCells(source)`: a non-core master swaps for nothing; a dont_use cell
    /// only for itself; a cell with no class for itself; else its class, in class order, without
    /// dont_use or non-link cells, cells with no master, cells whose LEF area or leakage exceeds
    /// the source's by more than the block's limits (4 each by default); a source with a `user_function_class` keeps
    /// only cells with the same one. (No site/VT keeping or footprint matching: off by default.)
    pub fn swappable_cells(&self, source: &str) -> Result<Vec<String>, Stop> {
        let Some(master) = self.masters.get(source).filter(|m| m.is_core) else { return Ok(Vec::new()) };
        if self.dont_use.contains(source) {
            return Ok(vec![source.to_string()]);
        }
        if self.equiv.undecided.contains(source) {
            return Err(Stop::refused("RSZ-EQUIV", format!("{source}: its equivalent cells need a sequential, statetable or timing-arc comparison, which is not modelled")));
        }
        let Some(&k) = self.equiv.class_of.get(source) else { return Ok(vec![source.to_string()]) };
        let source_cell = self.libs.link_cell(source).expect("a link cell");
        let source_area = master.area;
        // The source's leakage is read only when a leakage limit is in force.
        let source_leakage = if self.limits.leakage.is_some() { cell_leakage(source_cell) } else { None };
        let mut out = Vec::new();
        for name in &self.equiv.classes[k] {
            // Class members are link cells by construction.
            if self.dont_use.contains(name) {
                continue;
            }
            let Some(m) = self.masters.get(name) else { continue };
            if self.limits.area.is_some_and(|l| source_area != 0 && m.area as f64 / source_area as f64 > l) {
                continue;
            }
            let cell = self.libs.link_cell(name).expect("a link cell");
            // The ratio is float, compared with the double limit.
            if let (Some(limit), Some(src), Some(eq)) = (self.limits.leakage, source_leakage, cell_leakage(cell)) {
                if f64::from(eq / src) > limit {
                    continue;
                }
            }
            // `sizing_keep_site_`: the source's site only.
            if self.limits.keep_site && m.site != master.site {
                continue;
            }
            // `sizing_keep_vt_`: the same VT index — the same set of IMPLANT layers.
            if self.limits.keep_vt && implant_set(m) != implant_set(master) {
                continue;
            }
            if !source_cell.user_function_class.is_empty() && source_cell.user_function_class != cell.user_function_class {
                continue;
            }
            out.push(name.clone());
        }
        Ok(out)
    }

    /// `Resizer::getVTEquivCells(source)`, the source among them: none with fewer than two VT
    /// categories (`sorted_vt_categories`) or no equivalent cells; else each equivalent cell — the
    /// source kept as is — not dont_use, with a master, of ANOTHER VT category, the same area
    /// (fuzzily), site, footprint and user function class; stably sorted by leakage (none = 0),
    /// ascending; then of two neighbours in one VT category, the one sharing the longer name
    /// prefix with the source is kept (ties: the later). The last is the least leaky's — the best.
    pub fn vt_equiv_cells(&self, source: &str, vt_category_count: usize) -> Vec<String> {
        if vt_category_count < 2 {
            return Vec::new();
        }
        let (Some(&k), Some(src_master), Some(src_cell)) = (self.equiv.class_of.get(source), self.masters.get(source), self.libs.link_cell(source)) else { return Vec::new() };
        // `cellVTType(a) == cellVTType(b)`: the same set of IMPLANT layers (the index and name
        // follow from the set).
        let vt = implant_set;
        let mut out: Vec<String> = Vec::new();
        for name in &self.equiv.classes[k] {
            if name == source {
                out.push(name.clone());
                continue;
            }
            if self.dont_use.contains(name) {
                continue;
            }
            let (Some(m), Some(cell)) = (self.masters.get(name), self.libs.link_cell(name)) else { continue };
            if vt(m) == vt(src_master) || !vyges_sta::fuzzy::equal(m.area as f32, src_master.area as f32) || m.site != src_master.site || cell.footprint != src_cell.footprint || cell.user_function_class != src_cell.user_function_class {
                continue;
            }
            out.push(name.clone());
        }
        let leak = |n: &String| self.libs.link_cell(n).and_then(cell_leakage).unwrap_or(0.0);
        out.sort_by(|a, b| if leak(a) < leak(b) { std::cmp::Ordering::Less } else if leak(b) < leak(a) { std::cmp::Ordering::Greater } else { std::cmp::Ordering::Equal });
        let common = |a: &str| a.bytes().zip(source.bytes()).take_while(|(x, y)| x == y).count();
        let mut i = 0;
        while i + 1 < out.len() {
            let (c, n) = (&out[i], &out[i + 1]);
            if vt(&self.masters[c]) == vt(&self.masters[n]) {
                if common(c) > common(n) {
                    out.remove(i + 1);
                } else {
                    out.remove(i);
                }
            } else {
                i += 1;
            }
        }
        out
    }

    /// `(*target_load_map_)[cell]`: a cell not in the map reads 0 (the reference's operator[]).
    fn target_load(&self, cell: &str) -> f32 {
        self.target_loads.get(cell).copied().unwrap_or(0.0)
    }

    /// `Resizer::bufferDelay(cell, load_cap, tgt_slew_corner, max)`: over the arcs into the
    /// buffer's output (no checks), each at the target slew of its input transition and the lumped
    /// load, the larger delay per output transition (from −INF); then the larger of rise and fall.
    pub fn buffer_delay(&self, cell: &Cell, load_cap: f32) -> f32 {
        // The arcs' models at the target-slew corner (`gateTableModel(scene)`).
        let cell = self.libs.scene_cell(self.tgt_scene, &cell.name).unwrap_or(cell);
        let Some((_, output)) = cell.buffer_ports() else { return -INF };
        let mut delays = [-INF; 2];
        for set in cell.arc_sets.iter().filter(|s| s.to == output.name && !s.role.is_timing_check()) {
            for arc in &set.arcs {
                if let Model::Gate(m) = &arc.model {
                    let (delay, _) = m.gate_delay(self.tgt_slews[arc.from_rf], load_cap);
                    delays[arc.to_rf] = delays[arc.to_rf].max(delay);
                }
            }
        }
        delays[0].max(delays[1])
    }

    /// `Resizer::findTargetCell(cell, load_cap, revisiting_inst)`: among the swappable cells, in
    /// order, a buffer or inverter takes a cell that is faster with a target load within 10 % of
    /// the best distance, or closer with a delay within 10 %; any other cell the closer target
    /// load (only upsizing when revisiting). The 10 % compares in double.
    pub fn find_target_cell(&self, cell: &str, load_cap: f32, revisiting: bool) -> Result<String, Stop> {
        let swappable = self.swappable_cells(cell)?;
        let mut best = cell.to_string();
        if swappable.is_empty() {
            return Ok(best);
        }
        let c = self.libs.link_cell(cell).expect("a link cell");
        let is_buf_inv = c.is_buffer() || c.is_inverter();
        let target_load = self.target_load(cell);
        let mut best_load = target_load;
        let mut best_dist = (load_cap - target_load).abs();
        let mut best_delay = if is_buf_inv { self.buffer_delay(c, load_cap) } else { 0.0 };
        for name in &swappable {
            let t = self.libs.link_cell(name).expect("a link cell");
            let target_load = self.target_load(name);
            let delay = if is_buf_inv { self.buffer_delay(t, load_cap) } else { 0.0 };
            let dist = (load_cap - target_load).abs();
            let take = if is_buf_inv {
                (delay < best_delay && f64::from(dist) < f64::from(best_dist) * 1.1) || (dist < best_dist && f64::from(delay) < f64::from(best_delay) * 1.1)
            } else {
                dist < best_dist && (!revisiting || target_load > best_load)
            };
            if take {
                best = name.clone();
                best_dist = dist;
                best_load = target_load;
                best_delay = delay;
            }
        }
        Ok(best)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty::Library;
    use vyges_sta::liberty_parse::parse as lparse;

    fn lib(text: &str) -> Libs {
        Libs { libs: vec![Library::read(&lparse(text).unwrap()).unwrap()], ..Default::default() }
    }

    // Rule (Resizer::cellLeakage): `cell_leakage_power` when set, else the float mean of the
    // `leakage_power` groups, else none.
    #[test]
    fn leakage_is_the_cell_value_else_the_group_mean() {
        let mut c = Cell { leakage_power: Some(3.0), leakage_powers: vec![1.0, 2.0], ..Default::default() };
        assert_eq!(cell_leakage(&c), Some(3.0));
        c.leakage_power = None;
        assert_eq!(cell_leakage(&c), Some(1.5));
        c.leakage_powers.clear();
        assert_eq!(cell_leakage(&c), None);
    }

    fn ff(name: &str, ff_group: &str) -> String {
        format!(
            r#"cell ({name}) {{ {ff_group} pin (D) {{ direction : input ; }} pin (CK) {{ direction : input ; clock : true ; }}
              pin (Q) {{ direction : output ; function : "IQ" ; }} }}"#
        )
    }

    // Rule (equivCellSequentials): two flops with the same ff group (clock, next state, outputs)
    // are one class; a different next_state or a missing inverted output splits them.
    #[test]
    fn flops_with_the_same_ff_group_are_equivalent() {
        let g = r#"ff (IQ, IQN) { clocked_on : "CK" ; next_state : "D" ; }"#;
        let l = lib(&format!("library (t) {{ {} {} {} {} }}", ff("F1", g), ff("F2", g), ff("F3", r#"ff (IQ, IQN) { clocked_on : "CK" ; next_state : "!D" ; }"#), ff("F4", r#"ff (IQ) { clocked_on : "CK" ; next_state : "D" ; }"#)));
        let (a, b, c, d) = (&l.libs[0].cells["F1"], &l.libs[0].cells["F2"], &l.libs[0].cells["F3"], &l.libs[0].cells["F4"]);
        assert_eq!(equiv_cells(a, b), Some(true));
        assert_eq!(equiv_cells(a, c), Some(false));
        assert_eq!(equiv_cells(a, d), Some(false));
    }

    // Rule: a comparison this crate cannot decide makes BOTH classes undecided — the first
    // member must not silently lose a member it may have.
    #[test]
    fn an_undecided_comparison_marks_the_existing_class_too() {
        let st = r#"cell (S2) { statetable ("D", "IQ") { table : "H : - : H" ; } pin (D) { direction : input ; } pin (Q) { direction : output ; function : "IQ" ; } }"#;
        let s1 = r#"cell (S1) { pin (D) { direction : input ; } pin (Q) { direction : output ; function : "IQ" ; } }"#;
        let e = make_equiv_cells(&lib(&format!("library (t) {{ {s1} {st} }}")));
        assert!(e.undecided.contains("S1") && e.undecided.contains("S2"), "{:?}", e.undecided);
    }

    // Rule (getSwappableCells): a candidate leaking more than 4× the source is dropped even when
    // its area is within the limit (Nangate45: BUF_X4 is not a swap for BUF_X1).
    #[test]
    fn a_leakier_than_four_times_cell_is_not_swappable() {
        let buf = |n: &str, r: f32, leak: f32| {
            format!(
                r#"cell ({n}) {{ cell_leakage_power : {leak} ; pin (A) {{ direction : input ; }} pin (Z) {{ direction : output ; function : "A" ;
                  timing () {{ related_pin : "A" ; timing_sense : positive_unate ; fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }}"#
            )
        };
        let l = lib(&format!(
            r#"library (l) {{ lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
              index_1 ("0, 1") ; index_2 ("0, 1") ; }} {} {} {} }}"#,
            buf("B1", 4.0, 10.0),
            buf("B2", 2.0, 40.0),
            buf("B4", 1.0, 41.0)
        ));
        let masters: BTreeMap<String, Master> = ["B1", "B2", "B4"].iter().map(|n| (n.to_string(), Master { site: "s".into(), area: 1, width: 1, height: 1, is_core: true, logic_std: true, implant_obs: vec![] })).collect();
        let equiv = make_equiv_cells(&l);
        let (dont_use, loads) = (BTreeSet::new(), BTreeMap::new());
        let s = Sizing { libs: &l, masters: &masters, dont_use: &dont_use, equiv: &equiv, target_loads: &loads, tgt_slews: [0.0; 2], tgt_scene: 0, limits: SizingLimits::default() };
        assert_eq!(s.swappable_cells("B1").unwrap(), vec!["B1".to_string(), "B2".into()], "B4 leaks 4.1x B1");
    }

    // Rules (makeEquivCells, sta::equivCells): same ports and functions ⇒ one class, sorted by
    // DECREASING drive resistance (stable); a different function or port set is another class.
    #[test]
    fn classes_group_equivalent_cells_by_decreasing_drive() {
        let buf = |n: &str, r: f32| {
            format!(
                r#"cell ({n}) {{ pin (A) {{ direction : input ; }} pin (Z) {{ direction : output ; function : "A" ;
                  timing () {{ related_pin : "A" ; timing_sense : positive_unate ; fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }}"#
            )
        };
        let l = lib(&format!(
            r#"library (l) {{ lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
              index_1 ("0, 1") ; index_2 ("0, 1") ; }}
              {} {} {}
              cell (INV) {{ pin (A) {{ direction : input ; }} pin (Z) {{ direction : output ; function : "!A" ; }} }}
              cell (INV2) {{ pin (A) {{ direction : input ; }} pin (Z) {{ direction : output ; function : "A'" ; }} }} }}"#,
            buf("B1", 1.0),
            buf("B2", 4.0),
            buf("B3", 2.0)
        ));
        let e = make_equiv_cells(&l);
        assert_eq!(e.classes, vec![vec!["B2".to_string(), "B3".into(), "B1".into()], vec!["INV".into(), "INV2".into()]]);
    }
}
