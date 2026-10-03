// SPDX-License-Identifier: Apache-2.0
//! The resizer's preamble, in the reference's order (`Resizer::resizePreamble`): what the repair
//! reads about the LIBRARY before it looks at any net.
//!
//! Implemented here: `findBuffers` (the pruned buffer list and the weakest buffer),
//! `findTargetLoads` (the target slews and every cell's target load) and `computeSlewShapeFactor`.
//! One corner, and no liberty `k_` scale factors (the caller refuses both), so a model's value is
//! its table's. Each function below is one statement of the preamble and carries its
//! rules at the site.

use std::collections::BTreeMap;

use vyges_sta::liberty::{Cell, Library, Model, FALL, MAX, RISE};
use vyges_sta::table::GateModel;

use crate::order::std_sort_by;
use crate::Stop;

/// The libraries in the order they were read. The first is the default library; a cell's LINK
/// cell is the first library's cell of that name.
#[derive(Debug, Default)]
pub struct Libs {
    pub libs: Vec<Library>,
    /// The corners (`define_corners`), in order; none defined is one scene, `default`.
    pub scenes: Vec<String>,
    /// With corners: each scene's libraries (a `read_liberty -corner`'s, in read order) — the
    /// libraries its `sceneCell` / `scenePort` come from. Empty with one scene: it is `libs`.
    pub scene_libs: Vec<Vec<Library>>,
}

impl Libs {
    /// The number of scenes (at least one).
    pub fn scene_count(&self) -> usize {
        self.scene_libs.len().max(1)
    }

    /// A scene's name as the reference prints it.
    pub fn scene_name(&self, k: usize) -> &str {
        self.scenes.get(k).map_or("default", |s| s.as_str())
    }

    /// The libraries a scene's timer reads (its cells are the scene cells).
    pub fn scene_slice(&self, k: usize) -> &[Library] {
        if self.scene_libs.is_empty() {
            &self.libs
        } else {
            &self.scene_libs[k]
        }
    }

    /// `LibertyCell::sceneCell(scene)`: the scene's cell of that name, else the link cell.
    pub fn scene_cell(&self, k: usize, name: &str) -> Option<&Cell> {
        self.scene_slice(k).iter().find_map(|l| l.cells.get(name)).or_else(|| self.link_cell(name))
    }

    /// The scene's library holding a cell, else the link cell's library.
    pub fn scene_library(&self, k: usize, name: &str) -> Option<&Library> {
        self.scene_slice(k).iter().find(|l| l.cells.contains_key(name)).or_else(|| self.link_library(name))
    }

    /// The library of a link cell.
    pub fn link_library(&self, name: &str) -> Option<&Library> {
        self.libs.iter().find(|l| l.cells.contains_key(name))
    }

    /// `network_->defaultLibertyLibrary()`: the first library read.
    pub fn default_library(&self) -> Option<&Library> {
        self.libs.first()
    }

    /// `network_->findLibertyCell(name)`: the first library that has the name.
    pub fn link_cell(&self, name: &str) -> Option<&Cell> {
        self.libs.iter().find_map(|l| l.cells.get(name))
    }

    /// `isLinkCell(cell)`: the cell of library `lib` named `name` is the link cell.
    pub fn is_link_cell(&self, lib: usize, name: &str) -> bool {
        self.libs.iter().position(|l| l.cells.contains_key(name)) == Some(lib)
    }
}

/// What the preamble reads about a cell's LEF master.
#[derive(Debug, Clone, PartialEq)]
pub struct Master {
    pub site: String,
    /// `dbMaster::getArea()` (width × height, DBU²) and `isCore()`.
    pub area: i64,
    pub is_core: bool,
    /// `isLogicStdCell`'s test: the master's type is CORE exactly (not a CORE subtype).
    pub logic_std: bool,
    /// Names of the IMPLANT layers among the master's obstructions (`cellVTType`'s input).
    pub implant_obs: Vec<String>,
}

/// The output of `findBuffers`.
#[derive(Debug, Clone, PartialEq)]
pub struct Buffers {
    /// `buffer_cells_`, in the reference's order.
    pub cells: Vec<String>,
    /// `buffer_lowest_drive_`: the LAST of `cells`.
    pub lowest: String,
}

/// `bufferDriveResistance`: the buffer output port's drive resistance.
pub fn buffer_drive_resistance(cell: &Cell) -> f32 {
    cell.buffer_ports().map_or(0.0, |(_, out)| cell.drive_resistance(&out.name))
}

/// `LibertyPort::capacitance()`: the largest of the port's rise/fall min/max capacitances.
fn port_capacitance_max(c: &[[f32; 2]; 2]) -> f32 {
    c.iter().flatten().copied().fold(-1e30, crate::rebuffer::std_max)
}

/// `getBufferUse` with no `set_opt_config` clock-buffer pattern: a clock buffer is a cell with
/// `is_clock_cell`, or whose name contains `CLKBUF` in any case.
fn is_clock_buffer(cell: &Cell) -> bool {
    cell.is_clock_cell || cell.name.to_ascii_uppercase().contains("CLKBUF")
}

/// `Resizer::getBufferList` then `Resizer::findBuffers` (buffer pruning on, clock buffers
/// excluded — the defaults).
///
/// Rules, in order:
/// - libraries in read order; each library's `buffers()` — its cells in NAME order that are not
///   liberty `dont_use` and are buffers;
/// - kept: not a clock buffer, not in `dont_use`, not always-on / isolation / level shifter, the
///   link cell, and a LEF master exists;
/// - sorted by VT category, then by output drive resistance (`std::sort`);
/// - the footprint: when more than one appears, the first in name order whose share of the list
///   is above one half; only buffers with it are kept (all, when none is picked);
/// - five buckets of consecutive buffers (the first `n mod 5` one larger); each sorted by drive
///   resistance × input capacitance, and with one dominant site its first two taken;
/// - the weakest buffer is the last one taken.
///
/// - with buffers on more than one site: the two sites with the largest shares, and from each
///   bucket the first buffer (by R·Cin) on each of them, in site order — one per site, not two.
///
/// Refused rather than modelled (no corpus witness): a buffer master with IMPLANT obstructions
/// (the VT categories are numbered in the order masters are first asked about), and two sites
/// with equal shares (ordered by the reference's site pointers).
/// `Resizer::getBufferList`: the buffer list and what it tallies (`lib_data_`).
pub struct BufferList<'l> {
    /// Sorted by VT category, then output drive resistance (libc++ `std::sort`).
    pub cells: Vec<&'l Cell>,
    /// `cells_by_site` / `cells_by_footprint`, by name.
    pub by_site: BTreeMap<String, usize>,
    pub by_footprint: BTreeMap<String, usize>,
}

/// `Resizer::getBufferList`: libraries in read order; each library's `buffers()` — its cells in
/// NAME order that are not liberty `dont_use` and are buffers; a clock buffer skipped when
/// `exclude_clock_buffers` (setup) and kept otherwise (`repairHold`); kept: not in `dont_use`,
/// not always-on / isolation / level shifter, the link cell, with a LEF master. Sorted by VT
/// category then drive resistance — one category here (a master with IMPLANT obstructions is
/// refused), so by drive resistance, in libc++'s order for ties.
pub fn get_buffer_list<'l>(libs: &'l Libs, masters: &BTreeMap<String, Master>, dont_use: &std::collections::BTreeSet<String>, exclude_clock_buffers: bool) -> Result<BufferList<'l>, Stop> {
    let mut list: Vec<&Cell> = Vec::new();
    let mut by_site: BTreeMap<String, usize> = BTreeMap::new();
    let mut by_footprint: BTreeMap<String, usize> = BTreeMap::new();
    for (li, lib) in libs.libs.iter().enumerate() {
        for cell in lib.cells.values().filter(|c| !c.dont_use && c.is_buffer()) {
            if exclude_clock_buffers && is_clock_buffer(cell) {
                continue;
            }
            if dont_use.contains(&cell.name) || cell.always_on || cell.is_isolation_cell || cell.is_level_shifter || !libs.is_link_cell(li, &cell.name) {
                continue;
            }
            let Some(master) = masters.get(&cell.name) else { continue };
            if !master.implant_obs.is_empty() {
                return Err(Stop::refused("RSZ-VT", format!("buffer {} has IMPLANT obstructions ({}): VT categories are not modelled", cell.name, master.implant_obs.join(" "))));
            }
            *by_site.entry(master.site.clone()).or_default() += 1;
            if !cell.footprint.is_empty() {
                *by_footprint.entry(cell.footprint.clone()).or_default() += 1;
            }
            list.push(cell);
        }
    }
    crate::order::libcxx_sort_by(&mut list, |a, b| buffer_drive_resistance(a) < buffer_drive_resistance(b))
        .map_err(|h| Stop::refused("RSZ-ORDER", format!("{} buffers reach the sort's heap fallback, which is not modelled", h.len)))?;
    Ok(BufferList { cells: list, by_site, by_footprint })
}

pub fn find_buffers(libs: &Libs, masters: &BTreeMap<String, Master>, dont_use: &std::collections::BTreeSet<String>, exclude_clock_buffers: bool) -> Result<Buffers, Stop> {
    let BufferList { cells: list, by_site, by_footprint } = get_buffer_list(libs, masters, dont_use, exclude_clock_buffers)?;

    // findBuffers: the footprint.
    let mut best_footprint: Option<&str> = None;
    if by_footprint.len() > 1 {
        for (fp, &count) in by_footprint.iter().map(|(f, c)| (f.as_str(), c)) {
            let ratio = count as f32 / list.len() as f32;
            if f64::from(ratio) > 0.5 {
                best_footprint = Some(fp);
                break;
            }
        }
    }
    // The two dominant sites: by their share of the buffer list, largest first. The counts are
    // kept in a map keyed by the site's pointer, so a tie would be ordered by pointer: refused.
    let mut sites: Vec<(&str, usize)> = by_site.iter().map(|(s, c)| (s.as_str(), *c)).collect();
    sites.sort_by_key(|s| std::cmp::Reverse(s.1));
    if sites.len() > 1 && sites.windows(2).take(2).any(|w| w[0].1 == w[1].1) {
        return Err(Stop::refused("RSZ-SITES", format!("buffer sites with equal shares ({}): their order is the reference's pointer order, not modelled", sites.iter().map(|(s, c)| format!("{s}:{c}")).collect::<Vec<_>>().join(" "))));
    }
    let best_sites: Vec<&str> = sites.iter().take(2).map(|(s, _)| *s).collect();
    let kept: Vec<&Cell> = list.into_iter().filter(|c| best_footprint.is_none_or(|fp| c.footprint == fp)).collect();

    // Five buckets, two buffers each by R·Cin.
    const BUCKETS: usize = 5;
    let (n, size) = (kept.len(), kept.len() / BUCKETS);
    let remainder = n % BUCKETS;
    let mut cells = Vec::new();
    for bucket in 0..BUCKETS {
        let start = bucket * size + bucket.min(remainder);
        let end = start + size + usize::from(bucket < remainder);
        let mut members: Vec<(&Cell, f32)> = (start..end.min(n))
            .map(|i| {
                let c = kept[i];
                let cin = c.buffer_ports().map_or(0.0, |(input, _)| port_capacitance_max(&input.capacitance));
                (c, buffer_drive_resistance(c) * cin)
            })
            .collect();
        std_sort_by(&mut members, |a, b| a.1.partial_cmp(&b.1).expect("R·C is a number"))
            .map_err(|t| Stop::refused("RSZ-ORDER", format!("a bucket of {} buffers with equal R·C", t.len)))?;
        if best_sites.len() == 1 {
            cells.extend(members.iter().take(2).map(|(c, _)| c.name.clone()));
        } else {
            // One per dominant site, in site order: the bucket's first buffer on that site.
            for site in &best_sites {
                if let Some((c, _)) = members.iter().find(|(c, _)| masters.get(&c.name).is_some_and(|m| m.site == *site)) {
                    cells.push(c.name.clone());
                }
            }
        }
    }
    let lowest = cells.last().cloned().ok_or_else(|| Stop::error("RSZ-0022", "no buffers found.".into()))?;
    Ok(Buffers { cells, lowest })
}

/// `tgt_slew_load_cap_factor`: the target slew is a buffer's at ten times its input cap.
pub const TGT_SLEW_LOAD_CAP_FACTOR: f32 = 10.0;

/// `Resizer::findBufferTargetSlews` for one corner: `tgt_slews_[rise, fall]`.
///
/// Rules: every buffer's arcs from its input to its output, in arc-set then arc order; each gate
/// arc's slew at a load of `input cap (from transition, max) × 10`, the input slew 0 for a first
/// look-up and that look-up's slew for the second; summed per OUTPUT transition, averaged by a
/// float division by the count — per scene, over its scene buffers; the target slews and the
/// target-slew corner are the scene with the LARGEST rise slew (strictly: a later equal one does not
/// replace it). Returns the slews and that scene.
pub fn find_buffer_target_slews(libs: &Libs, buffers: &[String]) -> ([f32; 2], usize) {
    let mut tgt = [0.0f32; 2];
    let mut tgt_scene = 0;
    for k in 0..libs.scene_count() {
        let mut slews = [0.0f32; 2];
        let mut counts = [0i32; 2];
        for name in buffers {
            let Some(cell) = libs.scene_cell(k, name) else { continue };
            let Some((input, output)) = cell.buffer_ports() else { continue };
            for set in cell.arc_sets.iter().filter(|s| s.from == input.name && s.to == output.name) {
                for arc in &set.arcs {
                    if let Model::Gate(model) = &arc.model {
                        let load_cap = input.capacitance(arc.from_rf, MAX) * TGT_SLEW_LOAD_CAP_FACTOR;
                        let (_, slew) = model.gate_delay(0.0, load_cap);
                        let (_, slew) = model.gate_delay(slew, load_cap);
                        slews[arc.to_rf] += slew;
                        counts[arc.to_rf] += 1;
                    }
                }
            }
        }
        let rise = slews[RISE] / counts[RISE] as f32;
        let fall = slews[FALL] / counts[FALL] as f32;
        if rise > tgt[RISE] {
            tgt = [rise, fall];
            tgt_scene = k;
        }
    }
    (tgt, tgt_scene)
}

/// `Resizer::findTargetLoad(cell, arc, in_slew, out_slew)`: the load at which the arc's output
/// slew reaches `out_slew`, by bisection.
///
/// Rules: bounds 0 and 1 pF in double; the slew difference is a FLOAT (`gateSlewDiff` takes the
/// load as float and returns a slew); 0 when the slew at no load is already above the target;
/// the upper bound doubles while still below it; stop when the bounds are within 1 % of the
/// larger; the LOWER bound is the answer, narrowed to float.
pub fn find_target_load_arc(model: &GateModel, in_slew: f32, out_slew: f32) -> f32 {
    let diff = |load_cap: f64| -> f64 {
        let (_, slew) = model.gate_delay(in_slew, load_cap as f32);
        f64::from(slew - out_slew)
    };
    let (mut cap1, mut cap2) = (0.0f64, 1.0e-12f64);
    let tol = 0.01f64;
    if diff(cap1) > 0.0 {
        return 0.0;
    }
    let mut diff2 = diff(cap2);
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol {
        if diff2 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff2 = diff(cap2);
        } else {
            let cap3 = (cap1 + cap2) / 2.0;
            let diff3 = diff(cap3);
            if diff3 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff2 = diff3;
            }
        }
    }
    cap1 as f32
}

/// `Resizer::findTargetLoad(cell)`: the mean, over every arc of every non-check arc set (tristate
/// and clock-tree-path sets carry no arcs here), of that arc's target load for the target slew of
/// its input and of its output transition — summed in float, divided by the count. A non-gate arc
/// adds 0 and still counts.
pub fn find_target_load(cell: &Cell, tgt_slews: [f32; 2]) -> f32 {
    let mut sum = 0.0f32;
    let mut count = 0i32;
    for set in cell.arc_sets.iter().filter(|s| !s.role.is_timing_check()) {
        for arc in &set.arcs {
            sum += match &arc.model {
                Model::Gate(m) => find_target_load_arc(m, tgt_slews[arc.from_rf], tgt_slews[arc.to_rf]),
                Model::Check(_) => 0.0,
            };
            count += 1;
        }
    }
    if count > 0 {
        sum / count as f32
    } else {
        0.0
    }
}

/// `Resizer::findTargetLoads`: the target slews and corner, then every LINK cell that is not
/// `dont_use`, in library then name order, the target load of its cell AT the target-slew corner
/// (`sceneCell`), mapped to the link cell.
pub fn find_target_loads(libs: &Libs, buffers: &[String], dont_use: &std::collections::BTreeSet<String>) -> ([f32; 2], usize, BTreeMap<String, f32>) {
    let (tgt, tgt_scene) = find_buffer_target_slews(libs, buffers);
    let mut loads = BTreeMap::new();
    for (li, lib) in libs.libs.iter().enumerate() {
        for cell in lib.cells.values() {
            if libs.is_link_cell(li, &cell.name) && !dont_use.contains(&cell.name) {
                let corner_cell = libs.scene_cell(tgt_scene, &cell.name).unwrap_or(cell);
                loads.insert(cell.name.clone(), find_target_load(corner_cell, tgt));
            }
        }
    }
    (tgt, tgt_scene, loads)
}

/// `Resizer::computeSlewShapeFactor` over the default library: each transition cast as a falling
/// one (a rise's thresholds flipped, `1 − upper`, `1 − lower`), the RC crossing times
/// `−ln(threshold)` (taken in double and narrowed), their difference over the library's slew
/// derate; the largest over rise and fall, then 10 % pessimism. A factor outside [0.1, 10] is
/// RSZ-0101.
pub fn compute_slew_shape_factor(lib: &Library) -> Result<f32, Stop> {
    let mut factor = 0.0f32;
    for rf in [RISE, FALL] {
        let (th_low, th_high) = if rf == RISE {
            (1.0f32 - lib.slew_upper_threshold[rf], 1.0f32 - lib.slew_lower_threshold[rf])
        } else {
            (lib.slew_lower_threshold[rf], lib.slew_upper_threshold[rf])
        };
        let t_high = -(f64::from(th_high).ln()) as f32;
        let t_low = -(f64::from(th_low).ln()) as f32;
        let rf_factor = (t_low - t_high) / lib.slew_derate;
        if !(0.1..=10.0).contains(&rf_factor) {
            let name = if rf == RISE { "rise" } else { "fall" };
            return Err(Stop::error("RSZ-0101", format!("Elmore slew modeling shape factor is out of range: {rf_factor:.3e} for {name}")));
        }
        factor = factor.max(rf_factor);
    }
    const PESSIMISM: f32 = 0.10;
    Ok(factor * (1.0 + PESSIMISM))
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty_parse::parse;

    fn lib(text: &str) -> Libs {
        Libs { libs: vec![Library::read(&parse(text).unwrap()).unwrap()], ..Default::default() }
    }

    // A buffer whose fall slew at its largest load is `r` ns over 1 pF, input cap `cin` pF.
    fn buf(name: &str, r: f32, cin: f32, extra: &str) -> String {
        format!(
            r#"cell ({name}) {{ {extra} pin (A) {{ direction : input ; capacitance : {cin} ; }}
              pin (Z) {{ direction : output ; function : "A" ;
                timing () {{ related_pin : "A" ; timing_sense : positive_unate ;
                  fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }}"#
        )
    }

    fn library(cells: &[String]) -> Libs {
        lib(&format!(
            r#"library (l) {{ time_unit : "1ns" ; capacitive_load_unit (1, pf) ;
              lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
                index_1 ("0, 1") ; index_2 ("0, 1") ; }}
              {} }}"#,
            cells.join("\n")
        ))
    }

    /// One library of buffer B whose slews (both transitions) are `r` ns per pF.
    fn corner_lib(r: f32) -> Library {
        let text = format!(
            r#"library (l) {{ time_unit : "1ns" ; capacitive_load_unit (1, pf) ;
              lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ; index_1 ("0, 1") ; index_2 ("0, 1") ; }}
              cell (B) {{ pin (A) {{ direction : input ; capacitance : 0.01 ; }} pin (Z) {{ direction : output ; function : "A" ;
                timing () {{ related_pin : "A" ; timing_sense : positive_unate ;
                  rise_transition (t) {{ values ("0, {r}", "0, {r}") ; }} fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }} }}"#
        );
        Library::read(&parse(&text).unwrap()).unwrap()
    }

    // Rule (Resizer::findBufferTargetSlews): per scene, the scene buffers' mean slews; the target
    // slews and corner are the scene with the LARGEST rise slew — strictly, so a later scene with
    // an equal one does not take it.
    #[test]
    fn the_target_slew_corner_is_the_slowest_scene() {
        let scenes = |a: f32, b: f32| Libs { libs: vec![corner_lib(a), corner_lib(b)], scenes: vec!["c0".into(), "c1".into()], scene_libs: vec![vec![corner_lib(a)], vec![corner_lib(b)]] };
        let bufs = ["B".to_string()];
        let (slow_second, k) = find_buffer_target_slews(&scenes(1.0, 2.0), &bufs);
        assert_eq!(k, 1);
        let (fast_second, k0) = find_buffer_target_slews(&scenes(2.0, 1.0), &bufs);
        assert_eq!((k0, slow_second), (0, fast_second));
        assert_eq!(find_buffer_target_slews(&scenes(1.0, 1.0), &bufs).1, 0, "an equal later scene does not replace");
    }

    fn masters(names: &[&str]) -> BTreeMap<String, Master> {
        names.iter().map(|n| (n.to_string(), Master { site: "core".into(), area: 1, is_core: true, logic_std: true, implant_obs: vec![] })).collect()
    }

    // Rules (getBufferList, findBuffers): clock buffers, dont_use and master-less cells are out;
    // the rest by drive resistance; with five buffers each bucket holds one; the weakest — the
    // LAST taken — is the highest drive resistance.
    #[test]
    fn buffers_are_sorted_by_drive_and_the_last_is_the_weakest() {
        let libs = library(&[
            buf("B4", 4.0, 1.0, ""),
            buf("B1", 1.0, 1.0, ""),
            buf("B8", 8.0, 1.0, ""),
            buf("B2", 2.0, 1.0, ""),
            buf("B16", 16.0, 1.0, ""),
            buf("CLKBUF_X1", 0.5, 1.0, ""),
            buf("BX", 0.25, 1.0, "dont_use : true ;"),
            buf("NOMASTER", 3.0, 1.0, ""),
        ]);
        let m = masters(&["B1", "B2", "B4", "B8", "B16", "CLKBUF_X1", "BX"]);
        let b = find_buffers(&libs, &m, &Default::default(), true).unwrap();
        assert_eq!(b.cells, ["B1", "B2", "B4", "B8", "B16"]);
        assert_eq!(b.lowest, "B16");
    }

    // Rule (findBuffers buckets): n = 7 ⇒ sizes 2,2,1,1,1; a bucket keeps its two lowest R·Cin,
    // in that order.
    #[test]
    fn a_bucket_is_ordered_by_drive_times_input_cap() {
        let libs = library(&[
            buf("A1", 1.0, 3.0, ""),
            buf("A2", 2.0, 1.0, ""),
            buf("A3", 3.0, 1.0, ""),
            buf("A4", 4.0, 1.0, ""),
            buf("A5", 5.0, 1.0, ""),
            buf("A6", 6.0, 1.0, ""),
            buf("A7", 7.0, 1.0, ""),
        ]);
        let m = masters(&["A1", "A2", "A3", "A4", "A5", "A6", "A7"]);
        let b = find_buffers(&libs, &m, &Default::default(), true).unwrap();
        // bucket 0 = {A1 (R·C 3), A2 (2)} → A2, A1.
        assert_eq!(b.cells, ["A2", "A1", "A3", "A4", "A5", "A6", "A7"]);
    }

    // Rule (findBuffers footprint): with two footprints, the first in name order holding more
    // than half the list wins and the others are dropped.
    #[test]
    fn a_dominant_footprint_filters_the_list() {
        let libs = library(&[
            buf("F1", 1.0, 1.0, r#"cell_footprint : "buf" ;"#),
            buf("F2", 2.0, 1.0, r#"cell_footprint : "buf" ;"#),
            buf("D1", 3.0, 1.0, r#"cell_footprint : "dly" ;"#),
        ]);
        let m = masters(&["F1", "F2", "D1"]);
        assert_eq!(find_buffers(&libs, &m, &Default::default(), true).unwrap().cells, ["F1", "F2"]);
    }

    // Rule (findBuffers sites): two sites, the larger share first; each bucket gives, per site in
    // that order, its first buffer (by R·Cin) on the site — one per site, not its best two.
    // Sorted by R: A1 A2 | T1 A3 | A4 | T2 | A5 (sizes 2,2,1,1,1).
    #[test]
    fn two_sites_take_one_buffer_per_site_from_each_bucket() {
        let libs = library(&[
            buf("A1", 1.0, 1.0, ""),
            buf("A2", 2.0, 1.0, ""),
            buf("T1", 2.5, 1.0, ""),
            buf("A3", 3.0, 1.0, ""),
            buf("A4", 4.0, 1.0, ""),
            buf("T2", 4.5, 1.0, ""),
            buf("A5", 5.0, 1.0, ""),
        ]);
        let mut m = masters(&["A1", "A2", "T1", "A3", "A4", "T2", "A5"]);
        for t in ["T1", "T2"] {
            m.get_mut(t).unwrap().site = "tall".into();
        }
        let b = find_buffers(&libs, &m, &Default::default(), true).unwrap();
        assert_eq!(b.cells, ["A1", "A3", "T1", "A4", "T2", "A5"]);
        assert_eq!(b.lowest, "A5");
    }

    // Rules: IMPLANT obstructions (VT categories), and two sites with EQUAL shares (their order is
    // the reference's site pointers), are refused, never guessed.
    #[test]
    fn vt_categories_and_tied_sites_are_refused() {
        let libs = library(&[buf("B1", 1.0, 1.0, ""), buf("B2", 2.0, 1.0, "")]);
        let mut m = masters(&["B1", "B2"]);
        m.get_mut("B2").unwrap().site = "tall".into();
        assert!(matches!(find_buffers(&libs, &m, &Default::default(), true), Err(Stop::Refused { code: "RSZ-SITES", .. })));
        let mut m = masters(&["B1", "B2"]);
        m.get_mut("B1").unwrap().implant_obs = vec!["LVT".into()];
        assert!(matches!(find_buffers(&libs, &m, &Default::default(), true), Err(Stop::Refused { code: "RSZ-VT", .. })));
    }

    // Rule (findTargetLoad): for a slew linear in the load, the lower bisection bound within 1 %
    // of the exact crossing; above-target at no load gives 0.
    #[test]
    fn target_load_is_the_lower_bisection_bound() {
        use vyges_sta::table::{Axis, AxisVar, Table};
        // slew = 1 ns + load · (1 ns / 1 pF)
        let t = Table { axes: vec![Axis { var: AxisVar::TotalOutputNetCapacitance, values: vec![0.0, 1e-12] }], values: vec![1e-9, 2e-9] };
        let m = GateModel { delay: None, slew: Some(t) };
        let l = find_target_load_arc(&m, 0.0, 3e-9);
        assert!(l <= 2e-12 && l > 2e-12 * 0.99, "{l}");
        assert_eq!(find_target_load_arc(&m, 0.0, 0.5e-9), 0.0);
    }

    // Rule (computeSlewShapeFactor): the values the reference computes — 30/70 thresholds
    // 0.932027698, 20/80 thresholds 1.5249238 (`pre|shape=`).
    #[test]
    fn slew_shape_factor_matches_the_reference_for_both_threshold_sets() {
        for (lo, up, want) in [(30, 70, 0.932_027_7_f32), (20, 80, 1.524_923_8_f32)] {
            let l = lib(&format!(
                "library (l) {{ slew_lower_threshold_pct_rise : {lo} ; slew_lower_threshold_pct_fall : {lo} ;
                  slew_upper_threshold_pct_rise : {up} ; slew_upper_threshold_pct_fall : {up} ; }}"
            ));
            assert_eq!(compute_slew_shape_factor(l.default_library().unwrap()).unwrap(), want);
        }
    }
}
