//! `SetupCritVtSwapPolicy` (`CRIT_VT_SWAP`, the phase every legacy-compatible list ends with): from
//! the worst violating endpoints (at most 100), a backward walk through the violating fanin
//! (at most 50 critical instances per endpoint) collects the instances whose output slack
//! violates; each is swapped to its best VT-equivalent cell (`selectBetterRfDriveVtCell`) in one
//! batch, then the timer is updated once.
//!
//! The batch commits in discovery order (upstream a5756aefab: a vector, where an
//! `unordered_map<Instance*>` once gave a heap order) — the order shows through the
//! max-capacitance guard on a fanin net two swaps share.

use std::collections::{BTreeSet, HashSet};

use vyges_sta::fuzzy;
use vyges_sta::graph::EdgeKind;
use vyges_sta::liberty::{Direction, Role, MAX};
use vyges_sta::netlist::Conn;

use super::{replacement_preserves_max_cap, CapCheck, MoveResult, Repair, Stop};
use crate::order::LibcxxHeap;
use crate::repair_timing::{collect_violating, Move};

/// `kMaxCritEndpoints`, `kMaxCritInstancesPerEndpoint`.
const MAX_CRIT_ENDPOINTS: usize = 100;
const MAX_CRIT_INSTANCES_PER_ENDPOINT: usize = 50;

/// One critical instance: its cell, its best VT cell, its fanin nets' capacitance checks.
struct CritInst {
    inst: String,
    cell: String,
    best: String,
    /// `outputPin(inst)`: its first output pin in the instance's pin order ([`first_output_port`]).
    out_pin: Option<String>,
    /// `getInstanceSlack`: the target's slack.
    slack: f32,
    fanin_caps: Vec<(String, Vec<CapCheck>)>,
}

/// `Resizer::checkAndMarkVTSwappable`: not marked, not dont_touch, a logic standard cell with a
/// liberty cell and a better VT cell (`selectBetterRfDriveVtCell`) — the best cell; otherwise
/// marked not swappable.
fn check_and_mark_vt_swappable(ctx: &super::Ctx<'_>, dont_touch: &BTreeSet<String>, inst: &str, cell: &str, not_swappable: &mut HashSet<String>) -> Option<String> {
    if not_swappable.contains(inst) {
        return None;
    }
    let best = if dont_touch.contains(inst) || !ctx.sizing.masters.get(cell).is_some_and(|m| m.logic_std) || ctx.libs.link_cell(cell).is_none() {
        None
    } else {
        super::select_better_rf_drive_vt_cell(ctx, cell)
    };
    if best.is_none() {
        not_swappable.insert(inst.to_string());
    }
    best
}

impl Repair<'_, '_> {
    /// `SetupCritVtSwapPolicy::iterate`.
    pub(super) fn crit_vt_swap_phase(&mut self) -> Result<(), Stop> {
        if self.args.skip_crit_vt_swap || self.args.skip_vt_swap || self.ctx.vt_category_count < 2 {
            return Ok(());
        }
        self.trk_capture_pre_phase_slack()?;
        if self.swap_vt_crit_cells()? {
            self.retime(&[])?;
        }
        self.trk_phase_summary("VT Swap Phase Summary", None)
    }

    /// `swapVTCritCells`: the critical instances of the worst violating endpoints' fanin cones,
    /// each VT-swapped (`VtSwapCandidate::applyReplacement`: the max-capacitance guard, then
    /// `replaceCell`); the batch accepted and the timer updated when any swap was made.
    fn swap_vt_crit_cells(&mut self) -> Result<bool, Stop> {
        let mut ends = collect_violating(&self.timing.ends, self.ctx.margin);
        ends.truncate(MAX_CRIT_ENDPOINTS);
        let ends: Vec<String> = ends.into_iter().map(|e| e.pin).collect();
        let mut not_swappable: HashSet<String> = HashSet::new();
        let crit = self.collect_crit_insts(&ends, &mut not_swappable)?;
        self.debug("swap_crit_vt", 1, format!("identified {} critical instances", crit.len()));
        let dont_touch = self.design.net_info().dont_touch_insts.clone();
        let mut changed = false;
        for c in &crit {
            if check_and_mark_vt_swappable(self.ctx, &dont_touch, &c.inst, &c.cell, &mut not_swappable).is_none() {
                continue;
            }
            // Level 2: each swap's output pin becomes the current endpoint and is tracked as a
            // violator (`trackViolatorWithTimingInfo`: the pin's slack read now, the target's
            // slack); the attempt is tracked before the apply, whatever the guard decides.
            if let (true, Some(out)) = (self.moves.is_some(), c.out_pin.as_deref()) {
                self.trk_set_current_endpoint(out)?;
                self.trk_violator_with_timing_info(out, c.slack)?;
                self.trk_attempt(out, Move::VtSwap)?;
            }
            if !replacement_preserves_max_cap(self.ctx.libs, &c.cell, &c.best, &c.fanin_caps) {
                self.debug("vt_swap_move", 2, format!("REJECT VTSwapMove {}: {} -> {} violates max capacitance", c.inst, c.cell, c.best));
                continue;
            }
            self.design.swap_master(&c.inst, &c.best).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
            self.debug("vt_swap_move", 1, format!("ACCEPT VTSwapMove {}: {} -> {}", c.inst, c.cell, c.best));
            self.commit(MoveResult { kind: Move::VtSwap, count: 1, insts: vec![c.inst.clone()] });
            changed = true;
            self.debug("swap_crit_vt", 1, format!("inst {} did crit VT swap", c.inst));
        }
        // `acceptPendingMoves` / `rejectPendingMoves`, each with the tracker's flush.
        if changed {
            self.committer.accept_pending();
            if let Some(m) = self.moves.as_mut() {
                m.commit_moves();
            }
            self.retime(&[])?;
            self.num_viols = collect_violating(&self.timing.ends, self.ctx.margin).len() as i64;
        } else if let Some(m) = self.moves.as_mut() {
            m.reject_moves();
        }
        Ok(changed)
    }

    /// `traverseFaninCone` over each endpoint in order, in one timer read: from the endpoint, the
    /// violating fanin worst slack first (in-edges newest first; a register clock pin not followed), each
    /// swappable instance whose worst output slack violates collected once, at most 50 new ones
    /// per endpoint.
    fn collect_crit_insts(&mut self, ends: &[String], not_swappable: &mut HashSet<String>) -> Result<Vec<CritInst>, Stop> {
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        let ctx = self.ctx;
        let margin = ctx.margin;
        let design: &dyn super::SetupDesign = &*self.design;
        let dont_touch = &design.net_info().dont_touch_insts;
        let mut result: Vec<CritInst> = Vec::new();
        // Under a group, `findGroupFanout`'s side: whether its startpoints are inputs (None: any).
        let group_side = ctx.path_group.map(crate::path_group::group_starts_at_input);
        super::timed_all(ctx, design.as_design(), &mut self.timer, edits, |gs, ss, cs| {
            let (g, s) = (&gs[0], &ss[0]);
            let parasitics = std::slice::from_ref(design.parasitics(0));
            let sc = crate::timing::Scenes::new(std::slice::from_ref(g));
            let ideal: BTreeSet<usize> = g.ideal_clock.iter().copied().collect();
            // `Vertex::isRegClk`: the from pin of a clock-to-output arc.
            let is_reg_clk = |v: usize| is_reg_clk(g, v);
            let group_fanout = group_side.map(|from_inputs| find_group_fanout(g, &cs[0], from_inputs, &is_reg_clk));
            let mut visited: HashSet<usize> = HashSet::new();
            let mut crit_set: HashSet<String> = HashSet::new();
            for end in ends {
                let Some(ev) = (0..g.vertices.len()).find(|&x| &g.vertices[x].name == end && s.is_endpoint(x)) else { continue };
                if !visited.insert(ev) {
                    continue;
                }
                let mut endpoint_insts = 0;
                let fanins = |cur: usize| {
                    // Under a group: logic no group startpoint reaches carries none of its paths.
                    let in_group = |f: usize| !matches!(&group_fanout, Some(r) if !r.contains(&f));
                    g.in_edges[cur].iter().rev().map(|&e| g.edges[e].from).filter(|&f| !is_reg_clk(f) && in_group(f)).map(|f| (f, s.slack_of(f, MAX, None))).collect::<Vec<_>>()
                };
                walk_fanin_cone(ev, s.slack_of(ev, MAX, None), margin, &mut visited, fanins, |cur| {
                    // Under a group an instance counts only through an output (its own delay on
                    // the group's paths; the endpoint's register is reached by its data pin).
                    let counted = counts_instance(group_fanout.is_some(), g.vertices[cur].is_driver);
                    if let (true, Conn::Inst(k, _)) = (counted, &g.vertices[cur].conn) {
                        let (inst, cell) = (&g.netlist.insts[*k].0, &g.netlist.insts[*k].1);
                        if let Some(best) = check_and_mark_vt_swappable(ctx, dont_touch, inst, cell, not_swappable) {
                            // `getInstanceSlack`: the least (`std::min`) slack over the output pins.
                            let mut inst_slack = f32::MAX;
                            for (u, ux) in g.vertices.iter().enumerate() {
                                if ux.is_driver && matches!(&ux.conn, Conn::Inst(j, _) if j == k) {
                                    inst_slack = inst_slack.min(s.slack_of(u, MAX, None));
                                }
                            }
                            // `outputPin(inst)`: the first any-output pin in the instance's pin
                            // iterator order (its master's terminals, as LEF orders them).
                            let out_pin = first_output_port(ctx.master_pins.get(cell).map(Vec::as_slice).unwrap_or_default(), |port| ctx.libs.link_cell(cell).and_then(|c| c.port(port)).map(|p| p.direction))
                                .and_then(|port| g.vertices.iter().find(|ux| ux.port.as_deref() == Some(port) && matches!(&ux.conn, Conn::Inst(j, _) if j == k)).map(|ux| ux.name.clone()));
                            if fuzzy::less(inst_slack, margin) && crit_set.insert(inst.clone()) {
                                endpoint_insts += 1;
                                let mut fanin_caps = Vec::new();
                                for (u, ux) in g.vertices.iter().enumerate() {
                                    if ux.is_driver || !matches!(&ux.conn, Conn::Inst(j, _) if j == k) {
                                        continue;
                                    }
                                    let is_input = ctx.libs.link_cell(cell).and_then(|c| c.port(ux.port.as_deref()?)).is_some_and(|p| p.direction == Direction::Input);
                                    // The target has no scene or path (`Target::mode` is null), so no
                                    // register clock pin is skipped (`max_cap_skips_pin(_, None)`);
                                    // a scan pin is.
                                    let scan = ctx.libs.link_cell(cell).zip(ux.port.as_deref()).is_some_and(|(c, port)| super::is_scan_pin(c, port));
                                    if !is_input || scan {
                                        continue;
                                    }
                                    let checks = crate::timing::net_drivers(g, u)
                                        .into_iter()
                                        .map(|d| {
                                            let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, parasitics, &ideal, self.ctx.limits);
                                            CapCheck { cap, max_cap, slack, limited }
                                        })
                                        .collect();
                                    fanin_caps.push((ux.port.clone().unwrap_or_default(), checks));
                                }
                                result.push(CritInst { inst: inst.clone(), cell: cell.clone(), best, out_pin, slack: inst_slack, fanin_caps });
                            }
                        }
                    }
                    endpoint_insts < MAX_CRIT_INSTANCES_PER_ENDPOINT
                });
            }
            Ok(())
        })?;
        Ok(result)
    }
}

/// `Vertex::isRegClk`: the from pin of a clock-to-output arc.
/// `SetupCritVtSwapPolicy::outputPin`: the first of the instance's pins (`pinIterator`, its
/// master's terminals in order) whose direction `isAnyOutput` — output, tristate or bidirect.
fn first_output_port(ports: &[String], direction: impl Fn(&str) -> Option<Direction>) -> Option<&str> {
    ports.iter().map(String::as_str).find(|p| matches!(direction(p), Some(Direction::Output | Direction::Tristate | Direction::Bidirect)))
}

fn is_reg_clk(g: &vyges_sta::graph::Graph<'_>, v: usize) -> bool {
    g.out_edges[v].iter().any(|&e| match g.edges[e].kind {
        EdgeKind::Gate { set } => {
            let vx = &g.vertices[g.edges[e].to];
            vx.lib.and_then(|l| g.libs.get(l)).and_then(|l| l.cells.get(vx.cell.as_deref()?)).is_some_and(|c| matches!(c.arc_sets[set].role, Role::RegClkToQ | Role::LatchEnToQ))
        }
        EdgeKind::Wire => false,
    })
}

/// Whether `traverseFaninCone` counts the instance of a pin it pops: under a group only through
/// an output (its own delay on the group's paths; the endpoint's register is reached by its data
/// pin), else through any pin.
fn counts_instance(group: bool, driver: bool) -> bool {
    !group || driver
}

/// `findGroupFanout`: every vertex a group startpoint reaches — from each top input, or each
/// register output (`Resizer::isRegister`: a sequential cell's), on its side of the group
/// (`startpointInGroup`: a top input exactly when the group starts at inputs) and off the clock
/// network, forward through every edge but a timing check and one into a register clock pin.
fn find_group_fanout(g: &vyges_sta::graph::Graph<'_>, clock_network: &BTreeSet<usize>, from_inputs: Option<bool>, is_reg_clk: &dyn Fn(usize) -> bool) -> HashSet<usize> {
    let register = |v: usize| {
        let vx = &g.vertices[v];
        vx.lib.and_then(|l| g.libs.get(l)).and_then(|l| l.cells.get(vx.cell.as_deref()?)).is_some_and(|c| c.is_sequential())
    };
    let seeds: Vec<usize> = (0..g.vertices.len()).filter(|&v| is_group_start(g.vertices[v].is_driver, clock_network.contains(&v), g.is_port(v), register(v), from_inputs)).collect();
    // An edge is followed unless it is a timing check or ends at a register clock pin.
    let next = |v: usize| -> Vec<usize> {
        g.out_edges[v]
            .iter()
            .filter(|&&e| !matches!(g.edges[e].kind, EdgeKind::Gate { set } if g.arc_set(e, set).role.is_timing_check()))
            .map(|&e| g.edges[e].to)
            .filter(|&to| !is_reg_clk(to))
            .collect()
    };
    reach_forward(&seeds, next)
}

/// A `findGroupFanout` seed: a driver off the clock network that is a top input or a register
/// output, a top input exactly when the group starts at inputs (`startpointInGroup`; either when
/// it starts anywhere, `kAny`).
fn is_group_start(driver: bool, on_clock: bool, input: bool, register: bool, from_inputs: Option<bool>) -> bool {
    driver && !on_clock && (input || register) && from_inputs.is_none_or(|f| input == f)
}

/// Breadth first from `seeds` along `next`: every vertex reached, the seeds included.
fn reach_forward(seeds: &[usize], next: impl Fn(usize) -> Vec<usize>) -> HashSet<usize> {
    let mut reached: HashSet<usize> = HashSet::new();
    let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
    for &v in seeds {
        if reached.insert(v) {
            queue.push_back(v);
        }
    }
    while let Some(v) = queue.pop_front() {
        for to in next(v) {
            if reached.insert(to) {
                queue.push_back(to);
            }
        }
    }
    reached
}

/// `traverseFaninCone`'s walk (6d88a0eb5b8): a min-slack `std::priority_queue` of (slack, vertex),
/// ordered by slack alone, seeded with the endpoint; each popped vertex is visited, then its
/// fanins (in the order given) not yet visited whose slack violates the margin are queued and
/// marked visited. The walk stops when the queue empties or `visit` reports the cap reached.
/// Equal slacks pop in the libc++ heap's order.
fn walk_fanin_cone(start: usize, start_slack: f32, margin: f32, visited: &mut HashSet<usize>, fanins: impl Fn(usize) -> Vec<(usize, f32)>, mut visit: impl FnMut(usize) -> bool) {
    let mut queue = LibcxxHeap::new(|a: &(f32, usize), b: &(f32, usize)| a.0 > b.0);
    queue.push((start_slack, start));
    while let Some((_, cur)) = queue.pop() {
        let more = visit(cur);
        for (fanin, slack) in fanins(cur) {
            if !visited.contains(&fanin) && fuzzy::less(slack, margin) {
                queue.push((slack, fanin));
                visited.insert(fanin);
            }
        }
        if !more {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    /// Rule (`outputPin`, upstream a5756aefab's tracker): the output pin a CRIT_VT swap is tracked
    /// at is the first any-output pin in the master's terminal order, not the first driver vertex
    /// the graph made — on a two-output cell (`COUT`, `SUM`) the two can differ, and the tracker
    /// then counts the swap at another pin than a later reroute of the same instance.
    #[test]
    fn the_tracked_output_pin_is_the_first_any_output_in_terminal_order() {
        use vyges_sta::liberty::Direction::*;
        let ports: Vec<String> = ["A", "B", "CI", "COUT", "SUM"].iter().map(|p| p.to_string()).collect();
        let dir = |p: &str| Some(if matches!(p, "COUT" | "SUM") { Output } else { Input });
        assert_eq!(super::first_output_port(&ports, dir), Some("COUT"));
        let ports: Vec<String> = ["A", "Z", "Y"].iter().map(|p| p.to_string()).collect();
        assert_eq!(super::first_output_port(&ports, |p| Some(match p { "A" => Input, "Z" => Tristate, _ => Output })), Some("Z"));
        assert_eq!(super::first_output_port(&ports, |p| Some(if p == "A" { Input } else { Internal })), None);
    }

    use super::*;

    /// Rules (`SetupCritVtSwapPolicy::findGroupFanout`, `PathGroupFilter::startpointInGroup`):
    /// a group's startpoints are the top inputs (in2reg, in2out) or the register outputs
    /// (reg2reg, reg2out), never a clock-network pin nor a load; the fanout is everything they
    /// reach forward, the walk stopping at a timing check and at a register clock pin (handed in
    /// `next`).
    #[test]
    fn a_groups_fanout_starts_on_its_side() {
        // (driver, on_clock, input, register)
        assert!(is_group_start(true, false, true, false, Some(true)));
        assert!(!is_group_start(true, false, true, false, Some(false)), "an input is no reg2reg start");
        assert!(is_group_start(true, false, false, true, Some(false)));
        assert!(!is_group_start(true, false, false, true, Some(true)), "a register is no in2reg start");
        assert!(!is_group_start(true, true, false, true, Some(false)), "a clock-network pin is none");
        assert!(!is_group_start(false, false, false, true, Some(false)), "a register's input is none");
        assert!(!is_group_start(true, false, false, false, Some(false)), "a combinational output is none");
        // gated_clock (`kAny`): an input and a register output both start it.
        assert!(is_group_start(true, false, true, false, None) && is_group_start(true, false, false, true, None));
        let next = [vec![1], vec![2, 3], vec![], vec![1]];
        let r = reach_forward(&[0], |v| next[v].clone());
        assert_eq!(r, HashSet::from([0, 1, 2, 3]));
        assert_eq!(reach_forward(&[2], |v| next[v].clone()), HashSet::from([2]));
    }

    /// Rules (`findGroupFanout` on a graph): reg2reg reaches from every register output on; in2reg
    /// from the inputs off the clock network, and an input driving a register's clock pin stops
    /// there (that register's output is not reached through it).
    #[test]
    fn a_groups_fanout_stops_at_register_clock_pins() {
        const D: &str = r#"cell_rise (t) { values ("0.01, 0.01", "0.01, 0.01"); } rise_transition (t) { values ("0.01, 0.01", "0.01, 0.01"); }
          cell_fall (t) { values ("0.01, 0.01", "0.01, 0.01"); } fall_transition (t) { values ("0.01, 0.01", "0.01, 0.01"); }"#;
        const C: &str = r#"rise_constraint (t) { values ("0.01, 0.01", "0.01, 0.01"); } fall_constraint (t) { values ("0.01, 0.01", "0.01, 0.01"); }"#;
        let lib = format!(
            r#"library (t) {{ time_unit : "1ns"; capacitive_load_unit (1, pf);
          lu_table_template (t) {{ variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 ("0, 1"); index_2 ("0, 1"); }}
          cell (buf) {{ pin (A) {{ direction : input; capacitance : 0.001; }}
            pin (Z) {{ direction : output; function : "A"; timing () {{ related_pin : "A"; timing_sense : positive_unate; {D} }} }} }}
          cell (dff) {{ ff (IQ, IQN) {{ clocked_on : "CK"; next_state : "D"; }}
            pin (D) {{ direction : input; capacitance : 0.001; timing () {{ related_pin : "CK"; timing_type : setup_rising; {C} }} }}
            pin (CK) {{ direction : input; clock : true; capacitance : 0.001; }}
            pin (Q) {{ direction : output; function : "IQ"; timing () {{ related_pin : "CK"; timing_type : rising_edge; {D} }} }} }} }}"#
        );
        use vyges_sta::netlist::{inst, Conn, Net, Netlist, PortDir};
        let libs = vec![vyges_sta::liberty::Library::read(&vyges_sta::liberty_parse::parse(&lib).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let netlist = Netlist {
            insts: vec![inst("b1", "buf"), inst("r1", "dff"), inst("b2", "buf"), inst("r2", "dff"), inst("r3", "dff")],
            ports: vec![("clk".into(), PortDir::Input), ("a".into(), PortDir::Input), ("en".into(), PortDir::Input), ("q".into(), PortDir::Output)],
            nets: vec![
                Net { name: "clk".into(), pins: vec![Conn::Port(0), i(1, "CK"), i(3, "CK")] }.into(),
                Net { name: "a".into(), pins: vec![Conn::Port(1), i(0, "A"), i(4, "D")] }.into(),
                Net { name: "n1".into(), pins: vec![i(0, "Z"), i(1, "D")] }.into(),
                Net { name: "n2".into(), pins: vec![i(1, "Q"), i(2, "A")] }.into(),
                Net { name: "n3".into(), pins: vec![i(2, "Z"), i(3, "D")] }.into(),
                Net { name: "en".into(), pins: vec![Conn::Port(2), i(4, "CK")] }.into(),
                Net { name: "q".into(), pins: vec![i(4, "Q"), Conn::Port(3)] }.into(),
            ],
        };
        let g = vyges_sta::graph::Graph::build(&libs, &netlist).unwrap();
        let clocks = crate::timing::clock_pins(&g, &["clk".to_string()]);
        let names = |from_inputs: bool| -> BTreeSet<String> { find_group_fanout(&g, &clocks, Some(from_inputs), &|v| is_reg_clk(&g, v)).into_iter().map(|v| g.vertices[v].name.clone()).collect() };
        let set = |xs: &[&str]| -> BTreeSet<String> { xs.iter().map(|x| x.to_string()).collect() };
        assert_eq!(names(true), set(&["a", "b1/A", "b1/Z", "en", "r1/D", "r3/D"]));
        assert_eq!(names(false), set(&["b2/A", "b2/Z", "q", "r1/Q", "r2/D", "r2/Q", "r3/Q"]));
    }

    /// Rule (`traverseFaninCone` under -path_group): an instance counts only through an output pin;
    /// with no group, through any pin.
    #[test]
    fn under_a_group_an_instance_counts_through_its_output() {
        assert!(counts_instance(false, false) && counts_instance(false, true));
        assert!(counts_instance(true, true) && !counts_instance(true, false));
    }

    // Rule (SetupCritVtSwapPolicy::traverseFaninCone, 6d88a0eb5b8): the violating fanin is walked
    // worst slack first (a min-slack priority queue, not FIFO), so the per-endpoint cap keeps the
    // most critical gates; fanins at or above the margin are not followed.
    #[test]
    fn the_fanin_cone_is_walked_worst_slack_first() {
        // 0 <- {1: -0.1, 2: -0.5}; 1 <- {3: -0.9}; 2 <- {4: -0.2, 5: 0.0 (not violating)}
        let slack = [-1.0, -0.1, -0.5, -0.9, -0.2, 0.0];
        let fanin_of: [&[usize]; 6] = [&[1, 2], &[3], &[4, 5], &[], &[], &[]];
        let order = |cap: usize| {
            let mut seen = Vec::new();
            let mut visited = HashSet::from([0]);
            walk_fanin_cone(0, slack[0], 0.0, &mut visited, |v| fanin_of[v].iter().map(|&f| (f, slack[f])).collect(), |v| {
                seen.push(v);
                seen.len() < cap
            });
            seen
        };
        assert_eq!(order(usize::MAX), vec![0, 2, 4, 1, 3], "FIFO would give 0, 1, 2, 3, 4");
        assert_eq!(order(3), vec![0, 2, 4], "the cap keeps the worst branch");
    }
}
