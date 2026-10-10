//! `SetupCritVtSwapPolicy` (`CRIT_VT_SWAP`, the phase every legacy-compatible list ends with): from
//! the worst violating endpoints (at most 100), a backward walk through the violating fanin
//! (at most 50 critical instances per endpoint) collects the instances whose output slack
//! violates; each is swapped to its best VT-equivalent cell (`getVTEquivCells(..).back()`) in one
//! batch, then the timer is updated once.
//!
//! The reference commits the batch in its `unordered_map<Instance*>` order (heap addresses: its
//! own debug order differs run to run, its results do not); a swap's cell depends only on the
//! library, so the order can show only through the max-capacitance guard on a fanin net two swaps
//! share — ours commits in discovery order.

use std::collections::{BTreeSet, HashSet, VecDeque};

use vyges_sta::fuzzy;
use vyges_sta::graph::EdgeKind;
use vyges_sta::liberty::{Direction, Role, MAX};
use vyges_sta::netlist::Conn;

use super::{replacement_preserves_max_cap, CapCheck, MoveResult, Repair, Stop};
use crate::repair_timing::{collect_violating, Move};

/// `kMaxCritEndpoints`, `kMaxCritInstancesPerEndpoint`.
const MAX_CRIT_ENDPOINTS: usize = 100;
const MAX_CRIT_INSTANCES_PER_ENDPOINT: usize = 50;

/// One critical instance: its cell, its best VT cell, its fanin nets' capacitance checks.
struct CritInst {
    inst: String,
    cell: String,
    best: String,
    fanin_caps: Vec<(String, Vec<CapCheck>)>,
}

/// `Resizer::checkAndMarkVTSwappable`: not marked, not dont_touch, a logic standard cell with a
/// liberty cell and VT-equivalent cells whose last is not the cell itself — the best cell;
/// otherwise marked not swappable.
fn check_and_mark_vt_swappable(ctx: &super::Ctx<'_>, dont_touch: &BTreeSet<String>, inst: &str, cell: &str, not_swappable: &mut HashSet<String>) -> Option<String> {
    if not_swappable.contains(inst) {
        return None;
    }
    let best = if dont_touch.contains(inst) || !ctx.sizing.masters.get(cell).is_some_and(|m| m.logic_std) || ctx.libs.link_cell(cell).is_none() {
        None
    } else {
        ctx.sizing.vt_equiv_cells(cell, ctx.vt_category_count).last().filter(|b| *b != cell).cloned()
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
        if self.swap_vt_crit_cells()? {
            self.retime(&[])?;
        }
        Ok(())
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
        if changed {
            self.committer.accept_pending();
            self.retime(&[])?;
            self.num_viols = collect_violating(&self.timing.ends, self.ctx.margin).len() as i64;
        }
        Ok(changed)
    }

    /// `traverseFaninCone` over each endpoint in order, in one timer read: from the endpoint, the
    /// violating fanin (in-edges newest first; a register clock pin not followed), each
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
        super::timed_all(ctx, design.as_design(), &mut self.timer, edits, |gs, ss, cs| {
            let (g, s) = (&gs[0], &ss[0]);
            let parasitics = std::slice::from_ref(design.parasitics(0));
            let sc = crate::timing::Scenes::new(std::slice::from_ref(g));
            let ideal = if ctx.ideal_clock { cs[0].clone() } else { BTreeSet::new() };
            // `Vertex::isRegClk`: the from pin of a clock-to-output arc.
            let is_reg_clk = |v: usize| {
                g.out_edges[v].iter().any(|&e| match g.edges[e].kind {
                    EdgeKind::Gate { set } => {
                        let vx = &g.vertices[g.edges[e].to];
                        vx.lib.and_then(|l| g.libs.get(l)).and_then(|l| l.cells.get(vx.cell.as_deref()?)).is_some_and(|c| matches!(c.arc_sets[set].role, Role::RegClkToQ | Role::LatchEnToQ))
                    }
                    EdgeKind::Wire => false,
                })
            };
            let mut visited: HashSet<usize> = HashSet::new();
            let mut crit_set: HashSet<String> = HashSet::new();
            for end in ends {
                let Some(ev) = (0..g.vertices.len()).find(|&x| &g.vertices[x].name == end && s.is_endpoint(x)) else { continue };
                if !visited.insert(ev) {
                    continue;
                }
                let mut queue = VecDeque::from([ev]);
                let mut endpoint_insts = 0;
                while endpoint_insts < MAX_CRIT_INSTANCES_PER_ENDPOINT {
                    let Some(cur) = queue.pop_front() else { break };
                    if let Conn::Inst(k, _) = &g.vertices[cur].conn {
                        let (inst, cell) = (&g.netlist.insts[*k].0, &g.netlist.insts[*k].1);
                        if let Some(best) = check_and_mark_vt_swappable(ctx, dont_touch, inst, cell, not_swappable) {
                            // `getInstanceSlack`: the least (`std::min`) slack over the output pins.
                            let mut inst_slack = f32::MAX;
                            for (u, ux) in g.vertices.iter().enumerate() {
                                if ux.is_driver && matches!(&ux.conn, Conn::Inst(j, _) if j == k) {
                                    inst_slack = inst_slack.min(s.slack_of(u, MAX, None));
                                }
                            }
                            if fuzzy::less(inst_slack, margin) && crit_set.insert(inst.clone()) {
                                endpoint_insts += 1;
                                let mut fanin_caps = Vec::new();
                                for (u, ux) in g.vertices.iter().enumerate() {
                                    if ux.is_driver || !matches!(&ux.conn, Conn::Inst(j, _) if j == k) {
                                        continue;
                                    }
                                    let is_input = ctx.libs.link_cell(cell).and_then(|c| c.port(ux.port.as_deref()?)).is_some_and(|p| p.direction == Direction::Input);
                                    if !is_input {
                                        continue;
                                    }
                                    let checks = crate::timing::net_drivers(g, u)
                                        .into_iter()
                                        .map(|d| {
                                            let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, parasitics, &ideal);
                                            CapCheck { cap, max_cap, slack, limited }
                                        })
                                        .collect();
                                    fanin_caps.push((ux.port.clone().unwrap_or_default(), checks));
                                }
                                result.push(CritInst { inst: inst.clone(), cell: cell.clone(), best, fanin_caps });
                            }
                        }
                    }
                    for &e in g.in_edges[cur].iter().rev() {
                        let fanin = g.edges[e].from;
                        if is_reg_clk(fanin) || visited.contains(&fanin) {
                            continue;
                        }
                        if fuzzy::less(s.slack_of(fanin, MAX, None), margin) {
                            queue.push_back(fanin);
                            visited.insert(fanin);
                        }
                    }
                }
            }
            Ok(())
        })?;
        Ok(result)
    }
}
