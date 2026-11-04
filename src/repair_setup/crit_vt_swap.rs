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
            // Level 2 tracks each swap against its output pin as the current endpoint, and the
            // batch's moves are finalized against the LAST one — the order of an
            // `unordered_map<Instance*>`, a heap order no capture records.
            if self.moves.is_some() {
                return Err(Stop::refused("RSZ-ABSENT", "the move tracker at level 2 over a CRIT_VT_SWAP batch (its moves credited to the last instance of a heap-ordered map): not modelled".into()));
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
                let mut endpoint_insts = 0;
                let fanins = |cur: usize| {
                    g.in_edges[cur].iter().rev().map(|&e| g.edges[e].from).filter(|&f| !is_reg_clk(f)).map(|f| (f, s.slack_of(f, MAX, None))).collect::<Vec<_>>()
                };
                walk_fanin_cone(ev, s.slack_of(ev, MAX, None), margin, &mut visited, fanins, |cur| {
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
                                            let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, parasitics, &ideal, self.ctx.limits);
                                            CapCheck { cap, max_cap, slack, limited }
                                        })
                                        .collect();
                                    fanin_caps.push((ux.port.clone().unwrap_or_default(), checks));
                                }
                                result.push(CritInst { inst: inst.clone(), cell: cell.clone(), best, fanin_caps });
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
    use super::*;

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
