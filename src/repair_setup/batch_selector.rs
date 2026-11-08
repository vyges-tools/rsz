//! `BatchSelector` (PARALLEL_EVAL): the batch candidates a policy ranks and admits into one
//! conflict-free batch, in score order. A candidate waits when it changes what a batch member
//! changes, or when its timing depends on a batch member that its estimate did not see.
//!
//! The stages keep the reference's names and order: `select` ranks, then `fillWorstReserve`,
//! `fillByScore` and, when the reserve took a slot, `orderByScore`; `admit` tests one candidate
//! with `hardConflict` against each member in admission order, then `dependsOn`. The trace lines
//! (`brank` … `bstat`) are the printer's, written where it writes them.

use std::collections::HashMap;

use super::parallel_eval::{e9i, BatchSelectorConfig, MoveImpact};

/// `BatchCandidate`: one target's best move as the batch sees it — the owning evaluation's index in
/// the pool, its score before and after the cross-target priority weighting, the slack of the
/// endpoint the target sits on (the reserve ranks on it) and its impact.
#[derive(Debug, Clone, PartialEq)]
pub struct BatchCandidate {
    pub evaluation_index: usize,
    pub local_score: f32,
    pub priority_score: f32,
    pub endpoint_slack: f32,
    pub impact: MoveImpact,
}

/// `BatchConflictReason`, in declaration order: the stats' deferred counts are indexed by it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatchConflictReason {
    IncompleteImpact,
    WriteInstance,
    WriteNet,
    ReconnectPin,
    WriteReconnectInstance,
    ForwardDependency,
}

impl BatchConflictReason {
    /// Every reason, in declaration order (`kCount` of them).
    pub const ALL: [BatchConflictReason; 6] = [
        BatchConflictReason::IncompleteImpact,
        BatchConflictReason::WriteInstance,
        BatchConflictReason::WriteNet,
        BatchConflictReason::ReconnectPin,
        BatchConflictReason::WriteReconnectInstance,
        BatchConflictReason::ForwardDependency,
    ];

    /// `batchConflictReasonName`.
    pub fn name(self) -> &'static str {
        match self {
            BatchConflictReason::IncompleteImpact => "incomplete_impact",
            BatchConflictReason::WriteInstance => "write_instance",
            BatchConflictReason::WriteNet => "write_net",
            BatchConflictReason::ReconnectPin => "reconnect_pin",
            BatchConflictReason::WriteReconnectInstance => "write_reconnect_instance",
            BatchConflictReason::ForwardDependency => "forward_dependency",
        }
    }
}

/// `BatchSelectionStats`: the admission decisions and the conflict deferrals per reason.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BatchSelectionStats {
    pub considered: i32,
    pub batch_limited: i32,
    pub score_limited: i32,
    pub wns_reserved: i32,
    pub deferred: [i32; 6],
}

/// `BatchSelection`: the selected evaluation indices, in batch order, and their stats.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BatchSelection {
    pub evaluation_indices: Vec<usize>,
    pub stats: BatchSelectionStats,
}

/// `BatchSelector`: the score floor and the worst-endpoint reserve every selection uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatchSelector {
    config: BatchSelectorConfig,
}

/// `BatchSelector::Admission`: the batch under construction — the selection, its members' impacts
/// for the conflict checks, and the candidates already admitted (an `unordered_set` of pointers
/// upstream, lookup only: here one flag per candidate position).
struct Admission<'c> {
    selection: BatchSelection,
    impacts: Vec<&'c MoveImpact>,
    taken: Vec<bool>,
}

/// `intersects`: the two lists share an element.
fn intersects(lhs: &[String], rhs: &[String]) -> bool {
    lhs.iter().any(|value| rhs.contains(value))
}

impl BatchSelector {
    pub fn new(config: BatchSelectorConfig) -> BatchSelector {
        BatchSelector { config }
    }

    /// `select`: at most `batch_cap` candidates. The candidates ranked by priority score,
    /// descending (`stable_sort`: ties keep candidate order); the worst-endpoint reserve filled
    /// first (`reserve_worst`: the policy asks for it once WNS stops improving), then the rest by
    /// score; a selection the reserve took part in is put back in score order.
    pub fn select(&self, candidates: &[BatchCandidate], batch_cap: i32, reserve_worst: bool, trace: &mut String) -> BatchSelection {
        let mut ranked: Vec<usize> = (0..candidates.len()).collect();
        ranked.sort_by(|&l, &r| descending(candidates[l].priority_score, candidates[r].priority_score));
        let order: Vec<String> = ranked.iter().map(|&k| candidates[k].evaluation_index.to_string()).collect();
        trace.push_str(&format!("VYGP|brank|n={}|cap={batch_cap}|reserve_worst={}|{}\n", ranked.len(), u8::from(reserve_worst), order.join(",")));
        let mut admission = Admission { selection: BatchSelection::default(), impacts: Vec::with_capacity(ranked.len()), taken: vec![false; candidates.len()] };
        let reserve = self.worst_reserve(ranked.len(), batch_cap, reserve_worst);
        trace.push_str(&format!("VYGP|bres|reserve={reserve}\nVYGP|bphase|reserve\n"));
        self.fill_worst_reserve(candidates, &ranked, reserve, &mut admission, trace);
        trace.push_str("VYGP|bphase|score\n");
        self.fill_by_score(candidates, &ranked, batch_cap, &mut admission, trace);
        if admission.selection.stats.wns_reserved > 0 {
            order_by_score(candidates, &mut admission.selection);
        }
        let st = &admission.selection.stats;
        let deferred: Vec<String> = st.deferred.iter().map(i32::to_string).collect();
        trace.push_str(&format!(
            "VYGP|bstat|considered={}|batch_limited={}|score_limited={}|wns_reserved={}|deferred={}|ordered_by_score={}\n",
            st.considered,
            st.batch_limited,
            st.score_limited,
            st.wns_reserved,
            deferred.join(","),
            u8::from(st.wns_reserved > 0)
        ));
        admission.selection
    }

    /// `admit`: one candidate, unless its impact is incomplete or it conflicts with the batch:
    /// the first member (in admission order) it hard-conflicts with gives the reason; with none,
    /// the first member whose timing effect its estimate read is a forward dependency. The check
    /// is one way: a member that read timing this candidate changes does not stop it.
    fn admit<'c>(&self, candidate: &'c BatchCandidate, position: usize, admission: &mut Admission<'c>, trace: &mut String) -> bool {
        let selection = &mut admission.selection;
        selection.stats.considered += 1;
        let mut verdict = |name: &str, at: i64| trace.push_str(&format!("VYGP|badm|{}|{name}|at={at}\n", candidate.evaluation_index));
        if !candidate.impact.complete {
            selection.stats.deferred[BatchConflictReason::IncompleteImpact as usize] += 1;
            verdict(BatchConflictReason::IncompleteImpact.name(), -1);
            return false;
        }
        let mut conflict: Option<(BatchConflictReason, usize)> = None;
        for (k, selected) in admission.impacts.iter().enumerate() {
            if let Some(reason) = hard_conflict(selected, &candidate.impact) {
                conflict = Some((reason, k));
                break;
            }
        }
        if conflict.is_none() {
            for (k, selected) in admission.impacts.iter().enumerate() {
                if depends_on(&candidate.impact, selected) {
                    conflict = Some((BatchConflictReason::ForwardDependency, k));
                    break;
                }
            }
        }
        if let Some((reason, k)) = conflict {
            selection.stats.deferred[reason as usize] += 1;
            verdict(reason.name(), k as i64);
            return false;
        }
        verdict("admitted", -1);
        selection.evaluation_indices.push(candidate.evaluation_index);
        admission.impacts.push(&candidate.impact);
        admission.taken[position] = true;
        true
    }

    /// `worstReserve`: the slots kept for the worst endpoints — none unless the reserve is asked
    /// for, configured and the batch is oversubscribed (more candidates than the cap); else the
    /// cap's percentage (integer), clamped to [1, cap].
    fn worst_reserve(&self, candidate_count: usize, batch_cap: i32, reserve_worst: bool) -> i32 {
        let oversubscribed = candidate_count as i32 > batch_cap;
        if !reserve_worst || self.config.wns_reserve_pct <= 0 || !oversubscribed {
            return 0;
        }
        (batch_cap * self.config.wns_reserve_pct / 100).clamp(1, batch_cap)
    }

    /// `fillWorstReserve`: the ranked candidates re-sorted by endpoint slack, ascending
    /// (`stable_sort`), admitted in that order until the reserve is full or an endpoint is at or
    /// above the slack margin; only an admitted one counts toward the reserve.
    fn fill_worst_reserve<'c>(&self, candidates: &'c [BatchCandidate], ranked: &[usize], reserve: i32, admission: &mut Admission<'c>, trace: &mut String) {
        if reserve <= 0 {
            return;
        }
        let mut critical: Vec<usize> = ranked.to_vec();
        critical.sort_by(|&l, &r| ascending(candidates[l].endpoint_slack, candidates[r].endpoint_slack));
        let order: Vec<String> = critical.iter().map(|&k| format!("{}:{}", candidates[k].evaluation_index, e9i(candidates[k].endpoint_slack))).collect();
        trace.push_str(&format!("VYGP|bcrit|margin={}|{}\n", e9i(self.config.slack_margin), order.join(",")));
        for &k in &critical {
            if admission.selection.stats.wns_reserved >= reserve || candidates[k].endpoint_slack >= self.config.slack_margin {
                break;
            }
            if self.admit(&candidates[k], k, admission, trace) {
                admission.selection.stats.wns_reserved += 1;
            }
        }
    }

    /// `fillByScore`: the ranked candidates in order, up to the cap (the rest counted as batch
    /// limited) — those the reserve took skipped — under the score floor: once the batch holds
    /// `score_floor_start` moves, a local score below `min_score_ratio` of the best one is score
    /// limited; when the local scores are in ranked order that ends the fill (the rest counted),
    /// else that one candidate is skipped.
    fn fill_by_score<'c>(&self, candidates: &'c [BatchCandidate], ranked: &[usize], batch_cap: i32, admission: &mut Admission<'c>, trace: &mut String) {
        let mut best_local_score = 0.0f32;
        let mut local_score_ordered = true;
        for (index, &k) in ranked.iter().enumerate() {
            // std::max(a, b): b only when a < b.
            if best_local_score < candidates[k].local_score {
                best_local_score = candidates[k].local_score;
            }
            if index > 0 && candidates[k].local_score > candidates[ranked[index - 1]].local_score {
                local_score_ordered = false;
            }
        }
        let taken = admission.taken.iter().filter(|&&t| t).count();
        trace.push_str(&format!(
            "VYGP|bfill|best_local={}|ordered={}|ratio={}|floor_start={}|taken={taken}\n",
            e9i(best_local_score),
            u8::from(local_score_ordered),
            e9i(self.config.min_score_ratio),
            self.config.score_floor_start
        ));
        for (ranked_index, &k) in ranked.iter().enumerate() {
            let selection = &mut admission.selection;
            if selection.evaluation_indices.len() as i32 >= batch_cap {
                selection.stats.batch_limited = (ranked.len() - ranked_index) as i32;
                trace.push_str(&format!("VYGP|bcap|ranked_index={ranked_index}|limited={}\n", selection.stats.batch_limited));
                break;
            }
            if admission.taken[k] {
                continue;
            }
            let c = &candidates[k];
            if self.config.min_score_ratio > 0.0 && best_local_score > 0.0 && selection.evaluation_indices.len() as i32 >= self.config.score_floor_start && c.local_score < best_local_score * self.config.min_score_ratio {
                trace.push_str(&format!("VYGP|bfloor|{}|ordered={}\n", c.evaluation_index, u8::from(local_score_ordered)));
                if local_score_ordered {
                    selection.stats.score_limited = (ranked.len() - ranked_index) as i32;
                    break;
                }
                selection.stats.score_limited += 1;
                continue;
            }
            self.admit(c, k, admission, trace);
        }
    }
}

/// `lhs->score > rhs->score` as a sort's less-than: the larger first, equal (or unordered) ties
/// left to the stable sort.
fn descending(lhs: f32, rhs: f32) -> std::cmp::Ordering {
    ascending(rhs, lhs)
}

/// `lhs < rhs` as a sort's less-than.
fn ascending(lhs: f32, rhs: f32) -> std::cmp::Ordering {
    if lhs < rhs {
        std::cmp::Ordering::Less
    } else if rhs < lhs {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

/// `orderByScore`: the selection stably sorted by its candidates' priority scores, descending
/// (the score map is lookup only), so the reserve changes which moves are in the batch, not the
/// order they are applied in.
fn order_by_score(candidates: &[BatchCandidate], selection: &mut BatchSelection) {
    let score_of: HashMap<usize, f32> = candidates.iter().map(|c| (c.evaluation_index, c.priority_score)).collect();
    selection.evaluation_indices.sort_by(|l, r| descending(score_of[l], score_of[r]));
}

/// `hardConflict`: the first write or reconnect conflict between a member and a candidate, in this
/// order: a shared written instance, a shared written net, a shared reconnected pin, an instance
/// one writes that the other reconnects (either way).
fn hard_conflict(selected: &MoveImpact, candidate: &MoveImpact) -> Option<BatchConflictReason> {
    if intersects(&selected.write_instances, &candidate.write_instances) {
        return Some(BatchConflictReason::WriteInstance);
    }
    if intersects(&selected.write_nets, &candidate.write_nets) {
        return Some(BatchConflictReason::WriteNet);
    }
    if intersects(&selected.reconnect_pins, &candidate.reconnect_pins) {
        return Some(BatchConflictReason::ReconnectPin);
    }
    if intersects(&selected.write_instances, &candidate.reconnect_instances) || intersects(&candidate.write_instances, &selected.reconnect_instances) {
        return Some(BatchConflictReason::WriteReconnectInstance);
    }
    None
}

/// `dependsOn`: the candidate's estimate read timing (instances or nets) that the member changes.
fn depends_on(candidate: &MoveImpact, selected: &MoveImpact) -> bool {
    intersects(&selected.timing_effect_instances, &candidate.timing_dependency_instances) || intersects(&selected.timing_effect_nets, &candidate.timing_dependency_nets)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn impact(wi: &[&str], wn: &[&str], rp: &[&str], ri: &[&str], tei: &[&str], tdi: &[&str]) -> MoveImpact {
        let v = |xs: &[&str]| xs.iter().map(|x| x.to_string()).collect::<Vec<String>>();
        MoveImpact { complete: true, write_instances: v(wi), write_nets: v(wn), reconnect_pins: v(rp), reconnect_instances: v(ri), timing_effect_instances: v(tei), timing_dependency_instances: v(tdi), ..MoveImpact::default() }
    }

    fn cand(index: usize, score: f32, slack: f32, impact: MoveImpact) -> BatchCandidate {
        BatchCandidate { evaluation_index: index, local_score: score, priority_score: score, endpoint_slack: slack, impact }
    }

    fn plain() -> MoveImpact {
        impact(&[], &[], &[], &[], &[], &[])
    }

    fn selector(min_score_ratio: f32, wns_reserve_pct: i32) -> BatchSelector {
        BatchSelector::new(BatchSelectorConfig { score_floor_start: 2, min_score_ratio, wns_reserve_pct, slack_margin: 0.0 })
    }

    fn reason_count(s: &BatchSelection, r: BatchConflictReason) -> i32 {
        s.stats.deferred[r as usize]
    }

    // select: ranked by priority score, descending; std::ranges::stable_sort keeps candidate order
    // on equal scores. Without conflicts, the floor or the reserve, the batch is the ranked order.
    #[test]
    fn the_batch_is_the_ranked_order_with_ties_in_candidate_order() {
        let cs = vec![cand(0, 1.0, -1.0, plain()), cand(1, 3.0, -1.0, plain()), cand(2, 1.0, -1.0, plain()), cand(3, 2.0, -1.0, plain())];
        let mut t = String::new();
        let s = selector(0.0, 0).select(&cs, 8, false, &mut t);
        assert_eq!(s.evaluation_indices, [1, 3, 0, 2]);
        assert_eq!((s.stats.considered, s.stats.batch_limited, s.stats.score_limited, s.stats.wns_reserved), (4, 0, 0, 0));
        assert!(t.starts_with("VYGP|brank|n=4|cap=8|reserve_worst=0|1,3,0,2\nVYGP|bres|reserve=0\n"), "{t}");
    }

    // hardConflict: the reasons in order (write instance, write net, reconnect pin, write vs
    // reconnect instance either way), against the first member in admission order that
    // conflicts; an incomplete impact is deferred before any check.
    #[test]
    fn a_hard_conflict_defers_with_the_first_reason_of_the_first_conflicting_member() {
        let cs = vec![
            cand(0, 9.0, -1.0, impact(&["a"], &["n1"], &[], &[], &[], &[])),
            cand(1, 8.0, -1.0, impact(&["b"], &["n2"], &["p"], &["r"], &[], &[])),
            // Shares n2 with the second member and the instance a with the first: the first
            // member wins, with its own first reason (write_instance).
            cand(2, 7.0, -1.0, impact(&["a"], &["n2"], &[], &[], &[], &[])),
            cand(3, 6.0, -1.0, impact(&["c"], &["n2"], &["p"], &[], &[], &[])),
            cand(4, 5.0, -1.0, impact(&["d"], &[], &["p"], &[], &[], &[])),
            // Writes r, which the second member reconnects.
            cand(5, 4.0, -1.0, impact(&["r"], &[], &[], &[], &[], &[])),
            // Reconnects b, which the second member writes.
            cand(6, 3.5, -1.0, impact(&["e"], &[], &[], &["b"], &[], &[])),
            cand(7, 3.0, -1.0, MoveImpact { complete: false, write_instances: vec!["zz".into()], ..MoveImpact::default() }),
        ];
        let mut t = String::new();
        let s = selector(0.0, 0).select(&cs, 64, false, &mut t);
        assert_eq!(s.evaluation_indices, [0, 1]);
        assert_eq!(s.stats.deferred, [1, 1, 1, 1, 2, 0]);
        assert!(t.contains("VYGP|badm|2|write_instance|at=0\nVYGP|badm|3|write_net|at=1\nVYGP|badm|4|reconnect_pin|at=1\nVYGP|badm|5|write_reconnect_instance|at=1\nVYGP|badm|6|write_reconnect_instance|at=1\nVYGP|badm|7|incomplete_impact|at=-1\n"), "{t}");
    }

    // admit: with no hard conflict, a candidate whose estimate read an instance or net a member
    // changes is a forward dependency. The check is one way: a member that read what the
    // candidate changes does not stop it.
    #[test]
    fn a_forward_dependency_is_checked_one_way_only() {
        let reads_a = impact(&["x"], &[], &[], &[], &[], &["a"]);
        let mut changes_n = impact(&["y"], &[], &[], &[], &[], &[]);
        changes_n.timing_effect_nets = vec!["n".into()];
        let mut reads_n = impact(&["z"], &[], &[], &[], &[], &[]);
        reads_n.timing_dependency_nets = vec!["n".into()];
        let changes_a = impact(&["w"], &[], &[], &[], &["a"], &[]);
        let cs = vec![cand(0, 9.0, -1.0, reads_a), cand(1, 8.0, -1.0, changes_n), cand(2, 7.0, -1.0, reads_n), cand(3, 6.0, -1.0, changes_a)];
        let mut t = String::new();
        let s = selector(0.0, 0).select(&cs, 64, false, &mut t);
        // 3 changes what 0 read: admitted. 2 read what 1 changes: deferred.
        assert_eq!(s.evaluation_indices, [0, 1, 3]);
        assert_eq!(reason_count(&s, BatchConflictReason::ForwardDependency), 1);
        assert!(t.contains("VYGP|badm|2|forward_dependency|at=1\n"), "{t}");
    }

    // fillByScore: at the cap the rest of the ranked list is batch limited (counted from the
    // break, taken ones included); the score floor applies only once the batch holds
    // score_floor_start moves and the best local score is positive.
    #[test]
    fn the_cap_ends_the_fill_and_counts_the_rest() {
        let cs: Vec<BatchCandidate> = (0..5).map(|k| cand(k, 5.0 - k as f32, -1.0, plain())).collect();
        let mut t = String::new();
        let s = selector(0.0, 0).select(&cs, 2, false, &mut t);
        assert_eq!((s.evaluation_indices.as_slice(), s.stats.batch_limited, s.stats.considered), (&[0, 1][..], 3, 2));
        assert!(t.contains("VYGP|bcap|ranked_index=2|limited=3\n"), "{t}");
    }

    // fillByScore's floor: a local score below min_score_ratio of the best, once the batch holds
    // score_floor_start (2) moves; local scores in ranked order: the fill ends there and the rest
    // count as score limited. Out of order (a weighted priority): that candidate alone is skipped.
    #[test]
    fn the_score_floor_ends_an_ordered_fill_and_skips_in_an_unordered_one() {
        let cs = vec![cand(0, 10.0, -1.0, plain()), cand(1, 9.0, -1.0, plain()), cand(2, 0.5, -1.0, plain()), cand(3, 0.4, -1.0, plain())];
        let mut t = String::new();
        let s = selector(0.1, 0).select(&cs, 64, false, &mut t);
        assert_eq!((s.evaluation_indices.as_slice(), s.stats.score_limited), (&[0, 1][..], 2));
        // Below the floor before the batch holds two moves: still admitted.
        let early = vec![cand(0, 10.0, -1.0, plain()), cand(1, 0.5, -1.0, plain()), cand(2, 0.4, -1.0, plain())];
        let s = selector(0.1, 0).select(&early, 64, false, &mut t);
        assert_eq!((s.evaluation_indices.as_slice(), s.stats.score_limited), (&[0, 1][..], 1));
        // Priority order differs from local order: each low one skipped, a later high one taken.
        let mut un = vec![cand(0, 10.0, -1.0, plain()), cand(1, 9.0, -1.0, plain()), cand(2, 0.5, -1.0, plain()), cand(3, 2.0, -1.0, plain())];
        un[2].priority_score = 8.0;
        un[3].priority_score = 7.0;
        let s = selector(0.1, 0).select(&un, 64, false, &mut t);
        assert_eq!((s.evaluation_indices.as_slice(), s.stats.score_limited), (&[0, 1, 3][..], 1));
        // No ratio: no floor.
        let s = selector(0.0, 0).select(&cs, 64, false, &mut t);
        assert_eq!(s.evaluation_indices, [0, 1, 2, 3]);
    }

    // worstReserve: none unless asked for (the WNS stall), configured and oversubscribed; else
    // cap * pct / 100 clamped to [1, cap]. fillWorstReserve: by endpoint slack ascending
    // (stable), stopping at the reserve or at the slack margin; then fillByScore skips the taken;
    // orderByScore puts the batch back in priority order.
    #[test]
    fn the_worst_endpoint_reserve_takes_the_most_negative_first_then_the_batch_is_in_score_order() {
        let sel = selector(0.0, 10);
        assert_eq!((sel.worst_reserve(5, 4, false), sel.worst_reserve(4, 4, true), sel.worst_reserve(5, 4, true), sel.worst_reserve(100, 40, true)), (0, 0, 1, 4));
        assert_eq!(selector(0.0, 0).worst_reserve(5, 4, true), 0);
        let cs = vec![cand(0, 9.0, -1.0, plain()), cand(1, 8.0, -2.0, plain()), cand(2, 7.0, -1.0, plain()), cand(3, 6.0, -5.0, plain()), cand(4, 5.0, 1.0, plain())];
        let mut t = String::new();
        let s = sel.select(&cs, 4, true, &mut t);
        // Reserve 1: candidate 3 (the worst endpoint), then by score 0, 1, 2 to the cap (ranked
        // 3 and 4 left: batch limited 2); the batch re-sorted by priority.
        assert_eq!((s.evaluation_indices.as_slice(), s.stats.wns_reserved, s.stats.batch_limited), (&[0, 1, 2, 3][..], 1, 2));
        assert!(t.contains("VYGP|bcrit|margin=0.000000000e+00|3:-5.000000000e+00,1:-2.000000000e+00,0:-1.000000000e+00,2:-1.000000000e+00,4:1.000000000e+00\n"), "{t}");
        // The margin stops the reserve: no endpoint below it, no reserved slot.
        let above: Vec<BatchCandidate> = cs.iter().cloned().map(|mut c| { c.endpoint_slack = 0.0; c }).collect();
        assert_eq!(sel.select(&above, 4, true, &mut t).stats.wns_reserved, 0);
    }
}
