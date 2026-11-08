//! `BatchSelector` (PARALLEL_EVAL): the batch candidates a policy ranks and admits into one
//! conflict-free batch.

use super::parallel_eval::MoveImpact;

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
