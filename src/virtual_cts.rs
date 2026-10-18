//! Global placement's virtual clock tree (`-virtual_cts`): before each timing-driven reweight,
//! every register clock pin of each clock gets an ideal network latency from a minimum spanning
//! tree over the pins' placed instances — the farthest pin, through the tree from its hub, gets
//! `fraction × period`, the others in proportion. The latencies live for one reweight: the next
//! builds them afresh from the new placement.

use std::collections::{BTreeSet, HashMap, VecDeque};

use vyges_sta::graph::{EdgeKind, Graph};
use vyges_sta::liberty::Role;

/// The vertex of an edge's arc set role (`None` for a wire).
fn edge_role(g: &Graph<'_>, e: usize) -> Option<Role> {
    match g.edges[e].kind {
        EdgeKind::Gate { set } => Some(g.arc_set(e, set).role),
        EdgeKind::Wire => None,
    }
}

/// `Sta::findRegisterClkPins(clk, riseFall, registers, latches)`: from the clock's source pin, a
/// depth-first walk through the clock network (wires and combinational arcs); each register
/// clock pin it reaches (`isRegClk`) names a register, and every pin of that register an edge
/// leaves as clock-to-Q or enable-to-Q is collected. In pin id order (a `PinSet`), ids from
/// `pin_id` (pins without one last, by name).
pub fn register_clk_pins(g: &Graph<'_>, source: &str, pin_id: &HashMap<String, u64>) -> Vec<String> {
    let reg_clk: Vec<bool> = (0..g.vertices.len()).map(|v| g.out_edges[v].iter().any(|&e| matches!(edge_role(g, e), Some(Role::RegClkToQ | Role::LatchEnToQ)))).collect();
    let inst_of = |v: usize| g.vertices[v].name.rsplit_once('/').map(|(i, _)| i.to_string());
    let mut regs: BTreeSet<String> = BTreeSet::new();
    let mut visited = vec![false; g.vertices.len()];
    let mut stack: Vec<usize> = g.vertices.iter().enumerate().filter(|(_, x)| x.name == source).map(|(v, _)| v).collect();
    while let Some(v) = stack.pop() {
        if visited[v] {
            continue;
        }
        visited[v] = true;
        for &e in &g.out_edges[v] {
            let to = g.edges[e].to;
            if reg_clk[to] {
                regs.extend(inst_of(to));
            }
            if matches!(edge_role(g, e), None | Some(Role::Combinational)) {
                stack.push(to);
            }
        }
    }
    let mut pins: Vec<String> = (0..g.vertices.len()).filter(|&v| reg_clk[v] && inst_of(v).is_some_and(|i| regs.contains(&i))).map(|v| g.vertices[v].name.clone()).collect();
    pins.sort_by(|a, b| (pin_id.get(a).copied().unwrap_or(u64::MAX), a).cmp(&(pin_id.get(b).copied().unwrap_or(u64::MAX), b)));
    pins.dedup();
    pins
}

/// `ClockBase::buildVirtualTreeForClock` from the sinks' locations on: `None` with fewer than two
/// sinks or all within one unit of the hub; else each sink's latency. Prim's tree on Manhattan
/// distance from sink 0 (the first least key wins a tie), rooted at the sink nearest the sinks'
/// centroid (the first wins a tie), breadth-first tree distances, each scaled so the farthest is
/// `fraction × period` (`float(distance × (double(fraction) × period / farthest))`).
pub fn latencies(sinks: &[(i32, i32)], fraction: f32, period: f32) -> Option<Vec<f32>> {
    let n = sinks.len();
    if n < 2 {
        return None;
    }
    let dist = |a: usize, b: usize| f64::from((sinks[a].0 - sinks[b].0).abs() + (sinks[a].1 - sinks[b].1).abs());
    let mut parent: Vec<Option<usize>> = vec![None; n];
    let mut edge = vec![0.0f64; n];
    let mut in_mst = vec![false; n];
    let mut key = vec![f64::MAX; n];
    key[0] = 0.0;
    for _ in 0..n {
        let mut u = None;
        for i in 0..n {
            if !in_mst[i] && u.is_none_or(|k: usize| key[i] < key[k]) {
                u = Some(i);
            }
        }
        let u = u.expect("a vertex left");
        in_mst[u] = true;
        for v in 0..n {
            if !in_mst[v] {
                let d = dist(u, v);
                if d < key[v] {
                    key[v] = d;
                    parent[v] = Some(u);
                    edge[v] = d;
                }
            }
        }
    }
    let (mut sx, mut sy) = (0.0f64, 0.0f64);
    for s in sinks {
        sx += f64::from(s.0);
        sy += f64::from(s.1);
    }
    let (cx, cy) = (sx / n as f64, sy / n as f64);
    let mut root = 0;
    let mut best = f64::MAX;
    for (i, s) in sinks.iter().enumerate() {
        let d = (f64::from(s.0) - cx).abs() + (f64::from(s.1) - cy).abs();
        if d < best {
            best = d;
            root = i;
        }
    }
    let mut adj: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
    for i in 0..n {
        if let Some(p) = parent[i] {
            adj[p].push((i, edge[i]));
            adj[i].push((p, edge[i]));
        }
    }
    let mut tree = vec![-1.0f64; n];
    tree[root] = 0.0;
    let mut bfs = VecDeque::from([root]);
    while let Some(u) = bfs.pop_front() {
        for &(v, w) in &adj[u] {
            if tree[v] < 0.0 {
                tree[v] = tree[u] + w;
                bfs.push_back(v);
            }
        }
    }
    let max = tree.iter().copied().fold(f64::MIN, f64::max);
    if max < 1.0 {
        return None;
    }
    let scale = f64::from(fraction) * f64::from(period) / max;
    Some(tree.iter().map(|&t| (t * scale) as f32).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (ClockBase::buildVirtualTreeForClock): Prim's tree from sink 0, rooted at the sink
    // nearest the centroid; the farthest sink through the TREE gets fraction × period.
    // Sinks on a line at 0, 10, 30: tree 0-10-30, centroid 13.3 → root 10 (distance 3.3);
    // tree distances 10, 0, 20 → scaled by 0.1 × 2 / 20.
    #[test]
    fn the_farthest_sink_through_the_tree_gets_the_fraction_of_the_period() {
        let l = latencies(&[(0, 0), (10, 0), (30, 0)], 0.1, 2.0).unwrap();
        let scale = f64::from(0.1f32) * f64::from(2.0f32) / 20.0;
        assert_eq!(l, vec![(10.0 * scale) as f32, 0.0, (20.0 * scale) as f32]);
    }

    // Rule: fewer than two sinks, or every sink within one unit of the hub — no latency at all.
    #[test]
    fn one_sink_or_colocated_sinks_get_none() {
        assert_eq!(latencies(&[(5, 5)], 0.1, 2.0), None);
        assert_eq!(latencies(&[(5, 5), (5, 5)], 0.1, 2.0), None);
    }

    // Rule: a tie in the centroid distance keeps the FIRST sink as the hub (strict `<`).
    #[test]
    fn a_centroid_tie_keeps_the_first_sink_as_the_hub() {
        let l = latencies(&[(0, 0), (10, 0)], 0.5, 1.0).unwrap();
        assert_eq!(l, vec![0.0, (10.0 * (f64::from(0.5f32) * 1.0 / 10.0)) as f32]);
    }
}
