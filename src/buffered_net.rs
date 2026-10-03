// SPDX-License-Identifier: Apache-2.0
//! The buffered net: a net's Steiner tree as the repair walks it — loads, junctions and wires,
//! each carrying the capacitance, fanout and slew limit seen from it (`BufferedNet`), built by
//! `makeBufferedNetSteiner` / `makeBufferedNetFromTree`.
//!
//! Placement parasitics only: every wire is on no layer, its RC split by its horizontal and
//! vertical share of the signal wire RC.

use vyges_sta::graph::Graph;
use vyges_sta::netlist::Conn;

use crate::timing::{self, Limits, INF};

/// `BufferedNet::null_layer`.
pub const NULL_LAYER: i32 = -1;

/// A node of the buffered net (an arena index is its identity).
#[derive(Debug, Clone, PartialEq)]
pub enum Kind {
    /// A load pin (a timing-graph vertex).
    Load { pin: usize },
    /// `ref` then `ref2`.
    Junction { r: usize, r2: usize },
    /// A wire from this node's location to `ref`'s.
    Wire { r: usize },
}

#[derive(Debug, Clone, PartialEq)]
pub struct BNode {
    pub kind: Kind,
    pub x: i32,
    pub y: i32,
    pub layer: i32,
    /// The member defaults: cap 0, fanout 1, max load slew INF.
    pub cap: f32,
    pub fanout: f32,
    pub max_load_slew: f32,
}

/// The nodes, in creation order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BufferedNet {
    pub nodes: Vec<BNode>,
}

/// The signal wire RC per meter (`wireSignal{H,V}{Resistance,Capacitance}`), doubles.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WireRc {
    pub h_res: f64,
    pub v_res: f64,
    pub h_cap: f64,
    pub v_cap: f64,
}

/// `est::SteinerTree` as the buffered net reads it.
#[derive(Debug, Clone, PartialEq)]
pub struct Tree {
    /// `tree_.deg`: the points below it are pins.
    pub deg: usize,
    /// `(x, y, n)` per branch point.
    pub branch: Vec<(i32, i32, usize)>,
    /// The pins in the (x, y) sort, by name, with their locations — `loc_pin_map_` in insertion order.
    pub pinlocs: Vec<(String, i32, i32)>,
    /// `drvrPt()`.
    pub drvr_pt: Option<usize>,
}

impl Tree {
    /// `SteinerTree::pins(pt)`: below `deg`, the pins at that point's location, in insertion order.
    pub fn pins(&self, pt: usize) -> Option<Vec<&str>> {
        if pt >= self.deg {
            return None;
        }
        let (x, y, _) = self.branch[pt];
        let pins: Vec<&str> = self.pinlocs.iter().filter(|p| p.1 == x && p.2 == y).map(|p| p.0.as_str()).collect();
        (!pins.is_empty()).then_some(pins)
    }
    pub fn location(&self, pt: usize) -> (i32, i32) {
        (self.branch[pt].0, self.branch[pt].1)
    }
}

/// What node construction reads: the timer at the net's corner (`graph`: its scene's cells), the
/// libraries (the LINK cells some values come from), the SDC environment (port loads).
pub struct Ctx<'a, 'g> {
    pub graph: &'a Graph<'g>,
    pub libs: &'a crate::preamble::Libs,
    pub sdc: &'a vyges_sta::graph::SdcEnv,
    pub limits: &'a Limits,
    pub dbu: i32,
    pub rc: WireRc,
}

impl Ctx<'_, '_> {
    /// `Resizer::dbuToMeters(int)`: `dist / (dbu · 1e6)` in double.
    pub fn dbu_to_meters(&self, dist: i32) -> f64 {
        f64::from(dist) / (f64::from(self.dbu) * 1e6)
    }
}

fn manhattan(a: (i32, i32), b: (i32, i32)) -> i32 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

impl BufferedNet {
    pub fn location(&self, n: usize) -> (i32, i32) {
        (self.nodes[n].x, self.nodes[n].y)
    }

    /// `BufferedNet::length()`: Manhattan distance from a wire to its `ref`.
    pub fn length(&self, n: usize) -> i32 {
        match self.nodes[n].kind {
            Kind::Wire { r } => manhattan(self.location(n), self.location(r)),
            _ => 0,
        }
    }

    /// `BufferedNet::wireRC` on no layer: 0 for a zero-length wire; else each direction's share
    /// of the length (meters over meters) times that direction's signal RC.
    pub fn wire_rc(&self, n: usize, ctx: &Ctx<'_, '_>) -> (f64, f64) {
        let Kind::Wire { r } = self.nodes[n].kind else { return (0.0, 0.0) };
        let len = self.length(n);
        if len == 0 {
            return (0.0, 0.0);
        }
        let (a, b) = (self.location(n), self.location(r));
        let dx = ctx.dbu_to_meters((a.0 - b.0).abs()) / ctx.dbu_to_meters(len);
        let dy = ctx.dbu_to_meters((a.1 - b.1).abs()) / ctx.dbu_to_meters(len);
        (dx * ctx.rc.h_res + dy * ctx.rc.v_res, dx * ctx.rc.h_cap + dy * ctx.rc.v_cap)
    }

    /// The load constructor at the corner: a liberty load takes `portCapacitance(port, corner)`
    /// (the corner port's max-side capacitance, the larger of rise and fall), `portFanoutLoad`
    /// (the LINK port's `fanout_load`, else its library's default, else 0) and
    /// `maxInputSlew(port, corner)`; a top-level port, the larger over rise and fall of its
    /// `set_load -pin_load` (max), and the other member defaults.
    fn load(&mut self, ctx: &Ctx<'_, '_>, at: (i32, i32), pin: usize) -> usize {
        use vyges_sta::liberty::{FALL, MAX, RISE};
        let mut node = BNode { kind: Kind::Load { pin }, x: at.0, y: at.1, layer: NULL_LAYER, cap: 0.0, fanout: 1.0, max_load_slew: INF };
        let vx = &ctx.graph.vertices[pin];
        if let (Some(lib), Some(cell), Some(port)) = (vx.lib, vx.cell.as_deref(), vx.port.as_deref()) {
            let scene_lib = &ctx.graph.libs[lib];
            let link_lib = ctx.libs.link_library(cell).unwrap_or(scene_lib);
            if let (Some(p), Some(lp)) = (scene_lib.cells.get(cell).and_then(|c| c.port(port)), link_lib.cells.get(cell).and_then(|c| c.port(port))) {
                node.cap = p.capacitance[RISE][MAX].max(p.capacitance[FALL][MAX]);
                node.fanout = lp.fanout_load.or(link_lib.default_fanout_load).unwrap_or(0.0);
                node.max_load_slew = timing::max_input_slew_at(link_lib, scene_lib, p.direction, p.max_transition, ctx.limits);
            }
        } else if let Some(c) = ctx.sdc.port_pin_cap.get(&vx.name) {
            for rf in [RISE, FALL] {
                if let Some(pin_cap) = c[rf][MAX] {
                    node.cap = node.cap.max(pin_cap);
                }
            }
        }
        self.push(node)
    }

    /// The junction constructor: `ref` and `ref2`'s caps and fanouts summed, the smaller slew limit.
    pub(crate) fn junction(&mut self, at: (i32, i32), r: usize, r2: usize) -> usize {
        let (a, b) = (&self.nodes[r], &self.nodes[r2]);
        let node = BNode { kind: Kind::Junction { r, r2 }, x: at.0, y: at.1, layer: NULL_LAYER, cap: a.cap + b.cap, fanout: a.fanout + b.fanout, max_load_slew: a.max_load_slew.min(b.max_load_slew) };
        self.push(node)
    }

    /// The wire constructor: at `from`, on no layer; `ref`'s cap plus the wire's (length in meters
    /// times its cap per meter, in double, narrowed); `ref`'s fanout and slew limit.
    pub(crate) fn wire(&mut self, ctx: &Ctx<'_, '_>, at: (i32, i32), r: usize) -> usize {
        let (ref_cap, fanout, max_load_slew) = (self.nodes[r].cap, self.nodes[r].fanout, self.nodes[r].max_load_slew);
        let n = self.push(BNode { kind: Kind::Wire { r }, x: at.0, y: at.1, layer: NULL_LAYER, cap: 0.0, fanout, max_load_slew });
        let (_, wire_cap) = self.wire_rc(n, ctx);
        self.nodes[n].cap = (f64::from(ref_cap) + ctx.dbu_to_meters(self.length(n)) * wire_cap) as f32;
        n
    }

    fn push(&mut self, node: BNode) -> usize {
        self.nodes.push(node);
        self.nodes.len() - 1
    }

    /// `BufferedNet::maxLoadWireLength`.
    pub fn max_load_wire_length(&self, n: usize) -> i32 {
        match self.nodes[n].kind {
            Kind::Wire { r } => self.length(n) + self.max_load_wire_length(r),
            Kind::Junction { r, r2 } => self.max_load_wire_length(r).max(self.max_load_wire_length(r2)),
            Kind::Load { .. } => 0,
        }
    }
}

/// `makeBufferedNetSteiner`'s adjacency: for each branch point `i` whose `n` is another point,
/// `i` joins `n`'s list and `n` joins `i`'s, in branch order.
pub fn adjacents(tree: &Tree) -> Vec<Vec<usize>> {
    let mut adj = vec![Vec::new(); tree.branch.len()];
    for (i, &(_, _, j)) in tree.branch.iter().enumerate() {
        if j != i {
            adj[i].push(j);
            adj[j].push(i);
        }
    }
    adj
}

/// `makeBufferedNetFromTree(tree, from, to, …)`: at `to` — on the FIRST visit of its location —
/// each LOAD pin there a load node, folded left into junctions; then each adjacent point but
/// `from`, in adjacency order, recursively, folded likewise; then, below the root and when `to`
/// is not at `from`'s location, the whole wrapped in a wire at `from`.
#[allow(clippy::too_many_arguments)]
fn from_tree(bn: &mut BufferedNet, ctx: &Ctx<'_, '_>, tree: &Tree, from: Option<usize>, to: usize, adj: &[Vec<usize>], visited: &mut std::collections::HashSet<(i32, i32)>, index: &std::collections::HashMap<&str, usize>) -> Option<usize> {
    let mut bnet: Option<usize> = None;
    let to_loc = tree.location(to);
    if let Some(pins) = tree.pins(to) {
        if visited.insert(to_loc) {
            for name in pins {
                let Some(&v) = index.get(name) else { continue };
                // `network->isLoad(pin)`: an instance input, or a top-level output port.
                let is_load = !ctx.graph.vertices[v].is_driver;
                if is_load {
                    let n1 = bn.load(ctx, to_loc, v);
                    bnet = Some(match bnet {
                        Some(b) => bn.junction(to_loc, b, n1),
                        None => n1,
                    });
                }
            }
        }
    }
    for &a in &adj[to] {
        if Some(a) != from {
            if let Some(n1) = from_tree(bn, ctx, tree, Some(to), a, adj, visited, index) {
                bnet = Some(match bnet {
                    Some(b) => bn.junction(to_loc, b, n1),
                    None => n1,
                });
            }
        }
    }
    if let (Some(b), Some(f)) = (bnet, from) {
        if tree.location(to) != tree.location(f) {
            return Some(bn.wire(ctx, tree.location(f), b));
        }
    }
    bnet
}

/// `Resizer::makeBufferedNetSteiner`: from the driver's Steiner point, the buffered net and its
/// root — `None` without a driver point or without a load.
pub fn make_buffered_net_steiner(ctx: &Ctx<'_, '_>, tree: &Tree) -> Option<(BufferedNet, usize)> {
    let drvr_pt = tree.drvr_pt?;
    let adj = adjacents(tree);
    let index: std::collections::HashMap<&str, usize> = ctx.graph.vertices.iter().enumerate().map(|(i, v)| (v.name.as_str(), i)).collect();
    let mut bn = BufferedNet::default();
    let mut visited = std::collections::HashSet::new();
    let root = from_tree(&mut bn, ctx, tree, None, drvr_pt, &adj, &mut visited, &index)?;
    Some((bn, root))
}

/// The tree in pre-order, one `bn|level|type|x|y|cap|fanout|max load slew|layer|load pin` line
/// per node.
pub fn trace_tree(bn: &BufferedNet, n: usize, level: usize, g: &Graph<'_>, out: &mut Vec<String>) {
    use crate::trace::g9;
    let node = &bn.nodes[n];
    let (kind, pin) = match &node.kind {
        Kind::Load { pin } => ("load", g.vertices[*pin].name.clone()),
        Kind::Junction { .. } => ("junction", "-".to_string()),
        Kind::Wire { .. } => ("wire", "-".to_string()),
    };
    out.push(format!("bn|{level}|{kind}|{}|{}|{}|{}|{}|{}|{pin}", node.x, node.y, g9(f64::from(node.cap)), g9(f64::from(node.fanout)), g9(f64::from(node.max_load_slew)), node.layer));
    match node.kind {
        Kind::Wire { r } => trace_tree(bn, r, level + 1, g, out),
        Kind::Junction { r, r2 } => {
            trace_tree(bn, r, level + 1, g, out);
            trace_tree(bn, r2, level + 1, g, out);
        }
        Kind::Load { .. } => {}
    }
}

/// `makeBufferedNetSteiner`'s Steiner tree, as the reference prints it before building:
/// `stdrvr|driver|driver point|branch count`, then `st|i|x|y|n|pins at i,` per branch point.
pub fn trace_steiner(tree: &Tree, drvr_name: &str, out: &mut Vec<String>) {
    let Some(dp) = tree.drvr_pt else { return };
    out.push(format!("stdrvr|{drvr_name}|{dp}|{}", tree.branch.len()));
    for (i, &(x, y, n)) in tree.branch.iter().enumerate() {
        let pins: String = tree.pins(i).unwrap_or_default().iter().map(|p| format!("{p},")).collect();
        out.push(format!("st|{i}|{x}|{y}|{n}|{pins}"));
    }
}

/// Whether a vertex is a top-level port (its load node keeps the defaults).
pub fn is_port(g: &Graph<'_>, v: usize) -> bool {
    matches!(g.vertices[v].conn, Conn::Port(_))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (makeBufferedNetSteiner): each non-self `n` link joins both lists, in branch order.
    #[test]
    fn adjacency_is_both_ways_in_branch_order() {
        let t = Tree { deg: 3, branch: vec![(0, 0, 3), (5, 0, 3), (0, 5, 3), (0, 0, 3)], pinlocs: vec![], drvr_pt: Some(0) };
        assert_eq!(adjacents(&t), vec![vec![3], vec![3], vec![3], vec![0, 1, 2]]);
    }

    // Rule (SteinerTree::pins): only points below deg carry pins, those at that location.
    #[test]
    fn pins_only_below_deg() {
        let t = Tree { deg: 2, branch: vec![(0, 0, 1), (5, 0, 1), (0, 0, 1)], pinlocs: vec![("a".into(), 0, 0), ("b".into(), 5, 0), ("c".into(), 0, 0)], drvr_pt: Some(0) };
        assert_eq!(t.pins(0), Some(vec!["a", "c"]));
        assert_eq!(t.pins(2), None, "a Steiner point at a pin's location still has no pins");
    }
}
