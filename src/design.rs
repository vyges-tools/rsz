// SPDX-License-Identifier: Apache-2.0
//! The design as the repair edits it: the database, the netlist the timer reads from it, and the
//! estimated parasitics — cached per net, as the estimator keeps them during a repair.
//!
//! The estimator's incremental rules: an edit marks the nets it touches invalid; a net is
//! re-estimated only when asked for (`ensureWireParasitic`) — when invalid or when it has no
//! parasitic — at the routing alpha in force THEN; every other net keeps the estimate it had.

use std::collections::HashMap;

use vyges_sta::graph::NetParasitics;
use vyges_sta::netlist::Netlist;

use crate::buffered_net::Tree;
use crate::timing::NetInfo;

/// A repeater just inserted.
#[derive(Debug, Clone, PartialEq)]
pub struct Repeater {
    pub inst: String,
    /// Its input and output pin names, and the net it drives.
    pub input: String,
    pub output: String,
    pub out_net: String,
}

/// What the timer has not yet seen: the database's edit callbacks in order (`Db::edit_log_take`),
/// and the nets `updateParasitics` estimated since — each gets `delaysInvalidFromFanin`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TimerEdits {
    pub events: Vec<String>,
    pub updated_nets: Vec<String>,
}

/// What the repair asks of the design. The command line implements it over the database.
pub trait Design {
    /// The netlist as the database holds it now (the timer's vertex order).
    fn netlist(&self) -> &Netlist;
    /// [`Design::netlist`] to hold while the design is edited further (a timing graph borrows it).
    /// A design that keeps its netlist shared hands out the share instead of a copy.
    fn netlist_shared(&self) -> std::sync::Arc<Netlist> {
        std::sync::Arc::new(self.netlist().clone())
    }
    /// A count that moves whenever the netlist changes, and only then: equal counts mean equal
    /// netlists, so a holder can tell "still current" without comparing them. `None`: not kept —
    /// compare the netlists.
    fn netlist_generation(&self) -> Option<u64> {
        None
    }
    /// A scene's parasitic cache as the timer reads it now — an invalid net keeps its old entry
    /// until it is ensured (every scene is estimated together).
    fn parasitics(&self, scene: usize) -> &HashMap<String, NetParasitics>;
    /// Pin id order and the nets `repairDriver` passes over, as the database holds them now.
    fn net_info(&self) -> &NetInfo;
    /// `EstimateParasitics::ensureWireParasitic(drvr_pin, net)`.
    fn ensure_wire_parasitic(&mut self, net: &str) -> Result<(), String>;
    /// `EstimateParasitics::updateParasitics()`: every net marked invalid, re-estimated.
    fn update_parasitics(&mut self) -> Result<(), String>;
    /// `est::makeSteinerTree(drvr_pin)` for a net and its driver, at the current routing alpha.
    fn steiner(&self, net: &str, drvr_pin: &str) -> Option<Tree>;
    /// What `Resizer::makeBufferedNet` builds from for a net and its driver: the Steiner tree under
    /// placement parasitics, the global route under global-route parasitics.
    fn net_shape(&self, net: &str, drvr_pin: &str) -> Option<crate::buffered_net::NetShape> {
        self.steiner(net, drvr_pin).map(crate::buffered_net::NetShape::Steiner)
    }
    /// The net a pin is on, by name (`flatNet`), and whether it is special.
    fn pin_net(&self, _pin: &str) -> Option<(String, bool)> {
        None
    }
    /// `GlobalRouter::isNetResAware(db_net)`.
    fn net_res_aware(&self, _net: &str) -> bool {
        false
    }
    /// `getFRNetResistance` and `getFRNetResistanceOnMinResistanceLayer` of a net.
    fn net_resistances(&self, _net: &str) -> (f32, f32) {
        (0.0, 0.0)
    }
    /// `RerouteCandidate::apply`: `setResistanceAware(true)`, `addDirtyNet`, `setNetIsResAware`,
    /// `parasiticsInvalid` — the net routed again, resistance-aware, at the next update.
    fn reroute_net(&mut self, _net: &str) -> Result<(), String> {
        Err("rerouting needs global-route parasitics".into())
    }
    /// The net's global route in gcells of wire ([`crate::repair_design::route_gcell_length`]),
    /// and the tile size: `tryRerouteNet`'s short-net test. `None` without a route or a tile size.
    fn net_route_gcells(&self, _net: &str) -> Option<(i32, i32)> {
        None
    }
    /// `getParasiticsSrc() == kGlobalRouting`: the parasitics come from the global routes.
    fn global_routing_parasitics(&self) -> bool {
        false
    }
    /// The next `updateParasitics` re-routes nets resistance-aware: its `updateSlacks` reads the
    /// timer's net slacks mid-update (the edits pulled first, so the router's dirty nets are known).
    fn route_slacks_wanted(&mut self) -> bool {
        false
    }
    /// Those slacks (`sta_->slack(net, max)` per net), for the next `updateParasitics` only.
    fn set_route_slacks(&mut self, _slacks: std::collections::HashMap<String, f32>) {}
    /// `ensureWireParasitic(drvr_pin, net)` would call `updateRoutes`: global-route parasitics,
    /// and the net invalid or without a pi model (the edits pulled first).
    fn ensure_routes(&mut self, _net: &str) -> bool {
        false
    }
    /// `Resizer::insertBufferBeforeLoads(nullptr, loads, cell, &loc, reason)` with its
    /// `insertBufferPostProcess` (the location clamped to the core, placed).
    fn insert_repeater(&mut self, loads: &[String], cell: &str, loc: (i32, i32), reason: &str) -> Result<Repeater, String>;
    /// `Resizer::replaceCell`: `dbInst::swapMaster`.
    fn swap_master(&mut self, inst: &str, cell: &str) -> Result<(), String>;
    /// Per instance, the cell whose arc sets its output-to-output edges still carry after
    /// equivalent-arcs replacements ([`vyges_sta::graph::Graph::stale_out_arcs`]); none by default.
    fn stale_out_arcs(&self) -> Option<&std::collections::BTreeMap<String, String>> {
        None
    }
    /// An instance's location.
    fn inst_location(&self, inst: &str) -> (i32, i32);
    /// `dbNetwork::location(pin)`: an instance terminal's average XY (else its instance's origin),
    /// a port's first pin location (else (0, 0)).
    fn pin_location(&self, pin: &str) -> (i32, i32);
    /// `Network::visitConnectedPins(pin)` from a driver: its leaf pins (instance pins and ports;
    /// hierarchical pins are no loads) in visit order — through the module nets when the pin has
    /// one, else its flat net's instance terminals in the database's order, then its ports.
    fn visit_connected_pins(&self, pin: &str) -> Vec<String>;
    /// Begin recording the edits for the incremental timer.
    fn start_timer_edits(&mut self) -> Result<(), String> {
        Ok(())
    }
    /// The edits since the last take (see [`TimerEdits`]).
    fn take_timer_edits(&mut self) -> TimerEdits {
        TimerEdits::default()
    }
}
