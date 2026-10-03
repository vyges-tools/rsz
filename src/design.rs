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
    /// `Resizer::insertBufferBeforeLoads(nullptr, loads, cell, &loc, reason)` with its
    /// `insertBufferPostProcess` (the location clamped to the core, placed).
    fn insert_repeater(&mut self, loads: &[String], cell: &str, loc: (i32, i32), reason: &str) -> Result<Repeater, String>;
    /// `Resizer::replaceCell`: `dbInst::swapMaster`.
    fn swap_master(&mut self, inst: &str, cell: &str) -> Result<(), String>;
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
