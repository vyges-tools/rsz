// SPDX-License-Identifier: Apache-2.0
//! Electrical repair of a placed design: repeaters inserted along each net's Steiner tree where a
//! wire is too long, a driver sees too much capacitance or fans out too far, or a load's transition
//! is too slow; a driver resized where its own transition is.
//!
//! The modules follow the repair's call sequence:
//! - [`preamble`]: what the repair reads about the libraries before any net — the buffers, their
//!   target slews and each cell's target load, the slew shape factor.
//! - [`repair_design`]: the driver loop, a thin sequencer of the stages in their order.
//! - [`repair_timing`]: `repair_timing -setup` up to its first progress row.
//! - [`timing`]: what the repair asks the timer — slew, capacitance and fanout checks per scene, the
//!   forward pass, the driver order.
//! - [`driver_slew`]: a driver's own slew violation — the size that fits, else the load cap that
//!   would.
//! - [`fanout`]: fanout repair's load regions.
//! - [`buffered_net`]: the net's Steiner tree as the repair walks it.
//! - [`walk`]: the walk from the loads up, and the repeater it inserts.
//! - [`sizing`]: equivalent cells and which of them fits a load.
//! - [`design`]: what the repair asks of the design it edits.
//! - [`order`]: sorts whose order is a value.
//! - [`trace`]: the call-sequence trace, one line per decision.

pub mod buffer_ports;
pub mod buffered_net;
pub mod design;
pub mod driver_slew;
pub mod fanout;
pub mod max_wire_length;
pub mod order;
pub mod preamble;
pub mod repair_design;
pub mod repair_timing;
pub mod sizing;
pub mod timing;
pub mod trace;
pub mod walk;

/// Why a run stopped short of a result.
#[derive(Debug, Clone, PartialEq)]
pub enum Stop {
    /// The input is outside what is modelled. Never a pass.
    Refused { code: &'static str, msg: String },
    /// The reference would stop here too (its own error).
    Error { code: &'static str, msg: String },
}

impl Stop {
    pub fn refused(code: &'static str, msg: String) -> Stop {
        Stop::Refused { code, msg }
    }
    pub fn error(code: &'static str, msg: String) -> Stop {
        Stop::Error { code, msg }
    }
    pub fn code(&self) -> &'static str {
        match self {
            Stop::Refused { code, .. } | Stop::Error { code, .. } => code,
        }
    }
    pub fn message(&self) -> &str {
        match self {
            Stop::Refused { msg, .. } | Stop::Error { msg, .. } => msg,
        }
    }
}
