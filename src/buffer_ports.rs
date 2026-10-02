// SPDX-License-Identifier: Apache-2.0
//! `buffer_ports`: a buffer between each top-level port and the logic it touches — after each
//! input port, before each output port — so the port is driven, or loads, one known cell.
//!
//! One function per stage of the rule, in its call order: [`buffer_ports`] runs
//! [`buffer_inputs`] then [`buffer_outputs`]; each walks the top-level ports in the database's
//! port order, filters them, and buffers each survivor ([`buffer_input`] / [`buffer_output`]).
//! The insertion is the database's own (`insertBufferAfterDriver` / `insertBufferBeforeLoad`)
//! with the resizer's post-processing (location clamped to the core, placed).
//!
//! The estimator's state across the edit: each walk runs inside one incremental-parasitics
//! guard, so every insertion marks the nets it touches invalid — the port's net and the new one
//! — and the guard's exit re-estimates each of them at the placement as it now stands
//! (`updateParasitics`). Every other net keeps the estimate it had. A later `repair_design` starts
//! from that state.

use crate::design::Design;
use crate::Stop;

/// What `buffer_ports` reads of the design beyond what the repair reads.
pub trait PortDesign: Design {
    /// The top-level ports in the order the top instance's pin iterator visits them — the
    /// database's port order.
    fn top_ports(&self) -> Vec<String>;
    /// A port's direction as the database states it: INPUT, OUTPUT, INOUT or FEEDTHRU.
    fn port_direction(&self, port: &str) -> String;
    /// The flat net a port is on, if any.
    fn port_net(&self, port: &str) -> Option<String>;
    /// `dbNet::isDoNotTouch`.
    fn net_dont_touch(&self, net: &str) -> bool;
    /// `dbNet::isSpecial`.
    fn net_special(&self, net: &str) -> bool;
    /// `dbNet::getITerms` in the database's order, supply terminals left out (the net pin
    /// iterator's leaf pins on a flat net — its ports are not among them).
    fn net_iterms(&self, net: &str) -> Vec<String>;
    /// The net's ports, in the database's order.
    fn net_bterms(&self, net: &str) -> Vec<String>;
    /// `dbNet::getFirstOutput`: the first instance terminal on the net with an output signal.
    fn net_first_output(&self, net: &str) -> Option<String>;
    /// `dbInst::isDoNotTouch`.
    fn inst_dont_touch(&self, inst: &str) -> bool;
    /// The instance's liberty cell: `Some(is_buffer)`, `None` when it has none.
    fn inst_is_buffer(&self, inst: &str) -> Option<bool>;
    /// The net's drivers' liberty ports, as `(pin, tristate)`.
    fn net_drivers(&self, net: &str) -> Vec<(String, bool)>;
    /// `Sta::isClock(pin)` on a top-level port: whether the clock network holds it.
    fn port_is_clock(&self, port: &str) -> bool;
    /// `dbNet::insertBufferAfterDriver(drvr, master, nullptr, "input")` then
    /// `insertBufferPostProcess`. The callbacks mark both nets invalid. Returns the new instance.
    fn insert_buffer_after_driver(&mut self, drvr: &str, cell: &str, reason: &str) -> Result<String, String>;
    /// `dbNet::insertBufferBeforeLoad(load, master, nullptr, "output")` then
    /// `insertBufferPostProcess`. The callbacks mark both nets invalid. Returns the new instance.
    fn insert_buffer_before_load(&mut self, load: &str, cell: &str, reason: &str) -> Result<String, String>;
}

/// The command's options.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Options {
    pub inputs: bool,
    pub outputs: bool,
    /// `-buffer_cell`: the cell to use instead of the weakest buffer.
    pub buffer_cell: Option<String>,
    pub verbose: bool,
}

impl Options {
    /// The command line: `-inputs`, `-outputs`, `-buffer_cell C`, `-verbose`. Neither side named
    /// means both. `-max_utilization` is refused: it sets the limit a later repair checks against.
    pub fn parse(args: &[String]) -> Result<Options, String> {
        let mut o = Options::default();
        let mut it = args.iter();
        while let Some(a) = it.next() {
            match a.as_str() {
                "-inputs" | "-input" => o.inputs = true,
                "-outputs" | "-output" => o.outputs = true,
                "-verbose" => o.verbose = true,
                "-buffer_cell" => o.buffer_cell = Some(it.next().ok_or("buffer_ports -buffer_cell needs a cell")?.clone()),
                "-max_utilization" => return Err("buffer_ports -max_utilization: not modelled".into()),
                other => return Err(format!("buffer_ports {other}: not modelled")),
            }
        }
        if !o.inputs && !o.outputs {
            o.inputs = true;
            o.outputs = true;
        }
        Ok(o)
    }
}

/// One message line, with its code, in the order the command writes them.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub code: &'static str,
    pub warning: bool,
    pub text: String,
}

/// What a `buffer_ports` run did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Outcome {
    pub inserted_inputs: usize,
    pub inserted_outputs: usize,
    /// Ports the walks visited.
    pub ports_checked: usize,
    pub lines: Vec<Line>,
}

impl Outcome {
    fn info(&mut self, code: &'static str, text: String) {
        self.lines.push(Line { code, warning: false, text });
    }
    fn warn(&mut self, code: &'static str, text: String) {
        self.lines.push(Line { code, warning: true, text });
    }
}

/// The `buffer_ports` command: inputs first, then outputs. `weakest` is the weakest buffer
/// (`findBuffers`' `buffer_lowest_drive_`), used when no `-buffer_cell` is given.
pub fn buffer_ports(design: &mut dyn PortDesign, o: &Options, weakest: &str) -> Result<Outcome, Stop> {
    let mut out = Outcome::default();
    if o.inputs {
        buffer_inputs(design, o, weakest, &mut out)?;
    }
    if o.outputs {
        buffer_outputs(design, o, weakest, &mut out)?;
    }
    Ok(out)
}

/// `selectBufferCell`: the user's cell, else the weakest buffer.
fn select_buffer_cell<'a>(o: &'a Options, weakest: &'a str) -> &'a str {
    o.buffer_cell.as_deref().unwrap_or(weakest)
}

/// `Resizer::bufferInputs`.
pub fn buffer_inputs(design: &mut dyn PortDesign, o: &Options, weakest: &str, out: &mut Outcome) -> Result<(), Stop> {
    let cell = select_buffer_cell(o, weakest);
    if o.verbose {
        out.info("RSZ-0029", format!("Start input port buffering with {cell}."));
    }
    let mut inserted = 0;
    // The incremental-parasitics guard: edits inside mark nets invalid, its exit re-estimates them.
    for port in design.top_ports() {
        out.ports_checked += 1;
        if design.port_direction(&port) != "INPUT" {
            continue;
        }
        // Rule: an input port on no net is not modelled (the reference reads the net unchecked).
        let net = design.port_net(&port).ok_or_else(|| Stop::refused("RSZ-REFUSED", format!("input port {port} is on no net: not modelled")))?;
        // `isConstant` is refused at the command (no case analysis, no constant cells), so it
        // never holds here.
        if !design.net_dont_touch(&net) && !design.port_is_clock(&port) && !design.net_special(&net) && has_pins(design, &net) {
            // repair_design resizes it to the target slew.
            if buffer_input(design, &port, &net, cell, o.verbose, out)? {
                inserted += 1;
            }
        }
    }
    design.update_parasitics().map_err(|e| Stop::error("RSZ-ERROR", e))?;
    out.inserted_inputs += inserted;
    out.info("RSZ-0027", format!("Inserted {inserted} {cell} input buffers."));
    Ok(())
}

/// `Resizer::hasPins`: the net pin iterator has a first pin — an instance terminal, not a supply.
fn has_pins(design: &dyn PortDesign, net: &str) -> bool {
    !design.net_iterms(net).is_empty()
}

/// `Resizer::bufferInput`: buffered unless a load is dont-touch (RSZ-0085, and the walk stops
/// there) or every other pin on the net is a buffer's — a pin on an instance with no liberty
/// cell counts as neither, another port as a non-buffer.
fn buffer_input(design: &mut dyn PortDesign, port: &str, net: &str, cell: &str, verbose: bool, out: &mut Outcome) -> Result<bool, Stop> {
    let mut has_non_buffer = false;
    let mut has_dont_touch = false;
    // `connectedPinIterator(net)`: the net's instance terminals, then its ports.
    let pins: Vec<String> = design.net_iterms(net).into_iter().chain(design.net_bterms(net)).collect();
    for pin in pins {
        if pin == port {
            continue;
        }
        match pin.rsplit_once('/').filter(|_| !design.net_bterms(net).contains(&pin)) {
            Some((inst, _)) => {
                if design.inst_dont_touch(inst) {
                    has_dont_touch = true;
                    out.warn("RSZ-0085", format!("Input {net} can't be buffered due to dont-touch fanout {pin}"));
                    break;
                }
                if design.inst_is_buffer(inst) == Some(false) {
                    has_non_buffer = true;
                }
            }
            // Another port: the top instance, no database instance.
            None => has_non_buffer = true,
        }
    }
    if has_dont_touch || !has_non_buffer {
        if verbose {
            out.info("RSZ-0213", format!("Skipping input port {port} buffering."));
        }
        return Ok(false);
    }
    if verbose {
        out.info("RSZ-0214", format!("Buffering input port {port}."));
    }
    insert_buffer_after_driver(design, net, cell, "input")?;
    Ok(true)
}

/// `Resizer::insertBufferAfterDriver(net, …)`: the net's driver is its first output instance
/// terminal, else its first input (or inout) port.
fn insert_buffer_after_driver(design: &mut dyn PortDesign, net: &str, cell: &str, reason: &str) -> Result<String, Stop> {
    let drvr = match design.net_first_output(net) {
        Some(it) => it,
        None => design
            .net_bterms(net)
            .into_iter()
            .find(|b| matches!(design.port_direction(b).as_str(), "INPUT" | "INOUT"))
            .ok_or_else(|| Stop::error("RSZ-3002", format!("insertBufferAfterDriver: No driver found for net {net}")))?,
    };
    design.insert_buffer_after_driver(&drvr, cell, reason).map_err(|e| Stop::error("RSZ-3003", e))
}

/// `Resizer::bufferOutputs`.
pub fn buffer_outputs(design: &mut dyn PortDesign, o: &Options, weakest: &str, out: &mut Outcome) -> Result<(), Stop> {
    let cell = select_buffer_cell(o, weakest);
    if o.verbose {
        out.info("RSZ-0031", format!("Start output port buffering with {cell}."));
    }
    let mut inserted = 0;
    for port in design.top_ports() {
        out.ports_checked += 1;
        if design.port_direction(&port) != "OUTPUT" {
            continue;
        }
        let Some(net) = design.port_net(&port) else { continue };
        if !design.net_dont_touch(&net)
            && !design.net_special(&net)
            // DEF has no tristate output type, so the drivers say.
            && !has_tristate_or_dont_touch_driver(design, &net, out)
            && has_pins(design, &net)
        {
            buffer_output(design, &port, cell, o.verbose, out)?;
            inserted += 1;
        }
    }
    design.update_parasitics().map_err(|e| Stop::error("RSZ-ERROR", e))?;
    out.inserted_outputs += inserted;
    out.info("RSZ-0028", format!("Inserted {inserted} {cell} output buffers."));
    Ok(())
}

/// `Resizer::hasTristateOrDontTouchDriver`: a tristate driver, or a driver on a dont-touch
/// instance (RSZ-0084), in the drivers' order; the first one found answers.
fn has_tristate_or_dont_touch_driver(design: &dyn PortDesign, net: &str, out: &mut Outcome) -> bool {
    for (pin, tristate) in design.net_drivers(net) {
        if tristate {
            return true;
        }
        if let Some((inst, _)) = pin.rsplit_once('/').filter(|_| !design.net_bterms(net).contains(&pin)) {
            if design.inst_dont_touch(inst) {
                out.warn("RSZ-0084", format!("Output {net} can't be buffered due to dont-touch driver {pin}"));
                return true;
            }
        }
    }
    false
}

/// `Resizer::bufferOutput`.
fn buffer_output(design: &mut dyn PortDesign, port: &str, cell: &str, verbose: bool, out: &mut Outcome) -> Result<String, Stop> {
    if verbose {
        out.info("RSZ-0215", format!("Buffering output port {port}."));
    }
    design.insert_buffer_before_load(port, cell, "output").map_err(|e| Stop::error("RSZ-3017", e))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    /// Rule (the command's Tcl): naming neither side buffers both; `-buffer_cell` names the cell,
    /// else the weakest buffer is used (`selectBufferCell`).
    #[test]
    fn neither_side_named_means_both() {
        let o = Options::parse(&[]).unwrap();
        assert!(o.inputs && o.outputs);
        let o = Options::parse(&args(&["-outputs"])).unwrap();
        assert!(!o.inputs && o.outputs);
        let o = Options::parse(&args(&["-inputs", "-buffer_cell", "BUF_X4"])).unwrap();
        assert_eq!(select_buffer_cell(&o, "BUF_X1"), "BUF_X4");
        assert_eq!(select_buffer_cell(&Options::parse(&[]).unwrap(), "BUF_X1"), "BUF_X1");
    }

    /// `-max_utilization` sets the limit a later repair checks against: refused, not ignored.
    #[test]
    fn max_utilization_is_refused() {
        assert!(Options::parse(&args(&["-max_utilization", "60"])).unwrap_err().contains("not modelled"));
    }
}
