// SPDX-License-Identifier: Apache-2.0
//! `remove_buffers` (`Resizer::removeBuffers`): every buffer the block's instance order reaches
//! that can go, or only the named ones. One thin function per reference stage, in the
//! reference's order: [`remove_buffers`] → [`remove_buffer_if_possible`] → [`can_remove_buffer`]
//! → [`remove_buffer`].
//!
//! The rules, as the reference applies them:
//! - No instances named: the block's instances in its own order (`block_->getInsts()`), each
//!   with dont-touch and fixed placement honoured; a removal is seen by every later instance.
//! - Instances named: each in the order given, protection on the buffer and its two nets set
//!   aside; one that cannot go warns RSZ-0097.
//! - Then RSZ-0026 with the count.

/// What the remover reads and edits: the database, the liberty cells, the SDC.
pub trait RemovalDesign {
    /// `block_->getInsts()`: every instance, in the block's order.
    fn insts(&self) -> Vec<String>;
    /// `canRemoveBuffer`'s cell test: the instance has a liberty cell, is a logic standard cell
    /// (`isLogicStdCell`: master type CORE exactly) and the cell is a buffer — then its buffer
    /// ports (`LibertyCell::bufferPorts`), input then output.
    fn buffer_ports(&self, inst: &str) -> Option<(String, String)>;
    /// The flat net on `inst`'s pin `pin` (`dbNetwork::flatNet`), if any.
    fn net_of(&self, inst: &str, pin: &str) -> Option<String>;
    fn inst_dont_touch(&self, inst: &str) -> bool;
    /// FIXED (or any fixed status: `dbInst::isFixed`).
    fn inst_fixed(&self, inst: &str) -> bool;
    fn net_dont_touch(&self, net: &str) -> bool;
    /// The net has a top-level port on it.
    fn net_has_port(&self, net: &str) -> bool;
    /// `dbNet::canMergeNet(removed)` on `survivor`.
    fn can_merge_net(&self, survivor: &str, removed: &str) -> bool;
    /// The instances of the net's instance terminals, in its terminal order.
    fn net_insts(&self, net: &str) -> Vec<String>;
    /// `Sdc::isConstrained(pin)` on an instance pin `inst/pin`.
    fn pin_constrained(&self, pin: &str) -> bool;
    /// `Sdc::isConstrained(inst)`.
    fn inst_constrained(&self, inst: &str) -> bool;
    /// `Sdc::isConstrained(net)`.
    fn net_constrained(&self, net: &str) -> bool;
    /// `removeBuffer`'s edit once the input net is known: with no output net the buffer is
    /// destroyed; else the pins are disconnected, the removed net merged into the survivor
    /// (`keep_output`: the output net survives), the buffer deleted, and the survivor renamed to
    /// the removed net's name when it is the deeper of the two (`isDeeperThan`). `unprotect`:
    /// the buffer's and both nets' dont-touch set aside for the edit, the survivor's restored
    /// after (`BufferRemovalProtection`).
    fn remove(&mut self, inst: &str, in_pin: &str, out_pin: &str, unprotect: bool) -> Result<(), String>;
}

/// What one `remove_buffers` did: the count, and the reference's lines in order.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Outcome {
    /// The instances asked: every one in the block, or the named ones.
    pub checked: usize,
    pub removed: usize,
    pub lines: Vec<String>,
}

/// `Resizer::removeBuffers(insts)`.
pub fn remove_buffers(d: &mut dyn RemovalDesign, insts: &[String]) -> Result<Outcome, String> {
    let mut out = Outcome::default();
    if insts.is_empty() {
        let all = d.insts();
        out.checked = all.len();
        for inst in all {
            if remove_buffer_if_possible(d, &inst, true, &mut out)? {
                out.removed += 1;
            }
        }
    } else {
        out.checked = insts.len();
        for inst in insts {
            if remove_buffer_if_possible(d, inst, false, &mut out)? {
                out.removed += 1;
            } else {
                out.lines.push(format!("[WARNING RSZ-0097] Instance {inst} cannot be removed because it is not a buffer, functions as a feedthrough port buffer, or is constrained"));
            }
        }
    }
    out.lines.push(format!("[INFO RSZ-0026] Removed {} buffers.", out.removed));
    Ok(out)
}

/// `Resizer::removeBufferIfPossible`.
pub fn remove_buffer_if_possible(d: &mut dyn RemovalDesign, inst: &str, honor: bool, out: &mut Outcome) -> Result<bool, String> {
    if can_remove_buffer(d, inst, honor) {
        return remove_buffer(d, inst, honor, out);
    }
    Ok(false)
}

/// `bufferRemovalKeepsOutputNet` on a flat netlist: the output net survives only when it has a
/// port and the input net has none.
pub fn keeps_output_net(in_has_port: bool, out_has_port: bool) -> bool {
    !in_has_port && out_has_port
}

/// `Resizer::canRemoveBuffer(buffer, honor_dont_touch_fixed)`.
pub fn can_remove_buffer(d: &dyn RemovalDesign, inst: &str, honor: bool) -> bool {
    let Some((in_pin, out_pin)) = d.buffer_ports(inst) else { return false };
    let Some(in_net) = d.net_of(inst, &in_pin) else { return false };
    let out_net = d.net_of(inst, &out_pin);
    if honor && (d.inst_dont_touch(inst) || d.inst_fixed(inst) || d.net_dont_touch(&in_net) || out_net.as_deref().is_some_and(|n| d.net_dont_touch(n))) {
        return false;
    }
    let keep_output = keeps_output_net(d.net_has_port(&in_net), out_net.as_deref().is_some_and(|n| d.net_has_port(n)));
    let (survivor, removed) = if keep_output { (out_net.clone(), Some(in_net.clone())) } else { (Some(in_net.clone()), out_net.clone()) };
    if d.pin_constrained(&format!("{inst}/{in_pin}")) || d.pin_constrained(&format!("{inst}/{out_pin}")) || removed.as_deref().is_some_and(|n| d.net_constrained(n)) || d.inst_constrained(inst) {
        return false;
    }
    // A disconnected output: removable, the buffer is destroyed without a merge.
    let Some(removed) = removed else { return true };
    let survivor = survivor.expect("the output net survives only when it exists");
    if honor {
        return d.can_merge_net(&survivor, &removed);
    }
    // Named removal sets aside the buffer's and its nets' protection only: every other
    // dont-touch instance on the removed net still refuses the merge.
    !d.net_insts(&removed).iter().any(|i| i != inst && d.inst_dont_touch(i))
}

/// `Resizer::removeBuffer(buffer, honor_dont_touch_fixed)`.
pub fn remove_buffer(d: &mut dyn RemovalDesign, inst: &str, honor: bool, out: &mut Outcome) -> Result<bool, String> {
    let (in_pin, out_pin) = d.buffer_ports(inst).ok_or_else(|| format!("{inst}: not a buffer"))?;
    if d.net_of(inst, &in_pin).is_none() {
        out.lines.push(format!("[WARNING RSZ-0168] The input pin of buffer '{inst}' is undriven. Do not remove the buffer."));
        return Ok(false);
    }
    d.remove(inst, &in_pin, &out_pin, !honor)?;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};

    /// A toy netlist: instances in order, each with (master is buffer, pins → net).
    #[derive(Default)]
    struct Toy {
        insts: Vec<String>,
        buffers: BTreeSet<String>,
        pins: BTreeMap<(String, String), String>,
        dont_touch: BTreeSet<String>,
        fixed: BTreeSet<String>,
        ports: BTreeSet<String>,
        constrained: BTreeSet<String>,
        removed: Vec<String>,
    }

    impl Toy {
        fn buf(&mut self, inst: &str, a: Option<&str>, z: Option<&str>) {
            self.insts.push(inst.into());
            self.buffers.insert(inst.into());
            for (p, n) in [("A", a), ("Z", z)] {
                if let Some(n) = n {
                    self.pins.insert((inst.into(), p.into()), n.into());
                }
            }
        }
        fn cell(&mut self, inst: &str, a: &str, z: &str) {
            self.insts.push(inst.into());
            self.pins.insert((inst.into(), "A".into()), a.into());
            self.pins.insert((inst.into(), "ZN".into()), z.into());
        }
    }

    impl RemovalDesign for Toy {
        fn insts(&self) -> Vec<String> {
            self.insts.clone()
        }
        fn buffer_ports(&self, inst: &str) -> Option<(String, String)> {
            self.buffers.contains(inst).then(|| ("A".into(), "Z".into()))
        }
        fn net_of(&self, inst: &str, pin: &str) -> Option<String> {
            self.pins.get(&(inst.into(), pin.into())).cloned()
        }
        fn inst_dont_touch(&self, inst: &str) -> bool {
            self.dont_touch.contains(inst)
        }
        fn inst_fixed(&self, inst: &str) -> bool {
            self.fixed.contains(inst)
        }
        fn net_dont_touch(&self, net: &str) -> bool {
            self.dont_touch.contains(net)
        }
        fn net_has_port(&self, net: &str) -> bool {
            self.ports.contains(net)
        }
        fn can_merge_net(&self, survivor: &str, removed: &str) -> bool {
            !self.net_dont_touch(survivor) && !self.net_insts(removed).iter().any(|i| self.inst_dont_touch(i))
        }
        fn net_insts(&self, net: &str) -> Vec<String> {
            self.pins.iter().filter(|(_, n)| *n == net).map(|((i, _), _)| i.clone()).collect()
        }
        fn pin_constrained(&self, pin: &str) -> bool {
            self.constrained.contains(pin)
        }
        fn inst_constrained(&self, inst: &str) -> bool {
            self.constrained.contains(inst)
        }
        fn net_constrained(&self, net: &str) -> bool {
            self.constrained.contains(net)
        }
        fn remove(&mut self, inst: &str, in_pin: &str, out_pin: &str, _unprotect: bool) -> Result<(), String> {
            let a = self.pins.remove(&(inst.into(), in_pin.into())).unwrap();
            let z = self.pins.remove(&(inst.into(), out_pin.into()));
            if let Some(z) = z {
                let (keep, gone) = if keeps_output_net(self.ports.contains(&a), self.ports.contains(&z)) { (z, a) } else { (a, z) };
                for n in self.pins.values_mut().filter(|n| **n == gone) {
                    *n = keep.clone();
                }
            }
            self.insts.retain(|i| i != inst);
            self.removed.push(inst.into());
            Ok(())
        }
    }

    /// Rule (`removeBuffers` with no instances): the block's order, each removal seen by the
    /// next — a chain of two buffers both go, in order, and the count is reported (RSZ-0026).
    #[test]
    fn every_buffer_goes_in_block_order() {
        let mut t = Toy::default();
        t.cell("u0", "in", "n0");
        t.buf("b1", Some("n0"), Some("n1"));
        t.buf("b2", Some("n1"), Some("n2"));
        t.cell("u3", "n2", "out");
        let o = remove_buffers(&mut t, &[]).unwrap();
        assert_eq!(t.removed, ["b1", "b2"]);
        assert_eq!(o.removed, 2);
        assert_eq!(o.lines, ["[INFO RSZ-0026] Removed 2 buffers."]);
        assert_eq!(t.net_of("u3", "A").as_deref(), Some("n0"), "the input net survives");
    }

    /// Rule (`canRemoveBuffer`, honoured): a dont-touch or fixed buffer, a dont-touch net on
    /// either side, a constrained pin / instance / removed net, and a dont-touch sink on the
    /// removed net each keep the buffer.
    #[test]
    fn protection_and_constraints_keep_a_buffer() {
        let base = || {
            let mut t = Toy::default();
            t.cell("u0", "in", "n0");
            t.buf("b1", Some("n0"), Some("n1"));
            t.cell("u2", "n1", "out");
            t
        };
        let keeps = |f: &dyn Fn(&mut Toy)| {
            let mut t = base();
            f(&mut t);
            remove_buffers(&mut t, &[]).unwrap().removed == 0
        };
        assert!(!keeps(&|_| {}), "unprotected: removed");
        assert!(keeps(&|t| {
            t.dont_touch.insert("b1".into());
        }));
        assert!(keeps(&|t| {
            t.fixed.insert("b1".into());
        }));
        assert!(keeps(&|t| {
            t.dont_touch.insert("n0".into());
        }));
        assert!(keeps(&|t| {
            t.dont_touch.insert("n1".into());
        }));
        assert!(keeps(&|t| {
            t.dont_touch.insert("u2".into());
        }), "a dont-touch sink on the removed net: canMergeNet refuses");
        assert!(keeps(&|t| {
            t.constrained.insert("b1/A".into());
        }));
        assert!(keeps(&|t| {
            t.constrained.insert("b1/Z".into());
        }));
        assert!(keeps(&|t| {
            t.constrained.insert("b1".into());
        }));
        assert!(keeps(&|t| {
            t.constrained.insert("n1".into());
        }), "the removed (output) net constrained");
        assert!(!keeps(&|t| {
            t.constrained.insert("n0".into());
        }), "the surviving net constrained: not asked");
    }

    /// Rule (`bufferRemovalKeepsOutputNet`): with a port only on the output net, the input net is
    /// the one removed — its constraint is the one asked.
    #[test]
    fn a_port_on_the_output_net_alone_keeps_the_output_net() {
        assert!(keeps_output_net(false, true));
        assert!(!keeps_output_net(true, true));
        assert!(!keeps_output_net(false, false));
        assert!(!keeps_output_net(true, false));
        let mut t = Toy::default();
        t.cell("u0", "in", "n0");
        t.buf("b1", Some("n0"), Some("out"));
        t.ports.insert("out".into());
        t.constrained.insert("n0".into());
        assert_eq!(remove_buffers(&mut t, &[]).unwrap().removed, 0);
    }

    /// Rule (named `remove_buffers b1 b3`): protection on the buffer is set aside, a non-buffer
    /// warns RSZ-0097, and a disconnected output is still removable (destroyed); no input net
    /// keeps it.
    #[test]
    fn named_buffers_set_protection_aside_and_warn_on_the_rest() {
        let mut t = Toy::default();
        t.cell("u0", "in", "n0");
        t.buf("b1", Some("n0"), Some("n1"));
        t.cell("u2", "n1", "n2");
        t.buf("b3", Some("n2"), None);
        t.buf("b4", None, Some("n4"));
        t.dont_touch.insert("b1".into());
        let o = remove_buffers(&mut t, &["b1".into(), "u2".into(), "b3".into(), "b4".into()]).unwrap();
        assert_eq!(t.removed, ["b1", "b3"]);
        assert_eq!(o.removed, 2);
        assert_eq!(o.lines.len(), 3, "{:?}", o.lines);
        assert!(o.lines[0].contains("RSZ-0097") && o.lines[0].contains("u2"));
        assert!(o.lines[1].contains("RSZ-0097") && o.lines[1].contains("b4"));
        // a dont-touch sink other than the buffer on the removed net still refuses
        let mut t = Toy::default();
        t.buf("b1", Some("n0"), Some("n1"));
        t.cell("u2", "n1", "n2");
        t.dont_touch.insert("u2".into());
        assert_eq!(remove_buffers(&mut t, &["b1".into()]).unwrap().removed, 0);
    }
}
