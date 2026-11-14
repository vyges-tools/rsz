// SPDX-License-Identifier: Apache-2.0
//! `remove_buffers` through the job runner, on the database (`Resizer::removeBuffers`): the
//! block's buffers in its order, or the named ones with their protection set aside; the SDC's
//! `isConstrained` keeps a buffer, and what it does not model refuses.
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

/// `in1` → `b1` → `b2` → `drv` (fanout 2: `l1`, `l2`) → `out1`: a chain of two buffers.
const DEF: &str = "VERSION 5.8 ;
DIVIDERCHAR \"/\" ;
BUSBITCHARS \"[]\" ;
DESIGN chain ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 28000 ) ;
ROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;
COMPONENTS 4 ;
    - b1 BUF_X1 + PLACED ( 20000 0 ) N ;
    - b2 BUF_X1 + PLACED ( 40000 0 ) N ;
    - b3 BUF_X2 + PLACED ( 60000 0 ) N ;
    - b4 BUF_X1 + PLACED ( 80000 0 ) N ;
END COMPONENTS
PINS 2 ;
    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;
    - out1 + NET out1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 1400 ) N ;
END PINS
NETS 4 ;
    - in1 ( PIN in1 ) ( b1 A ) + USE SIGNAL ;
    - n1 ( b1 Z ) ( b2 A ) + USE SIGNAL ;
    - n2 ( b2 Z ) ( b3 A ) + USE SIGNAL ;
    - n3 ( b3 Z ) ( b4 A ) + USE SIGNAL ;
    - out1 ( b4 Z ) ( PIN out1 ) + USE SIGNAL ;
END NETS
END DESIGN
";

fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire")
}

/// One job: the design, `sdc` when not empty, then `steps` (JSON objects). The exit code, the
/// report and the DEF written.
fn run(name: &str, sdc: &str, steps: &str) -> (Option<i32>, serde_json::Value, String) {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("remove_buffers").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example().join("cells.lef"), dir.join("cells.lef")).unwrap();
    std::fs::copy(example().join("cells.lib"), dir.join("cells.lib")).unwrap();
    std::fs::write(dir.join("d.def"), DEF).unwrap();
    std::fs::write(dir.join("c.sdc"), sdc).unwrap();
    let read_sdc = if sdc.is_empty() { "" } else { r#"{ "cmd": "read_sdc", "args": ["c.sdc"] },"# };
    let job = format!(
        r#"{{ "write_def": "out.def", "steps": [
        {{ "cmd": "read_lef", "args": ["cells.lef"] }},
        {{ "cmd": "read_def", "args": ["d.def"] }},
        {{ "cmd": "read_liberty", "args": ["cells.lib"] }},
        {read_sdc}
        {steps} ] }}"#
    );
    std::fs::write(dir.join("job.json"), job).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz")).args(["remove_buffers", "job.json", "-o", "report.json"]).current_dir(&dir).output().unwrap();
    let r = std::fs::read_to_string(dir.join("report.json")).ok().and_then(|t| serde_json::from_str(&t).ok()).unwrap_or(serde_json::Value::Null);
    (out.status.code(), r, std::fs::read_to_string(dir.join("out.def")).unwrap_or_default())
}

fn components(def: &str) -> Vec<String> {
    def.lines().filter_map(|l| l.trim().strip_prefix("- ")).filter(|l| l.contains("BUF_X")).map(|l| l.split_whitespace().next().unwrap().to_string()).collect()
}

/// Rule (`removeBuffers` with no instances, `removeBuffer` merge and name): every buffer goes in
/// the block's order — the last one's output net keeps its port, so it is the survivor there —
/// RSZ-0026 counts them. With `b3` dont-touch only `b1` goes: `b3` is protected, and `b2` and
/// `b4` would each remove a net with `b3` on it (`dbNet::canMergeNet` refuses a dont-touch
/// instance on the removed net).
#[test]
fn every_buffer_goes_unless_protected() {
    let (code, r, def) = run("all", "", r#"{ "cmd": "remove_buffers", "args": [] }"#);
    assert_eq!(code, Some(0), "{r}");
    assert_eq!(r["removed_buffers"].as_u64(), Some(4), "{r}");
    assert!(components(&def).is_empty(), "{def}");
    assert_eq!(r["remove_buffers"][0]["lines"][0], "[INFO RSZ-0026] Removed 4 buffers.", "{r}");
    let (_, r, def) = run("dt", "", r#"{ "cmd": "set_dont_touch", "args": ["b3"] }, { "cmd": "remove_buffers", "args": [] }"#);
    assert_eq!(r["removed_buffers"].as_u64(), Some(1), "{r}");
    assert_eq!(components(&def), ["b2", "b3", "b4"], "{def}");
}

/// Rule (named removal: `honor_dont_touch_fixed` false, `BufferRemovalProtection`): a named
/// dont-touch buffer goes, and the surviving net's dont-touch is restored — a removal of every
/// buffer after it keeps `b1` and `b3`, both on that net, and takes `b4`.
#[test]
fn a_named_buffer_goes_despite_dont_touch() {
    let (code, r, def) = run("named", "", r#"{ "cmd": "set_dont_touch", "args": ["b2", "n1"] }, { "cmd": "remove_buffers", "args": ["b2"] }"#);
    assert_eq!(code, Some(0), "{r}");
    assert_eq!(components(&def), ["b1", "b3", "b4"], "{def}");
    assert_eq!(r["remove_buffers"][0]["removed"].as_u64(), Some(1), "{r}");
    let (_, r, def) = run("named_then_all", "", r#"{ "cmd": "set_dont_touch", "args": ["b2", "n1"] }, { "cmd": "remove_buffers", "args": ["b2"] }, { "cmd": "remove_buffers", "args": [] }"#);
    assert_eq!(components(&def), ["b1", "b3"], "{r}\n{def}");
}

/// Rule (`Sdc::isConstrained(pin)`: a case value on a pin): the buffer whose input pin the SDC
/// holds is kept. An exception through an instance is not modelled: refused.
#[test]
fn a_constrained_pin_keeps_its_buffer_and_an_instance_constraint_refuses() {
    let (code, r, def) = run("pin", "set_case_analysis 0 [get_pins {b2/A}]\n", r#"{ "cmd": "remove_buffers", "args": [] }"#);
    assert_eq!(code, Some(0), "{r}");
    assert_eq!(components(&def), ["b2"], "{def}");
    let (code, r, _) = run("cell", "create_clock -name c -period 1 [get_ports in1]\nset_false_path -through [get_cells {b3}]\n", r#"{ "cmd": "remove_buffers", "args": [] }"#);
    assert_eq!(code, Some(3), "{r}");
    assert!(r["reason"].as_str().unwrap_or_default().contains("not modelled"), "{r}");
}

/// `b1` → `drv` → `b2` → `l1` → `out1`, with `in1` also reaching `out2` (another port counts as
/// a non-buffer load, so `in1` is buffered).
const REUSE_DEF: &str = "VERSION 5.8 ;
DIVIDERCHAR \"/\" ;
BUSBITCHARS \"[]\" ;
DESIGN reuse ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 28000 ) ;
ROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;
COMPONENTS 4 ;
    - b1 BUF_X1 + PLACED ( 20000 0 ) N ;
    - drv BUF_X1 + PLACED ( 40000 0 ) N ;
    - b2 BUF_X1 + PLACED ( 60000 0 ) N ;
    - l1 BUF_X1 + PLACED ( 80000 0 ) N ;
END COMPONENTS
PINS 3 ;
    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;
    - out1 + NET out1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 1400 ) N ;
    - out2 + NET in1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 9400 ) N ;
END PINS
NETS 5 ;
    - in1 ( PIN in1 ) ( b1 A ) ( PIN out2 ) + USE SIGNAL ;
    - na ( b1 Z ) ( drv A ) + USE SIGNAL ;
    - nb ( drv Z ) ( b2 A ) + USE SIGNAL ;
    - n1 ( b2 Z ) ( l1 A ) + USE SIGNAL ;
    - out1 ( l1 Z ) ( PIN out1 ) + USE SIGNAL ;
END NETS
END DESIGN
";

/// Rule (odb `dbTable`: the block iterates by id, and `create` takes the LAST id `destroy` freed):
/// after `remove_buffers b1 b2` frees ids 1 and 3, `buffer_ports` makes `input1` at id 3 and
/// `output2` at id 1 — FIRST in the block's order, before `drv`, not at the end — and their nets
/// take freed net ids the same way (the reference at the pin: `output2` 1, `drv` 2, `input1` 3,
/// `l1` 4, `output3` 5; its DEF and ours identical). The netlist the timer reads must follow
/// (checked against a full read on every edit, `VYGES_RSZ_CHECK_REFRESH`) by inserting in place:
/// only the command's first edit reads every name (`VYGORDER|general`, O(design)); a full read
/// per port buffer took hours on a large block.
#[test]
fn a_buffer_made_after_a_removal_takes_the_freed_id_mid_order_without_a_full_read() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("remove_buffers").join("reuse");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example().join("cells.lef"), dir.join("cells.lef")).unwrap();
    std::fs::copy(example().join("cells.lib"), dir.join("cells.lib")).unwrap();
    std::fs::write(dir.join("d.def"), REUSE_DEF).unwrap();
    let job = r#"{ "write_def": "out.def", "steps": [
        { "cmd": "read_lef", "args": ["cells.lef"] },
        { "cmd": "read_def", "args": ["d.def"] },
        { "cmd": "read_liberty", "args": ["cells.lib"] },
        { "cmd": "remove_buffers", "args": ["b1", "b2"] },
        { "cmd": "buffer_ports", "args": [] } ] }"#;
    std::fs::write(dir.join("job.json"), job).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz"))
        .args(["buffer_ports", "job.json", "-o", "report.json"])
        .env("VYGES_RSZ_CHECK_REFRESH", "1")
        .env("VYGES_RSZ_ORDER_TRACE", "1")
        .current_dir(&dir)
        .output()
        .unwrap();
    let r = std::fs::read_to_string(dir.join("report.json")).unwrap_or_default();
    let stderr = String::from_utf8_lossy(&out.stderr);
    let trace: Vec<&str> = stderr.lines().filter(|l| l.starts_with("VYGORDER|")).collect();
    assert_eq!(out.status.code(), Some(0), "{r}\n{stderr}");
    // the DEF lists components by name: the order is the trace's (checked against the full read)
    let def = std::fs::read_to_string(dir.join("out.def")).unwrap();
    assert_eq!(components(&def), ["drv", "input1", "l1", "output2", "output3"], "{def}");
    assert!(trace.contains(&"VYGORDER|mid|output2|0"), "output2 takes b1's freed id, first: {trace:?}");
    assert_eq!(trace.iter().filter(|l| **l == "VYGORDER|general").count(), 1, "a full read after the first edit: {trace:?}");
}
