// SPDX-License-Identifier: Apache-2.0
//! `set_clock_latency` and `set_propagated_clock` on one clock: the later command wins
//! (`Sdc::setPropagatedClock` removes the clock's network latency; `Sdc::setClockLatency` removes
//! its propagation). The search reads the run's `set_propagated_clock` as coming after the file,
//! so a file with a network latency read AFTER the run propagated the clocks is refused; read
//! before, it is accepted.
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

const DEF: &str = "VERSION 5.8 ;
DIVIDERCHAR \"/\" ;
BUSBITCHARS \"[]\" ;
DESIGN buf1 ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 28000 ) ;
ROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;
COMPONENTS 1 ;
    - drv BUF_X1 + PLACED ( 20000 0 ) N ;
END COMPONENTS
PINS 3 ;
    - clk + NET clk + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 2400 ) N ;
    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;
    - out1 + NET out1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 1400 ) N ;
END PINS
NETS 3 ;
    - clk ( PIN clk ) + USE SIGNAL ;
    - in1 ( PIN in1 ) ( drv A ) + USE SIGNAL ;
    - out1 ( drv Z ) ( PIN out1 ) + USE SIGNAL ;
END NETS
END DESIGN
";

fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire")
}

/// A job with `set_propagated_clock` before or after `read_sdc`: the report.
fn run(name: &str, propagate_first: bool) -> serde_json::Value {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_latency_propagated").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["cells.lef", "cells.lib"] {
        std::fs::copy(example().join(f), dir.join(f)).unwrap();
    }
    std::fs::write(dir.join("d.def"), DEF).unwrap();
    std::fs::write(dir.join("c.sdc"), "create_clock -name clk -period 1 [get_ports clk]\nset_clock_latency 0.2 [get_clocks clk]\n").unwrap();
    let sdc = r#"{ "cmd": "read_sdc", "args": ["c.sdc"] }"#;
    let prop = r#"{ "cmd": "set_propagated_clock", "args": ["[all_clocks]"] }"#;
    let (a, b) = if propagate_first { (prop, sdc) } else { (sdc, prop) };
    let job = format!(
        r#"{{ "steps": [
        {{ "cmd": "read_lef", "args": ["cells.lef"] }},
        {{ "cmd": "read_def", "args": ["d.def"] }},
        {{ "cmd": "read_liberty", "args": ["cells.lib"] }},
        {a}, {b},
        {{ "cmd": "set_wire_rc", "args": ["-resistance", "0.0008", "-capacitance", "0.0002"] }},
        {{ "cmd": "estimate_parasitics", "args": ["-placement"] }},
        {{ "cmd": "repair_design", "args": [] }} ] }}"#
    );
    std::fs::write(dir.join("job.json"), job).unwrap();
    Command::new(env!("CARGO_BIN_EXE_vyges-rsz")).args(["repair_design", "job.json", "-o", "report.json"]).current_dir(&dir).output().unwrap();
    std::fs::read_to_string(dir.join("report.json")).ok().and_then(|t| serde_json::from_str(&t).ok()).unwrap_or(serde_json::Value::Null)
}

#[test]
fn a_latency_read_after_the_run_propagated_the_clocks_is_refused() {
    let r = run("propagate_first", true);
    assert_eq!(r["status"], "refused", "{r}");
    assert!(r["reason"].as_str().unwrap_or_default().contains("set_clock_latency read after set_propagated_clock"), "{r}");
    let r = run("propagate_after", false);
    assert_ne!(r["status"], "refused", "{r}");
}
