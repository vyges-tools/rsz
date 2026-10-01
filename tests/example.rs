// SPDX-License-Identifier: Apache-2.0
//! `examples/long_wire`, run through the binary: the report, the repaired design and the decision
//! trace must be the pinned ones. The three goldens were checked against the reference
//! implementation of the same command on the same files when they were pinned — the trace
//! byte-identical, the components and nets identical, the same closing summary lines.
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

const INPUTS: [&str; 4] = ["cells.lef", "cells.lib", "long_wire.def", "job.json"];

fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire")
}

fn read(p: &Path) -> String {
    std::fs::read_to_string(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

#[test]
fn the_long_wire_example_repairs_as_pinned() {
    // A fresh copy: the job writes its outputs next to itself.
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("long_wire");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    for f in INPUTS {
        std::fs::copy(example().join(f), dir.join(f)).unwrap();
    }
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz"))
        .args(["repair_design", "job.json", "-o", "report.json"])
        .current_dir(&dir)
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(0), "{}", String::from_utf8_lossy(&out.stderr));
    for (got, want) in [("report.json", "expected-report.json"), ("repaired.def", "expected-repaired.def"), ("repair.trace", "expected-repair.trace")] {
        assert_eq!(read(&dir.join(got)), read(&example().join(want)), "{got} differs from {want}");
    }
}

// The exit status is the claim: a job whose repair checks nothing is `vacuous`, exit 2 — not a
// pass. The same design with its nets and its pins removed (a DEF pin's `+ NET` makes its net on
// its own) leaves both drivers unconnected: each is passed over, none is checked.
#[test]
fn a_repair_that_checks_nothing_exits_two() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("long_wire_empty");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["cells.lef", "cells.lib", "job.json"] {
        std::fs::copy(example().join(f), dir.join(f)).unwrap();
    }
    let def = read(&example().join("long_wire.def"));
    let cut = |s: &str, from: &str, to: &str| -> String {
        let (a, b) = (s.find(from).unwrap(), s.find(to).unwrap() + to.len());
        format!("{}{}", &s[..a], &s[b..])
    };
    let def = cut(&cut(&def, "PINS 2 ;", "END PINS\n"), "NETS 3 ;", "END NETS\n");
    std::fs::write(dir.join("long_wire.def"), def).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz"))
        .args(["repair_design", "job.json", "-o", "report.json"])
        .current_dir(&dir)
        .output()
        .unwrap();
    let report = read(&dir.join("report.json"));
    assert_eq!(out.status.code(), Some(2), "{report}");
    assert!(report.contains(r#""status":"vacuous""#) && report.contains(r#""nets_checked":0"#), "{report}");
}
