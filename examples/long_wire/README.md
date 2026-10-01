# long_wire — a worked example

A two-buffer design whose one internal net, `n1`, runs 1.9 mm across a 2 mm row. The wire breaks
the slew limit (the library default, 0.2 ns) and the `-max_wire_length` of 400 µm, so the repair
buffers it.

Every file here is synthetic and written for this example: `cells.lef` (one site, two routing
layers, three buffer sizes), `cells.lib` (the same buffers, linear 3×3 delay tables), and
`long_wire.def` (the placed design).

```sh
cd examples/long_wire
vyges-rsz repair_design job.json -o report.json
```

`expected-report.json` is the report a correct build writes; `cargo test --features cli` runs this
example and compares the report and the repaired design (`expected-repaired.def`) against it.
