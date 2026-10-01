# vyges loom rsz — CLI reference

_Generated from `vyges loom rsz --help` — this page is the tool's own output, verbatim._

```text
vyges loom rsz — electrical repair of a placed design: repeaters along each net's Steiner tree,
drivers resized, where a wire is too long or a capacitance, fanout or transition limit is broken

USAGE:
  vyges loom rsz repair_design <job.json> [-o FILE]
  vyges loom rsz --describe
  vyges loom rsz --help
  vyges loom rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {"cmd": ..., "args": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 estimate_parasitics -placement, repair_design [options]
               an estimate_parasitics step may carry "db": the database as the estimate saw it,
               when cells were moved between it and the repair
  trace        write one line per decision, in the order the repair makes them, to this path
  write_def    write the design as the repair left it, as DEF, to this path
  dcalc_trace  (diagnostic) write the timer's delay-calculation trace of the design as read

REPAIR_DESIGN OPTIONS:
  -max_wire_length L    the longest wire, microns (0: none)
  -slew_margin P        percent taken off every slew limit
  -cap_margin P         percent taken off every capacitance limit
  -verbose              accepted
  refused: -pre_placement / -buffer_gain, -match_cell_footprint, -reroute, -max_utilization

CONSTRAINTS READ FROM SDC:
  create_clock, set_max_transition and set_max_fanout on the design, set_load on nets and ports,
  set_input_transition, set_driving_cell; any other timing-affecting command is refused

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

REPORT:
  status, nets_checked, nets_repaired, inserted_buffers, resized, drivers_skipped, violations
  {slew, capacitance, fanout, length}, and summary — the repair's closing lines, each with its code

EXIT STATUS:
  0  repaired     the design changed: buffers inserted or drivers resized
  0  up_to_date   drivers were checked and none needed a change (nets_checked)
  2  vacuous      no driver was checked. NOT a pass.
  2  error        usage, unreadable input, or an error the repair raises
  3  refused      an input or an option this engine does not model — see `reason`
```
