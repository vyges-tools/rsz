# vyges physical rsz — CLI reference

_Generated from `vyges physical rsz --help` — this page is the tool's own output, verbatim._

```text
vyges physical rsz — electrical repair of a placed design: repeaters along each net's Steiner
tree, drivers resized, where a wire is too long or a capacitance, fanout or transition limit is
broken

USAGE:
  vyges physical rsz repair_design <job.json> [-o FILE]
  vyges physical rsz buffer_ports <job.json> [-o FILE]  (the same job runner; a job may hold either)
  vyges physical rsz --describe
  vyges physical rsz --help
  vyges physical rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {"cmd": ..., "args": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 set_routing_layers -signal LO-HI [-clock LO-HI], global_route,
                 estimate_parasitics -placement | -global_routing, set_propagated_clock,
                 buffer_ports [options],
                 repair_design [options], repair_timing [options],
                 clock_buffers CELL..., repair_clock_nets [-max_wire_length L]
               clock_buffers states the resizer's clock buffer list as clock_tree_synthesis
               left it in the session (its -buf_list); repair_clock_nets repairs long wires in
               every clock network with it (L microns; without it, the length past which
               buffering pays, RSZ-0058)
               an estimate_parasitics step may carry "db": the database as the estimate saw it,
               when cells were moved between it and the repair
               a buffer_ports step may carry "write_def": the design as it left it, as DEF
               an estimate_parasitics or buffer_ports step may carry "sdc": the constraints in
               force when it ran, when a port's set_load comes after it
  trace        write one line per decision, in the order the repair makes them, to this path
  write_def    write the design as the repair left it, as DEF, to this path
  dcalc_trace  (diagnostic) write the timer's delay-calculation trace of the design as read

REPAIR_DESIGN OPTIONS:
  -max_wire_length L    the longest wire, microns (0: none)
  -slew_margin P        percent taken off every slew limit
  -cap_margin P         percent taken off every capacitance limit
  -verbose              accepted
  refused: -pre_placement / -buffer_gain, -match_cell_footprint, -reroute, -max_utilization

BUFFER_PORTS OPTIONS:
  -inputs / -outputs    which side (neither: both) — a buffer after each input port, before
                        each output port, unless its net is dont-touch, special or pinless, an
                        input is a clock source, an input's loads are all buffers or one is
                        dont-touch, or an output's driver is tristate or dont-touch
  -buffer_cell C        the buffer to use (default: the weakest buffer the repair would pick)
  -verbose              each port's decision in the report's lines
  refused: -max_utilization, a hierarchical design

REPAIR_TIMING:
  -setup: every move of the default sequence (UnbufferMove, SizeUpMove, SwapPinsMove,
  BufferMove, CloneMove, SplitLoadMove; SizeUpMatchMove) in the LEGACY phase and LAST_GASP —
  the move sequence (RSZ-0100), RSZ-0094 / RSZ-0099 (and RSZ-0221) or RSZ-0098, every progress
  row, the summary (RSZ-0051, RSZ-0062) in the report's repair_timing[].lines, and the design it
  leaves (write_def). A job's timing_trace file gets the pass-by-pass decisions.
  -hold (alone): the hold buffer, RSZ-0046 or RSZ-0033, every progress row, RSZ-0064 / RSZ-0066,
  RSZ-0132, RSZ-0032 and the buffers it inserts; -max_utilization or -max_buffer_percent reached
  ends it with RSZ-0050 / RSZ-0060 (status error).
  status repaired (the design changed), unrepaired (violations, nothing kept), up_to_date or
  error. Clocks (one or several, real or virtual), each ideal or propagated, with their I/O
  delays; latches (time borrowing); VT libraries (VtSwapMove); pins tied to supply nets.
  -setup and -hold together (or neither): the setup repair, then refused. Phases LEGACY, WNS,
  TNS, ENDPOINT_FANIN, STARTPOINT_FANOUT, LAST_GASP, CRIT_VT_SWAP and REROUTE are modelled (an
  empty list or an unknown first phase is the command's own error), on placement or global-route
  parasitics (the live incremental router; a resistance-aware re-route reads the timer's net
  slacks mid-update); the experimental policies
  GLOBAL_SIZING, MT1 and MEASURED_VT_SWAP each alone (tunables from RSZ_* environment variables,
  a setenv step), and LEGACY_MT as the first phase. -recover_power runs in full (RecoverPower: every row, the trace, the design; -match_cell_footprint
  and global-route parasitics refused). Refused before the lines: an experimental policy with
  other phases, LEGACY_MT after another phase or with SizeDownFanoutMove, parasitics other than -placement and -global_routing, setup clock
  uncertainty, clock latency / transition, derates, false and multicycle paths, clock groups,
  path delays other than the forms the timer models (see --describe); refused during the repair:
  -setup with -max_utilization, more than one repair per pass.

CONSTRAINTS READ FROM SDC:
  create_clock (-waveform: a rise then a fall within one period), set_input_delay /
  set_output_delay (-clock, one delay per port and clock; -add_delay adds a clock or merges),
  set_max_transition and set_max_fanout on the design, set_max_capacitance on the design, ports
  and pins, set_load on nets and ports, set_input_transition, set_driving_cell, set_max_delay /
  set_min_delay (repair_timing only, the forms above); any other timing-affecting command is
  refused

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

REPORT:
  buffer_ports (per step: inserted_inputs, inserted_outputs, ports_checked, lines — each with its
  code and severity), status, nets_checked, nets_repaired, inserted_buffers, resized, drivers_skipped, violations
  {slew, capacitance, fanout, length}, summary — the repair's closing lines, each with its code —
  warnings (RSZ-0065: -max_wire_length shorter than the length at which a buffer pays for itself),
  and max_wire_lengths (that length per buffer and scene, meters, as the check computed it)

EXIT STATUS:
  0  repaired     the design changed: buffers inserted or drivers resized
  0  up_to_date   drivers were checked and none needed a change (nets_checked)
  2  vacuous      no driver was checked. NOT a pass.
  2  error        usage, unreadable input, or an error the repair raises
  3  refused      an input or an option this engine does not model — see `reason`
```
