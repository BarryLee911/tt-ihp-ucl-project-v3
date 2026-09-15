# Interface tests

Run `make` in this directory with Icarus and the dependencies in `requirements.txt`.
The official workflow also runs these pin-only checks against the gate netlist.

Real 80 MHz division is checked at levels 0, 1, and 5 with the default 80,000-clock handoff. Area uses a history queue; peak uses an independent ranked candidate model. No internal RTL signals are read or forced. Functional simulation does not establish physical timing.

Reset is held for four edges. All output pins are checked from the first power-on reset edge; any X/Z fails. The manual `gate-validation` workflow can reuse a GDS only when RTL, clock configuration, and tiles match.
