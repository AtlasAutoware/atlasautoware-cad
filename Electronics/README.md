# AtlasPower-4S

One board that runs the new car's electronics off the 4S drive LiPo, so the Baseus power
bank and the USB-C PD trigger come off the car entirely. 88 x 68 mm, four M3 holes on a
76 x 56 pattern, mounted on the clipless system by `../cad/power_board_mount.py`.

    XT90 -> 10 A fuse -> TVS -> ideal-diode FET -> 5 mohm shunt -> VIN
    VIN  -> AP64501 -> 10.0 V  Jetson Orin Nano + SICK TiM561
         -> AP64501 ->  7.4 V  INJORA INJS235 servo
         -> AP64501 ->  5.0 V  USB hub, PCA9685, fan
    INA226 across the shunt -> I2C -> Jetson

## Why these numbers

The pack is 12.0 V at a 3.0 V/cell cutoff and 16.8 V full. The Jetson takes 9-20 V and the
TiM561 takes 9-28 V, so both would run straight off the pack, and that is the tempting
wrong answer: the pack rail carries the motor's switching noise, and a hard launch sags it
far enough to brown out the Jetson mid-run.

The compute rail is therefore **10 V, not 12 V**. At 12 V a buck has no headroom left when
the pack is nearly flat and you need a buck-boost, which is a four-switch converter and a
much harder board. At 10 V a plain synchronous buck still has 0.8 V of headroom even at an
11 V sag, which is the single decision that keeps this board simple enough to get right.

Servo peaks are handled by a 470 uF reservoir rather than by sizing the regulator for
stall: a 35 kg-class servo pulls 3-6 A stalled but only for milliseconds, and sizing three
rails for simultaneous peaks would have pushed the board to 97 W and a lot of heat.

## Files

| file | what |
| --- | --- |
| `design.py` | every electrical number, and the checks over them. Run it first. |
| `netlist.py` | parts and nets, built from `design.py` |
| `gen_kicad.py` | writes the KiCad project from `netlist.py` |
| `out/AtlasPower4S/` | the KiCad 9/10 project, schematic PDF, netlist, BOM, ERC report |
| `../cad/power_board_mount.py` | the printed tray, on the clipless foot interface |

Regenerate everything:

    python3 design.py && python3 netlist.py && python3 gen_kicad.py

## What has actually been verified

Everything below was run, not asserted:

- `design.py` passes all its checks: rail dividers land within 0.4 % of target on E96 pairs,
  the inductor is at or above minimum on every rail, saturation current clears 1.5x peak,
  every rail has headroom at an 11 V sag, and the shunt uses 49 % of the INA226's range
  without saturating it.
- The compensation values come from the AP64501 datasheet's own Eq. 17-20, and the method
  reproduces the datasheet's worked example (5 V, 5 A, 45 uF gives R5 = 15.8 k, as printed).
- KiCad parses the generated schematic and its extracted netlist matches `netlist.py`
  **exactly**: 42 nets, no missing, no extra, no membership differences.
- All 21 footprints resolve against the KiCad 10 libraries.
- 204 netlist pins, 204 global labels. BOM totals 84 parts against 84 in the netlist.
- ERC: **zero errors.** 84 warnings are `lib_symbol_issues`, which is `kicad-cli` not
  loading the project-local `sym-lib-table`; they disappear when the project is opened
  normally. 2 are `isolated_pin_label` on `ALERT` and `NC_VESC_5V`, both deliberate.

## What has NOT been verified, and must be before ordering

1. **The inductor land pattern.** KiCad has no SRP1265A footprint. `L_APV_APH1265` is the
   same 12.5 x 12.5 x 6.5 body class and is in there as a stand-in, but its pads have not
   been checked against the Bourns recommended layout. Check that drawing or pick an
   inductor whose footprint ships with KiCad. This is the one dimension on the board not
   traceable to a datasheet.
2. **There is no PCB layout yet.** The project has a schematic, a netlist and a BOM. Board
   outline, placement and routing are still to do, and for a switching supply the layout is
   not a formality: input ceramics hard against VIN/GND, the inductor tight to SW, feedback
   away from the switch node, and a solid ground pour with stitching vias under all three
   exposed pads. The datasheet's Figure 31 shows the intended arrangement.
3. **Thermals.** About 8 W total across the three regulators at full load, in SO-8EP
   packages rated 45 C/W. That needs copper, not hope. Model it once there is a layout.
4. Prices and stock were read from LCSC on 2026-09-07 and move.

## Two deliberate oddities

**J7 pin 2 is a dead net.** The VESC's PPM header carries 5 V, signal and ground. That 5 V
must never meet the servo BEC rail. The usual fix is "remember to pull the red pin from the
Y-lead", which is exactly the kind of instruction that gets forgotten once. Here the pin
physically goes nowhere, so the mistake is not available.

**The INA226 is powered from the Jetson's 3.3 V, not from a board rail.** Telemetry is
therefore only alive when the Jetson is, which is the correct dependency: there is nothing
to report to when it is off, and it keeps the sense chip off the noisy pack rail.

## Sources

- AP64501 datasheet DS41980 Rev. 5-2, Diodes Incorporated, December 2024 — pinout,
  0.8 V reference, 570 kHz, Eq. 6-20, Figure 31 layout guidance.
- LM74700-Q1 datasheet SNOSD17G, Texas Instruments — SOT-23-6 pinout, FET sizing rule.
- INA226 — 81.92 mV shunt full scale, 2.5 uV LSB.
- Jetson Orin Nano Developer Kit carrier board specification SP-11324-001 v1.1 — 9-20 V
  jack, 3.5 A rating.
- SICK TiM561-2050101 datasheet 1071419 — 9-28 V, external slow-blow fuse required.
