# Camera mast for the Orbbec Gemini 335 on the clipless mounting system

Replaces the wooden top plate: the camera stands on its own mast at the front of the car.

Files in `out/`: `camera_mast.step/.stl` (print one), `camera_mast_head.step/.stl` (print
one, the pitching saddle), `clipless_foot.step/.stl` (print two, from `omni20_mount.py`),
`camera_mast_assembly.step` (mast, head, feet, two clipless pieces in a stub of baseplate,
grey envelope of the camera), `camera_mast.png` (mast and head, shaded, two views),
`camera_mast_head.png`, `camera_mast_assembly.png`, `camera_mast_lines.png` and
`camera_mast_assembly_lines.png` (hidden-line). Source: `camera_mast_mount.py`, importing
`clipless.py`, `render.py`, helpers from `jetson_orin_nano_mount.py` and the slide channel
from `rplidar_c1_mount.py`.

## Component dimensions and where they come from

Orbbec Gemini 335 (model G40155-170). Product page
https://www.orbbec.com/products/stereo-vision-camera/gemini-335/ and store page
https://store.orbbec.com/products/gemini-335. Datasheet: "Gemini 330 Series Datasheet
v1.1", linked from https://www.orbbec.com/docs/g330-orbbec-gemini-330-series-datasheet/,
read at https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/06/Orbbec-Gemini-330-Series-Datasheet-v1.1-2.pdf

| Item | Value | Source |
| --- | --- | --- |
| Body | 90 x 25 x 30 mm (width x height x depth); front-view drawing gives 89.46 +-0.30 wide, side view 25.00 +-0.30 tall and 30.00 +-0.30 deep | datasheet section 2 table and 3.2.1 drawings (pages 16-17) |
| Mass | 97 g | section 2 table |
| Tripod thread | 1 x 1/4-20 UNC in the underside, "max insertion depth 5 mm", max torque 4.0 N.m; centred across the width | section 2 table, bottom-view drawing |
| Second fixing | 2 x M3, "M3 thread mounting points, max insertion depth 3 mm", max torque 0.4 N.m, 45.00 +-0.30 mm apart, in the REAR face between the heatsink fins | section 2 table, rear-view drawing |
| Data/power | USB Type-C on one end face, with two M2 points 15.00 +-0.10 apart for a locking cable | side-view drawing |
| Depth FOV | H 90 x V 65 degrees (1280 x 800) | data streams table |

The task text said to check for two M4 threads: on the Gemini 335 they are M3. M4 is the
335L (124 x 29 x 27 mm, 133 g), a different camera.

Estimates, marked as such in the script: the 1/4-20 socket's fore-aft position
(`CAM_TRIPOD_FROM_REAR` = 11 mm, from the bottom-view raster; it is not dimensioned), the
rear M3 holes and the lens axis at mid height (`CAM_M3_Z`, `CAM_LENS_Z` = 12.5), and
which end carries the USB-C (`CAM_USB_SIDE` = -y). The head's 1/4-20 slot has 8 mm of
fore-aft slack and the drum slot 15 degrees of extra arc to absorb the socket-position
estimate; the M3 tab sets the camera's fore-aft position regardless.

## Geometry assumed

Origin at the centre of the foot pair, +x forward, +z up. The lidar is taken to sit
`LIDAR_TO_CAM` = 120 mm behind the lens on the car's centreline, i.e. at x = -101, which
is roughly two clipless pitches behind the mast's foot row (the plinth of
`rplidar_c1_mount.py`, feet 49 mm apart along the car, lands there). Keep-out: a
cylinder of radius 60 mm about that point. The mast's rearmost point is the floor bar
at x = -20.25, so the margin is 20.75 mm; the head and camera are further forward still.

Being outside the keep-out does not remove the legs from the scan: a 2D lidar sees
everything crossing its plane at any range. The scan plane (44.93 mm above the board top
with the default plinth) crosses the four legs at |y| = 31 mm, x = -14.5 (rear pair) and
+14.5 (front pair). From the lidar they are at +-15.0 degrees (120 mm range, 4.3 degrees
wide) and +-19.7 degrees (92 mm range, 5.6 degrees wide). Mask two sectors of about 12
to 24 degrees either side of straight ahead in the lidar driver (angle_min/max or a
range filter below 0.15 m), or put the lidar plinth further back. The camera body itself
is well above the plane (underside at 137.5 mm, top at 162.5 mm).

## What the mast does

- Feet: two clipless feet ACROSS the car, `PITCH_Y` (env var, default 47.0) apart. Along
  the car would put the rear foot's floor 75 mm behind the lens, inside the keep-out. The
  floor bar is 40.5 (x) x 86 (y) x 7 mm and carries the same dovetail slide channel as the
  lidar plinth (`rplidar_c1_mount.slide_channel`), running along y and open on the +y
  side: feet into their pockets first, the mast slides on sideways over both flanges to
  the end stop, the two spring tongues snap up behind the +y flange, the 45 degree lips
  stop it lifting off. 1.75 mm of play between stop and bump plus the +-2 mm the pegs
  float. Set `PITCH_Y` to the measured pitch of the hole pair used.
- Legs: four 6 x 6 mm legs from the floor corners (x +-17.25, y +-38) converging to
  (x +-10, y +-19.5) at the drum yokes 99 mm up: a pyramid, triangulated in both planes.
  Leg half-angles 10.5 degrees laterally and 4.2 degrees fore-aft. Estimated tip
  deflection at 5 g on the 97 g camera plus 15 g head (PLA, E = 3 GPa, legs as a pinned
  truss): about 0.03 mm lateral and 0.2 mm fore-aft, first mode well above 50 Hz; leg
  buckling load is over 300 N against 6 N of load per leg. Legs are 4.3 to 5.6 degrees
  wide as seen by the lidar.
- Drum: a horizontal half-cylinder shell, R 20 / R 16, 44 mm wide, axis along y at
  z = 112.4, closed at the ends by the yokes and open underneath. Its outer surface is a
  cylindrical rack: 5 degree teeth (1.75 mm pitch, 0.8 mm deep) over +-65 degrees. An arc
  slot 7.5 mm wide from -15 to +35 degrees (from vertical towards the front) lets the
  1/4-20 bolt through the shell.
- Head (saddle): 33 x 44 mm pad with the matching toothed concave underside (0.15 mm
  radial clearance), 4 mm thick at the crest, flat top for the camera, and a 52 x 18 x 3
  rear tab with two 3.4 mm holes on the camera's 45 mm M3 pair. The camera bolts to the
  pad with its own 1/4-20 (a 1/4-20 x 1/2 in socket screw: 4 mm shell + 4 mm pad leaves
  4.7 mm in the camera, under the 5 mm limit; use a thin washer if the head marks the
  shell) from inside the drum, reached from below between the yokes with a hex key. The
  bolt clamps camera, pad and drum together, the teeth carry the pitch load, so the pitch
  cannot creep. Two M3 x 6 screws (3 mm engagement, the limit) through the tab into the
  rear of the camera stop it yawing on the single bolt and fix its fore-aft position.
- Pitch: loosen the bolt half a turn, lift the head one tooth, re-seat, tighten. 0, -5,
  -10, -15, -20 degrees nose-down all clear the mast (checked). The lens axis is
  `CAM_HEIGHT` (env var, default 150) above the board top at 0 degrees and 141.3 mm at
  -20; it moves forward 12 mm over the range. The USB-C end of the camera hangs 22 mm
  outboard of the pad on the -y side with nothing near it; the tab is 26 mm half-wide,
  the camera 44.7.
- Mass: mast 33.9 cm3, head 11.7 cm3; 42 + 15 = 57 g in solid PLA, 34 g at the 60 %
  figure the other notes use, plus two feet. Under the 60 g target either way.

## Verified in CadQuery (`python3 camera_mast_mount.py`)

```
mast 40.5 x 86.2 x 132.4 mm, 33.9 cm3 (42 g PLA solid, ~25 g at 60% effective); head 11.7 cm3 (15 g solid); feet pitch 47.0 mm across, CAM_HEIGHT 150.0
  solids_mast: 1
  solids_head: 1
  mast_x_head: 0.0
  mast_x_head_pitched: {5: 0.0, 10: 0.0, 15: 0.0, 20: 0.0}
  mast_x_envelope_pitched: {0: 0.0, 5: 0.0, 10: 0.0, 15: 0.0, 20: 0.0}
  head_x_envelope: 0.0
  mast_x_feet: 0.0
  feet_x_clipless: 0.0
  mast_x_plate: 0.0
  feet_x_envelope: 0.0
  lens_above_board_top_at_0: 150.0
  lens_above_board_top_at_-20: 141.3
  lens_x_from_feet_row: 19.0
  lidar_centre_x_from_feet_row: -101.0
  mast_rearmost_x: -20.25
  keepout_margin: 20.75
  legs_in_scan_plane_(x,y,range,angle,width_deg)_+y_side: [(-14.5, 31.0, 91.9, 19.7, 5.6), (14.5, 31.0, 119.6, 15.0, 4.3)]
  bolt_engagement_in_camera: 4.7
```

Intersection volumes in mm3: mast against the head at 0 and at each 5 degree pitch step to
-20, mast against the camera envelope (body with the rounded ends, USB-C plug stub) at
each step, head against the envelope, mast against the feet, feet against the clipless
pieces, mast against the baseplate stub, feet against the envelope.

## Printing

Mast: floor down, standing, no supports. The legs lean 4 to 11 degrees, the channel
lips are 45 degree flanks, and the drum prints as a 32 mm horizontal bore (the crown
sags a little on the inside, where only the bolt head sits) with the teeth as horizontal
ridges around its top; the two 4 mm wide flats at the shell's lower ends bridge 34 mm
between the yokes. 0.2 mm layers, 4 walls, 40 % infill; the legs are essentially all
wall.

Head: on its rear tab (the tab's back face on the bed), so the pad stands vertical and
the teeth print as vertical serrations at full resolution; the 1/4-20 slot is a 6.8 mm
bridge. No supports. PETG for both if the car lives outdoors in summer: the drum and
teeth are under constant bolt preload.

Hardware: 1 x 1/4-20 x 1/2 in socket head screw (do not use 5/8 in: 6.9 mm would exceed
the camera's 5 mm limit), 2 x M3 x 6 (3 mm engagement, the limit; M3 x 5 with the 3 mm
tab also works), two clipless feet.
