# 9751 Gemini — 2026 REBUILT robot — CAD prompt pack

Everything needed to hand to a text-to-CAD model (Zoo / Text-to-CAD, AdamCAD, CADGPT, or any LLM that
writes CadQuery / OpenSCAD / build123d) to get this robot into CAD.

Two things up front, because they decide how you should use this pack:

1. **No text-to-CAD tool will generate this robot from a single prompt.** They produce one solid body
   per prompt. The robot is ~10 subassemblies and ~80 parts, and most of those are COTS (REV
   MAXSwerve, NEOs, SPARK MAX/Flex, Kraken X60, MAXTube). This pack is therefore **one prompt per
   part**, plus a **master prompt** that places everything on a shared datum.
2. **The code does not contain most of the geometry.** It contains what the control system needs: the
   four wheel positions, wheel size, gearing, camera transforms, arm length, ball size, field size.
   It does *not* contain frame tube sizes, shooter plate outlines, or where the intake pivot sits.
   Section 6 is a 15-group tape-measure sheet that closes that gap — do it first, or the CAD will be
   fiction.

> **Want a whole-robot packaging model with less work?** Use
> [`CAD_MECHANISM_PROMPTS.md`](CAD_MECHANISM_PROMPTS.md) instead: eight self-contained prompts, one per
> mechanism, each placed in the robot by a stated transform. Use this document when you are ready to
> cut parts.

Files in this pack:

| File | What it is |
|---|---|
| `docs/ROBOT_CAD_PROMPT.md` | this document — the per-part prompts |
| `docs/CAD_MECHANISM_PROMPTS.md` | per-mechanism prompts for a packaging model you assemble yourself |
| `docs/cad/robot-datums.svg` / `.png` | datum drawing: wheel positions, frame envelope, cameras, rule limits |
| `docs/cad/robot-parameters.json` | every number used below, with provenance and confidence |
| `docs/cad/measurements.csv` | the tape-measure sheet, pre-filled where the code already answers |
| `docs/cad/source-map.csv` | which file/line every code-derived number came from |
| `docs/cad/generate_datums.py` | regenerates the drawing from `robot-parameters.json` (needs matplotlib) |

---

## 0. Ground rules that make text-to-CAD output usable

These matter more than the wording of any individual prompt.

| Rule | Why |
|---|---|
| **One body per prompt** | Every current generator makes a single solid. Ask for an assembly and you get fused nonsense. |
| **Millimetres, always** | Say `mm` on every number. Mixed units is the #1 silent failure. |
| **Name features in CAD vocabulary** | `counterbore`, `boss`, `pocket`, `fillet R3`, `M4 clearance (Ø4.5)`, `#10-32 clearance (Ø5.0)`. "hole for a bolt" gets you a guess. |
| **Anchor every position** | "two Ø5.0 holes, 50 mm from the left edge, 100 mm apart, on the centreline" — never "two holes on the left". |
| **One plate, one interface per prompt** | Split a part the moment it has more than ~10 features. |
| **Export STEP, not STL** | STEP is B-rep: measurable, machinable, re-featurable in Onshape/Fusion/SolidWorks. STL is a mesh and a dead end for manufacturing. |
| **Expect 2–3 fixes after import** | Budget for it instead of chasing a perfect first generation. |
| **Do not generate COTS** | Download vendor STEP. Prompting a MAXSwerve module wastes time and gets the dimensions wrong. |

**Download, don't generate:** REV-21-3005 (3in MAXSwerve), REV NEO / NEO 550 / NEO Vortex, SPARK MAX
/ SPARK Flex, CTRE Kraken X60, navX MXP, roboRIO, PDH, radio, MAXTube profiles, and the 2026 FUEL
ball (Ø150.11, 0.203 kg). Every vendor publishes STEP.

---

## 1. Context block — paste this in front of every prompt

```
CONTEXT (fixed for all parts)
Robot: FRC team 9751, 2026 REBUILT. 4-wheel swerve, ~65 kg.
Units: millimetres, angles in degrees.
Robot frame: +X forward (the intake side), +Y to the robot's left, +Z up.
Origin: centre of rotation, on the floor plane (z = 0) = midpoint of the four wheel centres.
Drivetrain: 4 x REV 3in MAXSwerve. Wheel centres at
  FL (+266.7, +336.55),  FR (+266.7, -336.55),
  RL (-266.7, +336.55),  RR (-266.7, -336.55).
Wheelbase 533.4 (X), track 673.1 (Y). Wheel Ø76.2 x 22.2 wide, axis along Y, contact patch on z = 0.
Scoring element: foam ball, Ø150.11, 0.203 kg.
Output: single solid body, millimetres, STEP. No fillets under R1. No organic shapes.
```

`+X = forward = intake` is deliberate: it matches WPILib, PathPlanner and the PhotonVision
transforms. Do not also flip into a "robot front = −X" CAD convention on top. The shooter sits on −X;
that sign is where teams lose parts.

---

## 2. Datums and coordinate system

```
ORIGIN      centre of rotation, floor plane, z = 0
X AXIS      +X toward the intake; the shooter fires along -X
Y AXIS      +Y = robot's left (right-handed with +Z up)
Z AXIS      +Z up from the carpet
WHEELS      FL (+266.7, +336.55)   FR (+266.7, -336.55)
            RL (-266.7, +336.55)   RR (-266.7, -336.55)
FRAME       smallest rectangle that hugs the module bodies:
            633.9 (X) x 773.6 (Y); with 12 mm corner chamfers the perimeter is 109.7 in
CAMERA A    (-311.15, -273.05, 434.98), yaw 180 deg   rear-facing, over the muzzle
CAMERA B    (-254.00, -330.20, 411.23), yaw  -90 deg   faces the robot's RIGHT
```

Keep `docs/cad/robot-datums.png` open next to any prompt. It is the drawing the prompts refer to.

---

## 3. Master prompt — the assembly skeleton

Run this once in a tool that holds a component tree (Zoo Design Studio, Onshape, Fusion, or an LLM
writing CadQuery). It creates **no detail**: four module placeholders, a frame box, a bumper box, two
camera points, and two reserved envelopes. Detail comes from section 4, one prompt at a time.

```
Create a 2026 FRC swerve robot layout, millimetres. Origin at the centre of rotation on the floor
plane (z = 0). +X forward, +Y left, +Z up. Do not add detail, fillets or fasteners.

1. Four placeholder blocks, 100.5 x 100.5 x 171.5 tall, centred on
   (+266.7, +336.55), (+266.7, -336.55), (-266.7, +336.55), (-266.7, -336.55).
   Each sits on the floor: top at z = 171.5. Add one Ø76.2 x 22.2 wide cylinder in each block,
   axis along Y, centre at z = 38.1, tread touching z = 0. Label FL, FR, RL, RR.

2. Frame box: 633.9 (X) x 773.6 (Y) x 300 tall, bottom at z = 32, 12 mm 45 deg chamfers on all four
   vertical corners, 3 mm wall, open top and bottom. Label FRAME. This 300 is a placeholder.

3. Bumper box, separate body: outer footprint 762.9 (X) x 902.6 (Y), from z = 69.85 to z = 139.7,
   wall thickness 64.5. Label BUMPERS.

4. Two camera blocks, 36 x 36 x 10, lens axis through the centre of the 36 x 36 face:
   CAM_A at (-311.15, -273.05, 435.0), lens axis pointing toward -X (yaw 180 deg).
   CAM_B at (-254.00, -330.20, 411.2), lens axis pointing toward -Y (yaw -90 deg).

5. Two empty reserves, do not model them, just keep the boxes:
   SHOOTER  x -467 .. -317,  y -140 .. +140,  z 38 .. 450
   INTAKE   x +317 .. +617,  y -150 .. +150,  z 0 .. 300

Report the overall bounding box and each body's mass at 1.0 g/cm3 so I can check the scale.
```

Expect the result to be roughly right and to disagree with the vendor module STEP by a few mm. That
is fine — its job is to lock the layout; you then replace each placeholder with `REV-21-3005.step`.

---

## 4. Part prompts

Each prompt is standalone: paste the context block (section 1), then the prompt. Replace every
`«MEASURE»` from `measurements.csv` before generating — a placeholder left in a prompt gets a
confident, wrong number.

---

### 4.1 Frame corner gusset

Eight of these (top and bottom at each corner). Generate one, mirror the rest.

```
Generate one flat aluminium corner gusset, 3.175 mm thick 6061-T6, millimetres.

Outline: right triangle, right angle at the plate's origin, leg along +X = 120, leg along -Y = 120,
R6 fillets at the two acute corners. Envelope 120 x 120.

Features (through holes unless stated):
- 4 x #10-32 clearance holes Ø5.0 on the +X leg at (15, -8), (30, -8), (95, -8), (110, -8).
- 4 x #10-32 clearance holes Ø5.0 on the -Y leg at (8, -15), (8, -30), (8, -95), (8, -110).
- 1 triangular pocket, 1.587 deep, inset 12 from all edges, R8 internal corners.
- 0.5 mm chamfer on both faces of the outline.

Constraint: every hole centre is exactly 16 mm from the outer edge of its own leg, so the plate sits
flush in 1 x 2 in MAXTube on the standard MAX grid. No other features.
```

---

### 4.2 Bumper segment

Four segments, mitred corners. This prompt is **one long side**; change two numbers for the short
sides.

```
Generate one FRC bumper segment as a fit-check solid, millimetres, no fasteners.
Plate-local frame for the cross-section: Y = outward from the robot, Z = up.
Inner face at y = 0, outer face at y = 64.5. Bottom at z = 0.

Three fused slabs:
- Backing plate: 6.35 thick in Y (y 0..6.35), 976.8 long in X, height H_BACK.
- Padding: 57.15 thick in Y (y 6.35..63.5), 976.8 long in X, height H_PAD, centred vertically on
  the backing plate.
- Cover: 1.0 thick in Y over the outer face (y 63.5..64.5), wrapping 25 mm onto the top and bottom
  faces.
Set H_BACK = «MEASURE_bumper_height» and H_PAD = «MEASURE_padding_height».
Mitre both ends at 45 deg so four segments form a rectangle. No fillets.
Label faces INNER, TOP, OUTER.
```

Rule floor for those two heights (2026): the padding must fill at least the 69.85–139.7 mm band above
the floor, must be ≥ 57.15 deep, and the whole bumper must not extend more than 101.6 from the frame
perimeter. Confirm against the current manual revision — bumpers are the one assembly where the rules
beat our numbers.

---

### 4.3 Shooter side plate (the opposed flywheel pair)

The part the code constrains hardest, and the one worth generating properly: the two flywheel axles
must sit exactly one ball-plus-compression apart.

```
Generate one 3.175 mm aluminium side plate for a fixed dual-flywheel FRC shooter, millimetres.
Plate frame: origin at the LOWER flywheel axle centre, X to the muzzle, Y = up the plate.

Two axle bores, both in the plate:
- Lower axle at (0, 0).
- Upper axle at (0, PITCH), where PITCH = 101.6 + (150.11 - 2 * COMPRESSION) = 213.6
  for COMPRESSION = 19.05.
- Both: Ø12.75 through bore (to be broached 1/2 in hex).

Outline: rounded rectangle 300 (X) x (PITCH + 120) tall, corner radius 12, centred on the midpoint
between the axles.

- Ball passage: through slot on the -X side, 162.1 wide (Y), 55 tall, R12 ends, centred on the
  midpoint between the axles, open to the -X edge.
- 4 x Ø5.0 holes (#10-32), 12 mm in from each edge.
- 2 x Ø12.0 lightening holes, 40 below the lower axle and 40 above the upper axle.
- R3 fillets on external corners only.

The -X edge faces the hub: the ball exits travelling -X and slightly up.
No hood, no turret bearing, no pivoting feature - this robot has none of those.
```

Why that shape: the code runs a fixed opposed pair — front 1500 rpm, back 2000 rpm, 1:1 gearing, no
hood, no turret, and aiming is done by rotating the whole drivetrain (`DriveSubsystem.getAngleToHub()`
returns the bearing to the hub **minus 180°**). A CAD model with a hood or turret is not this robot.

`COMPRESSION` is the one number nobody can read from code: it is a tuning choice between roughly
12.7 and 25.4 mm, and it is what sets exit speed. Set it, test on the field, update this prompt.

---

### 4.4 Flywheel

```
Generate one FRC flywheel, millimetres, single body, 6061.
- Ø101.6 outer, 25.4 wide.
- Ø12.75 through bore (to be broached 1/2 in hex).
- 6 x Ø6.5 lightening holes on a Ø70 bolt circle, evenly spaced; keep the outer rim at least 10 thick
  radially.
- 4 x Ø5.0 holes on a Ø45 circle, 90 deg apart, for the hub (tapped #10-32).
- 1.5 mm chamfer on both outer rims, 0.5 on the bore.
Report mass. Target ~0.65 kg.
```

The code's flywheel inertia is `kShooterFlywheelMoiKgMetersSquared = 0.0025` and is explicitly flagged
as an estimate to be replaced by SysId. A 101.6 x 25.4 aluminium disc is ≈0.65 kg → J ≈ 0.00084
kg·m²; two of them plus hubs lands in the right decade. **Take the real inertia from CAD mass
properties and put it in `Constants.java`** — that is the single most valuable thing this CAD work
buys the simulation.

---

### 4.5 Feeder drum

```
Generate one FRC feeder drum, millimetres, single body.
- Ø50.8 x 150 long.
- Ø12.75 through bore (1/2 in hex).
- One raised grip band in the middle 50 mm, 3.0 tall, R12 edges (smooth - do not model knurl teeth).
- Two grooves 4.0 wide x 3.0 deep, 10 mm in from each end.
- One 6.0 wide flat, 40 long, at the shaft end, for a collar.
Report mass; target ~0.74 kg in aluminium.
```

Drum diameter is not in the code — the code only requires 4500 rpm through a 1:1 Kraken X60. Ø50.8 is
the common FUEL feeder size; measure yours.

---

### 4.6 Intake deploy arm

The code pins the *motion*, not the shape: 300 mm long, 3 kg, 0 → 100° (105° hard limit), 212.4:1
total reduction, NEO Vortex on SPARK Flex.

```
Generate one FRC intake deploy arm, millimetres, single body, aluminium.
Pivot at the origin. Arm extends along +X when stowed (0 deg = horizontal, pointing forward).
Rotation axis = Y.

- Pivot boss: Ø38 outer, Ø13 through bore (1/2 in hex shaft), 8 thick, centred on the origin.
- Arm: 2 x 1 in MAXTube, modelled as a 280 (X) x 50.8 (Y) x 25.4 (Z) hollow box, 1.587 wall,
  centreline from (20, 0) to (300, 0).
- Roller clevis at the far end: two 6 mm plates 90 apart in Y, each with a Ø12.75 bore centred at
  (300, 0).
- Pivot mount: a second 6 mm plate parallel to the pivot boss at (-80, 0), same Ø13 bore, joined to
  the arm by a 60 x 50.8 web.
- 2 x Ø8.5 holes on the 1 in face at x = 60 and x = 240, on the centreline.
- R2 fillets at the pivot-to-arm junction only.

Pivot-to-roller-centre must be exactly 300.0. Do not model the roller, motor or gearbox.
```

That 300.0 and the 3 kg arm mass are what the simulation's arm model uses. Change them and the
MAXMotion tuning stops matching.

---

### 4.7 Intake roller

```
Generate one FRC intake roller, millimetres, single body.
- Core Ø25.4 x «MEASURE_intake_width» long (nominal 500), 6061.
- 8 silicone paddle strips, 25 wide, 6 tall, full length, evenly spaced, R2 on the leading edge.
- Ø12.75 through bore.
- 2 x Ø6.5 cross holes 30 mm from each end, 90 deg apart, for roll pins.
Report mass; target under 1.2 kg.
```

Code: 5600 rpm on a Kraken X60, 1:1 → about 7.4 m/s surface speed at Ø25.4. That is why the roller
needs a compliant paddle rather than a hard tube.

---

### 4.8 Conveyor (indexer)

```
Generate one FRC conveyor frame, millimetres, single body.
- Two side plates, 6.35 thick, 420 long (X) x 120 tall (Z), outer faces at y = +/-95.
- Per plate: 3 x Ø12.75 bores at (0, 20), (210, 55), (420, 20) in the plate's XZ plane, plus one
  Ø5.0 tensioner slot 20 long at the mid roller.
- Cross members: two 20 x 20 x 2 square tubes at x = 40 and x = 380.
- 3 mm polycarbonate floor 420 x 190 between the plates, front edge (x = 0) flared down 15 deg.
R2 fillets. Do not model belts or motor.
```

Motor: NEO on SPARK MAX, CAN 19, 4500 rpm, 1:1 — so the only ratio in that subsystem is the drum.

---

### 4.9 Camera A bracket (rear-facing, high on the shooter)

Both camera transforms are *measured from the robot centre in code* and are used identically by the
real robot and the PhotonVision simulation. Treat them as hard requirements.

```
Generate one folded aluminium bracket, 1.587 mm 5052, millimetres, single body, unfolded-rectangular.
The camera board is 36 x 36 x 10 with 4 x Ø2.5 holes on a 28 x 28 square.

- Fixing face 45 x 45 with four Ø2.5 holes on the 28 x 28 pattern; the lens axis passes through the
  centre of that face, perpendicular to it.
- Bend 1: 90 deg, tab 45 x 25.
- Bend 2: 45 deg, tab 25 x 20, with two Ø5.0 x 12 long slots for #10-32.
R1.6 inside bend radii. Report the unfolded blank size.
```

### 4.10 Camera B bracket (side-facing)

```
Same board and hole pattern, 1.587 mm 5052, single body.
- Fixing face 40 x 40, lens axis perpendicular through the centre.
- One 90 deg bend, tab 40 x 30, two Ø5.0 holes 25 apart, 10 from the bend line.
- One 30 deg bend at the far end, tab 20 x 15.
```

Placement — do not let the generator choose it:

| | X | Y | Z | yaw | pitch |
|---|---|---|---|---|---|
| CAM A | −311.15 | −273.05 | 434.98 | 180° | 0° |
| CAM B | −254.00 | −330.20 | 411.23 | −90° | 0° |

Both assume **zero pitch** (`Rotation3d(0, 0, yaw)` in `Constants.VisionConstants`). If the real
brackets tilt the cameras even 2–3°, that is a code change too — the sim feeds the same transform
into PhotonVision's `VisionSystemSim`, so a wrong transform makes the sim agree with a wrong robot.

---

### 4.11 Belly pan / electronics plate

```
Generate one belly pan, 3.175 mm 6061, millimetres, single body.
- Outline 580 (X) x 720 (Y), R8 corners.
- 20 pockets 2.0 deep, 40 x 40, on a 5 x 4 grid, 100 pitch in X, 150 in Y, centred.
- 16 x Ø5.0 perimeter holes, 10 from the edge, 100 pitch on all four sides.
- Component patterns, all Ø5.0 through:
  roboRIO: 4 holes on a 140 x 80 rectangle centred at (-120, 0)
  PDH:     4 holes on a 100 x 70 rectangle centred at (40, 0)
  radio:   2 holes 60 apart centred at (180, 120)
- 1 mm chamfer both faces.
```

The electronics layout is not in the code at all; those centres are placeholders. Fix them from
`measurements.csv` before generating, then re-check the CG, because 65 kg and 5.7 kg·m² are
placeholders too.

---

### 4.12 Climber — optional

`Climber.java` and `AutoClimb.java` are **entirely commented out**. If the mechanism is still on the
robot, prompt it from `measurements.csv` like anything else (motor CAN 35, soft limit 57.426 motor
rotations, ±0.5 duty). If it is gone, do not model it — and delete the dead files so CAD and code
agree.

---

## 5. Where every number comes from

Machine-readable version with confidence flags: `docs/cad/robot-parameters.json`. Index of file/line:
`source-map.csv`.

| Value | Size | Source | Trust |
|---|---|---|---|
| Wheelbase / track | 533.4 / 673.1 | `Constants.DriveConstants` (21 / 26.5 in) | **code** |
| Wheel | Ø76.2 x 22.2 | `Constants.ModuleConstants`, REV | **code** |
| Module body / tabs / height | 100.5 / 133.1 / 171.5 | REV REV-21-3005 spec | vendor |
| Drive reduction | 5.0769 : 1 (13T) | `(45*22)/(13*15)` | **code** |
| Steer reduction | 46.4236 : 1 | 9424 : 203 | **code** |
| Free speed | 4.46 m/s | 5676 rpm / 5.0769 x 0.2394 m | derived |
| Camera A | (−311.15, −273.05, 434.98), 180° | `VisionConstants` (inches) | **code** |
| Camera B | (−254.00, −330.20, 411.23), −90° | `VisionConstants` (inches) | **code** |
| Flywheel pair | Ø101.6, 1:1, 1500 / 2000 rpm | `Shooter.java` + comment in `Constants` | code + estimate |
| Feeder | Kraken X60, 4500 rpm, 1:1 | `Shooter.java` | **code** |
| Deploy arm | 300 long, 3 kg, 0 → 100°, 212.4:1 | `SimulationConstants` | code (shape estimate) |
| Roller / conveyor | 5600 / 4500 rpm, 1:1 | `Intake.java` | **code** |
| Robot mass | 65 kg | `Constants`, `settings.json` | **estimate** |
| Yaw inertia | 5.7 kg·m² | `Constants` | **estimate** |
| FUEL | Ø150.11, 0.203 kg | 2026 manual 5.10.1, AndyMark am-5801 | **code** |
| Frame tube, shooter plates, intake pivot, electronics | — | **not in the repository** | measure |

Three places where the code and physical reality are most likely to disagree:

1. **Mass.** 65 kg = 143.3 lb. 2026 limits are 115 lb without bumpers (R103) and 135 lb with
   (R408). Either 65 kg is the all-up weight and the robot is 8 lb over, or — far more likely — it
   is a default that was never replaced. Weigh it.
2. **`pathplanner/settings.json` says 902 x 876 mm.** That cannot be the frame (perimeter 140 in
   against a 110 in limit, R104) and cannot be derived from the 633.9 x 773.6 module geometry at all.
   It only feeds PathPlanner's collision model, which is why autonomous never noticed. Do not copy it
   into CAD; replace it with the measured envelope. (For reference: this frame plus a 64.5 mm bumper
   is 762.9 x 902.6 — the 902 matches, the 876 does not.)
3. **The camera names lie.** `kBackCameraName = "FrontCamera"` faces the rear, and
   `kSideCameraName = "GoodCamera_OldLeft"` carries a −90° yaw, i.e. it faces the robot's **right**.
   The transform is authoritative; the names are archaeology. Model what the transform says.

---

## 6. Measurement sheet — before you generate anything structural

Print `docs/cad/measurements.csv`, fill it in, convert to mm, then run the prompts. About 45 minutes with a
tape and calipers, and it is what turns the datum drawing into a real model.

| # | Dimension | Where | Feeds |
|---|---|---|---|
| 1 | Overall frame length x width, outer tube faces | drivetrain | master prompt, bumpers |
| 2 | Frame tube sizes and wall thickness | drivetrain | 4.1 |
| 3 | Frame height, floor to top of top tube | drivetrain | elevation, master prompt |
| 4 | Bumper L x W x H, padding depth, mounting height | bumpers | 4.2 |
| 5 | Shooter: both flywheel Ø, width, **axle centre distance** | shooter | 4.3, 4.4 |
| 6 | Shooter: plate outline L x H, plate thickness | shooter | 4.3 |
| 7 | Shooter: muzzle centre relative to origin (x, y, z) | shooter | aim (PR #18) |
| 8 | Shooter: ball exit angle to horizontal | field test | CAD + code |
| 9 | Feeder drum Ø and length | shooter | 4.5 |
| 10 | Intake: pivot (x, y, z), arm length, roller Ø and length, width | intake | 4.6, 4.7 |
| 11 | Conveyor: length, width, drum Ø, height above floor | indexer | 4.8 |
| 12 | roboRIO / PDH / radio / battery positions | belly pan | 4.11 |
| 13 | **All-up mass**, plus each subassembly | scale | code, `settings.json` |
| 14 | CG height, CG x / y (balance on a pipe) | scale | sim, tipping margin |
| 15 | Every COTS part number on the robot | BOM | which STEP to download |

Feed 13 and 14 straight back into `Constants.SimulationConstants.kRobotMassKg`,
`kRobotMOIKgMetersSquared` and `pathplanner/settings.json`. The simulation's acceleration and traction
behaviour depend on those two numbers, and right now both are guesses.

---

## 7. Closing the loop: CAD → code → sim

CAD is only worth it if the numbers flow back. In payoff order:

| CAD output | Goes to | Effect |
|---|---|---|
| Wheelbase / track (measured) | `DriveConstants.kWheelBase`, `kTrackWidth` | kinematics, PathPlanner (now 21 x 26.5 in) |
| Flywheel inertia from mass properties | `kShooterFlywheelMoiKgMetersSquared` | spin-up, shot recovery in sim |
| Deploy arm inertia | `kDeployArmLengthMeters`, `kDeployArmMassKg` | arm dynamics, MAXMotion tuning |
| Muzzle position | `ShooterConstants.kMuzzleOffsetMeters` (PR #18) | aim geometry, close shots |
| Camera transforms, re-measured | `VisionConstants` | vision fusion accuracy |
| Robot mass and yaw inertia | `Constants`, `pathplanner/settings.json` | traction, accel, tipping |
| Bumper envelope L x W | `settings.json` `robotLength` / `robotWidth` | PathPlanner collision model |
| Frame perimeter | engineering notebook / inspection | R104 compliance |

For a fully CAD-accurate sim later, `SimulationConstants` maps directly onto MapleSim's
`DriveTrainSimulationConfig` (mass, MOI, track, wheelbase, wheel radius, gearing, COF). Getting mass
properties right in CAD is what unlocks better simulation — not better renders.

---

## 8. Verification checklist — after every generation

- [ ] Every dimension in the prompt is measurable in the STEP (re-measure it in your CAD tool).
- [ ] One closed solid: no open surfaces, no zero-thickness walls.
- [ ] Holes are holes, not bosses. Diameters match the fastener callout: #10-32 → Ø5.0, #8-32 →
      Ø4.4, M4 → Ø4.5, M3 → Ø3.4, 1/2 in hex → Ø12.75.
- [ ] Units are mm and the bounding box matches the envelope you asked for.
- [ ] No unrequested features — the usual failure is extra fillets, extra ribs, or a symmetric pocket
      in a plate that must not have one.
- [ ] Load it next to the vendor STEP for anything it touches.
- [ ] Mass is plausible: aluminium 2.70 g/cm³, so 3.175 mm plate ≈ 8.3 kg/m².

When the result is wrong, decide first: **wrong structure → re-prompt; right structure, wrong
dimensions → feature-edit.** Re-prompting a nearly-correct part wastes the most time.

---

## 9. Sources

- REV 3in MAXSwerve (REV-21-3005): 100.5 mm body / 133.1 mm with tabs, 171.5 mm tall with NEO,
  1.72 kg, azimuth 9424:203 — REV product page and drawing.
- Wheelbase, track, wheel size, gearing, camera transforms, speeds: this repository
  (`Constants.java`, `Configs.java`, `Shooter.java`, `Intake.java`, `MAXSwerveModule.java`).
- FUEL Ø5.91 in (150.11 mm), 0.203 kg — 2026 Game Manual 5.10.1; AndyMark am-5801.
- Rules R103 / R104 / R402–R405 / R408 — 2026 Game Manual section 8 (re-check the current team update
  before inspection).
- Text-to-CAD limitations and prompting practice — Zoo/Text-to-CAD and AdamCAD documentation and
  reviews: one body per prompt, millimetres, named features, STEP export.
