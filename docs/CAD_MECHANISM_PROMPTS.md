# 9751 Gemini — mechanism-level CAD prompts — 2026 REBUILT

Eight self-contained prompts, one per mechanism. Each one carries **all** the context it needs: units,
coordinate frame, robot dimensions, the mechanism's interface holes, and the transform that puts it in
the robot. Paste a prompt, get a mechanism, insert it, and it lands in the right place.

This is the *other* half of the pack:

| Pack | Granularity | Use it for |
|---|---|---|
| `docs/CAD_MECHANISM_PROMPTS.md` (this file) | one fused body per mechanism | packaging, fit, mass/CG, legality, handing rigid bodies to the sim |
| `docs/ROBOT_CAD_PROMPT.md` | one body per *part* | manufacturing drawings, plate outlines, real detail |

**Say this out loud before you start:** a mechanism prompt produces a **packaging solid** — one closed
body with the right envelope, interfaces and functional features (axles, bores, ball passages), not a
manufacturable part. Text-to-CAD tools emit exactly one body per prompt, so that is the most you can
get in one shot, and it is the most useful thing to have first: it settles fit, mass and legality. The
part-level prompts then refine the pieces you actually cut.

---

## 0. Workflow

1. **Fill the numbers** (`docs/cad/measurements.csv`, 45 rows). Two ways:
   - You have the robot: 45 minutes with a tape, fill the `measured_value` column.
   - You do not: fill from the `reference_value` column, which comes from published robots with the same
     drivetrain and archetype (see [`CAD_REFERENCE_ROBOT.md`](CAD_REFERENCE_ROBOT.md)), and label the
     model **design intent**.

   Even better, if you cannot measure: **[`CAD_REFERENCE_ROBOT.md`](CAD_REFERENCE_ROBOT.md) §2** adapts a
   complete published CAD instead of generating eight mechanisms. Only use the prompts below if you
   want our own architecture rather than the reference robot's.
2. **Keep one layout file.** Section 1 is the nominal layout; edit it with your measurements before
   you generate anything. Every prompt repeats those numbers, so if the layout changes you re-paste.
3. **Generate one mechanism per prompt.** Don't ask for two mechanisms in one prompt.
4. **Assemble in this order:** drivetrain → bumpers → electronics → indexer → shooter → intake →
   vision A → vision B. Ground the drivetrain and insert the rest origin-to-origin with the yaw from
   the table in section 2. The mechanisms share interface holes, so if two of them do not line up, the
   error is in the interface, not in your assembly.
5. **Run the assembly check prompt** (section 6) — it reports interference, bounding box, mass, CG and
   perimeter, which is the whole reason for doing this.
6. **Feed the results back** (section 8): mass and inertia into `Constants.java`, the envelope into
   `pathplanner/settings.json`, and compare the perimeter against R104.

---

## 1. The nominal layout — one source of truth

Every number in this table is used verbatim inside the prompts. `(N)` = nominal, not measured.

```
FRAME                 outer 633.9 (X) x 773.6 (Y)                  (N, lower bound = module union)
RAILS                 25.4 thick x 50.8 tall MAXTube, two levels
  bottom level        z 50.6 .. 101.4                              (N)
  top level           z 219.2 .. 270.0                             (N)
  front / rear rails  at x = +/-303.7, spanning y -286.3 .. +286.3
  left / right rails  at y = +/-374.1, spanning x -216.45 .. +216.45
DECK                  z 270.0                                      (N)
BELLY PAN             z 20.0 .. 23.175, 580 x 720                  (N)
MODULES               4 x REV 3in MAXSwerve, body 100.5 sq, tabs 133.1 sq, 171.5 tall,
                      centres (+/-266.7, +/-336.55), tread touching z = 0
WHEEL                 dia 76.2 x 22.2 wide, axle along Y
BALL (FUEL)           dia 150.11, 0.203 kg
SHOOTER AXLE PITCH    213.6 = 101.6 flywheel + 150.11 ball - 2 x 19.05 compression
INTAKE ARM            300 pivot-to-roller, 500 wide                 (length from code, width N)
```

Two consequences worth knowing before you model:

- **The frame is four rails between the modules**, not a closed box: the module bodies already fill
  the corners. Corner gussets tie adjacent rail ends together.
- **The coded wheel positions are 0.8 in over the perimeter limit.** Module bodies alone give
  633.9 x 773.6 = 2815 mm = 110.8 in of hard-part perimeter against R104's 110 in; add the module
  mounting tabs (133.1 sq) and it is 666.5 x 806.2 = 116.0 in. So either the module centres, the
  tab overhang, or the recorded 21 x 26.5 in is off — the assembled model is what settles it. Measure
  M01–M03 before cutting anything.

---

## 2. Assembly: where each mechanism goes

Model each mechanism in its **own** local frame (stated in its prompt), then insert origin-to-origin.

| Mechanism | Local origin | Insert at X, Y, Z | Yaw about Z | Extra |
|---|---|---|---|---|
| Drivetrain (frame) | robot origin, floor | 0, 0, 0 | 0° | ground this one |
| Bumpers | robot origin, floor | 0, 0, 0 | 0° | separate body, never fused |
| Intake | pivot centre, arm along +X at 0° | 280, 0, 280 | 0° | then rotate **+50° about Y** (arm swings down) |
| Indexer | ball-entry end, +X = travel | 250, 0, 150 | 180° | travels toward −X |
| Shooter | between the flywheel axles, ball exits −X | −390, 0, 300 | 0° | |
| Vision A | lens centre, axis +X | −311.15, −273.05, 434.98 | 180° | from `VisionConstants` |
| Vision B | lens centre, axis +X | −254.00, −330.20, 411.23 | −90° | faces the robot's RIGHT |
| Electronics | pan centre | 0, 0, 20 | 0° | pan underside on z = 20 |

Why transforms instead of modelling everything in robot coordinates: most generators rebuild the body
at the origin on export, so an in-position model can silently lose its placement. A table survives
that. If your tool keeps world placement, tell it to place in robot coordinates and skip the table.

**Verification:** after inserting, measure from the robot origin to one known feature of each
mechanism and compare with the envelope in section 3. A 5 mm error is a transform problem; a 50 mm
error is a prompt problem.

---

## 3. Prompts

Eight prompts follow. Each is standalone — paste it whole, no preamble, no "see above".

---

### 3.1 DRIVETRAIN — frame

```
Generate a single FRC swerve-chassis frame, one fused solid body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. 4-wheel swerve, target mass 50 kg.
Frame: +X forward (intake side), +Y robot's left, +Z up. Origin = centre of rotation, on the floor
plane z = 0, at the midpoint of the four wheel centres.
Four REV 3in MAXSwerve modules stand on the floor at (+266.7, +336.55), (+266.7, -336.55),
(-266.7, +336.55), (-266.7, -336.55). Each module body is 100.5 x 100.5 in plan and 171.5 tall.
The module mounting tabs are 133.1 x 133.1, centred on the same points.
Scoring element: foam ball dia 150.11.

FRAME
Overall outer envelope 633.9 (X) x 773.6 (Y). Four rails of 25.4 x 50.8 rectangular tube, two levels:
- bottom level, z 50.6 to 101.4
- top level,    z 219.2 to 270.0
Per level:
- front rail: at x = +303.7, running y -286.3 to +286.3
- rear rail:  at x = -303.7, running y -286.3 to +286.3
- left rail:  at y = +374.1, running x -216.45 to +216.45
- right rail: at y = -374.1, running x -216.45 to +216.45
The rails stop short of the corners; the modules occupy those zones.

FEATURES
- Module mounting: 8 x dia 5.0 through holes per module (through both levels), on an 88.9 x 88.9
  square centred on each module centre.
- Corner gussets: 8 x flat L-brackets, 3.175 thick, 120 mm legs, tying each adjacent rail pair at
  both levels. Model them as triangular plates with R6 corners, fused to the rails.
- Bumper mounts: 8 x dia 5.0 through holes on the outer faces, at z = 90, two per side, 400 apart.
- Belly pan mounts: 16 x dia 5.0 through the bottom rails, on the inner edge, 100 pitch.
- 4 x dia 20 lightening holes in the front and rear rails, centred.

CONSTRAINTS
- Rails must not intrude into the module body volumes.
- No other features, no fillets over R6, no fasteners.
- Report the bounding box and the mass at 2.70 g/cm3.
```

---

### 3.2 BUMPERS

```
Generate an FRC bumper set as ONE closed body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Frame outer envelope 633.9 (X) x 773.6 (Y), two rail levels, deck at
z 270. Frame: +X forward, +Y left, +Z up. Origin = centre of rotation on the floor plane.
Scoring element: foam ball dia 150.11.

BUMPER
A closed rectangular ring, separate from the frame, never fused to it.
- Outer footprint 762.9 (X) x 902.6 (Y).
- Inner footprint 633.9 x 773.6 (matches the frame).
- Vertical extent z 69.85 to 139.7 (the R405 band).
- Wall build-up, from inside out: 6.35 aluminium backing plate, 57.15 padding, 1.0 cover.
  Total 64.5.
- Cut the ring into four segments at the corners with 45 deg mitres, then fuse the four segments back
  into one body so the mitre lines survive as edges.
- Cover wraps 25 mm around the top and bottom faces over the padding.

FEATURES
- 8 x dia 5.0 through holes in the inner face at z = 90, two per side, 400 apart, matching the frame
  bumper mounts.
- No other features. No fillets. No fasteners.

Report the bounding box and the mass at 1.4 g/cm3 (padding-dominated).
```

---

### 3.3 INTAKE

```
Generate an FRC intake (pivot arm + roller) as ONE fused body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Frame 633.9 (X) x 773.6 (Y), rails 25.4 x 50.8, deck at z 270.
Robot frame: +X forward, +Y left, +Z up, origin at the centre of rotation on the floor.
Scoring element: high-density foam ball, dia 150.11, 0.203 kg.
The intake picks balls off the floor in front of the robot and hands them to an indexer whose ball
path starts at x = +250, z = 150.

LOCAL FRAME (this is what you model in)
Origin = centre of the pivot shaft. The arm extends along +X when horizontal (0 deg). Rotation axis
is Y, positive rotation swings the arm down. Pivot shaft is 1/2 in hex.

GEOMETRY
- Pivot shaft: modelling proxy dia 12.75, spanning y -250 to +250, axis along Y, at the origin.
- Bearing plates: 2 x 6.35 thick plates at y = -60 and y = +60, each 90 x 90, with a dia 13.0 bore
  on the pivot. These bolt to the robot frame.
- Arm plates: 2 x 6.35 thick plates at y = -240 and y = +240, each a 300 (X) x 60 (Z) bar with a
  dia 13.0 pivot bore at the origin and a dia 12.75 roller bore at (300, 0).
- Cross member: 25.4 x 25.4 square tube joining the two arm plates at x = 150.
- Roller: dia 25.4 x 500 long, axis along Y at (300, 0), spanning y -250 to +250, with 8 silicone
  paddle strips 25 wide x 6 tall running the full length, evenly spaced, R2 leading edges.
- Roller drive: a 40 tooth HTD pulley boss on the roller at y = +200, dia 32 x 12 wide.

CONSTRAINTS
- Pivot centre to roller centre exactly 300.0.
- Nothing may protrude below z = -60 in this local frame (the arm hangs 50 mm above the floor when
  tilted 50 deg down).
- No fasteners, no belts. Report the bounding box in this local frame and the mass at 2.70 g/cm3.
```

Placement: insert at (280, 0, 280), yaw 0, then rotate **+50° about Y** — with the right-hand rule
that swings +X downward, putting the roller at x ≈ 473, z ≈ 50, about 156 mm in front of the frame
face. A larger angle puts the roller into the carpet and a smaller one leaves it too high to pick
up a Ø150 ball, so keep this pair together and set both from M22/M27 before you trust the layout. Confirm the real
angles from `measurements.csv` M22/M27; the code only tells you the arm travels 0 → 100°.

---

### 3.4 INDEXER (conveyor)

```
Generate an FRC indexer (conveyor) as ONE fused body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Frame 633.9 (X) x 773.6 (Y), top rails z 219.2 to 270.
Robot frame: +X forward, +Y left, +Z up, origin at the centre of rotation on the floor.
Scoring element: foam ball dia 150.11, 0.203 kg.
The indexer takes balls from the intake at the front and lifts them to a shooter whose ball path
centre is at z = 300 at the rear.

LOCAL FRAME
Origin = the ball entry end, on the ball-path floor. +X is the direction of travel. +Z up.

GEOMETRY
- 2 side plates, 6.35 thick, 430 long (X) x 160 tall (Z), outer faces at y = -100 and y = +100.
- Trough floor: 3.0 polycarbonate, 430 x 200, at z = 0, with the entry edge (x = 0) flared down
  20 mm over 40 mm.
- 3 rollers: dia 25.4 x 200, axis along Y, at (40, 0, 18), (215, 0, 18), (390, 0, 18).
- 6 x 15 mm grip slots in each side plate, on the centreline between rollers.
- Cross members: 2 x 20 x 20 square tube at x = 30 and x = 400, at z = 140.
- Motor mount boss: dia 60 x 8 plate on the outside of the +Y plate at (215, 0, 60).

FEATURES
- 4 x dia 5.0 through holes for the frame interface at (0, +/-80, 94.6) and (430, +/-80, 94.6).
- 2 x dia 5.0 holes at (430, +/-40, 0) for the shooter hand-off flange.

CONSTRAINTS
- Ball must be able to travel from x = 0 to x = 430 without exceeding 70 mm of side clearance.
- No belts, no motor, no fasteners. Report bounding box and mass at 2.70 g/cm3.
```

Placement: insert at (250, 0, 150), yaw 180°. It then spans x −180 … +250 and z 150 … 310, so its
exit lines up with the shooter's feeder at z ≈ 300.

---

### 3.5 SHOOTER

```
Generate a fixed dual-flywheel FRC shooter as ONE fused body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Frame 633.9 (X) x 773.6 (Y), deck at z 270.
Robot frame: +X forward, +Y left, +Z up, origin at the centre of rotation on the floor.
Scoring element: foam ball dia 150.11, 0.203 kg, fired at the hub which sits behind the robot.
This shooter has NO hood and NO turret: the whole drivetrain rotates to aim, and range comes only from
wheel speed. Do not add a pivoting hood or a turret bearing.

LOCAL FRAME
Origin = midpoint between the two flywheel axle centres, on the ball path. The ball exits along -X.

GEOMETRY
- Two flywheel axles, both along Y, dia 12.75 (to be broached 1/2 in hex), at (0, 0, +106.8) and
  (0, 0, -106.8): 213.6 apart, which is one ball plus a 19.05 compression.
- 2 flywheels: dia 101.6 x 25.4 wide on those axles, inner faces at z = +/-56 so the ball is
  squeezed 19.05 per side.
- 2 side plates, 6.35 thick, at y = -95 and y = +95: rounded rectangles 300 (X) x 334 (Z) centred on
  the origin, R12 corners, with dia 12.75 bores at both axle positions.
- Ball passage: through slot 55 (Z) x 162 (Y) between the flywheels, centred on the origin, open to
  the -X edge, R12 ends.
- Feeder: drum dia 50.8 x 150, axis along Y at (+120, 0, -20), with a 3.0 raised grip band.
- Entry funnel: two 3.175 plates at x = +60, angled 30 deg, forming a 170 wide mouth that guides the
  ball from +X into the flywheel nip.
- Mounting flanges: 4 x 3.175 tabs at y = +/-150, x = +86.3, z = -68 and -43, each 60 x 40 with a
  dia 5.0 hole centred.
- 4 x dia 12 lightening holes in each side plate, between the axles.

CONSTRAINTS
- The ball path from the feeder drum to the exit gap must be clear: no member inside the passage.
- Axle pitch exactly 213.6. Flywheel dia exactly 101.6.
- No motors, no belts, no fasteners. Report bounding box and mass at 2.70 g/cm3, excluding the
  flywheel mass separately if you can.
```

Placement: insert at (−390, 0, 300), yaw 0. Ball exit then sits at x ≈ −570, z ≈ 300 — outside the
rear frame face by 253 mm, which is the overhang the shooter needs. If your real robot's exit is
closer in, change the insert X and update the muzzle offset in code (PR #18 adds
`ShooterConstants.kMuzzleOffsetMeters` for exactly this).

---

### 3.6 VISION A — rear-facing camera bracket

```
Generate a camera bracket as ONE fused solid body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Robot frame: +X forward, +Y left, +Z up, origin at the centre of
rotation on the floor. This camera looks out over the shooter muzzle and is the one used for aiming.

LOCAL FRAME
Origin = centre of the camera lens on the front face of the camera. Optical axis = +X. The camera
board lies in the YZ plane with its 36 x 36 face perpendicular to the optical axis.

CAMERA
Board 36 x 36 x 10, four dia 2.5 holes on a 28 x 28 square, lens centred on the board.

BRACKET
- Fixing face: 45 x 45 x 1.587 aluminium, held 0 mm in front of the board face, with the four
  dia 2.5 holes on the 28 x 28 pattern. The lens must not be obstructed within a 75 deg cone about
  the optical axis.
- Bend 1: 90 deg away from the optical axis, tab 45 (Y) x 25 (X), 1.587 thick.
- Bend 2: 45 deg, tab 25 x 20, with two 5.0 x 12 slots for #10-32.
- Inside bend radii 1.6.

CONSTRAINTS
- Single body, 1.587 thick throughout, no fasteners.
- The bracket must stay inside a 45 x 45 x 45 box measured from the lens centre, except the two
  mounting tabs which may extend 30 mm further back (-X).
- Report the unfolded blank size and the mass at 2.70 g/cm3.
```

Placement: insert at (−311.15, −273.05, 434.98), yaw 180°. Both numbers come from
`Constants.VisionConstants` and are the same ones the simulation's PhotonVision model uses, so the CAD
and the sim agree.

---

### 3.7 VISION B — side-facing camera bracket

```
Generate a camera bracket as ONE fused solid body, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Robot frame: +X forward, +Y left, +Z up, origin at the centre of
rotation on the floor. This camera is a second, wider-context AprilTag camera looking out of the
robot's right side.

LOCAL FRAME
Origin = centre of the camera lens on the front face. Optical axis = +X. Board face in the YZ plane.

CAMERA
Board 36 x 36 x 10, four dia 2.5 holes on a 28 x 28 square.

BRACKET
- Fixing face 40 x 40 x 1.587, four dia 2.5 holes on the 28 x 28 pattern, lens unobstructed within a
  75 deg cone.
- Bend 1: 90 deg away from the optical axis, tab 40 x 30, two dia 5.0 holes 25 apart, 10 from the
  bend line.
- Bend 2: 30 deg at the far end of that tab, tab 20 x 15.
- Inside bend radii 1.6.

CONSTRAINTS
- Single body, 1.587 thick, no fasteners, inside a 60 x 60 x 60 box about the lens centre.
- Report the unfolded blank size and the mass at 2.70 g/cm3.
```

Placement: insert at (−254.00, −330.20, 411.23), yaw −90°. That transform points the camera along
**−Y**, i.e. out of the robot's right side — despite `kSideCameraName = "GoodCamera_OldLeft"`.

---

### 3.8 ELECTRONICS — belly pan and component proxies

```
Generate an electronics packaging body as ONE fused solid, millimetres.

ROBOT CONTEXT
Team 9751, 2026 FRC REBUILT. Frame 633.9 (X) x 773.6 (Y), bottom rails z 50.6 to 101.4.
Robot frame: +X forward, +Y left, +Z up, origin at the centre of rotation on the floor.
The belly pan hangs 20 mm off the carpet, below the bottom rails.

LOCAL FRAME
Origin = centre of the pan, on its underside. +Z up.

GEOMETRY
- Pan: 580 (X) x 720 (Y) x 3.175, 6061, R8 corners, top face at z = 3.175.
- 20 pockets 2.0 deep, 40 x 40, on a 5 x 4 grid, 100 pitch in X, 150 in Y, centred.
- 16 x dia 5.0 perimeter holes, 10 from the edge, 100 pitch on all four sides, counterbored 1.0.
- Component proxies, all fused to the pan top, as plain boxes with 2 mm corner radii:
    battery SLA     197 x 77 x 167   centred at (-180, 0, 86.5)
    roboRIO         152 x 102 x 51   centred at (60, 90, 28.5)
    PDH             152 x 76 x 44    centred at (60, -30, 26)
    radio           90 x 90 x 30     centred at (180, -120, 18)
    SPARK MAX stack 90 x 60 x 90     centred at (180, 120, 48)
- 4 x dia 5.0 holes on the pan for the battery tray, on a 210 x 90 rectangle about its centre.

CONSTRAINTS
- Single body. Do not model wiring, breakers, or the battery terminals.
- Every component box must sit fully inside the pan outline.
- Report the bounding box and the mass at 2.70 g/cm3 for the plate alone, then again with the
  proxies replaced by these masses: battery 5.9, roboRIO 0.5, PDH 0.6, radio 0.2, driver stack 1.2 kg.
```

Placement: insert at (0, 0, 20), yaw 0.

---

## 4. Interface registry — what makes the eight files one robot

These are the shared hole patterns. If two mechanisms do not line up on insert, the interface is what
to fix, not the assembly. All nominal until measured.

| ID | Between | Holes | Robot-frame position |
|---|---|---|---|
| IF-MODULE | frame ↔ 4 COTS modules | 4 × Ø5.0 on 88.9 × 88.9 per module | centred on each module centre, z 244.6 |
| IF-SHOOTER | frame ↔ shooter | 4 × Ø5.0 | x −303.7, y ±150, z 232.0 and 257.0 |
| IF-INTAKE | frame ↔ intake | 2 × Ø13.0 bearing bores + 4 × Ø5.0 | bores at (280, ±60, 280); holes at x 300, y ±90, z 250 |
| IF-INDEXER | frame ↔ indexer | 4 × Ø5.0 | x 250 and −180, y ±80, z 244.6 |
| IF-BELLY | frame ↔ electronics | 16 × Ø5.0 | 10 inboard of the frame edge, 100 pitch, z 58.2 → pan top |
| IF-SHOOTER-INDEXER | indexer ↔ shooter | 2 × Ø5.0 | must line up in **X and Z**: indexer exit at z 300 = shooter feeder inlet |
| IF-BUMPER | frame ↔ bumpers | 8 × Ø5.0 | outer faces at z 90, two per side, 400 apart |

The one that actually has to be right in 3D is `IF-SHOOTER-INDEXER`: the ball path is a continuous
line from the intake roller (z ≈ 50) up through the indexer (z 150 → 310) into the feeder (z ≈ 280)
and out between the flywheels (z 300). Any step in that line is a jam.

---

## 5. Legality and interference checks the assembled model answers

| Rule | Limit | How to check in the assembly |
|---|---|---|
| R104 perimeter | 2794 mm (110 in) | `PERIMETER` of the outermost hard parts, bumpers excluded. Our coded geometry already gives 2815 mm from module bodies alone — see section 1. |
| R104 starting height | 762 mm (30 in) | bounding box Z with the intake stowed, shooter at rest |
| R102 starting volume | nothing outside the perimeter projection | check the shooter overhang and the intake in the stowed pose |
| R403 bumper extension | ≤ 101.6 mm | bumper outer footprint minus frame: 64.5 ✓ |
| R405 bumper band | fills 69.85 → 139.7 | bumper body Z range ✓ |
| R103 mass | ≤ 52.16 kg | sum the eight bodies |
| R408 mass with bumpers | ≤ 61.23 kg | sum, bumpers included |

Interference checklist — the five collisions that actually happen on this layout:

1. **Intake vs floor.** At +50° tilt the roller centre sits at z ≈ 50, its lowest paddle tip at
   z ≈ 12. Verify nothing goes below 0 at the deployed angle; if it does, the tilt is too steep or
   the pivot too low (change the insert, not the model).
2. **Intake vs front bumpers.** At +50° the arm passes through the bumper band (69.85–139.7). Check
   the arm plates clear it; if not, move the pivot up or narrow the bumper segment there.
3. **Shooter vs deck.** Local z ±167 at insert z 300 gives 133 … 467, so the shooter hangs 137 mm
   below the deck. Check it misses the top rails at 219.2–270.
4. **Shooter vs indexer.** The feeder inlet must meet the indexer exit at z ≈ 300, and the indexer
   must not poke into the flywheel plane.
5. **Electronics vs drivetrain.** The pan is 580 × 720 inside a 583.1 × 723.2 opening: 1.5 mm per
   side. Either shrink the pan or move the rails.

---

## 6. Assembly check prompt

Paste this once the eight bodies are imported. It works in any CAD tool with a scripting interface,
or in an LLM that can read an exported report.

```
I have a CAD assembly of an FRC robot made of 8 bodies. Check it and report.

BODIES AND DECLARED ENVELOPES (robot frame, mm, +X forward, +Y left, +Z up, origin at the centre of
rotation on the floor):
  drivetrain   x -317 .. 317      y -387 .. 387      z 0 .. 270
  bumpers      x -381 .. 381      y -451 .. 451      z 69.85 .. 139.7
  intake       x 195 .. 520       y -250 .. 250      z 10 .. 345
  indexer      x -180 .. 250      y -100 .. 100      z 150 .. 310
  shooter      x -570 .. -330     y -150 .. 150      z 133 .. 467
  vision_a     x -360 .. -300     y -295 .. -250     z 410 .. 460
  vision_b     x -290 .. -230     y -355 .. -310     z 385 .. 440
  electronics  x -290 .. 290      y -360 .. 360      z 20 .. 165

Report, in this order:
1. Any body that exceeds its declared envelope, with the axis and the overshoot in mm.
2. Every interfering pair, with the interference volume in cm3. Ignore contacts under 0.5 cm3.
3. The overall bounding box of all bodies, and the perimeter of the outermost hard parts with the
   bumpers excluded (FRC R104 limit: 2794 mm).
4. Total mass, and the mass and centre of mass of each body given these densities:
   drivetrain 2.70 g/cm3, bumpers 1.40, intake 2.70, indexer 2.70, shooter 2.70, electronics 2.70.
5. Overall centre of gravity in X, Y, Z.
6. A list of the shared hole patterns you can find between bodies: for each pair, hole diameter,
   count, and whether the two patterns are concentric within 0.5 mm.
```

Then fix the layout, re-paste the affected mechanism prompts, and re-run. Two iterations is normal.

---

## 7. What the mechanism model is *not*

- **Not manufacturable.** A fused mechanism body has no part splits, no bend lines you can flat
  pattern, no fastener access. Use `ROBOT_CAD_PROMPT.md` for the parts you actually cut.
- **Not accurate inside the mechanism.** The envelopes and interfaces are nominal. The shooter's
  internal plate shape, the intake's arm architecture and the indexer's trough detail are invented
  placeholders that are geometrically consistent — that is all.
- **Not a substitute for the COTS STEP files.** The four modules, all motors, controllers and the
  battery should be downloaded, not generated. The prompts treat them as proxies only where they
  affect packaging.

---

## 8. Feeding the result back into the code

| From the assembly | Into | Why |
|---|---|---|
| Total mass | `Constants.SimulationConstants.kRobotMassKg`, `pathplanner/settings.json` `robotMass` | traction and acceleration in sim |
| Yaw inertia about Z at the CG | `kRobotMOIKgMetersSquared`, `settings.json` `robotMOI` | turn response |
| CG height | keep as documentation; the sim assumes a planar model | tipping margin, bumper height |
| Bounding box of the hard parts | `settings.json` `robotLength` / `robotWidth` | PathPlanner collision model (902 × 876 is unverified) |
| Shooter flywheel mass and inertia | `kShooterFlywheelMoiKgMetersSquared` | spin-up and shot recovery |
| Intake arm mass and length | `kDeployArmLengthMeters`, `kDeployArmMassKg` | arm dynamics |
| Muzzle centre | `ShooterConstants.kMuzzleOffsetMeters` (PR #18) | aim geometry |
| Wheelbase and track, re-measured | `DriveConstants.kWheelBase`, `kTrackWidth` | kinematics |

Run the nominal build-up in section 5 first: 7 (frame) + 6.88 (modules) + 8 (bumpers) + 4.5 (intake) +
3 (indexer) + 6.5 (shooter) + 12 (electronics) + 0.4 (vision) ≈ **48 kg**, plus fasteners ≈ 50 kg
(110 lb). That is comfortably under R103's 52.16 kg **and 15 kg lighter than the 65 kg the simulation
currently assumes** — which is the single biggest correction this exercise will produce.
