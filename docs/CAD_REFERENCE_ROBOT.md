# Reference robots — building our CAD without access to our robot

You cannot measure the robot right now. That is fine, but it changes what the CAD *is*. This document
sets out the honest version: pick a published robot that matches our archetype, use it as the
dimensional base, change the handful of numbers our code actually pins, and label everything that is
now an assumption rather than a measurement.

Two ways to use a reference robot, and they are not equivalent:

| Approach | What you get | Effort | Risk |
|---|---|---|---|
| **A. Adapt a published CAD** (recommended) | A complete, buildable, dimensionally real robot you can edit | Low — one download, ~10 edits | The reference's packaging choices are not ours |
| **B. Borrow dimensions into the prompts** | A packaging model of *our* architecture with plausible numbers | Medium — paste 8 prompts | Numbers are plausible, not real |

Both are in this document: section 2 is approach A, section 4 is approach B.

**Read this before you start.** Neither approach produces an as-built model of our robot. The sim
constants that come out of it (`kRobotMassKg`, `kRobotMOIKgMetersSquared`, wheelbase, muzzle offset)
become *design intent*, not truth. The SysId routines already in this repo are what replace them with
measured values, and a tape measure on the real robot is still the only way to close the loop. Write
"design intent" in the notes field so nobody mistakes it for measured later.

---

## 1. What "similar" has to mean

Our robot, from the code: 4 × REV 3in MAXSwerve, 21 × 26.5 in wheelbase/track, MAXTube structure,
single-jointed pivot intake with a driven roller, conveyor indexer, **fixed opposed dual flywheel, no
hood, no turret, no climber** (the climber files are commented out).

So a useful reference must have:

1. **The same drivetrain module** — REV-21-3005. This is the single most valuable match, because the
   module sets the frame, the corner geometry and the bumper height.
2. **The same structure system** — MAXTube 2×1 with the MAX hole pattern, so gussets and mounts carry over.
3. **The same game piece** — Ø150.11 FUEL.
4. Ideally the same intake→indexer→flywheel chain.

What it does *not* need to share: the shooter type. Ours is the odd one out (opposed pair, no hood), and
it is the one mechanism you should design rather than copy.

---

## 2. Reference A — REV 2026 ION FRC Starter Bot (primary base)

**This is the one to actually use.** It is a complete, published, configurable REBUILT robot built on
exactly our drivetrain hardware.

- Build guide: `docs.revrobotics.com/frc-kickoff-concepts/2026`
- Configurable Onshape model (checkbox for the MAXSwerve drivetrain and the MAXComposite intake)
- Full BOM and cut list for both EasySwerve and MAXSwerve configurations
- Robot code: `github.com/REVrobotics/2026-REV-ION-FRC-StarterBot`

### What it gives us, with published numbers

| Item | Value | Why it matters to us |
|---|---|---|
| Modules | 4 × REV-21-3005 3in MAXSwerve | identical to ours |
| Side rails | MAXTube 2×1 × 1/8 wall, **4 × 20 in** (MAXSwerve config; 19 in for EasySwerve) | our rails are the same stock, cut to suit a 21 in wheelbase |
| Cross tubes | MAXTube 1/2 × 1/2, **2 × 25 in** | our track is 26.5 in, so these grow ~1.5 in |
| Belly pan | 3 mm polycarb, **26.5 × 10.5 in** | a real electronics-floor precedent |
| Intake roller shaft | 1/2 in rounded hex, **cut to 18-7/16 in (468 mm)** | our nominal 500 mm intake width is within 1.3 in of it |
| Intake | compliant four-bar arm, passively deployed | our arm is powered and 300 mm; the *roller and bearings* still transfer |
| Launcher | 2 × NEO Vortex "upduction", 6 in flap wheels + compliant wheels, polycarb hood, on-pitch compression adjustment | **do not copy** — ours has no hood |
| Motors | NEO Vortex + SPARK Flex throughout | we run NEO + SPARK MAX on drive, Kraken X60 on roller/feeder |

### How to adapt it (approach A, the fast path)

1. Open the Onshape document and copy it to your account (or export STEP). Select the **MAXSwerve**
   configuration and the MAXComposite intake if you want the newer intake.
2. **Move the modules** to our wheel centres: ±10.5 in in X, ±13.25 in in Y (21 × 26.5 in). The four
   rail lengths follow from that; re-cut them in the model. Expect the side rails to land near 20 in
   and the cross tubes near 26.5 in.
3. **Keep** the rail construction, corner joints, belly pan, bumper mounting, and the conveyor. Those are
   the parts that cost the most time and carry over cleanly.
4. **Replace the launcher** with our opposed pair. Use the shooter prompt in
   `CAD_MECHANISM_PROMPTS.md` §3.5, with the compression and exit angle from reference B below.
5. **Add our two cameras** at the code transforms: CAM A (−311.15, −273.05, 434.98) yaw 180°,
   CAM B (−254.00, −330.20, 411.23) yaw −90°. These are the only two numbers in the whole model that
   are measured truth, because they come from `Constants.VisionConstants`.
6. **Re-run mass properties** and compare with every other reference in section 3.
7. Write `design intent` in the file properties, and in any note you leave the team.

Minimum edits to make it *our* robot rather than the starter bot: module positions, intake arm,
shooter, cameras, electronics layout. Everything else is structure.

---

## 3. References B, C, D — for the things reference A gets wrong for us

### B. Big Sky Robotics (Montana State) — Ri3D 2026 — for the shooter

- Onshape CAD and `github.com/Montana-State-Ri3D/2026-Ri3D`
- **Shooter: 0.41 in (10.4 mm) compression, ~70° exit angle.** Hopper: 0.16 in compression.
- Pivot intake, so the packaging problem is the same one we have.

Use it for the two numbers our code cannot supply: how much to squeeze the ball, and what angle to
fire at. Our opposed pair should run a little more total compression than a single wheel against a
hood (0.5–0.75 in / 12.7–19.05 mm), so treat Big Sky's 0.41 in as the lower bound and tune up from
there. The 70° exit angle is a good starting point for a shooter that fires from close to the hub.

### C. 2026 Everybot (Robonauts, team 118) — for mass

- Full CAD, docs and BOM: `118everybot.org/2026-frc-resources`
- **All-up mass 103 lb (~46.7 kg), including bumpers and battery.**
- Bumpers: single-piece unibumper, 5 in tall, 3/4 in plywood; planks 33.5 in and 28.5 in.

This is the most important number in the whole document, because it is a *weighed* robot rather than a
guess. Compare: our code assumes 65 kg (143 lb), which is above even the 135 lb with-bumpers limit
(R408). Reference robots in this archetype land in the 47–57 kg range. **The 65 kg is almost certainly
a placeholder that was never updated** — and it is the constant that most affects how the simulation
feels under acceleration.

### D. Penn State Ri3D 2026 — for the flywheel tradeoff

- 27 × 27 in chassis; final drive 2:1 to 6 in wheels, 1:1 to the flywheels, and they went from 2 to 4
  flywheels to offset a speed reduction.

Useful because it documents the tradeoff we are stuck with: with 1:1 NEOs and Ø101.6 wheels we have
limited stored energy, so ball-to-ball recovery is our weak point. If our shooter turns out to sag
during a stream, more flywheel mass is the lever, not more speed.

---

## 4. Approach B — borrowing dimensions into the prompts

If you would rather generate our own architecture from prompts than edit someone else's CAD, here is
what to fill in. Every row is a value that was previously "MEASURE"; the reference is where the
plausible number comes from.

| Our value | Reference-derived setting | Source | Confidence |
|---|---|---|---|
| Frame outer | 633.9 × 773.6 mm (25.0 × 30.5 in) | our module centres + module bodies; matches reference A's construction method | derived |
| Frame rails | 2×1 MAXTube × 1/8 wall, two levels | reference A | high |
| Cross tubes | 1/2×1/2 MAXTube | reference A | high |
| Intake width | 468 mm (18-7/16 in) | reference A roller shaft cut length | medium |
| Intake roller | Ø25.4, paddles per §3.3 | design choice | — |
| Shooter compression | 12.7–19.05 mm total (opposed pair) | reference B lower bound 10.4 mm, adjusted for a second driven wheel | medium |
| Shooter exit angle | 70° above horizontal | reference B | medium |
| Bumper height | 5 in (127 mm) of plywood core, padding to 139.7 mm band | reference C construction + R405 | medium |
| Bumper thickness | 57.15 mm padding + 6.35 backing + 1.0 cover | R402-A + reference C | high |
| Robot mass | 47–57 kg; use 52 kg for the model | reference C measured 46.7 kg | medium |
| Electronics layout | copy reference A's belly pan arrangement | reference A | low (ours differs) |

Everything not in this table is a design choice, not a measurement — say so in the file.

---

## 5. What this cannot do

Be blunt with the team about the limits, because CAD that looks authoritative is dangerous:

- **It cannot verify legality.** R104 (110 in perimeter) depends on the *real* outermost parts. We already
  know the coded wheel positions imply a 110.9 in perimeter with the module bodies flush, which is
  0.9 in over. A model built from a reference cannot tell you whether the real robot passes inspection.
- **It cannot confirm the coded numbers.** If 21 × 26.5 in, 65 kg, 5.7 kg·m² and 0.0025 kg·m² were typed
  in from memory, a model built from them will simply reproduce the same guesses in 3D.
- **It cannot set the muzzle position or camera tilts.** Those depend on how the real shooter and
  brackets ended up mounted. PR #18 added `ShooterConstants.kMuzzleOffsetMeters` precisely because
  this number has to be measured on the robot.
- **It cannot tune a shot.** Compression and exit angle from a reference are starting points; the
  distance→RPM table has to come from shooting on a field.

The useful framing: this exercise buys you **packaging, mass, interfaces and a buildable starting
point**. It does not buy you accuracy. Accuracy comes from `measurements.csv` when you next have the
robot, and from SysId for the inertia terms.

---

## 6. Checklist

- [ ] Copy or export the REV Starter Bot Onshape (MAXSwerve config)
- [ ] Move the modules to ±10.5 / ±13.25 in and re-cut the rails
- [ ] Replace the launcher with the opposed pair (`CAD_MECHANISM_PROMPTS.md` §3.5), 12.7–19.05 mm
      compression, 70° exit
- [ ] Set the intake to a 468 mm roller on a 300 mm powered arm
- [ ] Add CAM A and CAM B at the `VisionConstants` transforms
- [ ] Re-run mass properties; compare against 46.7 kg (Everybot) and 65 kg (our code)
- [ ] Update `pathplanner/settings.json` `robotLength` / `robotWidth` / `robotMass` from the model
- [ ] Mark the model "design intent — not measured" and log the assumptions next to
      `docs/cad/measurements.csv`
- [ ] When the robot is next accessible: 45 minutes with a tape closes every remaining row

---

## 7. Sources

- REV 2026 ION FRC Starter Bot — build guide, BOM, cut list, configurable Onshape, code repo
  (`docs.revrobotics.com/frc-kickoff-concepts/2026`; `github.com/REVrobotics/2026-REV-ION-FRC-StarterBot`).
- Big Sky Robotics Ri3D 2026 (Montana State) — Chief Delphi build thread, Onshape CAD, code repo.
- 2026 Everybot (Robonauts / team 118) — `118everybot.org/2026-frc-resources`, Everybot Docs, all-up
  mass 103 lb reported in the build thread.
- Penn State Ri3D 2026 — Chief Delphi build thread.
- Our own numbers: `docs/cad/robot-parameters.json` and `docs/cad/source-map.csv` (file:line in this repo).
- 2026 rules R103 / R104 / R402–R405 / R408, and FUEL Ø5.91 in — 2026 Game Manual.
