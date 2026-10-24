# Shooter Accuracy & Robustness — What Code Alone Can Fix

**Question:** without touching the mechanism, can our current shooter be made more
accurate and more robust?

**Short answer:** yes. Most of the inconsistency this robot shows is not the
flywheel — it is *when* the feeder is allowed to run, *what speed* the flywheel
is actually holding, and *where* the robot thinks the shot is going. All three
are software. This document lists what was wrong, what changed, what other 2026
teams with the same archetype do, and the exact calibration steps to get the
rest of the accuracy out of the mechanism you already have.

The changes are live in this branch. The *targeting* defaults are neutral - table
off, muzzle offset zero, heading trim zero, motion gain zero - so the robot aims
and spins exactly as it did. What changes unconditionally is the gate: the feeder
now waits for a real, settled shot instead of latching once and feeding until the
button is released. Every tuning knob can be reverted on its own.

---

## 1. Hardware as built (unchanged)

| Item | As-built |
| --- | --- |
| Flywheel motors | 2 x REV NEO on SPARK MAX, CAN 5 (front) / 6 (back), 1:1, coast, 60 A limit |
| Flywheel speeds | front 1500 RPM, back 2000 RPM (differential spin, ratio 0.75) |
| Feeder | CTRE Kraken X60 / TalonFX, CAN 7, brake, `VelocityVoltage` 4500 RPM |
| Indexing | NEO conveyor (velocity loop) + intake roller |
| Aiming | no turret, no hood: the whole drivetrain rotates; range comes only from RPM |
| Sensors | NEO hall encoders (RPM), navX, 2 PhotonVision cameras |

Because there is no hood, **RPM is the only range control**. Anything that makes
the actual wheel speed differ from the intended one becomes a distance error.

---

## 2. What the code was actually doing

### 2.1 The feeder was gated exactly once

`Shooter.periodic()` used a `waitingForSpinUp` latch: once the wheels reached
speed and the aim command finished, `waitingForSpinUp` was cleared and the
feeder plus conveyor ran until `stopAll()`. Nobody re-checked the wheel speed
after the first ball.

Every ball pulls kinetic energy out of a flywheel that is only two 1:1 NEOs on
a light wheel. Ball 2, 3 and 4 therefore leave slower than ball 1 — the exact
failure 6328 describes in their build thread (they solved it with extra
flywheel inertia and four motors; software can solve it by waiting) and the
reason 3464 added a debounced readiness signal.

### 2.2 "At speed" was one-sided and true at zero

```java
frontReady = current >= target && current <= target * 1.1;   // old
```

Two problems:

* the allowed error was **up to +10 %** (1650 RPM for a 1500 RPM setpoint) - at
  a fixed launch angle, a 10 % exit-speed error is roughly a 20 % range error;
* a target of **0 RPM counted as ready** because `0 >= 0` is true. That is the
  exact bug 3464 fixed in their PR "Prevent stale flywheel readiness": a
  stopped wheel comparing itself to a stale zero target looks ready. Our new
  gate requires a nonzero target, a percentage band, and a settle time.

### 2.3 Code could not set the flywheel speed

`periodic()` re-read `Front Shooter RPM Setpoint` from SmartDashboard into
`m_fronttargetRPM` **every loop**. Any distance-based target written by code
(`setFrontTargetRPM(...)`) was overwritten 20 ms later. This is why the
existing `InterpolatingDoubleTreeMap m_rpmMap` field was constructed with one
point (`215 in -> 5000 RPM`) and then never used - it could not have worked.

Now an operator edit is only accepted when the dashboard value **changes**, so
code and the dashboard can both drive the target.

### 2.4 Aiming stopped the moment the robot started ranging

The teleop shot was a command sequence:

```
aim until aligned  ->  drive to 100 in  ->  hold
```

The heading controller ran only in the first phase. Any push from defense, bump
into a wall, or wheel slip during the ranging drive left the robot firing
several degrees off - and the shooter happily fed, because the aim gate had
already latched "done".

Now one command aims and ranges at the same time, keeps correcting heading in
every phase, and the shooter itself refuses to feed while the heading error is
outside tolerance.

### 2.5 Aim geometry ignored the muzzle

```java
return Math.toDegrees(Math.atan2(hubY - robotY, hubX - robotX)) - 180.0;
```

The aim point is the **robot center**, not where the ball leaves. If the
shooter sits off the center line, close shots are systematically wrong: a 0.3 m
lateral offset is `atan(0.3 / 1.5) = 11°` at 1.5 m and still 5.7° at 3 m. The
same error also corrupts the distance used for RPM, which is why close-range
shots are usually the worst ones on this kind of robot.

`ShotMath.solveAim` now solves from the muzzle position with a few fixed-point
iterations (the same iteration that shoot-on-the-move solvers use), and exposes
a heading trim for the mechanical bias that every robot has. 6391 ships exactly
this trim (`shooterHeadingOffsetDegrees`), and 7415 applies a global 1.02 RPM
multiplier for the same "we systematically undershoot" reason.

Both constants default to zero, so the old behavior is bit-for-bit preserved
until someone measures the robot.

### 2.6 No motion gate, no motion compensation

The ball leaves with the robot's velocity added to it. Nothing in the old code
cared whether the robot was driving at 3 m/s while firing. Every top 2026 team
either gates on motion or compensates for it (6391, 7415, 6328: table + time of
flight + lead). We now gate by default and expose the compensation as a tunable.

### 2.7 The feedforward has no friction term

```java
kV = 12.0 / 5676.0 = 0.002114 V/RPM;   kS = 0
```

That is the theoretical back-EMF only. A real flywheel needs a few tenths of a
volt more to overcome bearing and belt friction, and with `kP = 0.0009 V/RPM`
and no integrator, the loop settles **below** the commanded speed. The size of
the deficit depends on battery voltage, temperature and break-in, i.e. it drifts
over a match.

This team has already been bitten by this class of bug: `PATH_ACCURACY_AUDIT.md`
records the 12x kV unit error that left the wheels at 1279 / 1705 RPM and made
`isAtTargetSpeed()` never true. `kS` is now wired into both the SPARK MAX and
the TalonFX configuration (still 0, so nothing changes until it is measured),
and every loop reports the actual RPM error so the deficit is visible instead of
silent.

### 2.8 There was no way to see why the robot was not firing

The new dashboard group answers that in one glance: `Shooter/Feed Status` shows
`SPINNING UP (312 RPM)`, `AIMING (4.1 deg)`, `RANGING`, `MOVING (...)`,
`FEEDING` or `OFF`.

---

## 3. What other 2026 teams with the same archetype do

| Team / source | What they do | What we took |
| --- | --- | --- |
| **7415** (technical binder) | 6-point distance-to-RPM interpolation table, a global **1.02** multiplier because they undershoot, shoot-on-the-move via pose extrapolation, per-lane distance | Editable table + trim multiplier |
| **6391** (2026 repo) | distance -> RPM, distance -> hood angle, distance -> **time of flight**; shoot-on-move iterates 3x; `shooterHeadingOffsetDegrees` trims physical misalignment | Fixed-point aim solve, heading trim, motion-comp formula |
| **5024** (2026 repo) | distance -> RPM table (2.44 m/2800 RPM ... 7.92 m/5100 RPM), `kS 0.2, kV 0.001764, kA 0.01` on the flywheel | kS support, table shape |
| **6328** (build thread) | four-phase flywheel control (start-up / idle / ball / recovery); velocity PID "applies output to compensate as soon as the ball enters", which they had to solve; more inertia on the comp robot | Per-shot recovery gating; documented as a hardware limit |
| **3464** (PR #11) | readiness requires active closed loop, **nonzero target**, connected sensor, and a debounce; clears on stop | Nonzero-target rule, settle time |
| **SubZero** (PR #32) | fixed "fed balls before the wheel was at speed", readiness compared against a stale zero setpoint, aiming cancelled by the shoot command, table missing sideways lead | Continuous gating, live aim gating |
| **6908** (2026 repo) | 35-95 % auto speed from Limelight distance, distance table calibration notes | Table trim workflow |
| **YETI shooter guide** | log RPM before/after each shot, recovery time and hit rate vs distance; feed only when `abs(v - setpoint) < tol` for N ms | Telemetry layout, settle-time rule |
| **Swartdogs 2026 `Flywheel`** | REVLib 2026 `closedLoop.feedForward.kS(kV)(kA)` with explicit `Volts.per(RPM)` units, follower motor config | kS wiring on the SPARK MAX config |

Common thread: everybody with this archetype converts distance to RPM with a
calibrated table, and everybody gates the feed on **live** wheel speed rather
than a one-time spin-up check.

---

## 4. What changed in this branch

| File | Change |
| --- | --- |
| `ShotMath.java` **(new)** | Pure aim geometry, table interpolation, speed ratio, closing-speed / motion compensation. No hardware. |
| `ShotGate.java` **(new)** | Shot-readiness state machine: speed band + settle + hysteresis, live aim gate, distance gate, motion gate, moving-shot distance envelope. No hardware. |
| `ShotInputs.java` **(new)** | The per-loop measurements the drivetrain hands to the shooter. |
| `ShooterCameraAim.java` **(new)** | Pure tag-residual math for the camera mounted on the shooter: bearing/range correction, ambiguity/range/staleness/implausible-correction filters. No hardware. |
| `ShooterTuning.java` **(new)** | Every new knob on the dashboard, defaults neutral, with `publishDefaults()` like `AutoTuning`. |
| `Shooter.java` | Targets refreshed and pushed every loop; distance table; trims; continuous feed gating; per-shot recovery; status telemetry; `kS` in both motor configs; SysId untouched. |
| `DriveSubsystem.java` | `getAimSolution()` / `getShotDistanceMeters()` built on `ShotMath`; `getAngleToHub()` delegates (identical at zero offset); `getShooterCameraAim()` publishes the newest tag residual from the shooter camera. |
| `RobotContainer.java` | Installs the live `ShotInputs` supplier; one `trackHubAndRange()` command that keeps aiming while ranging; distance measured from the muzzle; applies the shooter-camera residual to the aim when enabled. |
| `AutoAimAndShoot.java` | Same policy for autonomous: heading tracked in every phase, table mode skips ranging, muzzle distance. |
| `Constants.ShooterConstants` | Muzzle offset, facing offset, spin ratio, default table, nominal time of flight. |
| Tests | `ShotMathTest`, `ShotGateTest`, `ShooterCameraAimTest`, plus new cases in `ShooterTest`. |

### The new shot cycle

```
start()  ->  gate closed, feeder off
   loop:
     targets  = table(distance) * trim - motionComp        (or dashboard values)
     setpoints pushed to both SPARK MAX loops
     gate:    both wheels within 3% for 0.15 s
          and heading   <= 2 deg for 0.10 s
          and distance  <= 2 in (fixed-distance mode only)
          and |v| <= 0.35 m/s, |yaw| <= 45 deg/s   (unless the driver holds override)
     feeder runs while the gate is open; pauses the instant the wheel dips
     after each ball the wheel recovers and the gate reopens
```

### Dashboard keys added

`Shooter Tuning/` — `Use Distance Table`, `Table Trim`, `Back/Front Spin Ratio`,
`Speed Tolerance (%)`, `Speed Settle (s)`, `Aim Hysteresis (deg)`,
`Aim Settle (s)`, `Distance Hysteresis (in)`, `Max Fire Speed (m/s)`,
`Max Fire Yaw Rate (deg/s)`, `Motion Comp Gain (RPM per m/s)`,
`Heading Trim (deg)`, `Moving Shot Max Distance (m)`,
`Moving Shot Threshold (m/s)`, `Use Shooter Camera Aim`,
`Camera Aim Max Age (s)`, `Camera Aim Max Ambiguity`.

`Shooter Table/` — `1..5 Distance (m)` and `1..5 Front RPM`.

`Shooter/` (read-only) — `Ready To Fire`, `Feed Status`, `Speed Ready`, `Aimed`,
`At Distance`, `Motion OK`, `Front/Back RPM Error`, `Distance (m)`,
`Heading Error (deg)`, `Feed Bursts`, `Target Source`, `Table Valid`, `Table RPM`,
`Motion Comp (RPM)`, `Motion Gain (suggested)`, `Moving Shot Limit (m)`,
`Camera Aim Enabled`, `Camera Aim Used`, `Camera Aim Status`,
`Camera Aim Correction (deg)`, `Camera Range Correction (m)`.

---

## 5. Calibration, in order

Do these once, on the practice field, with the robot on a full battery. Each step
is independent; skip one and everything else still works.

### 5.1 Bench check: is the wheel actually reaching target? (5 min)

1. Put the robot on blocks, wheels free. Enable, hold the shoot button with the
   alignment override so the feeder runs.
2. Watch `Shooter/Front RPM Error` and `Shooter/Back RPM Error` at steady state.
3. **Pass:** both stay within about +/-30 RPM of zero.
   **If they settle low by 100+ RPM:** the feedforward needs `kS` (section 5.7).
   Raising `Speed Tolerance (%)` until it fires is a band-aid, not a fix.

### 5.2 Speed tolerance and settle (10 min)

1. Start at the defaults (3 %, 0.15 s).
2. Fire a controlled burst of 5 balls from ~2 m.
3. Watch `Shooter/Feed Status`. It should alternate
   `FEEDING -> SPINNING UP -> FEEDING` once per ball, not stay `FEEDING`.
4. If shots land short on balls 2-5, tighten `Speed Tolerance (%)` to 2.
   If the robot refuses to fire at all, loosen to 5 and check 5.1.

### 5.3 Distance table (20 min, the biggest win)

The table is what makes the robot score from wherever it ends up instead of
only from 100 in.

1. Mark distances on the floor: 1.0, 2.0, 3.0, 4.0, 5.0 m from the hub face.
2. Leave `Use Distance Table` **off**. At each mark, adjust
   `Front Shooter RPM Setpoint` until 3 of 3 shots go in - **from the same
   robot position each time**, and always from the same place on the robot
   (the muzzle should be at the measured distance).
3. Read the RPM off the dashboard and enter it in `Shooter Table/n Front RPM`,
   with the distance in `Shooter Table/n Distance (m)`. Distances must be
   ascending or `Shooter/Table Valid` goes false and the robot falls back to the
   fixed speed.
4. Turn `Use Distance Table` on. The robot now shoots from wherever it is: the
   teleop shot command stops driving to a fixed distance.
5. `Table Trim` (default 1.00) is the global correction. If every distance is
   5-15 cm short, try 1.02-1.05 instead of re-measuring the whole table (7415
   ended their season at 1.02 for exactly this reason).

Recommended: fill in the table at the distances you actually shoot from
(trench, outpost, hub front) before adding more points.

### 5.4 Heading trim (5 min)

1. From 2 m, aim and fire 5 balls. Note which side of the hub they miss.
2. `Heading Trim (deg)` rotates the commanded heading counter-clockwise as it
   increases. **Shots landing left of center -> lower the trim** by ~half the
   observed angular miss; shots landing right -> raise it.
3. Repeat once. +/-0.5 deg is usually enough. This is the cheap fix for a fixed
   camera/robot mounting bias that no amount of PID tuning can remove.

### 5.5 Muzzle offset (10 min, tape measure)

1. Measure the horizontal offset from the robot's rotation center to the center
   of the shooter exit, in robot axes (+x forward, +y left).
2. Enter it as `ShooterConstants.kMuzzleOffsetMeters`.
3. Verify: aim at the hub from 1.5 m. The shot should now go to the hub center
   rather than the center-plus-mounting-error point. Close shots improve the
   most.

### 5.6 Motion gate and shoot-on-the-move (15 min, optional)

1. Defaults: `Max Fire Speed 0.35 m/s`, `Max Fire Yaw Rate 45 deg/s`. The robot
   will not feed while driving faster than that unless the driver holds the
   alignment override. Tune to taste after 5.3.
2. To shoot while driving:
   * Set `Motion Comp Gain` to the value shown in `Shooter/Motion Gain
     (suggested)` (it is `dRPM/dDistance * time of flight`; with a typical
     400 RPM/m table and 0.45 s of flight that is about 180).
   * Raise `Max Fire Speed` to 1.0-1.5 m/s.
   * Verify 3 shots while driving toward the hub and 3 while driving away. If
     shots taken toward the hub land short *and* shots taken away land long,
     the gain is too high for this robot: the ball is not inheriting the full
     chassis velocity. Both sets landing long means the table itself is hot -
     fix §5.3 first.
   * The suggested value assumes **full** velocity inheritance (f = 1). Measure
     the real fraction and scale the gain by it (§9.7); the compensation is then
     a calibration, not a guess.
   * Keep moving shots inside `Moving Shot Max Distance (m)` until the gain has
     been measured. §9.5 explains the envelope and why a hood changes it.

### 5.7 Identified gains with SysId (15 min)

1. `Shooter Flywheels` -> quasistatic + dynamic, forward and reverse, from the
   SysId mechanism chooser.
2. In the analysis, record `kS` (volts), `kV` (V per rot/s) and `kA`.
3. Enter `kS` in `Shooter.kS` and the corrected `kV = kV_sysid / 60` (V per RPM)
   next to it. Same for the feeder: `FEEDER_kS`, `FEEDER_kV`.
4. Re-run 5.1: the steady-state error should collapse to a few RPM. This is
   what makes the speed tolerance meaningful at any RPM, not just at 1500.

---

## 6. Validation plan

1. `./gradlew test` — 12 new `ShotMath` tests, 9 new `ShotGate` tests, plus the
   added speed-band tests in `ShooterTest`. All are pure math; no robot needed.
   (They could not be executed in the sandbox this branch was written in — no
   JDK and no access to the FRC Maven. Please run them before deploying.)
2. Simulation: `./gradlew simulateJava`. Hold the shoot button; the robot should
   aim, range, then show `FEEDING`. Watch `Shooter/Feed Status` cycle between
   balls.
3. Bench: 5.1.
4. Practice field: 5.2, 5.3, 5.4, then 5.6 if wanted.
5. Match-like test: 10 cycles of "shoot 5, drive, shoot 5" from mixed distances
   and count makes. Compare against the previous code's number (`Feed Bursts` is
   a useful proxy for throughput).

---

## 7. What still needs hardware (and is worth it)

Code can make the mechanism repeat what it does; it cannot make it do something
mechanically impossible.

| Limit | Why software cannot fix it | Hardware fix |
| --- | --- | --- |
| Flywheel energy per ball | Two 1:1 NEOs dip a lot per ball; gating recovers accuracy but costs rate | Add inertia (steel disk / brass ring) or a third motor |
| Ball contact time ~30 ms | Not enough loops to control torque during contact (6328) | More inertia, or torque-mode control with a fast controller |
| Launch angle is fixed | RPM alone sets range; at one angle some distances are unreachable, and flight time (which is what limits a moving shot) cannot be shortened | Adjustable hood - §9.5 |
| No ball presence sensor | The feeder cannot meter one ball at a time; a burst may release 2-3 | Beam break at the shooter entry |
| Mechanical bias | Trim corrects a constant offset, not a varying one | Re-shim / re-mount the shooter |
| Wheel durometer, compression | Slip and spin vary with the game piece | Different wheels / compression |

The highest-value hardware item on this robot is not the shooter at all: a
beam break at the shooter throat plus one more flywheel disc would let the
software above run at full rate instead of waiting for recovery.

---

## 8. Rollback and safety notes

* `Use Distance Table` off (default) = the pre-existing fixed RPM behavior, with
  the new gating.
* `Heading Trim 0`, muzzle offset `(0,0)` = the pre-existing aim math exactly.
* `Motion Comp Gain 0` = no compensation.
* `Use Shooter Camera Aim` off (default) = the pose-based aim, byte for byte.
* `Moving Shot Max Distance 0` (default) = the motion gate as it was.
* `kS = 0` in both motor configs = the previous feedforward.
* The shooter only ever feeds when `systemEnabled`; `stopAll()` stops the
  flywheel, feeder, conveyor and shot cycle as before.
* SysId is untouched: when the system is disabled, `periodic()` does not write
  to the feeder, so the characterization voltage is not clobbered.
* If the robot will not fire and `Shooter/Feed Status` explains why, the driver
  can always hold the alignment override - but the flywheel speed gate still
  applies, deliberately.


---

## 9. The camera on the shooter: what it buys, and what a hood would have bought

### 9.1 Which camera it is

`VisionConstants` defines exactly two cameras, and only one of them is on the
shooter:

| Constant | Name on the network | Mount (robot frame, +x forward / +y left) | Looks |
| --- | --- | --- | --- |
| `kBackCameraName` | `FrontCamera` | 12.25 in back, 10.75 in **right**, 17.125 in up | yaw 180 = out of the **rear** |
| `kSideCameraName` | `GoodCamera_OldLeft` | 10 in back, 13 in right, 16.19 in up | yaw -90 = out of the **right** side |

The names are leftovers from an earlier robot (the "front" camera faces
backwards, the "left" camera faces right). The rear-facing unit is the one on
the top right of the shooter, looking out over the muzzle: it is the only one
whose line of sight is parallel to the shot. Everything below uses
`VisionConstants.kShooterCameraName`, which currently aliases it, so re-mounting
either the camera or the shooter only means updating one transform.

To confirm in 20 seconds: open the PhotonVision UI and cover each lens. The
camera that shows the hub tags while the robot is lined up to shoot is the one
this section is about.

### 9.2 What it was doing before this change

Nothing that touched the shot. Every accepted frame from either camera did
exactly one thing:

```
estimateCoprocMultiTagPose / estimateClosestToReferencePose
      -> outlier gate vs sampleAt(timestamp)
      -> innovation limit
      -> m_poseEstimator.addVisionMeasurement(...)
```

The aim itself was pure odometry: `getPose()` -> `ShotMath.solveAim()` ->
`getAngleToHub()`. Raw per-tag geometry (`getYaw`, `getPitch`, target area,
`getBestCameraToTarget`) was never read. So the camera improved the *inputs* to
the aim, filtered and delayed through the estimator, and never measured the aim
itself.

### 9.3 What it can measure that odometry cannot

The camera bolted to the shooter is the only sensor on the robot that measures
the angle to the target **from the frame the ball leaves from**. That matters
because the pose-based aim inherits every pose error, and the pose estimator is
deliberately smoothed:

| Error source | Pose-based aim | Shooter camera residual |
| --- | --- | --- |
| Gyro drift / wheel slip after a hit | full error | measured and removed |
| Fusion lag while turning (45 deg/s, ~100 ms) | ~4.5 deg | ~1.8 deg at 40 ms frame age |
| Unmeasured muzzle offset (`kMuzzleOffsetMeters` is still `(0,0)`) | full error | measured at the camera |
| Multi-tag solve needs 2+ tags to be accurate | fails with 1 tag | works with 1 tag |
| Ball inherits robot velocity, wheel slip, launch angle | invisible | invisible |

The first four are aim errors, and they are the ones that turn a well-tuned
table into a miss. 2 deg at 4 m is 14 cm; the hub throat does not have 14 cm of
margin. The last row is not an aim error, and no camera will ever see it - see
9.5.

### 9.4 What changed in code

`ShooterCameraAim.java` **(new, pure)** turns one frame into a *residual*, not a
second pose:

1. Project each detection from the camera frame into the field, using the pose
   the robot is currently driving on and the measured mount transform.
2. Compare it with the tag's pose from the official field layout.
3. Report the angular and radial difference, averaged over the acceptable tags
   in the frame.

That residual is exactly the amount by which the pose-based aim is wrong, so it
is *added* to the aim heading and the distance instead of replacing them, inside
`DriveSubsystem.getAimSolution()`. That one change point is deliberate: the
drive command, the shot gate, the PathPlanner rotation override and the muzzle
distance all read `getAimSolution()`, so they can never disagree about where the
robot is pointing. It also closes the loop physically - the drivetrain turns
until the tag is actually centred, not until the pose thinks it is.

Nothing in the pose estimator is touched, so a bad camera frame can never move
the robot's pose or the drivetrain's idea of where it is - the worst it can do is
be rejected.

Filters, all on the dashboard (`Shooter Tuning/`):

| Knob | Default | Rejects |
| --- | --- | --- |
| `Use Shooter Camera Aim` | **off** | the whole feature, when off |
| `Camera Aim Max Age (s)` | 0.08 | frames older than the latency budget |
| `Camera Aim Max Ambiguity` | 0.35 | single-tag solves too ambiguous to trust |
| - (`VisionConstants`) | 0.75-7.5 m | tags too close or too far to aim with |
| - (`VisionConstants`) | 8 deg / 0.75 m | corrections too large to be aim error |

The last row is the safety net: a wrong tag or a broken pose produces a
correction no physical aim error could produce, and the frame is dropped rather
than yanking the shot sideways. `Correction.reason()` reports which filter fired
(`STALE FRAME`, `AMBIGUOUS`, `TOO FAR`, `CORRECTION TOO LARGE`, `NO TARGET`,
`NO POSE`) on `Vision/ShooterCam Status`.

New read-only keys: `Vision/ShooterCam Aim Valid`, `Vision/ShooterCam Status`,
`Vision/ShooterCam Aim Error (deg)`, `Vision/ShooterCam Range Error (m)`,
`Vision/ShooterCam Frame Age (s)`, `Vision/ShooterCam Tags Used`,
`Shooter/Camera Aim Enabled`, `Shooter/Camera Aim Used`,
`Shooter/Camera Aim Correction (deg)`, `Shooter/Camera Range Correction (m)`.

**The residual is also a mount diagnostic.** Power the robot up, aim at the hub
and watch `Vision/ShooterCam Aim Error (deg)`:

* a constant few degrees at every range and heading = the mount yaw in
  `kRobotToBackCamera` is wrong (or the camera is not parallel to the shot);
* a residual that grows as the robot gets closer, at roughly `1/range` = the
  mount X/Y in `kRobotToBackCamera` is wrong;
* a residual that jumps around per tag = detection noise or vibration - see 9.6.

Either of the first two is worth fixing: the pose estimator is using the same
transform, so a wrong mount is quietly biasing every vision measurement too.

### 9.5 Shooting on the move without a hood: what is actually possible

Start with the honest mechanics. With a fixed launch angle and a distance->RPM
table, the launcher has exactly **one** control: wheel speed. Angle, spin and
ball contact are fixed by the mechanism. A hood adds a second control: it can
trade launch speed for launch angle at a given range, and the reason that
matters is **flight time**.

Everything bad about a moving shot scales with flight time:

* the fraction `f` of the chassis velocity the ball carries becomes a lateral
  miss of `f * v * t_flight` - 1 m/s cross-track over 0.5 s is **50 cm** at
  `f = 1`, and nothing at all at `f = 0`;
* the aim error from a stale heading grows as `yaw_rate * latency`, and the
  heading you commit to has to stay correct for the whole flight;
* the ball's own deviation from any unmodelled effect (slip, contact, spin)
  scales with time in the air.

At 4 m with a fixed angle, the flight time is whatever the table produces -
typically 0.4-0.6 s. A hood does not remove any of those terms either; it lets
you fire flatter and faster, which shortens the flight time and shrinks all
three at once.
That is the real value of a hood, and it is why "no hood" caps the *range* at
which moving shots are worth taking, not whether they are possible.

There are exactly two things to compensate on a moving shot, and one of them a
camera can measure:

1. **Aim at release.** `ShotGate` already refuses to fire unless the heading is
   within tolerance for a settle time. The camera is what makes that tolerance
   real instead of "within 2 deg of what the drifted pose believes". This is the
   half the shooter camera fixes, and it is why the feature and the gate belong
   together.
2. **Exit velocity inheritance.** When the robot is moving, the ball leaves with
   `v_exit(robot) + f * v_robot` in the field frame. `f` is somewhere between 0
   (the ball is launched in the robot frame and the robot slides under it) and 1
   (the ball leaves with the full chassis velocity). `ShotMath.motionCompensatedRpm`
   currently assumes **f = 1**: it lowers the commanded RPM when closing so the
   ball does not fly the extra `v * t_flight` metres. If the real `f` is 0.5, the
   correct gain is half of `Shooter/Motion Gain (suggested)`, and the current
   default over-compensates by the same factor.

   **A camera cannot measure `f`.** It is a property of the ball and the wheels,
   not of the geometry. Measure it once (10 min, 9.7) and put the answer into
   `Motion Comp Gain`; after that both compensation terms are calibrated.

So: shoot-on-the-fly without a hood is possible, but it is an **envelope**, not
a mode. Inside the envelope the shot is as good as a stationary one; outside it,
no amount of software recovers it. The envelope is:

* **short range** - the distance where the fixed-angle flight time is still
  short. `Moving Shot Max Distance (m)` enforces this in `ShotGate`
  (`0` = off, the default). Start at 3.5 m and widen only with data.
* **tight heading window** - keep `Max Fire Yaw Rate` at 45 deg/s or less.
  Turning is worse than translating, because it rotates the aim during the
  flight, and the gate's yaw term is the only thing watching it.
* **RPM at the table value for the range at release** - with `Motion Comp Gain`
  calibrated to the measured `f`, not the suggested full-inheritance value.
* **one shot per settled heading** - the feeder cannot recover the wheel between
  balls while the heading is still slewing.

| Shot | Envelope | Why |
| --- | --- | --- |
| Stationary, any table distance | unchanged | 0 m/s: neither compensation term applies |
| Moving at <= 3.5 m | allowed with the gate open | short flight, small `v * t` |
| Moving at 3.5-5 m | only with `f` calibrated and tested | flight time grows, `v * t` grows |
| Moving beyond 5 m | not worth it without a hood | flight time is the dominant error |

### 9.6 Vibration: the one thing to check before trusting this

The camera shares a structure with two spinning flywheels. A tag that is
smearing across pixels reads as a slightly wrong angle, and the residual has no
way to know. Before enabling `Use Shooter Camera Aim`, compare
`Vision/ShooterCam Aim Error (deg)` with the wheels stopped and at full speed:

* stable to ~0.3 deg in both cases: good, enable it;
* noisy or biased only while spinning: mount the camera on a compliant bracket,
  or leave the feature off and accept the pose-based aim. This is the one
  failure mode where the camera is *worse* than odometry, because it is
  confidently wrong.

### 9.7 Measure `f` (the velocity inheritance fraction), 10 min

1. Calibrate the table (§5.3) and the heading (§5.4) first.
2. `Motion Comp Gain = 0`. `Use Distance Table = on`.
3. Standing still at 3 m: fire 5 balls, note where they land. This is the
   reference group.
4. Strafing sideways at 1.0 m/s, launcher still pointed at the hub (the
   drivetrain's `getRotationTargetOverride()` holds the heading): fire 5 balls.
   Measure the mean lateral miss `d` at the hub. Then
   `f = d / (v * t_flight)`, with `t_flight` from the table's nominal 0.5 s.
5. Repeat at 1.5 m/s. `f` should come out the same; if it does not, the miss is
   dominated by something else (wheel recovery, heading lag) and moving shots
   should stay inside the short-range envelope until that is fixed.
6. Set `Motion Comp Gain = f * Shooter/Motion Gain (suggested)`.
7. Re-run step 4. The lateral miss should be small enough to ignore.

Sign check when reading the results: if shots taken **toward** the hub land
short *and* shots taken **away** from it land long, the gain is too high for the
real `f` - lower it. That is the signature of over-compensating, and it is the
most likely first result, because the suggested value assumes `f = 1`.

### 9.8 What this does not fix

* **The hood.** No camera changes the launch angle. `§7` still stands: at one
  angle, some distances are unreachable and the flight time is whatever the
  table says.
* **Wheel recovery.** The camera knows the aim is good; it has no idea the wheel
  dipped 300 RPM when the last ball left. `ShotGate`'s speed gate is still the
  only defence, and inertia is still the hardware fix.
* **Ball metering.** Without a beam break the feeder cannot promise one ball per
  burst; a camera pointed at the hub cannot see the throat.
* **Spin and compression.** Exit velocity and spin come from the wheel surface,
  not from the target geometry.


---

## Sources

* 7415 2026 technical binder - interpolation table, 1.02 trim, shoot-on-move:
  <https://2026.team7415.com/>
* 6391 2026 robot - distance/hood/TOF tables, 3x iteration, heading offset:
  <https://github.com/6391-Ursuline-Bearbotics/2026-6391-Rebuilt>
* 5024 2026 robot - distance table and `kS/kV/kA`:
  <https://github.com/frc5024/Rebuilt>
* 6328 2026 build thread - flywheel phases, per-ball recovery, inertia:
  <https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595>
* 3464 PR "Prevent stale flywheel readiness":
  <https://github.com/FRC-Team-3464/2026-code/pull/11>
* SubZero PR "Shooting and shoot-on-the-move fixes":
  <https://github.com/SubZero-Robotics/2026-CommandRobot/pull/32>
* 6908 2026 robot - distance-based speed and table calibration notes:
  <https://github.com/frc6908/2026-Robot>
* "Programming a hooded DECODE shooter" - velocity control needs an integrator;
  feed when aimed and within tolerance:
  <https://medium.com/@vikramaditya.nishant/programming-a-hooded-decode-shooter-463d6a71832f>
* YETI shooter design guide - distance tables, shot logging, feed gating:
  <https://wiki.yetirobotics.org/books/design-process/page/shooter-designs>
* Swartdogs 2026 `Flywheel` - REVLib 2026 `kS/kV/kA` configuration:
  <https://github.com/Swartdogs/RebuiltCode/blob/main/src/main/java/frc/robot/subsystems/shooter/Flywheel.java>
* Internal: `PATH_ACCURACY_AUDIT.md` (12x kV unit bug, prior flywheel stall),
  `SIMULATION.md` (flywheel physics calibration).
