# Robot Simulation Guide

This project now includes a full physics simulation of the robot: drivetrain,
shooter, intake, cameras, and battery. It follows the WPILib *Physics
Simulation* model combined with each vendor's official simulation support:

| Layer | Library | What it emulates |
|-------|---------|------------------|
| Swerve module motors | WPILib `DCMotorSim` + REVLib `SparkMaxSim` | NEO drive and NEO 550 steer motor physics (voltage/current/speed), SPARK MAX closed loops (velocity P+FF, position PID with wrap), encoder feedback, current limiting |
| Chassis | WPILib kinematics integration | Kinematic field pose integrated from encoder-derived measured wheel states; a `SimulatedGyro` sensor model (bias + noise) replaces the navX. This pose is not independent of the drive encoder scale. |
| Shooter flywheels | WPILib `FlywheelSim` + REVLib `SparkMaxSim` | Flywheel MOI, spin-up time, RPM feedback |
| Feeder & intake roller | WPILib `DCMotorSim` + Phoenix 6 `TalonFXSimState` | Kraken X60 physics; TalonFX closed loops run in Phoenix's own sim |
| Intake deploy arm | WPILib `SingleJointedArmSim` + REVLib `SparkFlexSim` | Arm gravity, gearing, MAXMotion profiled moves, `Mechanism2d` widget |
| Conveyor | WPILib `DCMotorSim` + REVLib `SparkMaxSim` | Drum physics, velocity PID |
| Cameras | PhotonVision `VisionSystemSim` | Both cameras with calibrated intrinsics/latency; publishes synthetic AprilTag frames your existing fusion code consumes |
| Battery | WPILib `BatterySim` + `RoboRioSim` | Voltage sag proportional to total simulated current draw |

Everything is gated behind `RobotBase.isSimulation()` – deploying to the
roboRIO is completely unaffected.

## Running the simulation

1. Open the project in WPILib VS Code.
2. Press `Ctrl+Shift+P` → **WPILib: Simulate Robot Code** → select
   **Use Sim GUI** (the default).
3. In the sim GUI, pick teleop or autonomous in the **Driver Station** tab.

What you should see:

- **Field** widget (`SmartDashboard → Field`): ground-truth robot pose plus a
  `Pose Estimator` ghost so you can watch your pose estimator converge.
- **Vision Sim Field**: AprilTags, camera FOV wedges, and which tags the
  simulated cameras currently “see”.
- **Intake Arm** (`Mechanism2d`): deploy/stow position as the MAXMotion
  profiles run.
- All your normal `SmartDashboard` values (RPMs, currents, distances) update, as
  does the battery-voltage log entry.
- PathPlanner autos run for real: module physics produce the commanded motion,
  and `resetOdometry()` re-spawns the simulated robot at the path's start pose.
- Auto-aim/heading-snap work against the hub coordinates and the simulated
  pose estimator, and camera sim feeds AprilTag corrections.

Driver controls: you can map a real Xbox controller / Thrustmaster in the sim
GUI’s Joysticks tab, or drag keyboard joysticks.

## Switching alliances in the simulation

The alliance can be changed at any time while the sim is running, so a
red-alliance match can be rehearsed without touching the code or re-authoring
any paths.

Two ways to do it:

| Control | Where |
|---------|-------|
| `Alliance` chooser | Glass (or any dashboard that renders `SendableChooser`s) - simulation only |
| Toggle button | Thrustmaster **button 12**, Xbox **Y** button - simulation only |

`Alliance Status` on the dashboard always shows the alliance the robot is
currently on. The button is the quickest way to flip sides mid-session; the
chooser is an explicit alternative for when you are driving from the dashboard
rather than from the controller.

What changes when the alliance switches:

- **Paths**: PathPlanner flips every blue-authored path for the red alliance
  (the origin stays on the blue side), so the same `.auto` file runs on both
  sides.
- **The robot is moved** to the matching spot on the other side of the field.
  The 2026 field is rotationally symmetric, so the robot is rotated 180°
  about the field center: `x → 16.54 − x`, `y → 8.07 − y`, `heading → heading
  − 180°`. This is exactly the transform PathPlanner uses when it flips a
  path, so the robot lands where the flipped path starts. Set
  `Mirror Robot On Alliance Change` to `false` on the dashboard to keep the
  robot where it is and only switch the alliance logic.
- **The heading** is set to the alliance default (180° on blue, 0° on red,
  facing your alliance wall) when the robot is mirrored. With mirroring
  disabled, its current pose and heading stay put; vision continues correcting
  from that prior.
- **The hub** follows the alliance, so auto-aim and the auto-distance
  controller shoot at the right hub.

The simulated driver station is kept in sync with the robot, so
`DriverStation.getAlliance()` reports the same alliance the robot code is
using. On the real robot the alliance comes from the FMS and cannot be
changed - the toggle and the chooser are simulation-only.

## Heading and pose estimation

Heading (and full pose) estimation fuses every available sensor through one
pipeline:

| Sensor | What it contributes | Where |
|---|---|---|
| Gyroscope (navX MXP) | Latency-free rotation between vision frames; the estimator trusts its angle deltas over wheel rotation | `getGyroRotation()` every loop → `SwerveDrivePoseEstimator.update` |
| Wheel odometry (4 swerve modules) | Translation between vision frames | `MAXSwerveModule.getPosition()` → the same estimator update |
| Vision (2 PhotonVision cameras) | Absolute field pose (translation AND heading), timestamped at image capture | `getAllUnreadResults()` → `VisionFusion` gates → `addVisionMeasurement` |
| PathPlanner start pose | Authoritative full pose (translation AND rotation) at auto start | `resetOdometry` |
| Gyro → vision feedback | The estimated field heading is fed back to the camera pipelines (`addHeadingData`) every loop so heading-aware PhotonVision solvers can use it | `addPhotonVisionMeasurement` |

How the fusion stays accurate, stable and smooth:

- **One capture, one measurement, in time order.** Both camera queues are
  drained once per loop and their frames are sorted by capture timestamp before
  fusion. The estimator latency-compensates each measurement to capture time;
  per-camera de-duplication prevents repeated frames, and a global monotonic
  timestamp guard drops late frames that would otherwise rewind the estimator
  after the other camera has already advanced it.
- **Trust scales with quality.** `VisionFusion.measurementStdDevs` turns
  ambiguity, camera-to-tag distance and the number of tags used into the
  standard deviations handed to the estimator. Multi-tag solutions provide
  the strongest heading; single-tag heading is deliberately downweighted
  because its PNP orientation can change hypotheses as targets appear and
  disappear. Ambiguous frames (ambiguity > 0.2) and degenerate near-field
  solves (< 0.22 m) are dropped before any pose math runs.
- **Outliers are rejected, drift is corrected.** Position and heading
  localization are tracked independently; once each component settles, a
  residual over 1.5 m / 45° is treated as a glitch. In-window measurements
  fuse with quality-scaled Kalman gains.
- **Vision reacquisition never resets the estimated pose.** Vision translation
  and heading converge through timestamped estimator updates, with each frame's
  innovation capped at 0.20 m and 2°. The heading therefore stays continuous
  while localization converges; single-tag heading is additionally weighted
  very lightly. PathPlanner's authored start pose is marked authoritative, so
  the first camera frame after an odometry reset cannot overwrite it.
- **Simulation starts from one heading truth.** The simulated ground-truth pose
  is initialized to the alliance default before the camera simulation starts;
  this prevents the first synthetic AprilTag frame from appearing to be a
  180° correction against the estimator.
- **Odometry resets honor the full authored pose.** `resetOdometry(pose)` is
  "the robot is now at this pose": translation AND rotation, in the estimator
  and (in simulation) in the ground truth. WPILib's odometry offsets the gyro
  internally, so a reset heading never needs to match the raw gyro reading.
  An earlier workaround kept the previous heading across resets; at auto start
  that left the robot disagreeing with the path's authored start rotation, so
  PathPlanner saw a huge rotation error and spun the robot. The root cause was
  the reset never establishing one consistent heading truth - not the
  controller gains.

Useful dashboard signals while diagnosing heading:
`Sim/Pose Estimate Error Heading (deg)` (estimate vs. ground truth),
`Field Heading Source`, `Vision Active`, `Vision/Outliers Rejected`, and
`Gyro Rotation Used`.

### The simulated gyro is a sensor, not the truth

In simulation the gyro is a `SimulatedGyro` model: the true yaw rate plus a
small zero-rate bias (0.02°/s ≈ 1.2°/min) and rate noise, integrated over
time (see `SimulationConstants`). It drifts like a real navX, so the vision
fusion gets to prove it rejects that drift. Watch
`Sim/Pose Estimate Error Heading (deg)` in the sim GUI: it grows slowly while
no tags are visible and converges back within a couple of frames as soon as a
camera sees tags again. Resetting the simulated robot (auto start, alliance
change) does not snap the gyro - the estimator's gyro offset absorbs the
difference, exactly like when the real robot is carried to a new starting
spot.

## Autonomous path tuning in Elastic Dashboard

The robot publishes its SmartDashboard/NetworkTables entries, which Elastic
Dashboard can display and edit. In Elastic, add a NetworkTables widget (or use
the data browser) and select the entries under `SmartDashboard/Auto Tuning`.
The path-following gains and auto-aim values are read live; edits take effect
while a command is running. Pathfind constraints are read when the next
pathfind command is created. No source edit or redeploy is required. Keep the
robot disabled or run in simulation while experimenting, and change one value
at a time.

### Values to tune

| Elastic entry | What it changes | Practical tuning approach |
|---|---|---|
| `Path Translation kP` | X/Y pose-error feedback in m/s per meter of error | Increase gradually if the robot trails the target; reduce if it oscillates or overshoots. |
| `Path Translation kI` | Removes persistent translation bias | Leave at 0 initially. Add only a small amount if a steady error remains after P tuning. |
| `Path Translation kD` | Damps translation error changes | Usually leave at 0 unless repeated overshoot remains; simulation encoders are clean, real sensors are noisier. |
| `Path Rotation kP` | Heading-error feedback in rad/s per radian of error | Increase if heading lags; reduce if it swings past the target. |
| `Path Rotation kI`, `Path Rotation kD` | Integral/derivative heading correction | Start at 0; tune only after P. |
| `Max Module Speed (mps)` | Runtime swerve-wheel speed desaturation limit | Keep at or below the measured/mechanically safe wheel limit. Lower it to reduce overall aggressiveness. |
| `Pathfind Max Speed (mps)`, `Pathfind Max Accel (mps2)` | Constraints for the trench pathfind commands | Reduce for smoother approaches; increase cautiously if motion is consistently too slow. Applies to newly scheduled pathfinds. |
| `Pathfind Max Angular Speed (degps)`, `Pathfind Max Angular Accel (degps2)` | Angular constraints for those pathfind commands | Lower if turns are abrupt; increase only if the robot can track the resulting rotation. |
| `Aim Heading kP`, `Aim Heading Tolerance (deg)` | Hub-facing auto-aim response and finish threshold | Adjust only when tuning auto-aim; avoid changing while evaluating path-following. |
| `Aim Distance kP`, `Aim Distance Max Speed (mps)`, `Aim Distance Tolerance (in)` | Auto-drive-to-shoot-distance behavior | Raise P for quicker approach, lower max speed for gentler approach; set tolerance to the acceptable shooting range. |
| `Auto Aim Target Distance (in)` | Desired distance from robot to hub | Set to the geometry required by the shot. |

The path PID defaults are 5.0 (translation) and 5.0 (rotation), with I and D
at zero, evaluated directly on each trajectory step with zero pose-lag against
`PathPlannerTrajectoryState` while reading live Elastic values on every loop.
Constraints entered in Elastic override the runtime pathfind commands and the
module desaturation limit (~4.46 m/s NEO free speed). **Normal PathPlanner autos
use constraints embedded in each `.path`/`.auto` asset**, so change those route
limits in the PathPlanner GUI and re-export when the authored trajectory itself
needs a new speed or acceleration profile. The dashboard `Max Module Speed`
still caps wheel outputs for those autos.

### Useful live telemetry

Use Elastic line graphs for these signals (and retain the NetworkTables/WPILib
DataLog for run-to-run comparison):

- `Auto/Target Pose X (m)`, `Auto/Target Pose Y (m)`, `Auto/Target Heading (deg)`
  against `Robot X (m)`, `Robot Y (m)`, and `Robot Heading (deg)`. `Auto/Translation
  Error X/Y` and `Auto/Heading Error` show signed tracking error directly.
- `Auto/Commanded Vx/Vy/Omega` against `Auto/Measured Vx/Vy/Omega`. Large or
  sustained command/measurement gaps indicate saturation, a speed constraint,
  or actuator response rather than a path geometry problem.
- In simulation, `Sim/Ground Truth X/Y/Heading` is integrated from the measured
  module encoder states. `Sim/Pose Estimate Error X/Y/Heading` can reveal
  gyro/vision/estimator disagreement, but it is **not an independent check of
  wheel scale or travel** because the pose streams share encoder-derived module
  motion. Compare `Robot X/Y` and the final pose to the PathPlanner target and
  the authored path distance as well. Inspect `Vision Active` and
  `Field Heading Source` before blaming a controller gain for an incorrect pose.
- For shoot-approach tuning, watch `Auto Distance Error (m)`, `Auto Distance
  Speed`, `Distance to Hub (m)`, and `Auto Aim Error (deg)`. Check battery voltage
  and module current when motion is unexpectedly slow.

### Repeatable trial-and-error loop

1. Start with a short, representative path at low speed. Confirm the field
   heading, start pose, alliance mirroring, and pose estimate are correct. For a
   repeatable axis check, run `x changes` and then `xy` from their authored start
   poses; compare final X/Y with the target, not only with the estimator.
2. Compare target pose to estimated pose and `Sim/Ground Truth`. If
   estimate-to-truth is poor, fix gyro, vision, or estimator issues first. If
   both poses agree but both miss the target by the same scale, inspect wheel
   conversion and drive/steer response: these pose streams currently share
   encoder-derived module motion. PID tuning cannot compensate for bad units.
3. Increase only translation P (or rotation P for heading error) in small steps
   until lag improves; back off if oscillation/overshoot appears. Keep I and D
   at zero at first.
4. If commanded speed is high but measured speed stays low, reduce path speed or
   acceleration and check module/battery telemetry before raising feedback gain.
5. Repeat the exact path and starting conditions, then compare peak and RMS
   translation/heading error, settling time, and end pose. Save known-good
   values outside the live dashboard before another trial.

See [`PATH_ACCURACY_AUDIT.md`](PATH_ACCURACY_AUDIT.md) for the confirmed REVLib
simulation-unit defect, corrective code changes, a focused validation sequence,
and a hardware-deployment checklist.

The dashboard entries are bounded in code to avoid accidental extreme values.
Dashboard edits are runtime tuning only; they do not update PathPlanner asset
files or replace physical system identification.

## Calibration data

All physics constants live in `Constants.SimulationConstants`; camera metadata
in `Constants.VisionConstants`. Sources of each value:

| Constant | Value | Source |
|----------|-------|--------|
| `kRobotMassKg` | 65.0 kg | PathPlanner GUI robot config (robot + bumpers + battery) |
| `kRobotMOIKgMetersSquared` | 5.7 kg·m² | PathPlanner GUI robot config |
| `kWheelRadiusMeters` | 0.0381 m | Measured 3" wheels (`ModuleConstants`) |
| `kDriveGearRatio` | 5.077 : 1 | MAXSwerve 13T pinion (`ModuleConstants`) |
| `kTurnGearRatio` | 9424/203 ≈ 46.42 : 1 | REV MAXSwerve steering spec |
| `kDriveWheelInertiaKgMetersSquared` | ≈ 0.0236 | (m/4)·r² per corner + wheel spin inertia |
| `kSteerInertiaKgMetersSquared` | 0.004 | Typical MAXSwerve azimuth estimate |
| Shooter flywheel MOI | 0.0025 kg·m² | Estimated 4" wheel + shaft; **re-identify with SysId** |
| Feeder / roller / conveyor MOI | 0.001 / 0.0008 / 0.002 | Drum estimates (1:1 confirmed by your FF gains ≈ 1/free-speed) |
| `kDeployArmGearRatio` | 212.4 : 1 | Calibrated so 59 motor rotations = 100° arm travel (matches `DEPLOY_POSITION = 59`) |
| Deploy arm length/mass | 0.30 m / 3.0 kg | Estimate — measure the arm |
| Cameras | 1280×720, 75° diag FOV, 30 fps, 30±5 ms latency | Typical OV9281-class cam; **re-calibrate with PhotonVision cal tool and paste in numbers** |

### CAD / mechanical measurements

The physics constants below are only as good as the measured robot. Two prompt packs turn the robot
into CAD: [`docs/CAD_MECHANISM_PROMPTS.md`](docs/CAD_MECHANISM_PROMPTS.md) has eight self-contained
text-to-CAD prompts, one per mechanism, each carrying its own context and the transform that places it
in the robot (assemble those for a packaging model); [`docs/ROBOT_CAD_PROMPT.md`](docs/ROBOT_CAD_PROMPT.md)
has the per-part prompts for the pieces that actually get cut. If the robot is not accessible for measurement, [`docs/CAD_REFERENCE_ROBOT.md`](docs/CAD_REFERENCE_ROBOT.md) gives a published reference robot to adapt and a reference value for every otherwise-unmeasurable dimension. The datum drawing
([`docs/cad/robot-datums.png`](docs/cad/robot-datums.png)), the nominal mechanism layout
([`docs/cad/robot-parameters.json`](docs/cad/robot-parameters.json)) and the tape-measure sheet
([`docs/cad/measurements.csv`](docs/cad/measurements.csv)) turn the values in this table from estimates
into measurements.

### Re-calibrating

- **Drivetrain / flywheel MOIs**: run
  [SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
  routines on the real robot, then plug identified kV/kA into the inertia
  estimates (J ≈ kA relation in `LinearSystemId`).
- **Wheel radius**: measure actual odometry distance over a known tape
  distance; adjust `ModuleConstants.kWheelDiameterMeters` (robot constant, it
  feeds both sim and real).
- **Deploy arm gearing**: jog the arm, record motor rotations to one full arm
  revolution; G = rotations·(360/measured degrees)·…, i.e.
  `motorRotations × 360 / degreesMoved`. Update `kDeployArmGearRatio`.
- **Cameras**: calibrate intrinsics in PhotonVision, then copy resolution, FOV,
  calib error, FPS and latency into `VisionConstants`.

## Unit tests

`./gradlew test` runs, without HAL or vendor libraries:

- `SimulationPhysicsTest` - sanity-checks the physics calibration (drive free
  speed ≈ 4.46 m/s matching the SPARK MAX + NEO cap, spin-up times, flywheel
  settling, arm travel range). If a physical change breaks the calibration the
  tests flag it.
- `VisionFusionTest`, `SimulatedGyroTest` - pin the vision measurement policy
  (de-duplication, quality gates, outlier rejection, trust scaling) and the
  simulated gyro sensor model.
- `HeadingFusionSimulationTest` - a closed-loop heading/pose-estimation
  simulation (drifting gyro + noisy latency-stamped vision through the same
  pipeline the robot runs): heading accuracy through arcs and in-place turns,
  gyro-drift recovery after vision outages, single-tag flicker, heading-safe
  first localization, jump-free updates, and duplicate-frame handling.

The same heading scenarios are also mirrored in pure Python
(`tests/test_heading_estimation.py`), together with the auto-start reset
regression ("the authored start rotation is authoritative"). Run them without
WPILib via `python3 -m unittest discover -s tests`; they also check that the
Python mirror's constants still match `VisionFusion.java`.

## Known limitations (and upgrade path)

- **No tire slip / skidding**: wheel-ground traction is ideal. For full
  rigid-body physics (drift, collision, fuel game pieces) the next step is the
  [MapleSim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Sim)
  vendordep — our `SimulationConstants` map directly onto its
  `DriveTrainSimulationConfig`.
- Rotational inertia of the chassis emerges from the per-corner mass model
  rather than the MOI constant directly (rotational response may be ~10–20%
  softer than the real robot).
- The simulated gyro has bias and rate noise but no axis cross-coupling, no
  temperature drift and no latency; real navX behavior is close but not
  identical (tune `SimulationConstants.kSimGyro*` to bracket it).
- Game-piece (fuel) pickup/scoring is not simulated.

## Troubleshooting signs/units

- **Module turns the wrong way / swerves fight each other**: flip the sign of
  the velocity passed to `m_turningSparkSim.iterate(...)` in
  `MAXSwerveModule.simulationPeriodic` (REV encoder `inverted(true)` versions
  can differ).
- **Back shooter wheel never reaches speed**: apply the back-motor inversion
  in the sim by feeding `-m_backPhysics.getAngularVelocityRPM()`.
- **Arm hangs instead of stowing**: expected — with no command the MAXMotion
  loop is idle and gravity wins, same as hardware in brake-less conditions.
  Schedule a stow on robot boot if you want it held.

## Startup errors and loop overruns

- PathPlanner assets must live in `src/main/deploy/pathplanner` (two n's),
  including `settings.json`, `autos/*.auto`, and `paths/*.path`. File names
  are case-sensitive on Linux. Both `blue left trench.path` and
  `blue right trench.path` are bundled for teleop trench pathfinding.
- Joystick axis/button/POV warnings mean the selected control profile is reading
  inputs absent from DS port 0. Map a controller (or a suitably configured
  simulated joystick) to that port and select the matching driver profile.
  They are separate from the PathPlanner startup failure.
- Camera raw/processed video and wireframe rendering are disabled to reduce
  desktop CPU load; synthetic AprilTag measurements and the debug field remain
  enabled. Pose telemetry remains on SmartDashboard instead of printing every
  second. The 20 ms loop deadline and overrun diagnostics remain unchanged.
  If overruns persist after warmup, run without the Java debugger and profile
  the simulation before changing physics or camera update rates.
- Check deploy asset names/references without WPILib using
  `python3 -m unittest discover -s tests`. Run `./gradlew test` in a WPILib Java
  environment for the Java/physics tests.
