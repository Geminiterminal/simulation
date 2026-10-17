# Robot Simulation Guide

This project now includes a full physics simulation of the robot: drivetrain,
shooter, intake, cameras, and battery. It follows the WPILib *Physics
Simulation* model combined with each vendor's official simulation support:

| Layer | Library | What it emulates |
|-------|---------|------------------|
| Swerve module motors | WPILib `DCMotorSim` + REVLib `SparkMaxSim` | NEO motor physics (voltage/current/speed), SPARK MAX closed loops (velocity P+FF, position PID with wrap), encoder feedback, current limiting |
| Chassis | WPILib kinematics integration | Ground-truth pose from measured wheel states; simulated gyro replaces the navX |
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
- **The heading** is re-seeded to the alliance default (180° on blue, 0° on
  red, i.e. facing your own alliance wall) and vision re-establishes the
  heading from the new spot on the field.
- **The hub** follows the alliance, so auto-aim and the auto-distance
  controller shoot at the right hub.

The simulated driver station is kept in sync with the robot, so
`DriverStation.getAlliance()` reports the same alliance the robot code is
using. On the real robot the alliance comes from the FMS and cannot be
changed - the toggle and the chooser are simulation-only.

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

`./gradlew test` runs `SimulationPhysicsTest`, which sanity-checks the physics
calibration (drive free speed ≈ 4.46 m/s vs 4.8 m/s cap, spin-up times,
flywheel settling, arm travel range). If a phyiscal change breaks the
calibration the tests flag it.

## Known limitations (and upgrade path)

- **No tire slip / skidding**: wheel-ground traction is ideal. For full
  rigid-body physics (drift, collision, fuel game pieces) the next step is the
  [MapleSim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Sim)
  vendordep — our `SimulationConstants` map directly onto its
  `DriveTrainSimulationConfig`.
- Rotational inertia of the chassis emerges from the per-corner mass model
  rather than the MOI constant directly (rotational response may be ~10–20%
  softer than the real robot).
- The simulated gyro is noise-free and never drifts — real navX drift won't be
  reproduced.
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
