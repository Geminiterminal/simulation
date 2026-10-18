# Autonomous path accuracy audit

## Executive finding

The strongest directly evidenced cause of the simulated robot falling short in **both X and Y** is a motor-simulation unit error in `MAXSwerveModule.simulationPeriodic()`. The WPILib motor plants produce mechanism-side angular velocity, but the code passed wheel **m/s** to `SparkMaxSim.iterate()`, which expects motor-shaft **RPM**. REVLib then applies the configured encoder conversion factor a second time. `DriveSubsystem.simulationPeriodic()` integrates the robot's ground-truth pose from the resulting encoder-derived module states, so this corrupts both the simulated movement and the pose estimate used by PathPlanner. Increasing path PID gains cannot compensate for a roughly 1,272:1 scale error.

The simulation now converts wheel/steer mechanism rates to RPM before calling REVLib, explicitly advances the steering through-bore encoder, models the MAXSwerve steering gear-direction reversal, uses the actual NEO 550 steering motor model (the previous plant used a full-size NEO), and discretizes PathPlanner chassis speeds before inverse kinematics. Regression tests were added for the unit and discretization contracts.

## Evidence and code review

### 1. Confirmed simulation scale error — root cause of the shared X/Y shortfall

- `Configs.MAXSwerveModule` converts the drive encoder to m/s with `wheel circumference / drive reduction / 60` (`Configs.java`). For the current 3-inch wheel and 13T ratio, that factor is about **0.000785874 m/s per motor RPM**.
- The drive physics model (`m_drivePhysicsSim`) reports wheel-output rad/s. The previous code converted that to wheel m/s and passed the result to `SparkMaxSim.iterate()`.
- REVLib documents `SparkSim.iterate()` as taking the externally calculated motor velocity in RPM; the configured encoder conversion is applied to produce the sensor's reported units. At a real simulated wheel speed of 1 m/s, the motor should be about **1,272.5 RPM**. Passing the number `1.0` as RPM makes the encoder report about **0.000786 m/s** instead of 1 m/s.
- The chassis simulation then gets its measured speeds from the module encoders and integrates those speeds into its `Sim/Ground Truth` pose. Thus the error is not just a dashboard/estimator display discrepancy: the simulated robot itself barely moves, and PathPlanner sees the same understated measured motion. Important limitation: this field pose is **not an independent plant truth**—it shares the module encoder source with odometry—so the target endpoint and simulated physical motion must also be checked; estimator-to-truth error alone can hide a common sensor-scale error.
- Steering had the analogous mismatch: mechanism rad/s was passed to a motor API expecting RPM, and the simulated through-bore absolute encoder was not separately advanced. Since steer angle is part of each module state, that can also rotate the wheel vectors incorrectly and create apparent X/Y drift.
- The steering plant also used `DCMotor.getNEO(1)`, even though the REV MAXSwerve steering motor is a NEO 550. The plant and `SparkMaxSim` now use `DCMotor.getNeo550(1)`, which models its lower free speed and torque more faithfully.

This is why the observed translation issue is more plausibly a **unit/feedback-path defect** than a sign error in X/Y or a bad translation PID gain.

### 2. Odometry and coordinate conventions — largely sound in structure; hardware calibration remains unverified

The implementation uses the standard WPILib field convention: blue-origin field coordinates, +X toward the red wall, +Y to the left, and counter-clockwise-positive heading. Robot-relative chassis speeds use +X forward and +Y left. The code's `fromFieldRelativeSpeeds()` call uses the estimated heading, while PathPlanner's `getRobotRelativeSpeeds()` and `driveRobotRelative()` are robot-relative as required.

The four module locations in `Constants.DriveConstants.kDriveKinematics` are FL, FR, BL, BR, and the position/state arrays use the same order. Their offsets match the explicit FL/FR/BL/BR module coordinates in `pathplanner/settings.json`. `SwerveDrivePoseEstimator.update()` runs in `periodic()`, and `resetOdometry(Pose2d)` resets both translation and rotation. Existing tests cover field-relative transforms, alliance mirroring, and reset heading semantics.

Those code-level facts do **not** prove the physical sensors are calibrated. On the real robot, verify the wheel diameter under load, drive reduction, each absolute steering zero/offset, and navX sign. In particular, `kGyroReversed == false` currently means `getGyroRotation()` negates the navX angle; confirm that a physical CCW turn increases the WPILib heading. Measure a known straight distance in both +X and +Y and compare encoder-derived odometry to a tape measure.

### 3. Path tracking and discrete motion

PathPlanner's holonomic controller adds pose-error feedback to trajectory motion, and the project overrides that feedback with field-coordinate X/Y and continuous-angle PID controllers. The X and Y corrections are calculated from field pose errors, and the PathPlanner output is converted through robot-relative swerve kinematics. These frames are consistent; the custom gains are not an explanation for the simulator's 1,272:1 encoder scale error.

A separate, smaller source of X/Y error is that the path output previously went straight from continuous `ChassisSpeeds` to module states. With translation and rotation combined, finite 20 ms steps can skew the realized translation. The PathPlanner output now uses WPILib's `ChassisSpeeds.discretize(...)` before inverse kinematics. If the wheel speeds are then saturated, however, discretization cannot fully recover the requested chassis motion; avoid routinely saturating the modules by matching path limits to measured capability.

The current live defaults are translation kP = 2.5 and rotation kP = 4.0, with I/D at zero. Keep these as the comparison baseline until units, scale, gyro, steering, and motor response are verified; only then evaluate P conservatively on low-speed paths. Do not add I to hide a scale, offset, or saturation error. The feedback overrides are static PathPlanner callbacks and are used only while a path is active; retain one clear feedback source and reset controller state when beginning a path if nonzero I/D gains are introduced.

### 4. Motion-profile and robot-config mismatch to resolve from measurements

Path assets contain materially different limits: the short test paths are generally 2.2 m/s and 2.8 m/s², while the selected Blue Right routes reach 3.8 m/s and 2.8–3.2 m/s²; several other paths are limited to 1.1 m/s. The motor configuration in Java uses a NEO free-speed constant (5,676 RPM; about 4.45 m/s theoretical wheel speed), while `settings.json` says `driveMotorType: "vortex"` and `maxDriveSpeed: 4.8`. Those values must describe the **actual installed motor** and a measured loaded top speed. If the robot uses NEOs, the present 4.8 m/s planner cap is above theoretical free speed; if it uses Vortex motors, the Java motor model/free-speed constants need to be updated instead. Do not choose between them from code alone.

Mass, MOI, wheel COF, gearing, wheel radius and measured loaded top speed feed PathPlanner's trajectory constraints. If a generated profile demands more acceleration or speed than the real drive can supply, feedforward lag and module desaturation leave the robot behind the time-indexed target; a path can finish with residual endpoint error even when the coordinate frame is right. Start calibration with an uncomplicated path at roughly 0.5–1.0 m/s and 0.5–1.0 m/s², then raise limits only after measured velocity follows the reference. Keep the authored final velocity at zero for stop-at-goal paths.

## Code changes made

1. **Drive simulation units:** convert the wheel-output rad/s to drive-motor RPM using the drive reduction before `SparkMaxSim.iterate()`. The configured encoder conversion now maps that RPM back to wheel m/s exactly once.
2. **Steering simulation:** use the correct NEO 550 motor model, account for the MAXSwerve steering output's reversed direction in the plant input and signed reduction, and call the absolute-encoder simulator with output-shaft RPM so the through-bore sensor and steering feedback move at the mechanism rate.
3. **Path output discretization:** discretize robot-relative PathPlanner speeds at the configured 20 ms loop period before generating module states.
4. **Regression coverage:** add Java tests for drive and steer conversion round-trips and the discretization pose property; add Python checks for the REVLib unit contract and production call sites.

## Systematic validation plan

### A. Fast regression checks

1. Run `python3 -m unittest discover -s tests` (portable, no WPILib runtime required).
2. In a WPILib Java environment, run `./gradlew test`; specifically inspect the new `DriveSubsystemTest` conversion/discretization tests and existing pose/vision tests.
3. Run the robot simulator, command a steady 0.5 m/s straight drive for 2 seconds, and check that the encoder-derived travel is near 1 m (allowing for acceleration and startup delay), not millimeters. Compare the endpoint to the command as well as comparing `Sim/Ground Truth X/Y` to the estimator pose; those two pose streams currently share encoder-derived motion.

### B. Controlled simulation path sequence

Use the existing straight `x changes` path first, then the `xy` path, with the sim on Blue. Both have zero start/end velocity and are useful axis checks. Monitor `Vision Active` and estimator-to-sim-pose error; if a pure drivetrain-only baseline is needed, add a temporary sim-only vision-disable switch rather than assuming vision is off. Repeat each from the exact authored start pose:

1. **Straight X:** check that only field X changes; Y and heading remain nearly constant.
2. **Strafe Y:** check that only field Y changes; X and heading remain nearly constant.
3. **Diagonal/rotation path:** check X/Y together and with a heading change; this exercises discretization and steering sign.
4. **Red mirror:** rerun the same blue-authored path on Red and confirm the expected field-center flip, not an axis swap or a single-axis sign reversal.
5. **Vision on:** rerun with simulated AprilTags; compare estimator pose to `Sim/Ground Truth` separately from target-pose error.

Graph the existing dashboard signals: `Auto/Target Pose X/Y`, `Robot X/Y`, `Auto/Translation Error X/Y`, `Auto/Commanded Vx/Vy/Omega`, `Auto/Measured Vx/Vy/Omega`, `Sim/Ground Truth X/Y/Heading`, `Sim/Pose Estimate Error X/Y/Heading`, module desired speed, and battery voltage. Save each run's logs. For low-speed, no-collision simulation trials, use provisional goals of **≤0.10 m final XY error**, **≤2° final heading error**, and **<0.05 m estimator-to-ground-truth translation error**. These are acceptance targets to measure, not claims about current performance.

Interpretation:

- Estimate and ground truth both short, with commanded speed greater than measured speed: drive model, motor saturation, module speed limits, or profile too aggressive.
- Ground truth follows but estimated pose does not: encoder conversion/steer angle/gyro/vision odometry.
- X and Y both have nearly the same fractional scale error: wheel diameter or encoder conversion; one axis alone is more suggestive of a module order/offset/sign or coordinate transform.
- Motion matches until turns/combined translation begin: steering sign/offset, chassis geometry, or discretization/saturation.
- Low-speed tests pass but high-speed tests lag: tune the physical feedforward/loaded speed and lower path speed/acceleration before raising pose PID gains.

### C. Hardware gate before competition deployment

Once sim tests pass, verify on blocks and then on an open field: each wheel's direction/azimuth, absolute encoder zero, gyro CCW sign, and measured straight-line distance. Run WPILib SysId or equivalent safe characterization for drivetrain kS/kV/kA and use actual loaded maximum speed to correct PathPlanner robot settings. Test a low-speed 1 m X move, 1 m Y move, diagonal, and 90-degree rotation with repeated trials; log signed X/Y bias and endpoint error. Only then tune translation/rotation P and route constraints on the physical robot.

## Research references

- [REVLib SparkSim API, 2026.0.3](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparksim?version=2026.0.3) — `iterate()` takes externally calculated motor RPM; selected feedback and conversion factors are reflected in the simulated sensor.
- [REVLib SparkAbsoluteEncoderSim API, 2026.0.3](https://codedocs.revrobotics.com/java/com/revrobotics/sim/sparkabsoluteencodersim?version=2026.0.3) and [auxiliary encoder simulation guide](https://docs.revrobotics.com/revlib/spark/sim/simulating-additional-sensors-and-auxiliary-devices) — separately advancing the output encoder after its mechanism/gearing.
- [WPILib Swerve Drive Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html) — module order, distance in meters, gyro sign and periodic updates.
- [WPILib Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html) and [ChassisSpeeds](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html) — field/robot frames and field-relative conversion.
- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html) — latency-compensated vision fusion and standard-deviation tuning.
- [WPILib ChassisSpeeds API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html) — 20 ms discretization and translation-skew behavior.
- [PathPlanner Robot Config](https://pathplanner.dev/robot-config.html) — measured loaded top speed, wheel radius, gearing, COF and module offsets.
- [PathPlanner AutoBuilder](https://pathplanner.dev/pplib-build-an-auto.html) and [feedback overrides](https://pathplanner.dev/pplib-override-feedback.html) — robot-relative speed supplier/output and custom field-relative correction semantics.
- [WPILib Feedforward / SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) — characterize feedforward rather than compensating with excessive PID.
