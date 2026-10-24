# Autonomous Path Accuracy & Simulation Audit (SPARK MAX + NEO)

## Root causes identified and resolved

1. **REVLib 2026 `SparkSim.iterate()` & `SparkAbsoluteEncoderSim.iterate()` unit mismatch (`MAXSwerveModule.java`)**
   - In REVLib `2026.0.3`, `SparkSim.iterate(velocity, vbus, dt)` divides `velocity` internally by `encoder.velocityConversionFactor` (`0.000785874 (m/s)/RPM` for drive) and multiplies back by that conversion factor when reporting `getEncoder().getVelocity()`.
   - Passing motor-shaft RPM into `SparkMaxSim.iterate()` caused a **1,272.5×** runaway encoder velocity scale error on drive and a **9.55×** scale error plus unstable sign inversion (`-m_turningSparkSim.getAppliedOutput()`) on steer.
   - `MAXSwerveModule.simulationPeriodic()` now passes wheel linear speed (`m/s`) to `m_drivingSparkSim.iterate(...)`, positive steer output voltage to `m_turnPhysicsSim`, and module angular velocity (`rad/s`) to `m_turningSparkSim.iterate(...)`. (The azimuth encoder state itself is no longer re-integrated with `SparkAbsoluteEncoderSim.iterate()` - see item 6.)

2. **REVLib 2026 `FeedForwardConfig.kV` 12× voltage unit bug (`Configs.java`, `Shooter.java`, `Intake.java`)**
   - REVLib 2025/2026 closed-loop feedforward (`closedLoop.feedForward.kV(...)` / deprecated `velocityFF(...)`) specifies voltage gain in **Volts per velocity unit**, not duty-cycle `[0, 1]` per velocity unit.
   - Drive modules (`Configs.java`) previously set `1.0 / kDriveWheelFreeSpeedRps` (`0.2242` instead of `12.0 / 4.4606 = 2.6902 V/(m/s)`), starving drive feedforward by **12×** (`8.3%` voltage).
   - Shooter flywheels (`Shooter.java`) previously set `kF = 0.000175` instead of `12.0 / 5676.0 = 0.002114 V/RPM`, causing front (`1,279 RPM < 1,500 RPM`) and back (`1,705 RPM < 2,000 RPM`) flywheels to stall below `Shooter.isAtTargetSpeed()` so `ShootForXseconds` / `AutoAimAndShoot` never fed game pieces.
   - Drive, shooter, and conveyor closed loops now use `closedLoop.feedForward.kV(12.0 / freeSpeed)`.

3. **Standardized SPARK MAX + NEO hardware configuration (`settings.json`, `Constants.java`, `AutoTuning.java`)**
   - Standardized `deploy/pathplanner/settings.json` on `"driveMotorType": "NEO"`, `"driveWheelRadius": 0.0381`, `"robotTrackwidth": 0.673` (`26.5 in`, matching `flModuleY = ±0.337 m`), and `"maxDriveSpeed": 4.2` (`<= 4.46 m/s` NEO free speed with headroom for loaded battery sag).
   - Aligned `DriveConstants.kMaxSpeedMetersPerSecond` to `ModuleConstants.kDriveWheelFreeSpeedRps` (`~4.46 m/s`).

4. **PathPlanner `DriveFeedforwards`, zero-lag trajectory feedback, and path spline fixes (`DriveSubsystem.java`, `MAXSwerveModule.java`, `paths/*.path`)**
   - Wired `(speeds, feedforwards)` in `AutoBuilder.configure(...)` so per-module `robotRelativeForcesXNewtons()` / `robotRelativeForcesYNewtons()` are projected onto each optimized module wheel direction and applied as arbitrary voltage feedforward (`DCMotor.getVoltage(motorTorqueNm, 0.0)`).
   - Replaced static `overrideXYFeedback` / `overrideRotationFeedback` callbacks (which read a 20 ms stale `m_pathTargetPose`) with direct `PPHolonomicDriveController.calculateRobotRelativeSpeeds(...)` evaluation using live `AutoTuning` gains on the exact current `PathPlannerTrajectoryState`.
   - Fixed reversed Bezier control handles and accidental `"unlimited": true` flags in `Blue left` path assets (`Blue Left trench to fuel.path`, `Blue left fuel intake.path`, `Blue left fuel to shoot.path`), and added mirrored `blue right trench.path` for `driveToRightTrench()`.

5. **Battery simulation voltage & regen current guards (`MAXSwerveModule.java`, `Shooter.java`, `Intake.java`, `RobotContainer.java`)**
   - Clamped simulated supply currents to finite positive draws and guarded `RoboRioSim.setVInVoltage(...)` so braking regeneration never drives simulated bus voltage negative, zero, or `NaN`.

6. **"Drivetrain Steer" SysId looked dead in simulation (`MAXSwerveModule.java`, `DriveSubsystem.java`)**
   - The azimuth routine holds the drive wheels at `0 V`, so the chassis never translates or rotates and nothing on the field widget moved. `MAXSwerveModule.simulationPeriodic()` also re-integrated the Through Bore encoder with `SparkAbsoluteEncoderSim.iterate()` on top of the position/velocity that `SparkMaxSim.iterate()` already mirrors into the selected feedback sensor: the reported module angle led the plant by a full loop of travel (5-15° during a sweep) and, because `SparkAbsoluteEncoderSim.iterate()` only integrates position and never publishes velocity, the logged azimuth velocity depended entirely on REVLib's internal mirroring. A flat-zero velocity trace makes the SysId OLS fit degenerate, which presents as "the steer test does nothing".
   - The simulated encoder state is now written straight from the steer plant - position wrapped to `[0, 2π)` like real hardware (so the multi-turn unwrap SysId relies on is exercised in sim), velocity in `rad/s` - keeping the logged position and velocity pair consistent.
   - `DriveSubsystem` publishes per-module sim telemetry (`Sim/Steer <FL|FR|RL|RR> Angle (deg)` / `Rate (deg/s)` / `Volts`, `Sim/Drive <corner> Speed (mps)` / `Volts`, `Sim/Steer Motor Current (A)`, `Sim/Steer FL Continuous Angle (rot)`, `Sim/SysId Commanded Volts`, `Sim/SysId Last Test`) and draws a `<corner> Module` heading arrow at each module's ground-truth field position, so azimuth motion is visible while the chassis stands still.
   - A SysId run that commands voltage the simulated SPARKs never apply (REVLib forces `0 V` output while the Driver Station is disabled) now reports a one-shot Driver Station warning instead of silently logging a flat run, and the steer entry points require the drivetrain from their first instant (`Commands.runOnce(this::clearHeadingHoldForSysId, this)`) like the linear/angular pre-align phase does.
   - Remaining sim-only fidelity gap, documented in `SIMULATION.md`: REVLib estimates motor current from the selected sensor's RPM, so the 46.42:1 azimuth reduction makes the estimate miss nearly all back-EMF and the 20 A smart current limit throttles the simulated steer motor to ~2-3 V for a 6 V command. Sim-identified azimuth kV/kA are pipeline checks, not real-robot gains.

## Closed-loop simulation verification results (all 5 autonomous routines)

| Autonomous Routine | Final Pose `(X, Y)` | Target Pose `(X, Y)` | Final XY Error | Final Heading Error | Peak Dynamic Error |
|---|---|---|---|---|---|
| `x changes` | `(7.028, 0.686)` | `(7.000, 0.686)` | `2.80 cm` | `0.01°` | `2.40 cm` |
| `y changes` | `(2.800, 0.973)` | `(2.800, 1.000)` | `2.69 cm` | `0.00°` | `2.63 cm` |
| `xy changes` | `(7.999, 7.525)` | `(8.000, 7.500)` | `2.49 cm` | `0.01°` | `7.88 cm` |
| `Blue left` | `(2.375, 3.560)` | `(2.369, 3.587)` | `2.80 cm` | `0.02°` | `9.75 cm` |
| `Blue right` | `(2.328, 3.712)` | `(2.325, 3.707)` | `0.51 cm` | `0.18°` | `5.28 cm` |
