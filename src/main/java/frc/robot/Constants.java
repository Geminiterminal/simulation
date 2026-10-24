// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Matched to MAXSwerve NEO 1.1 unloaded 12 V free speed (~4.46 m/s)
    // so commanded wheel speeds never exceed what SPARK MAX + NEO hardware can deliver.
    public static final double kMaxSpeedMetersPerSecond = ModuleConstants.kDriveWheelFreeSpeedRps;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration:
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 17;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 11;

    public static final int kFrontLeftTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 10;

    // Flip when the navX is mounted inverted (rotated 180 degrees about the
    // robot's z axis). Both the gyro heading and the gyro rate sign follow
    // this flag so they can never disagree with each other.
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5.0;
    public static final double kPYController = 5.0;
    public static final double kPThetaController = 5.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  /**
   * Geometry of the 2026 "Rebuilt" field, in the WPILib field coordinate
   * system (origin on the blue alliance wall, +x toward the red alliance wall).
   *
   * <p>The field is rotationally symmetric: the red half of the field is the
   * blue half rotated 180 degrees about the field center. That is the same
   * transform PathPlanner uses when it flips a blue-authored path for the red
   * alliance, so a mirrored pose and a flipped path always agree.
   */
  public static final class FieldConstants {
    /** Field length (blue wall to red wall). Matches PathPlanner's default. */
    public static final double kFieldLengthMeters = 16.54;

    /** Field width (one long side to the other). Matches PathPlanner's default. */
    public static final double kFieldWidthMeters = 8.07;

    /** Center of the blue alliance hub. */
    public static final double kBlueHubX = 4.626;
    public static final double kBlueHubY = 4.035;

    /** Center of the red alliance hub. */
    public static final double kRedHubX = 11.919;
    public static final double kRedHubY = 4.029;
  }

  /**
   * Vision hardware metadata shared by the real robot code (DriveSubsystem) and
   * the PhotonVision simulation (VisionSubsystem). Keeping the camera names and
   * robot-to-camera transforms in one place guarantees the simulated cameras see
   * exactly what the real cameras see, so the pose-fusion pipeline behaves
   * identically on and off the field.
   */
  public static final class VisionConstants {
    // NOTE: the physical "FrontCamera" unit is mounted facing the REAR of the robot.
    public static final String kBackCameraName = "FrontCamera";
    public static final String kSideCameraName = "GoodCamera_OldLeft";

    // Transform for the rear-facing camera (measured from robot center).
    public static final Transform3d kRobotToBackCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-12.25), // X
            Units.inchesToMeters(-10.75), // Y
            Units.inchesToMeters(17.125) // Z
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0), // Roll
            Units.degreesToRadians(0.0), // Pitch
            Units.degreesToRadians(180.0) // Yaw
        ));

    // Transform for the left-side-facing camera (measured from robot center).
    public static final Transform3d kRobotToSideCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-10),
            Units.inchesToMeters(-13),
            Units.inchesToMeters(16.19)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-90.0)
        ));

    // Simulated camera characteristics (calibrated to the team's cameras).
    // Re-measure intrinsics with the PhotonVision camera calibration tool and
    // update these numbers if the real calibration deviates.
    public static final int kSimCameraWidthPx = 1280;
    public static final int kSimCameraHeightPx = 720;
    public static final double kSimCameraDiagonalFovDeg = 75.0;
    public static final double kSimCameraCalibErrorPx = 0.25;
    public static final double kSimCameraCalibErrorStdPx = 0.08;
    public static final double kSimCameraFps = 30.0;
    public static final double kSimCameraAvgLatencyMs = 30.0;
    public static final double kSimCameraLatencyStdDevMs = 5.0;
    // PhotonVision multi-tag/PNP rarely solves reliably beyond this range.
    public static final double kSimCameraMaxSightRangeMeters = 6.0;
  }

  /**
   * Shooter geometry and defaults for the code-only accuracy improvements.
   *
   * <p>Every value here is a software setting: no mechanism has to move. The
   * two entries marked "measure" are one-time tape measurements; leaving them
   * at zero reproduces the original aim model exactly (aim the robot center,
   * fire straight out of the rear), so the new math is an opt-in improvement
   * rather than a behavior change. See SHOOTER_ACCURACY.md.
   */
  public static final class ShooterConstants {
    /**
     * MEASURE: horizontal offset from the robot center to the point where FUEL
     * leaves the shooter (the muzzle), robot frame (+x forward, +y left).
     *
     * <p>This matters most for close shots: a 0.3 m lateral offset changes the
     * required heading by ~11 degrees from 1.5 m away. A rough tape measure is
     * already much better than zero.
     */
    public static final Translation2d kMuzzleOffsetMeters = new Translation2d(0.0, 0.0);

    /** Robot-frame direction the shooter fires in, degrees (180 = straight out of the rear). */
    public static final double kShooterFacingOffsetDegrees = 180.0;

    /** Back/front flywheel speed ratio that preserves the tuned backspin. */
    public static final double kDefaultSpinRatio = 2000.0 / 1500.0;

    /** Fixed flywheel speeds used while the distance table is disabled. */
    public static final double kDefaultFrontRpm = 1500.0;
    public static final double kDefaultBackRpm = 2000.0;

    /** Number of editable distance-to-RPM points published to the dashboard. */
    public static final int kTablePoints = 5;

    /**
     * Default distance table. Every point holds the current fixed speed, so
     * turning the table on before calibrating does not change the shot. Replace
     * the points with measured values (SHOOTER_ACCURACY.md, "Calibrating the
     * distance table").
     *
     * <p>Distances must be entered in ascending order or the shooter falls back
     * to the fixed speed and reports "Table Valid = false".
     */
    public static final double[] kDefaultTableDistancesMeters = {1.0, 2.0, 3.0, 4.0, 5.0};
    public static final double[] kDefaultTableFrontRpm = {
      1500.0, 1500.0, 1500.0, 1500.0, 1500.0
    };

    /** Time of flight assumed when converting table slope into a motion-comp gain. */
    public static final double kNominalTimeOfFlightSeconds = 0.5;
  }

  /**
   * Physics calibration used ONLY in simulation (RobotBase.isSimulation()).
   *
   * <p>These values describe the physical robot so the WPILib physics models
   * (DCMotorSim / FlywheelSim / SingleJointedArmSim) reproduce real behavior:
   * acceleration limits, spin-up times, current draw and battery sag. They do
   * not affect code deployed to the roboRIO.
   *
   * <p>See SIMULATION.md for how each number was measured/estimated and how to
   * re-calibrate (e.g. with SysId) if the robot changes.
   */
  public static final class SimulationConstants {
    // ------------------------------------------------------------------
    // Chassis (matches the PathPlanner GUI robot configuration, which was
    // itself measured: full robot + bumpers + battery)
    // ------------------------------------------------------------------
    public static final double kRobotMassKg = 65.0;
    public static final double kRobotMOIKgMetersSquared = 5.7; // about the vertical axis

    // Wheel radius: 3-inch diameter wheels (ModuleConstants.kWheelDiameterMeters).
    public static final double kWheelRadiusMeters = ModuleConstants.kWheelDiameterMeters / 2.0;

    // Drive gearbox: MAXSwerve with 13T pinion -> (45*22)/(13*15) = 5.077:1
    public static final double kDriveGearRatio = ModuleConstants.kDrivingMotorReduction;

    // Steering gearbox: MAXSwerve azimuth reduction is 9424:203 (~46.42:1) per REV.
    public static final double kTurnGearRatio = 9424.0 / 203.0;

    // Effective inertia of one drive wheel as seen at the wheel axle: the
    // quarter-chassis it pushes ((m/4)*r^2) plus the wheel's own spin inertia
    // (~0.5 * m_wheel * r^2 for a ~50 g Colson wheel).
    public static final double kDriveWheelInertiaKgMetersSquared =
        (kRobotMassKg / 4.0) * kWheelRadiusMeters * kWheelRadiusMeters
            + 0.5 * 0.050 * kWheelRadiusMeters * kWheelRadiusMeters;

    // Rotational inertia of one swerve module about its steering axis (typical
    // MAXSwerve value; small effects only - steering just sets wheel direction).
    public static final double kSteerInertiaKgMetersSquared = 0.004;

    // ------------------------------------------------------------------
    // Shooter. The 1:1 motor ratios are confirmed by the feedforward gain
    // (kF = 1 / NEO free speed RPM, i.e. full voltage at free speed, so the
    // controller output units are motor-side RPM).
    // ------------------------------------------------------------------
    public static final double kShooterFlywheelGearRatio = 1.0;
    // Estimate for one 4" flywheel + shaft/hardware (~1.3 kg at 5 cm radius,
    // 0.5*m*r^2) plus shaft and pulleys. Re-identify with SysId for accuracy.
    public static final double kShooterFlywheelMoiKgMetersSquared = 0.0025;

    public static final double kFeederGearRatio = 1.0;
    public static final double kFeederMoiKgMetersSquared = 0.001;

    // ------------------------------------------------------------------
    // Intake mechanisms
    // ------------------------------------------------------------------
    // Roller (Kraken X60) and conveyor (NEO): small drums, roughly 1:1.
    public static final double kIntakeRollerGearRatio = 1.0;
    public static final double kIntakeRollerMoiKgMetersSquared = 0.0008;
    public static final double kConveyorGearRatio = 1.0;
    public static final double kConveyorMoiKgMetersSquared = 0.002;

    // Deploy arm: the code commands positions 0 (stowed) to 59 motor rotations
    // (deployed). Calibrated so that 59 rotations = 100 degrees of arm travel;
    // total reduction = 59 * 360 / 100 ~ 212.4:1 (gearbox + external stage).
    public static final double kDeployArmGearRatio = 59.0 * 360.0 / 100.0;
    public static final double kDeployArmLengthMeters = 0.30;
    public static final double kDeployArmMassKg = 3.0;
    // Stowed = 0 deg (horizontal), deployed = +100 deg (down over the bumper).
    public static final double kDeployArmStowedAngleRad = Units.degreesToRadians(0.0);
    public static final double kDeployArmDeployedAngleRad = Units.degreesToRadians(100.0);
    public static final double kDeployArmMaxAngleRad = Units.degreesToRadians(105.0);

    // ------------------------------------------------------------------
    // Simulated gyroscope (the navX MXP's stand-in).
    //
    // The gyroscope is the primary heading sensor: the pose estimator fuses
    // its angle deltas with wheel odometry and corrects the accumulated drift
    // with vision. A real navX has a small zero-rate bias and angle noise, so
    // the simulated one does too - otherwise the simulation would hide every
    // heading-estimation problem the real robot can have. See SimulatedGyro.
    // ------------------------------------------------------------------
    /** Constant zero-rate bias: 0.02 deg/s ~ 1.2 deg/min of drift. */
    public static final double kSimGyroDriftDegreesPerSecond = 0.02;

    /** White-noise sigma on the reported yaw rate. */
    public static final double kSimGyroNoiseDegreesPerSecond = 0.02;

    /** Seed for the gyro noise generator (keeps simulation runs repeatable). */
    public static final long kSimGyroRandomSeed = 2026L;

    // ------------------------------------------------------------------
    // Simulation runtime
    // ------------------------------------------------------------------
    // Standard robot loop period; the physics are stepped at this rate.
    public static final double kLoopPeriodSeconds = 0.020;
  }
}