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
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
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
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
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
    // Simulation runtime
    // ------------------------------------------------------------------
    // Standard robot loop period; the physics are stepped at this rate.
    public static final double kLoopPeriodSeconds = 0.020;
  }
}