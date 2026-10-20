package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/**
 * Runtime autonomous tuning controls published through NetworkTables. Elastic
 * Dashboard can edit these SmartDashboard entries while the robot is running.
 * Values are read at use-time (PID gains every scheduler loop; constraints when
 * a pathfind command is created), so no code edit or robot restart is needed.
 */
public final class AutoTuning {
  private AutoTuning() {}

  private static final String PREFIX = "Auto Tuning/";

  public static final String TARGET_DISTANCE_IN = "Auto Aim Target Distance (in)";
  private static final String TRANSLATION_P = PREFIX + "Path Translation kP";
  private static final String TRANSLATION_I = PREFIX + "Path Translation kI";
  private static final String TRANSLATION_D = PREFIX + "Path Translation kD";
  private static final String ROTATION_P = PREFIX + "Path Rotation kP";
  private static final String ROTATION_I = PREFIX + "Path Rotation kI";
  private static final String ROTATION_D = PREFIX + "Path Rotation kD";
  private static final String MAX_MODULE_SPEED = PREFIX + "Max Module Speed (mps)";
  private static final String PATH_MAX_SPEED = PREFIX + "Pathfind Max Speed (mps)";
  private static final String PATH_MAX_ACCEL = PREFIX + "Pathfind Max Accel (mps2)";
  private static final String PATH_MAX_ANGULAR_SPEED = PREFIX + "Pathfind Max Angular Speed (degps)";
  private static final String PATH_MAX_ANGULAR_ACCEL = PREFIX + "Pathfind Max Angular Accel (degps2)";
  private static final String AIM_KP = PREFIX + "Aim Heading kP";
  private static final String AIM_TOLERANCE = PREFIX + "Aim Heading Tolerance (deg)";
  private static final String DISTANCE_KP = PREFIX + "Aim Distance kP";
  private static final String DISTANCE_MAX_SPEED = PREFIX + "Aim Distance Max Speed (mps)";
  private static final String DISTANCE_TOLERANCE = PREFIX + "Aim Distance Tolerance (in)";

  /** Publish defaults only when an entry does not already exist, preserving dashboard edits. */
  public static void publishDefaults() {
    putDefault(TARGET_DISTANCE_IN, 100.0);
    putDefault(TRANSLATION_P, 5.0);
    putDefault(TRANSLATION_I, 0.0);
    putDefault(TRANSLATION_D, 0.0);
    putDefault(ROTATION_P, 5.0);
    putDefault(ROTATION_I, 0.0);
    putDefault(ROTATION_D, 0.0);
    putDefault(MAX_MODULE_SPEED, DriveConstants.kMaxSpeedMetersPerSecond);
    putDefault(PATH_MAX_SPEED, 2.0);
    putDefault(PATH_MAX_ACCEL, 2.0);
    putDefault(PATH_MAX_ANGULAR_SPEED, 360.0);
    putDefault(PATH_MAX_ANGULAR_ACCEL, 540.0);
    putDefault(AIM_KP, 0.03);
    putDefault(AIM_TOLERANCE, 2.0);
    putDefault(DISTANCE_KP, 1.0);
    putDefault(DISTANCE_MAX_SPEED, 0.35);
    putDefault(DISTANCE_TOLERANCE, 2.0);
  }

  private static void putDefault(String key, double value) {
    if (Double.isNaN(SmartDashboard.getNumber(key, Double.NaN))) {
      SmartDashboard.putNumber(key, value);
    }
  }

  private static double get(String key, double fallback, double min, double max) {
    return MathUtil.clamp(SmartDashboard.getNumber(key, fallback), min, max);
  }

  public static double translationP() { return get(TRANSLATION_P, 5.0, 0.0, 20.0); }
  public static double translationI() { return get(TRANSLATION_I, 0.0, 0.0, 5.0); }
  public static double translationD() { return get(TRANSLATION_D, 0.0, 0.0, 10.0); }
  public static double rotationP() { return get(ROTATION_P, 5.0, 0.0, 20.0); }
  public static double rotationI() { return get(ROTATION_I, 0.0, 0.0, 5.0); }
  public static double rotationD() { return get(ROTATION_D, 0.0, 0.0, 10.0); }
  public static double maxModuleSpeed() {
    return get(
        MAX_MODULE_SPEED,
        DriveConstants.kMaxSpeedMetersPerSecond,
        0.5,
        DriveConstants.kMaxSpeedMetersPerSecond);
  }
  public static double pathMaxSpeed() {
    return get(PATH_MAX_SPEED, 2.0, 0.2, DriveConstants.kMaxSpeedMetersPerSecond);
  }
  public static double pathMaxAcceleration() { return get(PATH_MAX_ACCEL, 2.0, 0.2, 8.0); }
  public static double pathMaxAngularSpeedRadPerSec() {
    return Units.degreesToRadians(get(PATH_MAX_ANGULAR_SPEED, 360.0, 30.0, 720.0));
  }
  public static double pathMaxAngularAccelerationRadPerSecSq() {
    return Units.degreesToRadians(get(PATH_MAX_ANGULAR_ACCEL, 540.0, 30.0, 1440.0));
  }
  public static double aimHeadingP() { return get(AIM_KP, 0.03, 0.0, 1.0); }
  public static double aimHeadingToleranceDeg() { return get(AIM_TOLERANCE, 2.0, 0.1, 15.0); }
  public static double aimDistanceKp() { return get(DISTANCE_KP, 1.0, 0.0, 5.0); }
  public static double aimDistanceMaxSpeed() { return get(DISTANCE_MAX_SPEED, 0.35, 0.05, 2.0); }
  public static double aimDistanceToleranceMeters() {
    return Units.inchesToMeters(get(DISTANCE_TOLERANCE, 2.0, 0.25, 24.0));
  }
  public static double aimTargetDistanceInches() {
    return get(TARGET_DISTANCE_IN, 100.0, 0.0, 300.0);
  }
}
