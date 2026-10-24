package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Runtime shooter tuning, published through NetworkTables next to
 * {@link AutoTuning}.
 *
 * <p>These are the knobs a driver coach can change from Elastic between
 * matches (or between shots on the practice field) without a redeploy. They
 * only affect firing policy and the distance table; the flywheel PID gains
 * live in {@link frc.robot.subsystems.Shooter} because they are baked into the
 * SPARK MAX configuration at boot.
 *
 * <p>Recommended first pass on a real robot:
 * <ol>
 *   <li>Leave {@code Use Distance Table} off; shoot at the fixed distance and
 *       confirm {@code Shooter/Speed Error (RPM)} settles near zero. A steady
 *       error means the flywheel feedforward needs calibrating (see
 *       SHOOTER_ACCURACY.md).
 *   <li>Tune {@code Speed Tolerance (%)} down until shots stop landing short
 *       after a rapid burst, then raise it one step for margin.
 *   <li>Calibrate the table, then turn {@code Use Distance Table} on.
 * </ol>
 */
public final class ShooterTuning {
  private ShooterTuning() {}

  private static final String PREFIX = "Shooter Tuning/";

  public static final String USE_DISTANCE_TABLE = PREFIX + "Use Distance Table";
  public static final String TABLE_TRIM = PREFIX + "Table Trim";
  public static final String SPIN_RATIO = PREFIX + "Back/Front Spin Ratio";
  public static final String SPEED_TOLERANCE_PERCENT = PREFIX + "Speed Tolerance (%)";
  public static final String SPEED_SETTLE_SECONDS = PREFIX + "Speed Settle (s)";
  public static final String AIM_HYSTERESIS_DEG = PREFIX + "Aim Hysteresis (deg)";
  public static final String AIM_SETTLE_SECONDS = PREFIX + "Aim Settle (s)";
  public static final String DISTANCE_HYSTERESIS_IN = PREFIX + "Distance Hysteresis (in)";
  public static final String MAX_FIRE_SPEED_MPS = PREFIX + "Max Fire Speed (m/s)";
  public static final String MAX_FIRE_YAW_DPS = PREFIX + "Max Fire Yaw Rate (deg/s)";
  public static final String MOTION_GAIN = PREFIX + "Motion Comp Gain (RPM per m/s)";
  public static final String HEADING_TRIM_DEG = PREFIX + "Heading Trim (deg)";
  public static final String MOVING_SHOT_MAX_DISTANCE_M = PREFIX + "Moving Shot Max Distance (m)";
  public static final String MOVING_SHOT_SPEED_THRESHOLD_MPS = PREFIX + "Moving Shot Threshold (m/s)";
  public static final String USE_SHOOTER_CAMERA_AIM = PREFIX + "Use Shooter Camera Aim";
  public static final String CAMERA_AIM_MAX_AGE_SECONDS = PREFIX + "Camera Aim Max Age (s)";
  public static final String CAMERA_AIM_MAX_AMBIGUITY = PREFIX + "Camera Aim Max Ambiguity";

  private static final double DEFAULT_SPEED_TOLERANCE_PERCENT = 3.0;
  private static final double DEFAULT_SPEED_SETTLE_SECONDS = 0.15;
  private static final double DEFAULT_AIM_HYSTERESIS_DEG = 1.5;
  private static final double DEFAULT_AIM_SETTLE_SECONDS = 0.10;
  private static final double DEFAULT_DISTANCE_HYSTERESIS_IN = 1.0;
  private static final double DEFAULT_MAX_FIRE_SPEED_MPS = 0.35;
  private static final double DEFAULT_MAX_FIRE_YAW_DPS = 45.0;
  private static final double DEFAULT_MOVING_SHOT_MAX_DISTANCE_M = 0.0;
  private static final double DEFAULT_MOVING_SHOT_SPEED_THRESHOLD_MPS = 0.20;
  private static final double DEFAULT_CAMERA_AIM_MAX_AGE_SECONDS = 0.08;
  private static final double DEFAULT_CAMERA_AIM_MAX_AMBIGUITY = 0.35;

  /** Publishes defaults for keys that do not exist yet, preserving dashboard edits. */
  public static void publishDefaults() {
    putNumberDefault(TABLE_TRIM, 1.0);
    putNumberDefault(
        SPIN_RATIO, Constants.ShooterConstants.kDefaultSpinRatio);
    putNumberDefault(SPEED_TOLERANCE_PERCENT, DEFAULT_SPEED_TOLERANCE_PERCENT);
    putNumberDefault(SPEED_SETTLE_SECONDS, DEFAULT_SPEED_SETTLE_SECONDS);
    putNumberDefault(AIM_HYSTERESIS_DEG, DEFAULT_AIM_HYSTERESIS_DEG);
    putNumberDefault(AIM_SETTLE_SECONDS, DEFAULT_AIM_SETTLE_SECONDS);
    putNumberDefault(DISTANCE_HYSTERESIS_IN, DEFAULT_DISTANCE_HYSTERESIS_IN);
    putNumberDefault(MAX_FIRE_SPEED_MPS, DEFAULT_MAX_FIRE_SPEED_MPS);
    putNumberDefault(MAX_FIRE_YAW_DPS, DEFAULT_MAX_FIRE_YAW_DPS);
    putNumberDefault(MOTION_GAIN, 0.0);
    putNumberDefault(HEADING_TRIM_DEG, 0.0);
    putNumberDefault(MOVING_SHOT_MAX_DISTANCE_M, DEFAULT_MOVING_SHOT_MAX_DISTANCE_M);
    putNumberDefault(MOVING_SHOT_SPEED_THRESHOLD_MPS, DEFAULT_MOVING_SHOT_SPEED_THRESHOLD_MPS);
    putNumberDefault(CAMERA_AIM_MAX_AGE_SECONDS, DEFAULT_CAMERA_AIM_MAX_AGE_SECONDS);
    putNumberDefault(CAMERA_AIM_MAX_AMBIGUITY, DEFAULT_CAMERA_AIM_MAX_AMBIGUITY);
    putBooleanDefault(USE_DISTANCE_TABLE, false);
    putBooleanDefault(USE_SHOOTER_CAMERA_AIM, false);
  }

  /** Writes {@code value} only if the key has never been published. */
  public static void putBooleanDefault(String key, boolean value) {
    SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, value));
  }

  /** Writes {@code value} only if the key has never been published. */
  public static void putNumberDefault(String key, double value) {
    if (Double.isNaN(SmartDashboard.getNumber(key, Double.NaN))) {
      SmartDashboard.putNumber(key, value);
    }
  }

  private static double get(String key, double fallback, double min, double max) {
    return MathUtil.clamp(SmartDashboard.getNumber(key, fallback), min, max);
  }

  /** When true the robot shoots from wherever it is, using the distance table. */
  public static boolean useDistanceTable() {
    return SmartDashboard.getBoolean(USE_DISTANCE_TABLE, false);
  }

  /** Multiplies every table RPM; the 7415-style "we undershoot a little" correction. */
  public static double tableTrim() {
    return get(TABLE_TRIM, 1.0, 0.80, 1.20);
  }

  /** Back wheel RPM divided by front wheel RPM, preserving the tuned backspin. */
  public static double spinRatio() {
    return get(SPIN_RATIO, Constants.ShooterConstants.kDefaultSpinRatio, 0.50, 2.00);
  }

  /** Fractional flywheel speed tolerance, e.g. 0.03 = +/-3% of target. */
  public static double speedToleranceFraction() {
    return get(SPEED_TOLERANCE_PERCENT, DEFAULT_SPEED_TOLERANCE_PERCENT, 0.5, 20.0) / 100.0;
  }

  public static double speedSettleSeconds() {
    return get(SPEED_SETTLE_SECONDS, DEFAULT_SPEED_SETTLE_SECONDS, 0.0, 1.0);
  }

  public static double aimHysteresisDeg() {
    return get(AIM_HYSTERESIS_DEG, DEFAULT_AIM_HYSTERESIS_DEG, 0.0, 10.0);
  }

  public static double aimSettleSeconds() {
    return get(AIM_SETTLE_SECONDS, DEFAULT_AIM_SETTLE_SECONDS, 0.0, 1.0);
  }

  public static double distanceHysteresisInches() {
    return get(DISTANCE_HYSTERESIS_IN, DEFAULT_DISTANCE_HYSTERESIS_IN, 0.0, 12.0);
  }

  public static double maxFireSpeedMps() {
    return get(MAX_FIRE_SPEED_MPS, DEFAULT_MAX_FIRE_SPEED_MPS, 0.0, 3.0);
  }

  public static double maxFireYawRateDps() {
    return get(MAX_FIRE_YAW_DPS, DEFAULT_MAX_FIRE_YAW_DPS, 5.0, 180.0);
  }

  /** Shoot-on-the-move gain. 0 disables motion compensation entirely. */
  public static double motionCompensationGain() {
    return get(MOTION_GAIN, 0.0, 0.0, 1000.0);
  }

  /** Mechanical aim bias, added to the commanded heading. See SHOOTER_ACCURACY.md. */
  public static double headingTrimDegrees() {
    return get(HEADING_TRIM_DEG, 0.0, -15.0, 15.0);
  }

  /**
   * Furthest shot allowed while the robot is moving. 0 disables the check, so a
   * stationary robot is unaffected and a moving one is limited only by
   * {@link #maxFireSpeedMps()}. See SHOOTER_ACCURACY.md, "Shooting on the move".
   */
  public static double movingShotMaxDistanceMeters() {
    return get(MOVING_SHOT_MAX_DISTANCE_M, DEFAULT_MOVING_SHOT_MAX_DISTANCE_M, 0.0, 12.0);
  }

  /** Speed below which a shot counts as stationary and the distance check is skipped. */
  public static double movingShotSpeedThresholdMps() {
    return get(MOVING_SHOT_SPEED_THRESHOLD_MPS, DEFAULT_MOVING_SHOT_SPEED_THRESHOLD_MPS, 0.0, 3.0);
  }

  /** When true, the shooter camera's tag residual corrects the pose-based aim. */
  public static boolean useShooterCameraAim() {
    return SmartDashboard.getBoolean(USE_SHOOTER_CAMERA_AIM, false);
  }

  /** Oldest camera frame still used for aim correction. */
  public static double cameraAimMaxAgeSeconds() {
    return get(CAMERA_AIM_MAX_AGE_SECONDS, DEFAULT_CAMERA_AIM_MAX_AGE_SECONDS, 0.02, 0.25);
  }

  /** Largest single-tag pose ambiguity still used for aim correction. */
  public static double cameraAimMaxAmbiguity() {
    return get(CAMERA_AIM_MAX_AMBIGUITY, DEFAULT_CAMERA_AIM_MAX_AMBIGUITY, 0.0, 1.0);
  }
}
