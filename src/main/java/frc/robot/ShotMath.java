package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pure shot geometry and table math for the shooter.
 *
 * <p>Nothing in this class touches motors, NetworkTables or the command
 * scheduler, so every rule here is exercised by {@code ShotMathTest} on a
 * laptop with no robot attached. {@link frc.robot.subsystems.DriveSubsystem}
 * and {@link frc.robot.subsystems.Shooter} are thin wrappers around it.
 */
public final class ShotMath {
  private ShotMath() {}

  /** Fixed-point iterations used by {@link #solveAim} by default (matches shoot-on-the-move solvers). */
  public static final int kDefaultIterations = 3;

  /** Below this length a distance vector is treated as degenerate. */
  private static final double kEpsilon = 1e-6;

  /**
   * Result of {@link #solveAim}.
   *
   * @param headingDegrees robot heading that points the shooter at the target, wrapped to [-180, 180)
   * @param distanceMeters distance from the muzzle (not the robot center) to the target
   */
  public record AimSolution(double headingDegrees, double distanceMeters) {}

  /**
   * Field position of the shooter muzzle.
   *
   * @param robotTranslation robot center in the field frame
   * @param muzzleOffsetRobotFrame muzzle offset from the robot center, robot frame (+x forward, +y left)
   * @param robotHeading robot heading
   * @return the muzzle's field position
   */
  public static Translation2d muzzlePosition(
      Translation2d robotTranslation,
      Translation2d muzzleOffsetRobotFrame,
      Rotation2d robotHeading) {
    return robotTranslation.plus(muzzleOffsetRobotFrame.rotateBy(robotHeading));
  }

  /**
   * Solves for the robot heading that points the shooter at {@code target}.
   *
   * <p>When the muzzle is not exactly at the robot center, its field position
   * depends on the heading that is being solved for, so there is no closed-form
   * answer. A few fixed-point iterations converge to the exact aim - the same
   * technique teams use to lead a moving target. With a zero offset the first
   * iteration is already exact, so this reproduces "aim the robot center" bit
   * for bit.
   *
   * @param robotPose current robot pose
   * @param target field position of the hub / pass target
   * @param muzzleOffsetRobotFrame shooter exit point in the robot frame
   * @param facingOffsetDegrees direction the shooter fires in, robot frame (180 = out of the rear)
   * @param headingTrimDegrees bias added to the commanded heading (see {@link ShooterTuning})
   * @param iterations fixed-point iterations to run (use {@link #kDefaultIterations})
   */
  public static AimSolution solveAim(
      Pose2d robotPose,
      Translation2d target,
      Translation2d muzzleOffsetRobotFrame,
      double facingOffsetDegrees,
      double headingTrimDegrees,
      int iterations) {
    Translation2d muzzle =
        muzzlePosition(
            robotPose.getTranslation(), muzzleOffsetRobotFrame, robotPose.getRotation());
    double heading = robotPose.getRotation().getDegrees();
    double distance = target.minus(muzzle).getNorm();

    int steps = Math.max(1, iterations);
    for (int i = 0; i < steps; i++) {
      Translation2d toTarget = target.minus(muzzle);
      distance = toTarget.getNorm();
      if (distance < kEpsilon) {
        break;
      }
      double bearingDegrees = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
      heading = bearingDegrees - facingOffsetDegrees + headingTrimDegrees;
      muzzle =
          muzzlePosition(
              robotPose.getTranslation(),
              muzzleOffsetRobotFrame,
              Rotation2d.fromDegrees(heading));
    }
    return new AimSolution(MathUtil.inputModulus(heading, -180.0, 180.0), distance);
  }

  /**
   * Linear interpolation over a distance table with ascending keys, clamped at
   * both ends. Returns {@link Double#NaN} for an unusable table or a
   * non-finite key so the caller can fall back instead of firing a guess.
   *
   * @param keys ascending distances in meters
   * @param values table values (RPM) matching {@code keys}
   * @param key distance to sample
   */
  public static double interpolate(double[] keys, double[] values, double key) {
    if (keys.length == 0 || keys.length != values.length) {
      return Double.NaN;
    }
    if (!Double.isFinite(key)) {
      return Double.NaN;
    }
    if (key <= keys[0]) {
      return values[0];
    }
    for (int i = 1; i < keys.length; i++) {
      if (key <= keys[i]) {
        double span = keys[i] - keys[i - 1];
        if (span <= kEpsilon) {
          return values[i];
        }
        double t = (key - keys[i - 1]) / span;
        return values[i - 1] + t * (values[i] - values[i - 1]);
      }
    }
    return values[values.length - 1];
  }

  /** True when every key is finite and strictly increasing. */
  public static boolean isStrictlyAscending(double[] keys) {
    for (int i = 1; i < keys.length; i++) {
      if (!(keys[i] > keys[i - 1])) {
        return false;
      }
    }
    return true;
  }

  /**
   * Measured/target speed ratio, or {@link Double#NaN} when there is no valid
   * target. A zero target never counts as "at speed" (the stale-readiness bug
   * that makes a stopped wheel look ready).
   */
  public static double speedRatio(double measuredRpm, double targetRpm) {
    if (!(targetRpm > 0.0) || !Double.isFinite(measuredRpm)) {
      return Double.NaN;
    }
    return measuredRpm / targetRpm;
  }

  /**
   * Speed of the robot along the muzzle-to-target line.
   *
   * @return positive when the robot is closing on the target, negative when
   *     receding, in meters per second
   */
  public static double closingSpeed(
      Translation2d muzzle, Translation2d target, double fieldVx, double fieldVy) {
    Translation2d toTarget = target.minus(muzzle);
    double norm = toTarget.getNorm();
    if (norm < kEpsilon) {
      return 0.0;
    }
    return (toTarget.getX() * fieldVx + toTarget.getY() * fieldVy) / norm;
  }

  /**
   * Launch-speed correction for a moving robot: closing on the target adds the
   * robot's speed to the ball's ground speed, so the wheel is commanded slower.
   */
  public static double motionCompensatedRpm(
      double baseRpm, double closingSpeedMetersPerSecond, double gainRpmPerMps) {
    return baseRpm - gainRpmPerMps * closingSpeedMetersPerSecond;
  }

  /**
   * Suggests a {@link #motionCompensatedRpm} gain from the local slope of the
   * distance table: {@code dRPM/dDistance * timeOfFlight}. A flat table (the
   * default, uncalibrated one) returns 0, which disables compensation.
   */
  public static double suggestedMotionGain(
      double[] keys, double[] values, double key, double timeOfFlightSeconds) {
    if (keys.length < 2 || keys.length != values.length) {
      return 0.0;
    }
    double window = 0.25;
    double low = interpolate(keys, values, key - window);
    double high = interpolate(keys, values, key + window);
    if (!Double.isFinite(low) || !Double.isFinite(high)) {
      return 0.0;
    }
    double slope = (high - low) / (2.0 * window);
    return Math.max(0.0, slope * Math.max(0.0, timeOfFlightSeconds));
  }
}
