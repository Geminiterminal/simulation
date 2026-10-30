package frc.robot;

/**
 * One loop of shooter-relevant measurements, produced by {@link RobotContainer}
 * and consumed by {@link frc.robot.subsystems.Shooter}.
 *
 * <p>Keeping the measurements in a record decouples the shooter subsystem from
 * the drivetrain: the shooter only needs to know where the hub is relative to
 * the muzzle, not how odometry or vision produced that number.
 *
 * @param distanceMeters distance from the muzzle to the hub; {@link Double#NaN} when unknown
 * @param headingErrorDeg degrees between the current heading and the shot
 * @param linearSpeedMps chassis translation speed (m/s)
 * @param yawRateDps chassis yaw rate (deg/s, counter-clockwise positive)
 * @param closingSpeedMps robot speed toward the hub (m/s, positive = closing)
 */
public record ShotInputs(
    double distanceMeters,
    double headingErrorDeg,
    double linearSpeedMps,
    double yawRateDps,
    double closingSpeedMps) {

  /** Neutral inputs: no distance information and no motion. */
  public static final ShotInputs NONE =
      new ShotInputs(Double.NaN, 0.0, 0.0, 0.0, 0.0);
}
