package frc.robot;

/**
 * Decides, every loop, whether it is safe to feed a game piece into the
 * flywheels.
 *
 * <p>This replaces the old "spin up once, then feed forever" latch. Four
 * independent conditions have to hold at the same time, and each one has the
 * timing behaviour a real match needs:
 *
 * <ul>
 *   <li><b>Speed</b> - both flywheels within a percentage of target, for a
 *       short settle time, with a hysteresis band so a wheel sitting on the
 *       tolerance edge cannot chatter the feeder on and off.
 *   <li><b>Aim</b> - heading error inside the tolerance, held for a settle
 *       time, with hysteresis so a robot being pushed by defense keeps firing
 *       instead of stuttering.
 *   <li><b>Distance</b> - only used when the robot is driving to a fixed
 *       shooting distance; ignored when the distance/RPM table is in charge.
 *   <li><b>Motion</b> - the chassis is slow enough that the ball leaves where
 *       the shot was aimed.
 * </ul>
 *
 * <p>Pure logic: no hardware, no NetworkTables. {@code ShotGateTest} covers
 * every branch.
 */
public final class ShotGate {

  /**
   * Once ready, the flywheel speed may drift this much past the tolerance band
   * before readiness is dropped. Prevents motor noise from cycling the feeder.
   */
  public static final double kSpeedHysteresisFactor = 1.5;

  /** Status of the last {@link #update} call. */
  public record Status(boolean speedReady, boolean aimed, boolean atDistance, boolean motionOk) {}

  /** Status used before the first update, and after {@link #reset()}. */
  public static final Status NOT_READY = new Status(false, false, false, false);

  /**
   * Per-loop measurements.
   *
   * @param speedRatio measured/target RPM of the <em>worst</em> flywheel;
   *     {@link Double#NaN} when there is no valid nonzero target
   * @param headingErrorDeg degrees between the current heading and the shot
   * @param distanceErrorMeters meters from the desired shooting distance;
   *     {@link Double#NaN} disables the distance gate
   * @param distanceMeters muzzle-to-hub distance; {@link Double#NaN} when
   *     unknown. Used only by the moving-shot envelope.
   * @param linearSpeedMps chassis translation speed
   * @param yawRateDps chassis yaw rate
   */
  public record Inputs(
      double speedRatio,
      double headingErrorDeg,
      double distanceErrorMeters,
      double distanceMeters,
      double linearSpeedMps,
      double yawRateDps) {}

  /**
   * Per-loop tolerances, normally built from {@link ShooterTuning}.
   *
   * @param movingSpeedThresholdMps speed above which a shot counts as "on the
   *     move" and the moving-shot distance limit applies
   * @param movingMaxDistanceMeters furthest shot allowed while moving;
   *     {@code <= 0} disables the limit, which is the default, so raising
   *     {@code maxLinearSpeedMps} on its own never changes behaviour
   */
  public record Config(
      double speedToleranceFraction,
      double speedSettleSeconds,
      double aimEnterToleranceDeg,
      double aimHysteresisDeg,
      double aimSettleSeconds,
      double distanceEnterToleranceMeters,
      double distanceHysteresisMeters,
      double maxLinearSpeedMps,
      double maxYawRateDps,
      double movingSpeedThresholdMps,
      double movingMaxDistanceMeters) {}

  /** "In band for N seconds" timer. */
  private static final class Dwell {
    private boolean active;
    private double seconds;

    boolean update(boolean insideBand, double requiredSeconds, double dtSeconds) {
      if (!insideBand) {
        active = false;
        seconds = 0.0;
        return false;
      }
      seconds += dtSeconds;
      if (seconds >= requiredSeconds) {
        active = true;
      }
      return active;
    }

    boolean active() {
      return active;
    }

    void reset() {
      active = false;
      seconds = 0.0;
    }
  }

  private final Dwell speedDwell = new Dwell();
  private final Dwell aimDwell = new Dwell();
  private boolean atDistance;

  /** Clears all timing state; call when the shooter is started or stopped. */
  public void reset() {
    speedDwell.reset();
    aimDwell.reset();
    atDistance = false;
  }

  /** True once the flywheels have been within tolerance for the settle time. */
  public boolean speedReady() {
    return speedDwell.active();
  }

  /**
   * Runs one gate update.
   *
   * @param dtSeconds loop period
   * @return the combined status
   */
  public Status update(double dtSeconds, Inputs inputs, Config config) {
    double dt = sanitizeDt(dtSeconds);

    // Flywheel speed: percentage band with hysteresis and a settle time.
    double speedTolerance = Math.abs(config.speedToleranceFraction());
    double speedBand =
        speedDwell.active() ? speedTolerance * kSpeedHysteresisFactor : speedTolerance;
    boolean speedInBand =
        Double.isFinite(inputs.speedRatio()) && Math.abs(inputs.speedRatio() - 1.0) <= speedBand;
    speedDwell.update(speedInBand, Math.max(0.0, config.speedSettleSeconds()), dt);

    // Heading: enter on the tight tolerance, leave on the wider one.
    double aimEnter = Math.abs(config.aimEnterToleranceDeg());
    double aimExit = aimEnter + Math.max(0.0, config.aimHysteresisDeg());
    double aimError = finiteOrZero(inputs.headingErrorDeg());
    boolean aimInBand = aimError <= (aimDwell.active() ? aimExit : aimEnter);
    aimDwell.update(aimInBand, Math.max(0.0, config.aimSettleSeconds()), dt);

    // Distance: only meaningful when the robot is ranging to a fixed distance.
    if (!Double.isFinite(inputs.distanceErrorMeters())) {
      atDistance = true;
    } else {
      double distanceEnter = Math.abs(config.distanceEnterToleranceMeters());
      double distanceExit = distanceEnter + Math.max(0.0, config.distanceHysteresisMeters());
      double distanceError = Math.abs(inputs.distanceErrorMeters());
      atDistance = distanceError <= (atDistance ? distanceExit : distanceEnter);
    }

    // Chassis motion: a spinning or sprinting robot changes where the ball goes.
    // On top of the flat speed limit, a moving shot can be restricted to a
    // shorter range band: with a fixed launch angle the flight time grows with
    // distance, so the further out the robot is the more its own motion and
    // the table's slope error show up at the hub.
    boolean motionOk =
        withinLimit(inputs.linearSpeedMps(), config.maxLinearSpeedMps())
            && withinLimit(inputs.yawRateDps(), config.maxYawRateDps())
            && insideMovingShotEnvelope(inputs, config);

    return new Status(speedDwell.active(), aimDwell.active(), atDistance, motionOk);
  }

  /**
   * Moving-shot envelope: while the robot is rolling, only trust shots inside
   * {@code movingMaxDistanceMeters}. A missing distance or a disabled limit
   * never blocks the shot - the same "do not block on a sensor glitch" rule the
   * other gates use.
   */
  private static boolean insideMovingShotEnvelope(Inputs inputs, Config config) {
    double limit = config.movingMaxDistanceMeters();
    if (!Double.isFinite(limit) || limit <= 0.0) {
      return true;
    }
    double threshold = Math.abs(config.movingSpeedThresholdMps());
    boolean moving = Double.isFinite(inputs.linearSpeedMps())
        && Math.abs(inputs.linearSpeedMps()) > threshold;
    if (!moving) {
      return true;
    }
    if (!Double.isFinite(inputs.distanceMeters())) {
      return true;
    }
    return inputs.distanceMeters() <= limit;
  }

  private static boolean withinLimit(double value, double limit) {
    if (!Double.isFinite(value)) {
      return true; // no measurement: do not block the shot on a sensor glitch
    }
    return Math.abs(value) <= Math.abs(limit);
  }

  private static double finiteOrZero(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }

  private static double sanitizeDt(double dtSeconds) {
    if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0) {
      return 0.02;
    }
    return Math.min(dtSeconds, 0.1);
  }
}
