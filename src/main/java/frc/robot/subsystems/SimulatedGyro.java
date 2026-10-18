package frc.robot.subsystems;

import java.util.Random;

/**
 * Sensor model for the simulated gyroscope (the navX MXP's stand-in).
 *
 * <p>The gyroscope is the primary heading sensor: the pose estimator trusts its
 * (latency-free) angle deltas over the wheel odometry, and the vision pipeline
 * is given the resulting field heading. A real gyro is not perfect - it has a
 * small zero-rate bias and angle noise, and its angle drifts over time. Modelling
 * that here means the simulation exercises the same sensor fusion the real robot
 * needs: wheel odometry carries translation, the gyro carries rotation between
 * vision frames, and vision measurements correct the accumulated drift.
 *
 * <p>This class is pure math (no WPILib) so its behavior is pinned by unit
 * tests and mirrored by the Python harness in {@code tests/}.
 */
public class SimulatedGyro {
  private final double m_driftDegreesPerSecond;
  private final double m_noiseStdDevDegreesPerSecond;
  private final Random m_random;

  // CCW-positive, matching the WPILib/field convention used everywhere else.
  private double m_angleDegrees = 0.0;
  private double m_rateDegreesPerSecond = 0.0;

  /**
   * Creates a simulated gyro.
   *
   * @param driftDegreesPerSecond constant zero-rate bias (degrees per second)
   * @param noiseStdDevDegreesPerSecond standard deviation of the rate white noise
   * @param seed seed for the noise generator (deterministic simulations)
   */
  public SimulatedGyro(
      double driftDegreesPerSecond, double noiseStdDevDegreesPerSecond, long seed) {
    m_driftDegreesPerSecond = driftDegreesPerSecond;
    m_noiseStdDevDegreesPerSecond = Math.max(0.0, noiseStdDevDegreesPerSecond);
    m_random = new Random(seed);
  }

  /**
   * Integrates the gyro over one loop using the robot's true yaw rate.
   *
   * <p>The angle is integrated from the measured rate (plus bias and noise)
   * rather than read from the true pose, so the simulated gyro drifts exactly
   * like the real sensor. Teleporting the simulated ground truth (resetting
   * odometry, changing alliance) therefore does NOT snap the gyro - the pose
   * estimator's gyro offset absorbs the difference, as it does when the real
   * robot is carried to a new starting pose.
   *
   * @param trueYawRateDegreesPerSecond the robot's actual yaw rate (CCW-positive)
   * @param dtSeconds loop period
   */
  public void update(double trueYawRateDegreesPerSecond, double dtSeconds) {
    m_rateDegreesPerSecond =
        trueYawRateDegreesPerSecond
            + m_driftDegreesPerSecond
            + m_random.nextGaussian() * m_noiseStdDevDegreesPerSecond;
    m_angleDegrees += m_rateDegreesPerSecond * dtSeconds;
  }

  /** @return the current gyro angle in degrees (CCW-positive) */
  public double getAngleDegrees() {
    return m_angleDegrees;
  }

  /** @return the current gyro rate in degrees per second (CCW-positive) */
  public double getRateDegreesPerSecond() {
    return m_rateDegreesPerSecond;
  }

  /**
   * Forces the gyro to read the given angle. Used by tests and at construction;
   * normal operation never needs this because odometry offsets absorb the
   * gyro-to-pose difference at every reset.
   *
   * @param degrees the angle to set
   */
  public void resetTo(double degrees) {
    m_angleDegrees = degrees;
    m_rateDegreesPerSecond = 0.0;
  }
}
