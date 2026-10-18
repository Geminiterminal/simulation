package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for the simulated gyro sensor model. Pure math - runs anywhere
 * with {@code ./gradlew test}.
 */
class SimulatedGyroTest {
  private static final double EPSILON = 1e-9;

  @Test
  void angleIntegratesTrueRateWhenPerfect() {
    SimulatedGyro gyro = new SimulatedGyro(0.0, 0.0, 0L);

    for (int i = 0; i < 100; i++) {
      gyro.update(90.0, 0.02); // 90 deg/s for 2 seconds
    }

    assertEquals(180.0, gyro.getAngleDegrees(), 1e-6);
    assertEquals(90.0, gyro.getRateDegreesPerSecond(), EPSILON);
  }

  @Test
  void driftAccumulatesLikeARealGyroBias() {
    double drift = 0.05; // deg/s
    SimulatedGyro gyro = new SimulatedGyro(drift, 0.0, 0L);

    for (int i = 0; i < 500; i++) {
      gyro.update(0.0, 0.02); // 10 seconds standing still
    }

    assertEquals(drift * 10.0, gyro.getAngleDegrees(), 1e-6);
  }

  @Test
  void angleIsIntegratedFromRateNotTruthSoTeleportsDoNotSnapIt() {
    SimulatedGyro gyro = new SimulatedGyro(0.0, 0.0, 0L);
    gyro.update(10.0, 0.5); // 5 degrees of real rotation

    // The simulated ground truth may be teleported (odometry reset / alliance
    // change) but the physical gyro keeps its angle - the estimator's gyro
    // offset absorbs the difference.
    assertEquals(5.0, gyro.getAngleDegrees(), 1e-6);
  }

  @Test
  void noiseIsBoundedAndSeededDeterministically() {
    SimulatedGyro a = new SimulatedGyro(0.0, 1.0, 12345L);
    SimulatedGyro b = new SimulatedGyro(0.0, 1.0, 12345L);

    double maxRate = 0.0;
    for (int i = 0; i < 1000; i++) {
      a.update(0.0, 0.02);
      b.update(0.0, 0.02);
      assertEquals(a.getAngleDegrees(), b.getAngleDegrees(), 0.0, "same seed must reproduce the same run");
      maxRate = Math.max(maxRate, Math.abs(a.getRateDegreesPerSecond()));
    }

    // A 1-sigma rate noise of 1 deg/s should not produce absurd rates.
    assertTrue(maxRate > 0.0 && maxRate < 6.0, "noise rate out of bounds: " + maxRate);
  }

  @Test
  void resetToForcesTheReportedAngle() {
    SimulatedGyro gyro = new SimulatedGyro(0.1, 0.1, 0L);
    gyro.update(50.0, 1.0);
    gyro.resetTo(42.0);

    assertEquals(42.0, gyro.getAngleDegrees(), EPSILON);
    assertEquals(0.0, gyro.getRateDegreesPerSecond(), EPSILON);
  }
}
