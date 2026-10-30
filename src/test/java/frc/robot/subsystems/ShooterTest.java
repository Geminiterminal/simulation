package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterTest {
  @Test
  void normalShotWaitsForFlywheelAimAndDistance() {
    assertFalse(Shooter.isReadyToFeed(false, true, true, false));
    assertFalse(Shooter.isReadyToFeed(true, false, true, false));
    assertFalse(Shooter.isReadyToFeed(true, true, false, false));
    assertTrue(Shooter.isReadyToFeed(true, true, true, false));
  }

  @Test
  void alignmentOverrideFeedsWhenFlywheelsReachTargetSpeed() {
    assertTrue(Shooter.isReadyToFeed(true, false, false, true));
  }

  @Test
  void alignmentOverrideNeverBypassesFlywheelSpeed() {
    assertFalse(Shooter.isReadyToFeed(false, true, true, true));
    assertFalse(Shooter.isReadyToFeed(false, false, false, true));
  }

  @Test
  void enablingAlignmentOverrideImmediatelyChangesReadiness() {
    assertFalse(Shooter.isReadyToFeed(true, false, false, false));
    assertTrue(Shooter.isReadyToFeed(true, false, false, true));
  }

  @Test
  void speedToleranceIsAPercentageBandThatRejectsZeroTargets() {
    assertTrue(Shooter.isSpeedWithinTolerance(1500.0, 1520.0, 0.03));
    assertTrue(Shooter.isSpeedWithinTolerance(1500.0, 1470.0, 0.03));
    assertFalse(Shooter.isSpeedWithinTolerance(1500.0, 1400.0, 0.03));
    assertFalse(Shooter.isSpeedWithinTolerance(1500.0, 1700.0, 0.03));

    // A stopped wheel must never look "at speed" for a zero or invalid target.
    assertFalse(Shooter.isSpeedWithinTolerance(0.0, 0.0, 0.03));
    assertFalse(Shooter.isSpeedWithinTolerance(-10.0, 0.0, 0.03));
    assertFalse(Shooter.isSpeedWithinTolerance(1500.0, Double.NaN, 0.03));
  }
}
