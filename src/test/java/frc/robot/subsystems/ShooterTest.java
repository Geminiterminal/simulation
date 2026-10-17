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
}