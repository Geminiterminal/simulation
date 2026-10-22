package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.SysIdSupport;
import frc.robot.driverprofiles.ThrustmasterProfile;
import java.util.Set;
import org.junit.jupiter.api.Test;

class SysIdCharacterizationTest {
  private static final double EPSILON = 1e-9;

  @Test
  void angularSysIdModuleAnglesProducePureCounterClockwiseSpinWithZeroScrub() {
    Rotation2d[] angles = DriveSubsystem.computeAngularSysIdModuleAngles();
    Translation2d[] modulePositions = {
      new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
      new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
      new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
      new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0)
    };

    double radiusMeters = Math.hypot(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0);
    SwerveModuleState[] unitTangentialStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      double rx = modulePositions[i].getX();
      double ry = modulePositions[i].getY();
      double tx = angles[i].getCos();
      double ty = angles[i].getSin();

      // Zero radial component -> wheels never scrub against each other on rectangular chassis.
      double radialDot = rx * tx + ry * ty;
      assertEquals(0.0, radialDot, EPSILON, "module " + i + " radial scrub");

      // Positive z-cross-product -> +Volts drives every module in the +CCW direction.
      double ccwMoment = rx * ty - ry * tx;
      assertEquals(radiusMeters, ccwMoment, EPSILON, "module " + i + " tangential moment arm");

      unitTangentialStates[i] = new SwerveModuleState(1.0, angles[i]);
    }

    ChassisSpeeds resultingSpeeds =
        DriveConstants.kDriveKinematics.toChassisSpeeds(unitTangentialStates);
    assertEquals(0.0, resultingSpeeds.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, resultingSpeeds.vyMetersPerSecond, EPSILON);
    assertEquals(1.0 / radiusMeters, resultingSpeeds.omegaRadiansPerSecond, EPSILON);
  }

  @Test
  void steerEncoderUnwrappingTracksMultipleTurnsAcrossTwoPiBoundary() {
    double continuous = 0.0;
    double prevRaw = 0.0;

    // Spin +2.5 full turns forward in 0.25-radian steps, wrapping raw encoder into [0, 2*pi).
    double totalExpected = 0.0;
    for (int step = 0; step < 63; step++) {
      totalExpected += 0.25;
      double wrappedRaw = totalExpected % (2.0 * Math.PI);
      continuous = MAXSwerveModule.unwrapAngleStepRadians(continuous, prevRaw, wrappedRaw);
      prevRaw = wrappedRaw;
      assertEquals(totalExpected, continuous, EPSILON);
    }

    // Spin back -3.0 full turns across 0 -> 2*pi wraparound.
    for (int step = 0; step < 75; step++) {
      totalExpected -= 0.25;
      double wrappedRaw = ((totalExpected % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
      continuous = MAXSwerveModule.unwrapAngleStepRadians(continuous, prevRaw, wrappedRaw);
      prevRaw = wrappedRaw;
      assertEquals(totalExpected, continuous, EPSILON);
    }
  }

  @Test
  void sparkAppliedVoltsUsesControllerBusOnHardwareAndSafeFallbackInSim() {
    assertEquals(
        5.75, MAXSwerveModule.computeAppliedVolts(0.5, 11.5, 12.2, false), EPSILON);
    assertEquals(
        5.75, MAXSwerveModule.computeAppliedVolts(0.5, 11.5, 12.2, true), EPSILON);
    // Tick-0 simulation fallback before SparkSim.iterate populates busVoltage.
    assertEquals(
        6.1, MAXSwerveModule.computeAppliedVolts(0.5, 0.0, 12.2, true), EPSILON);
  }

  @Test
  void deployArmSysIdLogsTrueJointRotationsAndEnforcesDirectionalTravelLimits() {
    // 0 motor rotations = 0 deg horizontal stowed; 59 motor rotations = 100 deg deployed.
    assertEquals(0.0, Intake.deployMotorRotationsToArmRotations(0.0), EPSILON);
    assertEquals(100.0 / 360.0, Intake.deployMotorRotationsToArmRotations(59.0), EPSILON);
    assertEquals(
        1.0 / SimulationConstants.kDeployArmGearRatio,
        Intake.deployMotorRpmToArmRotationsPerSecond(60.0),
        EPSILON);

    // At stowed (0.0 rotations): forward (+Volts) is allowed, reverse (-Volts) is blocked.
    assertEquals(2.5, Intake.clampDeploySysIdVolts(2.5, 0.0), EPSILON);
    assertEquals(0.0, Intake.clampDeploySysIdVolts(-2.5, 0.0), EPSILON);
    assertFalse(Intake.hasReachedDeploySysIdLimit(SysIdRoutine.Direction.kForward, 0.0));
    assertTrue(Intake.hasReachedDeploySysIdLimit(SysIdRoutine.Direction.kReverse, 0.0));

    // At deployed (56.0 rotations >= 55.0): forward (+Volts) is blocked, reverse (-Volts) allowed.
    assertEquals(0.0, Intake.clampDeploySysIdVolts(2.5, 56.0), EPSILON);
    assertEquals(-2.5, Intake.clampDeploySysIdVolts(-2.5, 56.0), EPSILON);
    assertTrue(Intake.hasReachedDeploySysIdLimit(SysIdRoutine.Direction.kForward, 56.0));
    assertFalse(Intake.hasReachedDeploySysIdLimit(SysIdRoutine.Direction.kReverse, 56.0));
  }

  @Test
  void thrustmasterSysIdButtonsAreDistinctValidAndDisjointFromTeleopBindings() {
    Set<Integer> teleopUsedButtons = Set.of(1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 14);
    int[] sysIdButtons = {
      ThrustmasterProfile.kSysIdQuasistaticForwardButton,
      ThrustmasterProfile.kSysIdQuasistaticReverseButton,
      ThrustmasterProfile.kSysIdDynamicForwardButton,
      ThrustmasterProfile.kSysIdDynamicReverseButton
    };

    Set<Integer> uniqueSysId = Set.of(
        sysIdButtons[0], sysIdButtons[1], sysIdButtons[2], sysIdButtons[3]);
    assertEquals(4, uniqueSysId.size());

    for (int btn : sysIdButtons) {
      assertTrue(btn >= 1 && btn <= 16, "button out of T.16000M range: " + btn);
      assertFalse(teleopUsedButtons.contains(btn), "collides with teleop button: " + btn);
    }
  }

  @Test
  void sysIdAutoKeysAreDistinctFromPathPlannerAutos() {
    assertFalse(SysIdSupport.isSysIdAutoKey(null));
    assertFalse(SysIdSupport.isSysIdAutoKey("NONE"));
    assertFalse(SysIdSupport.isSysIdAutoKey("Blue left"));

    assertTrue(SysIdSupport.isSysIdAutoKey(SysIdSupport.autoKeySelectedFull()));
    for (SysIdSupport.TestType type : SysIdSupport.TestType.values()) {
      assertTrue(SysIdSupport.isSysIdAutoKey(SysIdSupport.autoKeySelectedTest(type)));
    }
    for (SysIdSupport.Mechanism mech : SysIdSupport.Mechanism.values()) {
      if (mech != SysIdSupport.Mechanism.DISABLED) {
        assertTrue(SysIdSupport.isSysIdAutoKey(SysIdSupport.autoKeyMechanismFull(mech)));
      }
    }
  }
}
