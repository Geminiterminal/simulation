package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.Test;

class DriveSubsystemTest {
  private static final double EPSILON = 1e-9;

  @Test
  void fieldRelativeDrivingUsesVisionSeededHeading() {
    Pose2d visionPose = new Pose2d(4.5, 6.25, Rotation2d.fromDegrees(42.0));
    ChassisSpeeds robotSpeeds = DriveSubsystem.calculateFieldRelativeSpeeds(
        1.0, 0.0, 0.0, visionPose.getRotation());
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        robotSpeeds, visionPose.getRotation());

    assertEquals(1.0, fieldSpeeds.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, fieldSpeeds.vyMetersPerSecond, EPSILON);
  }

  @Test
  void fieldRelativeDrivingNeverFallsBackToRobotRelative() {
    ChassisSpeeds speeds = DriveSubsystem.calculateFieldRelativeSpeeds(
        1.0, 0.0, 0.0, Rotation2d.fromDegrees(90.0));

    assertEquals(0.0, speeds.vxMetersPerSecond, EPSILON);
    assertEquals(-1.0, speeds.vyMetersPerSecond, EPSILON);
  }

  @Test
  void robotRelativeAutoDistanceIsUnaffectedByEstimatorHeading() {
    ChassisSpeeds expected = DriveSubsystem.calculateRobotRelativeNormalizedSpeeds(
        -0.35, 0.0, 0.0);

    for (double headingDegrees : new double[] {0.0, 90.0, 180.0}) {
      // Unlike a field-relative command, estimator heading is deliberately not
      // an input to robot-relative normalized motion.
      ChassisSpeeds actual = DriveSubsystem.calculateRobotRelativeNormalizedSpeeds(
          -0.35, 0.0, 0.0);

      assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, EPSILON,
          "vx at estimator heading " + headingDegrees);
      assertEquals(-0.35 * DriveConstants.kMaxSpeedMetersPerSecond,
          actual.vxMetersPerSecond, EPSILON);
      assertEquals(0.0, actual.vyMetersPerSecond, EPSILON);
    }
  }

  @Test
  void normalFieldRelativeCommandsRemainHeadingDependent() {
    ChassisSpeeds atZero = DriveSubsystem.calculateFieldRelativeSpeeds(
        1.0, 0.0, 0.0, Rotation2d.fromDegrees(0.0));
    ChassisSpeeds atNinety = DriveSubsystem.calculateFieldRelativeSpeeds(
        1.0, 0.0, 0.0, Rotation2d.fromDegrees(90.0));
    ChassisSpeeds atOneEighty = DriveSubsystem.calculateFieldRelativeSpeeds(
        1.0, 0.0, 0.0, Rotation2d.fromDegrees(180.0));

    assertEquals(1.0, atZero.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, atZero.vyMetersPerSecond, EPSILON);
    assertEquals(0.0, atNinety.vxMetersPerSecond, EPSILON);
    assertEquals(-1.0, atNinety.vyMetersPerSecond, EPSILON);
    assertEquals(-1.0, atOneEighty.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, atOneEighty.vyMetersPerSecond, EPSILON);
  }

  @Test
  void allianceDefaultsUseOpposingFieldHeadings() {
    assertEquals(180.0,
        DriveSubsystem.getAllianceDefaultHeading(DriverStation.Alliance.Blue).getDegrees(),
        EPSILON);
    assertEquals(0.0,
        DriveSubsystem.getAllianceDefaultHeading(DriverStation.Alliance.Red).getDegrees(),
        EPSILON);
  }

}