package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
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

  @Test
  void simulatedStartupPoseUsesTheAllianceHeadingAsGroundTruth() {
    Pose2d simulatedTruth = new Pose2d(1.25, 2.5, Rotation2d.fromDegrees(0.0));

    Pose2d blueStart = DriveSubsystem.withAllianceDefaultHeading(
        simulatedTruth, DriverStation.Alliance.Blue);
    Pose2d redStart = DriveSubsystem.withAllianceDefaultHeading(
        simulatedTruth, DriverStation.Alliance.Red);

    assertEquals(simulatedTruth.getTranslation(), blueStart.getTranslation());
    assertEquals(simulatedTruth.getTranslation(), redStart.getTranslation());
    assertEquals(180.0, blueStart.getRotation().getDegrees(), EPSILON);
    assertEquals(0.0, redStart.getRotation().getDegrees(), EPSILON);
  }

  @Test
  void oneVisionFrameCannotOfferAnUnboundedPoseCorrection() {
    double[] bounded = VisionFusion.limitMeasurementInnovation(
        1.0, 2.0, 179.0,
        5.0, 6.0, -30.0);

    double translationCorrection = Math.hypot(bounded[0] - 1.0, bounded[1] - 2.0);
    double headingCorrection = Math.abs(
        VisionFusion.wrapDegrees(bounded[2] - 179.0));

    assertEquals(VisionFusion.kMaxTranslationInnovationMeters, translationCorrection, EPSILON);
    assertEquals(VisionFusion.kMaxHeadingInnovationDegrees, headingCorrection, EPSILON);
  }

  @Test
  void headingInnovationUsesTheShortPathAcrossWraparound() {
    double[] bounded = VisionFusion.limitMeasurementInnovation(
        1.0, 2.0, 179.0,
        1.0, 2.0, -179.0);

    assertEquals(2.0, VisionFusion.wrapDegrees(bounded[2] - 179.0), EPSILON);
  }

  @Test
  void hubLocationFollowsTheAlliance() {
    assertEquals(FieldConstants.kBlueHubX,
        DriveSubsystem.getHubLocation(DriverStation.Alliance.Blue).getX(), EPSILON);
    assertEquals(FieldConstants.kBlueHubY,
        DriveSubsystem.getHubLocation(DriverStation.Alliance.Blue).getY(), EPSILON);
    assertEquals(FieldConstants.kRedHubX,
        DriveSubsystem.getHubLocation(DriverStation.Alliance.Red).getX(), EPSILON);
    assertEquals(FieldConstants.kRedHubY,
        DriveSubsystem.getHubLocation(DriverStation.Alliance.Red).getY(), EPSILON);
  }

  @Test
  void redHubIsTheBlueHubRotatedAboutTheFieldCenter() {
    // The 2026 field is rotationally symmetric, so the red hub has to be the
    // blue hub turned 180 degrees around the center of the field.
    Translation2d blueHub = DriveSubsystem.getHubLocation(DriverStation.Alliance.Blue);
    Translation2d redHub = DriveSubsystem.getHubLocation(DriverStation.Alliance.Red);

    assertEquals(FieldConstants.kFieldLengthMeters - redHub.getX(), blueHub.getX(), 0.01);
    assertEquals(FieldConstants.kFieldWidthMeters - redHub.getY(), blueHub.getY(), 0.01);
  }

  @Test
  void mirroringMovesTheRobotToTheRotatedSpotOnTheOtherSide() {
    Pose2d bluePose = new Pose2d(4.626, 4.035, Rotation2d.fromDegrees(30.0));
    Pose2d mirrored = DriveSubsystem.mirrorPoseAcrossField(bluePose);

    assertEquals(FieldConstants.kFieldLengthMeters - 4.626, mirrored.getX(), EPSILON);
    assertEquals(FieldConstants.kFieldWidthMeters - 4.035, mirrored.getY(), EPSILON);
    assertEquals(-150.0, mirrored.getRotation().getDegrees(), EPSILON);
  }

  @Test
  void mirroringIsItsOwnInverse() {
    Pose2d pose = new Pose2d(3.25, 6.5, Rotation2d.fromDegrees(42.0));
    Pose2d roundTrip =
        DriveSubsystem.mirrorPoseAcrossField(DriveSubsystem.mirrorPoseAcrossField(pose));

    assertEquals(pose.getX(), roundTrip.getX(), EPSILON);
    assertEquals(pose.getY(), roundTrip.getY(), EPSILON);
    assertEquals(pose.getRotation().getDegrees(), roundTrip.getRotation().getDegrees(), EPSILON);
  }

  @Test
  void mirroredPoseIsWherePathPlannerFlipsThePath() {
    // PathPlanner flips a blue-authored path for the red alliance by rotating
    // it 180 degrees about the field center, so the mirrored robot has to end
    // up exactly where the flipped path starts.
    Pose2d startPose = new Pose2d(5.0, 2.5, Rotation2d.fromDegrees(0.0));
    Pose2d flippedStart = DriveSubsystem.mirrorPoseAcrossField(startPose);

    assertEquals(FieldConstants.kFieldLengthMeters - 5.0, flippedStart.getX(), EPSILON);
    assertEquals(FieldConstants.kFieldWidthMeters - 2.5, flippedStart.getY(), EPSILON);
    assertEquals(-180.0, flippedStart.getRotation().getDegrees(), EPSILON);
  }

}