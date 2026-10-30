package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/**
 * Covers the pure shot math behind the shooter improvements: aim geometry,
 * distance-table interpolation and shoot-on-the-move compensation. These tests
 * run with nothing but WPILib math, so {@code ./gradlew test} exercises them on
 * any laptop.
 */
class ShotMathTest {
  private static final double EPS = 1e-6;
  private static final Translation2d HUB = new Translation2d(4.626, 4.035);

  @Test
  void zeroOffsetReproducesTheLegacyCenterAimFormula() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0.0));

    ShotMath.AimSolution solution =
        ShotMath.solveAim(pose, HUB, new Translation2d(), 180.0, 0.0, 3);

    double legacy =
        Math.toDegrees(Math.atan2(HUB.getY() - pose.getY(), HUB.getX() - pose.getX())) - 180.0;
    assertEquals(legacy, solution.headingDegrees(), 1e-9);
    assertEquals(pose.getTranslation().getDistance(HUB), solution.distanceMeters(), EPS);
  }

  @Test
  void solvedHeadingPointsTheMuzzleAtTheTarget() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(37.0));
    Translation2d offset = new Translation2d(-0.25, 0.30);

    ShotMath.AimSolution solution = ShotMath.solveAim(pose, HUB, offset, 180.0, 0.0, 3);

    Translation2d muzzle =
        ShotMath.muzzlePosition(
            pose.getTranslation(), offset, Rotation2d.fromDegrees(solution.headingDegrees()));
    double bearing =
        Math.toDegrees(
            Math.atan2(HUB.getY() - muzzle.getY(), HUB.getX() - muzzle.getX()));
    double facing =
        MathUtil.inputModulus(solution.headingDegrees() + 180.0, -180.0, 180.0);

    assertEquals(0.0, MathUtil.inputModulus(bearing - facing, -180.0, 180.0), 0.5);
  }

  @Test
  void muzzleOffsetChangesTheRequiredHeading() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(37.0));

    double center =
        ShotMath.solveAim(pose, HUB, new Translation2d(), 180.0, 0.0, 3).headingDegrees();
    double offset =
        ShotMath.solveAim(pose, HUB, new Translation2d(-0.25, 0.30), 180.0, 0.0, 3)
            .headingDegrees();

    assertTrue(
        Math.abs(center - offset) > 0.5,
        "a 0.39 m lateral muzzle offset should move the aim by degrees, got "
            + (center - offset));
  }

  @Test
  void headingIsWrappedIntoTheContinuousPidRange() {
    Pose2d pose = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(10.0));

    ShotMath.AimSolution solution =
        ShotMath.solveAim(pose, HUB, new Translation2d(), 180.0, 0.0, 3);

    assertTrue(solution.headingDegrees() > -180.0 && solution.headingDegrees() <= 180.0);
    // The hub is almost straight behind the robot, so the commanded heading is ~0.
    assertEquals(0.0, solution.headingDegrees(), 10.0);
  }

  @Test
  void headingTrimShiftsTheCommandedHeading() {
    Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0.0));

    double base =
        ShotMath.solveAim(pose, HUB, new Translation2d(), 180.0, 0.0, 3).headingDegrees();
    double trimmed =
        ShotMath.solveAim(pose, HUB, new Translation2d(), 180.0, 2.5, 3).headingDegrees();

    assertEquals(2.5, trimmed - base, 1e-9);
  }

  @Test
  void interpolationHitsEndpointsMidpointsAndClamps() {
    double[] keys = {1.0, 2.0, 3.0};
    double[] values = {1000.0, 1500.0, 2200.0};

    assertEquals(1000.0, ShotMath.interpolate(keys, values, 0.5), EPS);
    assertEquals(1000.0, ShotMath.interpolate(keys, values, 1.0), EPS);
    assertEquals(1250.0, ShotMath.interpolate(keys, values, 1.5), EPS);
    assertEquals(1500.0, ShotMath.interpolate(keys, values, 2.0), EPS);
    assertEquals(1850.0, ShotMath.interpolate(keys, values, 2.5), EPS);
    assertEquals(2200.0, ShotMath.interpolate(keys, values, 9.0), EPS);
  }

  @Test
  void interpolationRejectsUnusableTables() {
    double[] keys = {1.0, 2.0};
    double[] values = {1000.0, 1500.0};

    assertTrue(Double.isNaN(ShotMath.interpolate(keys, values, Double.NaN)));
    assertTrue(Double.isNaN(ShotMath.interpolate(new double[0], new double[0], 1.5)));
    assertTrue(Double.isNaN(ShotMath.interpolate(keys, new double[0], 1.5)));
    assertTrue(Double.isNaN(ShotMath.interpolate(keys, new double[] {1.0}, 1.5)));
  }

  @Test
  void strictlyAscendingRejectsDuplicatesAndNaNs() {
    assertTrue(ShotMath.isStrictlyAscending(new double[] {1.0, 2.0, 3.0}));
    assertTrue(ShotMath.isStrictlyAscending(new double[] {1.0}));
    assertFalse(ShotMath.isStrictlyAscending(new double[] {1.0, 1.0, 2.0}));
    assertFalse(ShotMath.isStrictlyAscending(new double[] {2.0, 1.0}));
    assertFalse(ShotMath.isStrictlyAscending(new double[] {1.0, Double.NaN}));
  }

  @Test
  void speedRatioRejectsZeroAndInvalidTargets() {
    assertEquals(0.9, ShotMath.speedRatio(1350.0, 1500.0), EPS);
    assertTrue(Double.isNaN(ShotMath.speedRatio(1200.0, 0.0)));
    assertTrue(Double.isNaN(ShotMath.speedRatio(100.0, -5.0)));
    assertTrue(Double.isNaN(ShotMath.speedRatio(Double.NaN, 1500.0)));
  }

  @Test
  void closingSpeedAndCompensationHaveTheRightSigns() {
    Translation2d muzzle = new Translation2d();
    Translation2d target = new Translation2d(3.0, 0.0);

    assertEquals(2.0, ShotMath.closingSpeed(muzzle, target, 2.0, 0.0), EPS);
    assertEquals(-1.5, ShotMath.closingSpeed(muzzle, target, -1.5, 0.0), EPS);
    assertEquals(0.0, ShotMath.closingSpeed(muzzle, target, 0.0, 4.0), EPS);

    // Closing on the target means firing slower; backing away means firing harder.
    assertEquals(1400.0, ShotMath.motionCompensatedRpm(1500.0, 1.0, 100.0), EPS);
    assertEquals(1600.0, ShotMath.motionCompensatedRpm(1500.0, -1.0, 100.0), EPS);
    assertEquals(1500.0, ShotMath.motionCompensatedRpm(1500.0, 1.0, 0.0), EPS);
  }

  @Test
  void suggestedMotionGainFollowsTheLocalTableSlope() {
    double[] keys = {1.0, 2.0, 3.0, 4.0};
    double[] values = {1100.0, 1500.0, 1900.0, 2300.0}; // 400 RPM per meter

    assertEquals(200.0, ShotMath.suggestedMotionGain(keys, values, 2.5, 0.5), 5.0);

    double[] flat = {1500.0, 1500.0, 1500.0, 1500.0};
    assertEquals(0.0, ShotMath.suggestedMotionGain(keys, flat, 2.5, 0.5), EPS);
    assertEquals(0.0, ShotMath.suggestedMotionGain(new double[] {2.0}, new double[] {1500.0}, 2.0, 0.5), EPS);
  }

  @Test
  void muzzlePositionRotatesWithTheRobot() {
    Translation2d position =
        ShotMath.muzzlePosition(
            new Translation2d(5.0, 5.0), new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(90.0));

    assertEquals(5.0, position.getX(), EPS);
    assertEquals(6.0, position.getY(), EPS);
  }
}
