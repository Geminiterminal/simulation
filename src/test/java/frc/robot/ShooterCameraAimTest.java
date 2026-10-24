package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Covers the shooter-camera aim residual: the sign convention, the zero-error
 * case, per-target rejection (staleness, ambiguity, range, implausible
 * corrections) and the multi-tag average.
 *
 * <p>All geometry is set up on the unit circle so the expected angles are exact:
 * the robot sits at the field origin with heading 0 unless a test says
 * otherwise, and the camera is at the robot origin facing backwards (yaw 180),
 * i.e. along -x.
 */
class ShooterCameraAimTest {
  private static final ShooterCameraAim.Mount CENTER_MOUNT =
      new ShooterCameraAim.Mount(0.0, 0.0, 180.0);

  private static ShooterCameraAim.Config config() {
    return new ShooterCameraAim.Config(
        0.08, // max frame age (s)
        0.35, // max ambiguity
        0.75, // min range (m)
        7.5, // max range (m)
        8.0, // max aim correction (deg)
        0.75); // max range correction (m)
  }

  /** A tag 1 m straight out of the camera's lens: robot frame (-1, 0). */
  private static ShooterCameraAim.Target tag1mAt(int id, double tagFieldX, double tagFieldY) {
    return new ShooterCameraAim.Target(id, 1.0, 0.0, 0.05, tagFieldX, tagFieldY);
  }

  private static ShooterCameraAim.Correction evaluate(
      List<ShooterCameraAim.Target> targets, double headingDeg, ShooterCameraAim.Mount mount) {
    return ShooterCameraAim.evaluate(targets, 0.0, 0.0, headingDeg, mount, 0.02, config());
  }

  @Test
  void matchingPoseProducesNoCorrection() {
    ShooterCameraAim.Correction correction =
        evaluate(List.of(tag1mAt(1, -1.0, 0.0)), 0.0, CENTER_MOUNT);

    assertTrue(correction.valid());
    assertEquals("OK", correction.reason());
    assertEquals(0.0, correction.aimErrorDeg(), 1e-9);
    assertEquals(0.0, correction.rangeErrorMeters(), 1e-9);
    assertEquals(1, correction.acceptedTargets());
  }

  @Test
  void cameraOffsetAtTheMuzzleIsStillZeroResidual() {
    // Camera on the shooter, 12 in back and 10 in right of the robot centre,
    // looking out of the rear. The tag is placed exactly where that geometry
    // says it is, so the residual must still be zero: the mount offset is
    // modelled, not ignored. With the camera at (-0.3, -0.27) and yaw 180,
    // forward 1.0 / left 0 puts the tag at (-1.3, -0.27).
    ShooterCameraAim.Mount mount = new ShooterCameraAim.Mount(-0.3, -0.27, 180.0);

    ShooterCameraAim.Correction correction =
        evaluate(List.of(tag1mAt(1, -1.3, -0.27)), 0.0, mount);

    assertTrue(correction.valid());
    assertEquals(0.0, correction.aimErrorDeg(), 1e-9);
    assertEquals(0.0, correction.rangeErrorMeters(), 1e-9);
  }

  @Test
  void robotHeadingClockwiseOfThePoseNeedsMoreCounterClockwiseTurn() {
    // The pose says heading 0 but the tag was measured as if the robot were
    // rotated 5 deg counter-clockwise: the field point lands 5 deg further
    // round, so the pose is 5 deg clockwise of truth and the aim error has to
    // grow by +5 deg.
    ShooterCameraAim.Correction correction =
        evaluate(List.of(tag1mAt(1, -1.0, 0.0)), 5.0, CENTER_MOUNT);

    assertTrue(correction.valid());
    assertEquals(5.0, correction.aimErrorDeg(), 1e-9);
  }

  @Test
  void rangeResidualIsMeasuredMinusPredicted() {
    // Camera measures 1.2 m but the layout says the tag is 1.0 m away, so the
    // pose is under-reading the distance by 0.2 m.
    ShooterCameraAim.Target measured = new ShooterCameraAim.Target(1, 1.2, 0.0, 0.05, -1.0, 0.0);

    ShooterCameraAim.Correction correction =
        evaluate(List.of(measured), 0.0, CENTER_MOUNT);

    assertTrue(correction.valid());
    assertEquals(0.0, correction.aimErrorDeg(), 1e-9);
    assertEquals(0.2, correction.rangeErrorMeters(), 1e-9);
  }

  @Test
  void staleFramesAreRejected() {
    ShooterCameraAim.Correction fresh =
        ShooterCameraAim.evaluate(
            List.of(tag1mAt(1, -1.0, 0.0)), 0.0, 0.0, 0.0, CENTER_MOUNT, 0.02, config());
    assertTrue(fresh.valid());

    ShooterCameraAim.Correction stale =
        ShooterCameraAim.evaluate(
            List.of(tag1mAt(1, -1.0, 0.0)), 0.0, 0.0, 0.0, CENTER_MOUNT, 0.40, config());

    assertFalse(stale.valid());
    assertEquals("STALE FRAME", stale.reason());
    assertEquals(0.0, stale.aimErrorDeg(), 1e-9);
  }

  @Test
  void ambiguousTargetIsDroppedButUnknownAmbiguityIsAccepted() {
    ShooterCameraAim.Target ambiguous = new ShooterCameraAim.Target(1, 1.0, 0.0, 0.60, -1.0, 0.0);
    assertFalse(evaluate(List.of(ambiguous), 0.0, CENTER_MOUNT).valid());
    assertEquals("AMBIGUOUS", evaluate(List.of(ambiguous), 0.0, CENTER_MOUNT).reason());

    ShooterCameraAim.Target unknown = new ShooterCameraAim.Target(1, 1.0, 0.0, -1.0, -1.0, 0.0);
    assertTrue(evaluate(List.of(unknown), 0.0, CENTER_MOUNT).valid());
  }

  @Test
  void targetsOutsideTheUsefulRangeAreRejected() {
    ShooterCameraAim.Target far = new ShooterCameraAim.Target(1, 9.0, 0.0, 0.02, -9.0, 0.0);

    ShooterCameraAim.Correction correction = evaluate(List.of(far), 0.0, CENTER_MOUNT);

    assertFalse(correction.valid());
    assertEquals("TOO FAR", correction.reason());
  }

  @Test
  void implausibleCorrectionsAreRejectedInsteadOfYankingTheAim() {
    // 20 deg of apparent error is a bad tag or a broken pose, not an aim error.
    double twentyDeg = Math.toRadians(20.0);
    ShooterCameraAim.Target wrong =
        new ShooterCameraAim.Target(1, 1.0, 0.0, 0.02, -Math.cos(twentyDeg), -Math.sin(twentyDeg));

    ShooterCameraAim.Correction correction = evaluate(List.of(wrong), 0.0, CENTER_MOUNT);

    assertFalse(correction.valid());
    assertEquals("CORRECTION TOO LARGE", correction.reason());
  }

  @Test
  void severalTargetsAreAveraged() {
    // The measured projection sits 1 deg (tag 1) and 3 deg (tag 2)
    // counter-clockwise of where the layout puts them, so the pose over-reports
    // heading by 1 and 3 deg and the frame average is +2 deg.
    ShooterCameraAim.Target oneDeg =
        new ShooterCameraAim.Target(1, 1.0, 0.0, 0.02,
            -Math.cos(Math.toRadians(1.0)), Math.sin(Math.toRadians(1.0)));
    ShooterCameraAim.Target threeDeg =
        new ShooterCameraAim.Target(2, 1.0, 0.0, 0.02,
            -Math.cos(Math.toRadians(3.0)), Math.sin(Math.toRadians(3.0)));

    ShooterCameraAim.Correction correction =
        evaluate(List.of(oneDeg, threeDeg), 0.0, CENTER_MOUNT);

    assertTrue(correction.valid());
    assertEquals(2.0, correction.aimErrorDeg(), 1e-9);
    assertEquals(2, correction.acceptedTargets());
  }

  @Test
  void emptyFrameAndMissingPoseAreReportedNotGuessed() {
    assertEquals("NO TARGET", evaluate(List.of(), 0.0, CENTER_MOUNT).reason());
    assertEquals("NO POSE", evaluate(List.of(tag1mAt(1, -1.0, 0.0)), 0.0, null).reason());

    ShooterCameraAim.Correction noPose =
        ShooterCameraAim.evaluate(
            List.of(tag1mAt(1, -1.0, 0.0)), Double.NaN, 0.0, 0.0, CENTER_MOUNT, 0.02, config());
    assertFalse(noPose.valid());
    assertEquals("NO POSE", noPose.reason());
  }

  @Test
  void wrapDegreesNormalisesEveryAngle() {
    assertEquals(0.0, ShooterCameraAim.wrapDegrees(0.0), 1e-9);
    assertEquals(170.0, ShooterCameraAim.wrapDegrees(170.0), 1e-9);
    assertEquals(-170.0, ShooterCameraAim.wrapDegrees(190.0), 1e-9);
    assertEquals(-90.0, ShooterCameraAim.wrapDegrees(270.0), 1e-9);
    assertEquals(180.0, ShooterCameraAim.wrapDegrees(-180.0), 1e-9);
    assertEquals(0.0, ShooterCameraAim.wrapDegrees(360.0), 1e-9);
    assertEquals(0.0, ShooterCameraAim.wrapDegrees(Double.NaN), 1e-9);
  }
}
