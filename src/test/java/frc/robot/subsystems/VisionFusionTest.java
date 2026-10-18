package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for the vision measurement policy: frame de-duplication, quality
 * gates, outlier rejection and trust scaling. Pure logic - runs anywhere with
 * {@code ./gradlew test}, no HAL or vendor libraries required.
 */
class VisionFusionTest {
  private static final double EPSILON = 1e-9;

  @Test
  void frameIsProcessedOncePerCapture() {
    VisionFusion fusion = new VisionFusion();

    assertTrue(fusion.shouldProcessFrame("cam", 10.0), "new frame must be processed");
    assertFalse(fusion.shouldProcessFrame("cam", 10.0), "same capture must be skipped");
    assertFalse(fusion.shouldProcessFrame("cam", 10.0 + 1e-9), "near-identical timestamps are the same capture");
    assertTrue(fusion.shouldProcessFrame("cam", 10.05), "next capture must be processed");
  }

  @Test
  void framesAreTrackedPerCamera() {
    VisionFusion fusion = new VisionFusion();

    assertTrue(fusion.shouldProcessFrame("back", 5.0));
    assertTrue(fusion.shouldProcessFrame("side", 5.0), "same timestamp on another camera is a different frame");
    assertFalse(fusion.shouldProcessFrame("back", 5.0));
  }

  @Test
  void fusionTimestampsMustAdvanceAcrossAllCameras() {
    VisionFusion fusion = new VisionFusion();
    fusion.resetFusionTimestamp(10.0);

    assertFalse(fusion.isFusionTimestampInOrder(9.99), "frames from before a pose reset are stale");
    assertTrue(fusion.isFusionTimestampInOrder(10.02), "a newer capture may be fused");
    fusion.recordFusionTimestamp(10.02);
    assertFalse(fusion.isFusionTimestampInOrder(10.01), "late frames must not rewind estimator history");
    assertFalse(fusion.isFusionTimestampInOrder(10.02), "one timestamp cannot be fused twice");
    assertTrue(fusion.isFusionTimestampInOrder(10.04), "the next capture remains eligible");
  }

  @Test
  void qualityGateRejectsAmbiguousAndDegenerateSolves() {
    assertFalse(VisionFusion.isFrameAcceptable(0.5, 2.0), "high ambiguity");
    assertFalse(VisionFusion.isFrameAcceptable(0.25, 2.0), "ambiguity above the gate");
    assertTrue(VisionFusion.isFrameAcceptable(0.1, 2.0), "clean solve");
    assertTrue(VisionFusion.isFrameAcceptable(-1.0, 2.0), "unknown ambiguity (multi-tag)");
    assertFalse(VisionFusion.isFrameAcceptable(0.1, 0.1), "near-field degenerate distance");
    assertFalse(VisionFusion.isFrameAcceptable(0.1, Double.NaN), "non-finite distance");
  }

  @Test
  void outlierGateRejectsGlitchesButAllowsDriftCorrection() {
    // Small disagreements (drift correction) pass.
    assertTrue(VisionFusion.isConsistentWithEstimate(10.0, 12.0, 0.3));
    // Wrap-around disagreements are measured on the circle.
    assertTrue(VisionFusion.isConsistentWithEstimate(179.0, -179.0, 0.3));
    // Wild heading or translation disagreements are glitches.
    assertFalse(VisionFusion.isConsistentWithEstimate(0.0, 90.0, 0.3));
    assertFalse(VisionFusion.isConsistentWithEstimate(0.0, 2.0, 3.0));
    assertFalse(VisionFusion.isConsistentWithEstimate(0.0, 2.0, Double.NaN));
  }

  @Test
  void trustScalesWithMeasurementQuality() {
    double[] clean = VisionFusion.measurementStdDevs(3, 0.02, 1.0, true);
    double[] ambiguous = VisionFusion.measurementStdDevs(1, 0.2, 1.0, false);
    double[] far = VisionFusion.measurementStdDevs(3, 0.02, 5.0, true);

    for (int i = 0; i < 3; i++) {
      assertTrue(clean[i] < ambiguous[i], "multi-tag clean solve must be trusted more (dim " + i + ")");
      assertTrue(clean[i] < far[i], "close solve must be trusted more (dim " + i + ")");
    }
  }

  @Test
  void trustScalingIsMonotonicInAmbiguityAndDistance() {
    double previous = 0.0;
    for (double ambiguity = 0.0; ambiguity <= 0.2 + 1e-9; ambiguity += 0.05) {
      double stddev = VisionFusion.measurementStdDevs(1, ambiguity, 2.0, false)[2];
      assertTrue(stddev > previous, "heading stddev must grow with ambiguity");
      previous = stddev;
    }

    previous = 0.0;
    for (double distance = 0.5; distance <= 5.0; distance += 1.0) {
      double stddev = VisionFusion.measurementStdDevs(1, 0.05, distance, false)[0];
      assertTrue(stddev > previous, "position stddev must grow with distance");
      previous = stddev;
    }
  }

  @Test
  void multiTagIsTrustedMoreThanSingleTag() {
    double[] multiTag = VisionFusion.measurementStdDevs(2, 0.05, 2.0, true);
    double[] singleTag = VisionFusion.measurementStdDevs(1, 0.05, 2.0, false);

    assertTrue(multiTag[0] < singleTag[0]);
    assertTrue(multiTag[2] < singleTag[2]);
    assertTrue(
        singleTag[2] > multiTag[2] * 5.0,
        "single-tag heading must be substantially less authoritative than multi-tag heading");
  }

  @Test
  void unknownAmbiguityIsTreatedAsClean() {
    assertEquals(
        VisionFusion.measurementStdDevs(1, 0.0, 2.0, false)[0],
        VisionFusion.measurementStdDevs(1, -1.0, 2.0, false)[0],
        EPSILON);
  }

  @Test
  void angleWrapIsSymmetric() {
    assertEquals(0.0, VisionFusion.wrapDegrees(0.0), EPSILON);
    assertEquals(-180.0, VisionFusion.wrapDegrees(180.0), EPSILON);
    assertEquals(179.0, VisionFusion.wrapDegrees(-181.0), EPSILON);
    assertEquals(-179.0, VisionFusion.wrapDegrees(181.0), EPSILON);
    assertEquals(10.0, VisionFusion.wrapDegrees(370.0), EPSILON);
    assertEquals(-10.0, VisionFusion.wrapDegrees(-370.0), EPSILON);
  }
}
