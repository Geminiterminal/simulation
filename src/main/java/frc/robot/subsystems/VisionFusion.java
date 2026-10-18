package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

/**
 * Policy layer for vision pose measurements: frame intake bookkeeping, quality
 * gates, outlier rejection and measurement-trust (standard deviation) scaling.
 *
 * <p>This class is deliberately free of WPILib and PhotonVision types so the
 * policy can be unit tested anywhere ({@code ./gradlew test}) and mirrored by
 * the Python simulation harness in {@code tests/test_heading_estimation.py}.
 * {@link DriveSubsystem} adapts PhotonVision results to these primitives and
 * hands the returned standard deviations to the WPILib pose estimator.
 *
 * <p>Why this exists: fusing every camera frame at a fixed trust makes the
 * estimated heading jump whenever a noisy or ambiguous solution arrives, and
 * fusing the same captured frame twice double-counts one measurement. The
 * estimator's Kalman gain only produces smooth, consistent heading updates if
 * each captured frame is fused exactly once and its trust reflects how good the
 * measurement actually is.
 */
public final class VisionFusion {
  /** Reject single-tag poses with a solve ambiguity above this (1 = worst). */
  public static final double kMaxPoseAmbiguity = 0.2;

  /** Pose ambiguity below zero means "not reported" (e.g. multi-tag). */
  public static final double kUnknownPoseAmbiguity = -1.0;

  /** Closer than this the tag pose solve is degenerate (near-field PNP blowup). */
  public static final double kMinCameraToTagDistanceMeters = 0.22;

  /**
   * Post-seed outlier gate: a fused measurement may differ from the current
   * estimate by at most this much before it is treated as a glitch rather than
   * a correction. Sized to allow real drift correction while rejecting single
   * bad solves.
   */
  public static final double kMaxVisionTranslationMeters = 1.5;

  /** Heading counterpart of {@link #kMaxVisionTranslationMeters}, in degrees. */
  public static final double kMaxVisionHeadingDegrees = 45.0;

  // --- Measurement trust scaling (see measurementStdDevs) ---
  /** 1-sigma position trust of a clean, close, multi-tag frame. */
  private static final double kBasePositionStdDevMeters = 0.25;

  /** 1-sigma heading trust of a clean, close, multi-tag frame (radians). */
  private static final double kBaseHeadingStdDevRadians = Math.toRadians(6.0);

  /** Ambiguity penalty factor: trust scales with (1 + k * ambiguity). */
  private static final double kAmbiguityPenalty = 2.0;

  /** Distance penalty: trust scales with (1 + distance / k). */
  private static final double kDistancePenaltyMeters = 3.0;

  /** Multi-tag solutions are geometrically stronger than any single-tag solve. */
  private static final double kMultiTagBonus = 0.6;

  /** Timestamps of the most recent frame accepted for each camera. */
  private final Map<String, Double> m_lastProcessedFrame = new HashMap<>();

  /** Creates the frame bookkeeping for one robot's worth of cameras. */
  public VisionFusion() {
    // Static policy + per-camera frame bookkeeping; constructed once per robot.
  }

  /**
   * Returns whether this frame should be processed and records it. Cameras
   * publish at 30 fps while the robot loop runs at 50 Hz, and NetworkTables can
   * redeliver values, so one capture can show up more than once. Fusing one
   * capture twice counts the same measurement twice and makes the heading
   * update in visible steps.
   *
   * @param cameraName the camera the frame came from
   * @param timestampSeconds the frame's capture timestamp
   * @return true if this is a frame that has not been processed before
   */
  public synchronized boolean shouldProcessFrame(String cameraName, double timestampSeconds) {
    Double last = m_lastProcessedFrame.get(cameraName);
    if (last != null && Math.abs(last - timestampSeconds) < 1e-6) {
      return false;
    }
    m_lastProcessedFrame.put(cameraName, timestampSeconds);
    return true;
  }

  /**
   * Coarse quality gate applied before a pose solve is attempted.
   *
   * @param poseAmbiguity the best target's pose ambiguity, or negative if unknown
   * @param cameraToTagDistanceMeters distance from the camera to the tag
   * @return true if the frame is worth solving
   */
  public static boolean isFrameAcceptable(double poseAmbiguity, double cameraToTagDistanceMeters) {
    if (!Double.isFinite(cameraToTagDistanceMeters)
        || cameraToTagDistanceMeters < kMinCameraToTagDistanceMeters) {
      return false;
    }
    return isAmbiguityAcceptable(poseAmbiguity);
  }

  /**
   * Returns whether a solve ambiguity is trustworthy enough to use.
   *
   * @param poseAmbiguity the solve ambiguity, or negative if unknown
   * @return true if the ambiguity is acceptable
   */
  public static boolean isAmbiguityAcceptable(double poseAmbiguity) {
    return poseAmbiguity == kUnknownPoseAmbiguity
        || (poseAmbiguity >= 0.0 && poseAmbiguity <= kMaxPoseAmbiguity);
  }

  /**
   * Outlier gate used after the heading has been seeded: rejects measurements
   * that disagree wildly with the current estimate (glitches, misidentified
   * tags) while still allowing vision to correct gyro drift and accumulated
   * odometry error.
   *
   * @param estimatedHeadingDegrees current estimated heading
   * @param measuredHeadingDegrees heading from the vision solve
   * @param translationDeltaMeters distance between estimated and measured position
   * @return true if the measurement is consistent enough to fuse
   */
  public static boolean isConsistentWithEstimate(
      double estimatedHeadingDegrees, double measuredHeadingDegrees, double translationDeltaMeters) {
    if (!Double.isFinite(translationDeltaMeters)
        || translationDeltaMeters > kMaxVisionTranslationMeters) {
      return false;
    }
    double headingDeltaDegrees = Math.abs(wrapDegrees(measuredHeadingDegrees - estimatedHeadingDegrees));
    return Double.isFinite(headingDeltaDegrees) && headingDeltaDegrees <= kMaxVisionHeadingDegrees;
  }

  /**
   * Scales measurement trust by how good the solve actually is: multi-tag
   * solutions beat single-tag, clean ambiguities beat marginal ones, and nearby
   * tags beat far ones. The values are handed to the WPILib pose estimator as
   * per-measurement standard deviations, so a bad frame moves the heading a
   * little and a great frame moves it a lot - smoothly, in one step per frame.
   *
   * @param targetsUsed how many targets went into the solve
   * @param poseAmbiguity best-target ambiguity, or negative if unknown
   * @param cameraToTagDistanceMeters camera-to-tag distance of the best target
   * @param multiTagSolution whether the solve was a multi-tag PNP solution
   * @return {xStdDevMeters, yStdDevMeters, headingStdDevRadians}
   */
  public static double[] measurementStdDevs(
      int targetsUsed,
      double poseAmbiguity,
      double cameraToTagDistanceMeters,
      boolean multiTagSolution) {
    double ambiguity = poseAmbiguity < 0.0 ? 0.0 : Math.max(0.0, poseAmbiguity);
    double distance = Math.max(0.0, cameraToTagDistanceMeters);
    double quality = (1.0 + kAmbiguityPenalty * ambiguity) * (1.0 + distance / kDistancePenaltyMeters);
    double tagStrength = (multiTagSolution || targetsUsed >= 2) ? kMultiTagBonus : 1.0;
    double scale = quality * tagStrength;
    return new double[] {
      kBasePositionStdDevMeters * scale,
      kBasePositionStdDevMeters * scale,
      kBaseHeadingStdDevRadians * scale
    };
  }

  /**
   * Wraps an angle to the range [-180, 180) degrees.
   *
   * @param degrees the angle
   * @return the wrapped angle
   */
  public static double wrapDegrees(double degrees) {
    double wrapped = Math.IEEEremainder(degrees, 360.0);
    // IEEEremainder maps -180 to -180 and 180 to 180; fold 180 to -180 so the
    // range is symmetric and comparisons are unambiguous.
    return wrapped == 180.0 ? -180.0 : wrapped;
  }
}
