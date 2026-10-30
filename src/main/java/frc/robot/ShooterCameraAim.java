package frc.robot;

import java.util.List;

/**
 * Turns a single PhotonVision frame from the camera mounted on the shooter into
 * a small correction to the pose-based aim.
 *
 * <p>Why this exists: {@code DriveSubsystem}'s aim comes from the fused pose
 * estimator, which is smoothed on purpose (innovation limits, frame ranking,
 * outlier rejection). That is the right trade for driving, but it means the
 * number the shooter aims with is up to ~100 ms of fused motion behind the
 * robot, and it is only as good as the odometry under it. A camera bolted to
 * the shooter measures the <em>angle to the target</em> directly, in the same
 * frame the ball has to leave from, so it can tell us how wrong the pose is
 * right now.
 *
 * <p>The correction is deliberately expressed as a <b>residual</b>, not as a
 * second pose estimate:
 *
 * <ul>
 *   <li>Take one AprilTag detection, projected into field coordinates with the
 *       pose the robot is currently using.
 *   <li>Compare it with where the field layout says that tag actually is, as
 *       seen from the camera.
 *   <li>Report the angular and radial difference.
 * </ul>
 *
 * <p>If the pose is right the residual is zero and nothing changes. If the pose
 * is wrong - a defender hit, wheels slipping, a drifted gyro, an unmeasured
 * muzzle offset - the residual is exactly the error that would have missed the
 * shot, and adding it to {@code ShotInputs.headingErrorDeg} /
 * {@code ShotInputs.distanceMeters} fixes the aim without touching the pose
 * estimator, without a multi-tag solve, and without a full PnP solve.
 *
 * <p>Sign convention: {@code aimErrorDeg} is <b>added</b> to the heading error
 * the shooter already uses (degrees, counter-clockwise positive). A positive
 * value means the robot must turn further counter-clockwise than the pose-based
 * solution said. {@code rangeErrorMeters} is <b>added</b> to the distance.
 *
 * <p>Pure logic: no hardware, no NetworkTables, no WPILib. {@code
 * ShooterCameraAimTest} covers every branch.
 */
public final class ShooterCameraAim {
  private ShooterCameraAim() {}

  /**
   * One AprilTag detection, in the two frames the math needs.
   *
   * @param tagId fiducial id, for telemetry only
   * @param forwardMeters target centre distance along the camera's boresight
   *     (PhotonVision camera frame +X, out of the lens)
   * @param leftMeters target centre offset to the camera's left (camera frame
   *     +Y); negative means the tag is to the right of the lens axis
   * @param ambiguity PhotonVision pose ambiguity, {@code -1} when unknown
   * @param tagFieldXMeters tag centre X from the AprilTag field layout
   * @param tagFieldYMeters tag centre Y from the AprilTag field layout
   */
  public record Target(
      int tagId,
      double forwardMeters,
      double leftMeters,
      double ambiguity,
      double tagFieldXMeters,
      double tagFieldYMeters) {}

  /**
   * Where the camera sits on the robot, in the robot frame (+x forward, +y
   * left), plus which way it looks.
   *
   * @param xMeters camera X in the robot frame
   * @param yMeters camera Y in the robot frame
   * @param yawDeg camera boresight direction in the robot frame, CCW positive
   *     from +x (180 = looking out of the back of the robot)
   */
  public record Mount(double xMeters, double yMeters, double yawDeg) {}

  /**
   * Acceptance limits.
   *
   * @param maxAgeSeconds oldest frame worth trusting (latency budget)
   * @param maxAmbiguity largest pose ambiguity to believe; {@code -1} readings
   *     are always accepted
   * @param minRangeMeters closest tag to believe (guards against a tag that is
   *     actually a reflection or a partially visible edge)
   * @param maxRangeMeters furthest tag to believe (distant tags have poor
   *     angle resolution and belong to other field elements)
   * @param maxAimCorrectionDeg largest angular residual to apply; anything
   *     bigger is a bad pose or a mis-identified tag, not an aim error
   * @param maxRangeCorrectionMeters largest radial residual to apply
   */
  public record Config(
      double maxAgeSeconds,
      double maxAmbiguity,
      double minRangeMeters,
      double maxRangeMeters,
      double maxAimCorrectionDeg,
      double maxRangeCorrectionMeters) {}

  /**
   * The result of one frame.
   *
   * @param valid true when at least one target produced a usable correction
   * @param reason short human-readable explanation, shown on the dashboard
   * @param aimErrorDeg degrees to add to {@code ShotInputs.headingErrorDeg}
   * @param rangeErrorMeters metres to add to {@code ShotInputs.distanceMeters}
   * @param ageSeconds frame age at the moment it was evaluated
   * @param acceptedTargets number of detections that passed every filter
   * @param meanAmbiguity mean ambiguity of the accepted detections
   */
  public record Correction(
      boolean valid,
      String reason,
      double aimErrorDeg,
      double rangeErrorMeters,
      double ageSeconds,
      int acceptedTargets,
      double meanAmbiguity) {}

  /** Returned when there is no frame at all (camera unplugged, no tag). */
  public static final Correction NONE =
      new Correction(false, "NO FRAME", 0.0, 0.0, Double.NaN, 0, Double.NaN);

  /**
   * Evaluates one frame.
   *
   * @param targets every detection in the frame, already in the units above
   * @param robotXMeters current fused pose X
   * @param robotYMeters current fused pose Y
   * @param robotHeadingDeg current fused heading, CCW positive
   * @param mount camera mounting transform
   * @param ageSeconds {@code Timer.getFPGATimestamp() - frame.getTimestampSeconds()}
   * @param config acceptance limits
   * @return a correction; {@link Correction#valid()} is false when nothing is
   *     usable, and the correction terms are then zero
   */
  public static Correction evaluate(
      List<Target> targets,
      double robotXMeters,
      double robotYMeters,
      double robotHeadingDeg,
      Mount mount,
      double ageSeconds,
      Config config) {
    if (targets == null || targets.isEmpty()) {
      return new Correction(false, "NO TARGET", 0.0, 0.0, ageSeconds, 0, Double.NaN);
    }
    if (!finite(robotXMeters)
        || !finite(robotYMeters)
        || !finite(robotHeadingDeg)
        || mount == null
        || !finite(mount.xMeters())
        || !finite(mount.yMeters())
        || !finite(mount.yawDeg())) {
      return new Correction(false, "NO POSE", 0.0, 0.0, ageSeconds, 0, Double.NaN);
    }
    // Clock skew between the coprocessor and the roboRIO can make a perfectly
    // fresh frame look a few milliseconds in the future; clamp that to zero
    // instead of throwing the frame away.
    double age = finite(ageSeconds) ? Math.max(0.0, ageSeconds) : Double.NaN;
    if (!finite(age) || age > Math.abs(config.maxAgeSeconds())) {
      return new Correction(false, "STALE FRAME", 0.0, 0.0, ageSeconds, 0, Double.NaN);
    }

    // Camera and robot are rigidly connected, so both rotations are fixed for
    // the whole frame: compute them once.
    double mountRad = Math.toRadians(mount.yawDeg());
    double cosMount = Math.cos(mountRad);
    double sinMount = Math.sin(mountRad);
    double headingRad = Math.toRadians(robotHeadingDeg());
    double cosHeading = Math.cos(headingRad);
    double sinHeading = Math.sin(headingRad);

    // Camera position in field coordinates.
    double cameraFieldX =
        robotXMeters + cosHeading * mount.xMeters() - sinHeading * mount.yMeters();
    double cameraFieldY =
        robotYMeters + sinHeading * mount.xMeters() + cosHeading * mount.yMeters();

    double maxAmbiguity = config.maxAmbiguity() < 0.0 ? Double.POSITIVE_INFINITY : config.maxAmbiguity();
    double aimSum = 0.0;
    double rangeSum = 0.0;
    double ambiguitySum = 0.0;
    int accepted = 0;
    int rejectedAmbiguous = 0;
    int rejectedRange = 0;
    int rejectedLarge = 0;

    for (Target target : targets) {
      if (target == null
          || !finite(target.forwardMeters())
          || !finite(target.leftMeters())
          || !finite(target.tagFieldXMeters())
          || !finite(target.tagFieldYMeters())) {
        continue;
      }

      double ambiguity = target.ambiguity();
      if (ambiguity >= 0.0 && ambiguity > maxAmbiguity) {
        rejectedAmbiguous++;
        continue;
      }

      double measuredRange = Math.hypot(target.forwardMeters(), target.leftMeters());
      if (measuredRange < Math.abs(config.minRangeMeters())
          || measuredRange > Math.abs(config.maxRangeMeters())) {
        rejectedRange++;
        continue;
      }

      // Detection -> robot frame -> field frame.
      double robotFrameX =
          mount.xMeters() + cosMount * target.forwardMeters() - sinMount * target.leftMeters();
      double robotFrameY =
          mount.yMeters() + sinMount * target.forwardMeters() + cosMount * target.leftMeters();
      double measuredFieldX =
          robotXMeters + cosHeading * robotFrameX - sinHeading * robotFrameY;
      double measuredFieldY =
          robotYMeters + sinHeading * robotFrameX + cosHeading * robotFrameY;

      double measuredBearing = Math.atan2(measuredFieldY - cameraFieldY, measuredFieldX - cameraFieldX);
      double predictedBearing =
          Math.atan2(target.tagFieldYMeters() - cameraFieldY, target.tagFieldXMeters() - cameraFieldX);

      // Positive = the tag is further counter-clockwise than the pose predicted,
      // i.e. the robot is further clockwise than the pose believes.
      double aimErrorDeg = wrapDegrees(Math.toDegrees(measuredBearing - predictedBearing));
      double rangeErrorMeters =
          measuredRange
              - Math.hypot(
                  target.tagFieldXMeters() - cameraFieldX,
                  target.tagFieldYMeters() - cameraFieldY);

      if (Math.abs(aimErrorDeg) > Math.abs(config.maxAimCorrectionDeg())
          || Math.abs(rangeErrorMeters) > Math.abs(config.maxRangeCorrectionMeters())) {
        rejectedLarge++;
        continue;
      }

      aimSum += aimErrorDeg;
      rangeSum += rangeErrorMeters;
      ambiguitySum += ambiguity;
      accepted++;
    }

    if (accepted == 0) {
      String reason = "NO USABLE TAG";
      if (rejectedAmbiguous > 0) {
        reason = "AMBIGUOUS";
      } else if (rejectedRange > 0) {
        reason = "TOO FAR";
      } else if (rejectedLarge > 0) {
        reason = "CORRECTION TOO LARGE";
      }
      return new Correction(false, reason, 0.0, 0.0, ageSeconds, 0, Double.NaN);
    }

    return new Correction(
        true,
        "OK",
        aimSum / accepted,
        rangeSum / accepted,
        ageSeconds,
        accepted,
        ambiguitySum / accepted);
  }

  /** Wraps an angle in degrees into (-180, 180]. */
  public static double wrapDegrees(double degrees) {
    if (!finite(degrees)) {
      return 0.0;
    }
    double wrapped = ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    return wrapped <= -180.0 ? 180.0 : wrapped;
  }

  private static boolean finite(double value) {
    return !Double.isNaN(value) && !Double.isInfinite(value);
  }
}
