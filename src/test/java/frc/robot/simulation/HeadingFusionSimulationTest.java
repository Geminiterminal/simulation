package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SimulatedGyro;
import frc.robot.subsystems.VisionFusion;

/**
 * Closed-loop heading/pose-estimation simulation: drives the exact estimator
 * pipeline the robot uses (gyro + wheel odometry + latency-stamped vision
 * fusion through the {@link VisionFusion} policy) against a noisy drifting gyro
 * and noisy vision, and checks that the estimated heading stays accurate,
 * stable and smooth during movement, turns and navigation.
 *
 * <p>Runs nothing but WPILib math (no HAL, no vendor libraries), so it runs
 * anywhere: {@code ./gradlew test}. The same scenarios are mirrored by
 * {@code tests/test_heading_estimation.py}, which also pins the auto-start
 * reset behavior that first exposed the heading bug.
 */
class HeadingFusionSimulationTest {
  private static final double DT = 0.020; // 50 Hz robot loop
  private static final double VISION_PERIOD = 1.0 / 30.0; // 30 fps cameras
  private static final double VISION_LATENCY = 0.030; // seconds
  private static final double AMBIGUITY = 0.05;
  private static final double TAG_DISTANCE = 2.0; // meters
  private static final int TARGETS_USED = 2; // multi-tag solutions in these scenarios

  /** One simulated sensor sample + estimator update. */
  private static final class Step {
    final double time;
    final Pose2d truth;
    final Pose2d estimate;

    Step(double time, Pose2d truth, Pose2d estimate) {
      this.time = time;
      this.truth = truth;
      this.estimate = estimate;
    }
  }

  /** History of ground truth so vision frames can be rendered at capture time. */
  private static final class TruthHistory {
    private final List<double[]> m_samples = new ArrayList<>(); // {t, x, y, headingRad}

    void add(double t, Pose2d pose) {
      m_samples.add(
          new double[] {
            t, pose.getX(), pose.getY(), pose.getRotation().getRadians()
          });
    }

    Pose2d at(double t) {
      double[] prev = m_samples.get(0);
      double[] next = prev;
      for (double[] sample : m_samples) {
        if (sample[0] > t) {
          next = sample;
          break;
        }
        prev = sample;
        next = sample;
      }
      if (next == prev || next[0] == prev[0]) {
        return new Pose2d(prev[1], prev[2], new Rotation2d(prev[3]));
      }
      double alpha = (t - prev[0]) / (next[0] - prev[0]);
      double heading = prev[3] + alpha * Math.IEEEremainder(next[3] - prev[3], 2.0 * Math.PI);
      return new Pose2d(
          prev[1] + alpha * (next[1] - prev[1]),
          prev[2] + alpha * (next[2] - prev[2]),
          new Rotation2d(heading));
    }
  }

  /** The robot-side pipeline under test, wired exactly like DriveSubsystem. */
  private static final class Harness {
    final SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    final SwerveDrivePoseEstimator estimator;
    final SimulatedGyro gyro;
    final VisionFusion fusion = new VisionFusion();
    final TruthHistory history = new TruthHistory();
    final Random visionNoise = new Random(4242L);

    Pose2d truth = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    final double[] moduleDistance = new double[4];
    double time = 0.0;
    double nextVisionTime = 0.1;
    boolean seeded = false;

    Harness(double gyroDriftDegPerSec) {
      gyro = new SimulatedGyro(gyroDriftDegPerSec, 0.02, 987L);
      gyro.resetTo(truth.getRotation().getDegrees());
      estimator =
          new SwerveDrivePoseEstimator(
              kinematics,
              gyroRotation(),
              modulePositions(kinematics.toSwerveModuleStates(new ChassisSpeeds())),
              truth);
      seeded = true; // Harness starts from a known pose, like an odometry reset.
      history.add(0.0, truth);
    }

    Rotation2d gyroRotation() {
      return Rotation2d.fromDegrees(gyro.getAngleDegrees());
    }

    SwerveModulePosition[] modulePositions(SwerveModuleState[] states) {
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        positions[i] = new SwerveModulePosition(moduleDistance[i], states[i].angle);
      }
      return positions;
    }

    /** Advances one robot loop: physics, gyro, odometry and (maybe) vision. */
    Step step(ChassisSpeeds robotRelativeSpeeds, boolean visionEnabled) {
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
      for (int i = 0; i < 4; i++) {
        moduleDistance[i] += states[i].speedMetersPerSecond * DT;
      }

      truth =
          truth.exp(
              new Twist2d(
                  robotRelativeSpeeds.vxMetersPerSecond * DT,
                  robotRelativeSpeeds.vyMetersPerSecond * DT,
                  robotRelativeSpeeds.omegaRadiansPerSecond * DT));
      gyro.update(Units.radiansToDegrees(robotRelativeSpeeds.omegaRadiansPerSecond), DT);
      time += DT;
      history.add(time, truth);

      // Estimator update: gyro angle + wheel odometry (the robot's periodic()).
      Pose2d estimate =
          estimator.updateWithTime(time, gyroRotation(), modulePositions(states));

      // Vision frames arrive at 30 fps, stamped with their capture time
      // (arrival minus pipeline latency, exactly like PhotonVision does).
      if (visionEnabled) {
        while (time >= nextVisionTime) {
          fuseVisionFrameAt(nextVisionTime - VISION_LATENCY);
          nextVisionTime += VISION_PERIOD;
        }
      }
      return new Step(time, truth, estimator.getEstimatedPosition());
    }

    void fuseVisionFrameAt(double captureTime) {
      Pose2d truthAtCapture = history.at(captureTime);
      boolean multiTag = TARGETS_USED >= 2;
      double headingNoiseDeg = multiTag ? 0.8 : 2.0;
      Pose2d measurement =
          new Pose2d(
              truthAtCapture.getX() + visionNoise.nextGaussian() * 0.03,
              truthAtCapture.getY() + visionNoise.nextGaussian() * 0.03,
              truthAtCapture
                  .getRotation()
                  .plus(Rotation2d.fromDegrees(visionNoise.nextGaussian() * headingNoiseDeg)));

      // --- The exact DriveSubsystem fusion policy (VisionFusion) ---
      if (!fusion.shouldProcessFrame("sim-cam", captureTime)) return;
      if (!VisionFusion.isFrameAcceptable(AMBIGUITY, TAG_DISTANCE)) return;
      if (!seeded) {
        estimator.resetPosition(gyroRotation(), modulePositions(currentStates()), measurement);
        seeded = true;
        return;
      }
      if (!VisionFusion.isConsistentWithEstimate(
          estimator.getEstimatedPosition().getRotation().getDegrees(),
          measurement.getRotation().getDegrees(),
          estimator.getEstimatedPosition().getTranslation().getDistance(measurement.getTranslation()))) {
        return;
      }
      double[] stddevs =
          VisionFusion.measurementStdDevs(TARGETS_USED, AMBIGUITY, TAG_DISTANCE, multiTag);
      estimator.addVisionMeasurement(
          measurement, captureTime, VecBuilder.fill(stddevs[0], stddevs[1], stddevs[2]));
    }

    SwerveModuleState[] currentStates() {
      return kinematics.toSwerveModuleStates(new ChassisSpeeds());
    }
  }

  private static double headingErrorDegrees(Step step) {
    return Math.abs(
        VisionFusion.wrapDegrees(
            step.estimate.getRotation().getDegrees() - step.truth.getRotation().getDegrees()));
  }

  @Test
  void headingStaysAccurateThroughDriveArcAndTurn() {
    Harness harness = new Harness(0.02); // realistic navX-ish drift
    List<Step> steps = new ArrayList<>();

    // Drive forward while arcing, then turn in place, then translate in field.
    for (int i = 0; i < 150; i++) {
      ChassisSpeeds speeds;
      if (i < 50) {
        speeds = new ChassisSpeeds(1.0, 0.0, Units.degreesToRadians(30.0)); // arc
      } else if (i < 100) {
        speeds = new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(90.0)); // in-place turn
      } else {
        speeds = new ChassisSpeeds(0.0, 1.0, Units.degreesToRadians(-45.0)); // strafe while turning
      }
      steps.add(harness.step(speeds, true));
    }

    double maxError = 0.0;
    double sumSquares = 0.0;
    for (Step step : steps) {
      double error = headingErrorDegrees(step);
      maxError = Math.max(maxError, error);
      sumSquares += error * error;
    }
    double rms = Math.sqrt(sumSquares / steps.size());

    assertTrue(maxError < 3.0, "heading error peaked at " + maxError + " deg during motion");
    assertTrue(rms < 1.5, "heading RMS error " + rms + " deg during motion");
  }

  @Test
  void visionRejectsGyroDriftAfterAnOutage() {
    // Exaggerated drift (0.5 deg/s) so the outage is clearly visible: after
    // vision returns, the fusion must pull the heading back to truth.
    Harness harness = new Harness(0.5);
    Step last = null;

    // Move while vision is available.
    for (int i = 0; i < 50; i++) {
      last = harness.step(new ChassisSpeeds(0.5, 0.0, Units.degreesToRadians(10.0)), true);
    }
    double errorBeforeOutage = headingErrorDegrees(last);
    assertTrue(errorBeforeOutage < 2.0, "pre-outage heading error " + errorBeforeOutage);

    // 5 s vision outage while drifting.
    for (int i = 0; i < 250; i++) {
      last = harness.step(new ChassisSpeeds(0.5, 0.0, 0.0), false);
    }
    assertTrue(headingErrorDegrees(last) > 0.5, "drift should accumulate without vision");

    // Vision returns: heading must converge back to truth.
    for (int i = 0; i < 50; i++) {
      last = harness.step(new ChassisSpeeds(0.5, 0.0, 0.0), true);
    }
    double errorAfter = headingErrorDegrees(last);
    assertTrue(errorAfter < 2.0, "heading did not recover after vision returned: " + errorAfter + " deg");
  }

  @Test
  void headingUpdatesSmoothlyWithoutJumps() {
    Harness harness = new Harness(0.02);
    Step previous = null;
    double maxExcessStep = 0.0;

    for (int i = 0; i < 150; i++) {
      ChassisSpeeds speeds =
          i < 75
              ? new ChassisSpeeds(1.0, 0.0, Units.degreesToRadians(60.0))
              : new ChassisSpeeds(0.0, 0.0, Units.degreesToRadians(-120.0));
      Step step = harness.step(speeds, true);
      if (previous != null) {
        double estimateStep =
            Math.abs(
                VisionFusion.wrapDegrees(
                    step.estimate.getRotation().getDegrees()
                        - previous.estimate.getRotation().getDegrees()));
        double truthStep =
            Math.abs(
                VisionFusion.wrapDegrees(
                    step.truth.getRotation().getDegrees() - previous.truth.getRotation().getDegrees()));
        // Heading changes the real motion does not explain can only come from
        // the estimator/vision pipeline. Every such change must be a small
        // Kalman correction - never a snap or a flip.
        maxExcessStep = Math.max(maxExcessStep, estimateStep - truthStep);
      }
      previous = step;
    }

    assertTrue(
        maxExcessStep < 2.0,
        "estimated heading jumped " + maxExcessStep + " deg beyond real motion in one loop");
  }

  @Test
  void authoredStartHeadingIsAuthoritativeAtAutoStart() {
    // Regression for the heading bug: an auto authored at 0 deg must start at
    // 0 deg even when the pre-auto heading was the 180 deg alliance default.
    Harness harness = new Harness(0.02);

    // Robot sitting at the alliance default (180 deg) before the auto starts.
    Pose2d pathStart = new Pose2d(4.17, 0.69, Rotation2d.fromDegrees(0.0));
    harness.estimator.resetPosition(
        harness.gyroRotation(), harness.modulePositions(harness.currentStates()), pathStart);
    harness.truth = pathStart; // resetOdometry teleports ground truth too
    harness.history.add(harness.time, pathStart);

    // The first path waypoint holds heading 0 - no spin may be commanded.
    for (int i = 0; i < 25; i++) {
      harness.step(new ChassisSpeeds(0.0, 0.0, 0.0), true);
    }
    Step step = harness.step(new ChassisSpeeds(0.0, 0.0, 0.0), true);

    assertTrue(headingErrorDegrees(step) < 1.0, "heading diverged from the authored start heading");
    assertTrue(
        Math.abs(step.estimate.getRotation().getDegrees() - pathStart.getRotation().getDegrees()) < 1.0,
        "estimate heading " + step.estimate.getRotation().getDegrees() + " does not match the path start");
  }

  @Test
  void duplicatedFrameTimestampsAreNeverFusedTwice() {
    Harness harness = new Harness(0.02);

    // Build up odometry history without any automatic vision frames.
    for (int i = 0; i < 10; i++) {
      harness.step(new ChassisSpeeds(0.0, 0.0, 0.0), false);
    }
    double captureTime = harness.time - VISION_LATENCY;

    // Same capture handed to the pipeline twice: only the first may be fused.
    Pose2d beforeFirst = harness.estimator.getEstimatedPosition();
    harness.fuseVisionFrameAt(captureTime);
    Pose2d afterFirst = harness.estimator.getEstimatedPosition();
    harness.fuseVisionFrameAt(captureTime);
    Pose2d afterSecond = harness.estimator.getEstimatedPosition();

    assertFalse(beforeFirst.equals(afterFirst), "first copy of the frame must be fused");
    assertTrue(afterFirst.equals(afterSecond), "duplicate frame must not move the pose again");
    assertFalse(harness.fusion.shouldProcessFrame("sim-cam", captureTime));
  }
}
