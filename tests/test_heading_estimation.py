"""Closed-loop heading/pose-estimation simulation.

This mirrors, in Python, exactly what the robot code does:

* WPILib's 2026 ``PoseEstimator`` fusion math (gyro + wheel odometry, vision
  measurements fused at their capture timestamps with a constant Kalman gain
  and persistent vision compensation) - ported from wpimath ``PoseEstimator`` /
  ``Odometry`` (WPILib 2026.2.1).
* The robot's fusion policy in ``VisionFusion.java`` (frame de-duplication,
  quality gates, outlier rejection, measurement-trust scaling).
* The simulated gyro sensor model in ``SimulatedGyro.java`` (zero-rate bias +
  rate noise integrated from the true yaw rate).
* The ``DriveSubsystem.resetOdometry`` semantics: an odometry reset (e.g. the
  PathPlanner auto-start pose) is authoritative for translation AND rotation,
  in the estimator and in the simulated ground truth.

Scenarios cover movement, turns and navigation: auto-start heading alignment,
arc + in-place-turn accuracy with a drifting gyro and noisy vision, drift
rejection after a vision outage, smooth (jump-free) heading updates, duplicate
frame handling, and latency-correct fusion while turning.

Run with:  python3 -m unittest discover -s tests
"""

import math
import random
import re
import unittest
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

DT = 0.020  # 50 Hz robot loop
VISION_PERIOD = 1.0 / 30.0  # 30 fps cameras
VISION_LATENCY = 0.030  # seconds

# ---------------------------------------------------------------------------
# Geometry (WPILib Pose2d / Transform2d / Twist2d semantics)
# ---------------------------------------------------------------------------


def wrap_radians(angle):
    return math.remainder(angle, 2.0 * math.pi)


def wrap_degrees(degrees):
    wrapped = math.remainder(degrees, 360.0)
    return -180.0 if wrapped == 180.0 else wrapped


@dataclass
class Pose2d:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, CCW positive

    def exp(self, dx, dy, dtheta):
        """WPILib Pose2d.exp(Twist2d): apply a body-frame twist."""
        sin_t = math.sin(dtheta)
        cos_t = math.cos(dtheta)
        if abs(dtheta) < 1e-9:
            s = 1.0 - dtheta * dtheta / 6.0
            c = 0.5 * dtheta
        else:
            s = sin_t / dtheta
            c = (1.0 - cos_t) / dtheta
        sdx = dx * s - dy * c
        sdy = dx * c + dy * s
        return self.transform_by(sdx, sdy, dtheta)

    def transform_by(self, dx, dy, dtheta):
        """WPILib Pose2d.transformBy(Transform2d)."""
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        return Pose2d(
            self.x + dx * cos_t - dy * sin_t,
            self.y + dx * sin_t + dy * cos_t,
            wrap_radians(self.theta + dtheta),
        )

    def minus(self, other):
        """WPILib Pose2d.minus(Pose2d) -> Transform2d (other -> self, in other's frame)."""
        dx = self.x - other.x
        dy = self.y - other.y
        cos_t = math.cos(other.theta)
        sin_t = math.sin(other.theta)
        return (
            cos_t * dx + sin_t * dy,
            -sin_t * dx + cos_t * dy,
            wrap_radians(self.theta - other.theta),
        )

    def plus(self, transform):
        return self.transform_by(*transform)

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

    @staticmethod
    def interpolate(a, b, alpha):
        return Pose2d(
            a.x + alpha * (b.x - a.x),
            a.y + alpha * (b.y - a.y),
            wrap_radians(a.theta + alpha * wrap_radians(b.theta - a.theta)),
        )


# ---------------------------------------------------------------------------
# WPILib 2026 PoseEstimator port (wpimath PoseEstimator.java / Odometry.java)
# ---------------------------------------------------------------------------


class PoseEstimator:
    """Fuses gyro + wheel odometry with latency-stamped vision measurements.

    Ports the 2026 wpimath implementation: odometry carries the pose between
    vision frames (rotation from the gyro angle via an internal offset), and
    each vision measurement contributes a scaled residual that is applied as a
    persistent compensation relative to the odometry pose at the capture time.
    """

    BUFFER_DURATION = 1.5

    def __init__(self, initial_pose: Pose2d, gyro_angle: float, state_std=(0.1, 0.1, 0.1)):
        self.state_std = tuple(state_std)
        self.pose = initial_pose
        self.odo_pose = initial_pose
        self.gyro_offset = wrap_radians(initial_pose.theta - gyro_angle)
        self.previous_angle = initial_pose.theta
        self.odo_buffer = []  # (t, Pose2d) odometry-only poses
        self.vision_updates = []  # (t, vision_pose, odometry_pose)
        self.vision_k = self._gains_from_std((0.9, 0.9, 0.9))

    def _gains_from_std(self, vision_std):
        gains = []
        for q_std, r_std in zip(self.state_std, vision_std):
            q = q_std * q_std
            r = r_std * r_std
            gains.append(0.0 if q == 0.0 else q / (q + math.sqrt(q * r)))
        return gains

    def set_vision_measurement_std(self, vision_std):
        # Same closed-form Kalman gain as PoseEstimator.setVisionMeasurementStdDevs.
        self.vision_k = self._gains_from_std(vision_std)

    def reset_position(self, gyro_angle: float, pose: Pose2d):
        # Odometry.resetPosition: the library offsets the gyro internally, so
        # the reset heading never has to match the raw gyro reading.
        self.pose = pose
        self.odo_pose = pose
        self.gyro_offset = wrap_radians(pose.theta - gyro_angle)
        self.previous_angle = pose.theta
        self.odo_buffer.clear()
        self.vision_updates.clear()

    def _odo_sample_at(self, t):
        if not self.odo_buffer:
            return None
        if t <= self.odo_buffer[0][0]:
            return self.odo_buffer[0][1]
        if t >= self.odo_buffer[-1][0]:
            return self.odo_buffer[-1][1]
        for (t0, p0), (t1, p1) in zip(self.odo_buffer, self.odo_buffer[1:]):
            if t0 <= t <= t1:
                alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return Pose2d.interpolate(p0, p1, alpha)
        return self.odo_buffer[-1][1]

    def _compensated_sample_at(self, t):
        odo_sample = self._odo_sample_at(t)
        if odo_sample is None:
            return None
        applicable = [u for u in self.vision_updates if u[0] <= t]
        if not applicable:
            return odo_sample
        return self._compensate(applicable[-1], odo_sample)

    @staticmethod
    def _compensate(update, pose):
        _, vision_pose, odometry_pose = update
        return vision_pose.plus(pose.minus(odometry_pose))

    def add_vision_measurement(self, vision_pose: Pose2d, timestamp: float, vision_std):
        self.set_vision_measurement_std(vision_std)
        if not self.odo_buffer:
            return
        if self.odo_buffer[-1][0] - self.BUFFER_DURATION > timestamp:
            return
        odometry_sample = self._odo_sample_at(timestamp)
        vision_sample = self._compensated_sample_at(timestamp)
        if odometry_sample is None or vision_sample is None:
            return

        # Measure the residual between the vision pose and the estimate at the
        # capture time, scale it by the Kalman gain, and record the result as a
        # persistent compensation - exactly PoseEstimator.addVisionMeasurement.
        tx, ty, ttheta = vision_pose.minus(vision_sample)
        scaled = Pose2d(
            vision_sample.x + self.vision_k[0] * tx,
            vision_sample.y + self.vision_k[1] * ty,
            wrap_radians(vision_sample.theta + self.vision_k[2] * ttheta),
        )
        self.vision_updates = [u for u in self.vision_updates if u[0] <= timestamp]
        self.vision_updates.append((timestamp, scaled, odometry_sample))
        self.pose = self._compensate(self.vision_updates[-1], self.odo_pose)

    def update_with_time(self, t: float, gyro_angle: float, body_dx: float, body_dy: float):
        """Odometry.update + PoseEstimator.updateWithTime for one loop."""
        angle = wrap_radians(gyro_angle + self.gyro_offset)
        twist_dtheta = wrap_radians(angle - self.previous_angle)
        new_pose = self.odo_pose.exp(body_dx, body_dy, twist_dtheta)
        # Odometry.update forces the rotation to the gyro-derived angle.
        self.odo_pose = Pose2d(new_pose.x, new_pose.y, angle)
        self.previous_angle = angle
        self.odo_buffer.append((t, self.odo_pose))

        if not self.vision_updates:
            self.pose = self.odo_pose
        else:
            self.pose = self._compensate(self.vision_updates[-1], self.odo_pose)
        return self.pose


# ---------------------------------------------------------------------------
# Policy ports (VisionFusion.java / SimulatedGyro.java)
# ---------------------------------------------------------------------------


class VisionFusion:
    """Python mirror of VisionFusion.java (values are checked against the Java)."""

    K_MAX_POSE_AMBIGUITY = 0.2
    K_UNKNOWN_POSE_AMBIGUITY = -1.0
    K_MIN_CAMERA_TO_TAG_DISTANCE_METERS = 0.22
    K_MAX_VISION_TRANSLATION_METERS = 1.5
    K_MAX_VISION_HEADING_DEGREES = 45.0

    K_BASE_POSITION_STD_DEV_METERS = 0.25
    K_BASE_HEADING_STD_DEV_RADIANS = math.radians(6.0)
    K_AMBIGUITY_PENALTY = 2.0
    K_DISTANCE_PENALTY_METERS = 3.0
    K_MULTI_TAG_BONUS = 0.6

    def __init__(self):
        self._last_processed_frame = {}

    def should_process_frame(self, camera_name, timestamp_seconds):
        last = self._last_processed_frame.get(camera_name)
        if last is not None and abs(last - timestamp_seconds) < 1e-6:
            return False
        self._last_processed_frame[camera_name] = timestamp_seconds
        return True

    @classmethod
    def is_frame_acceptable(cls, pose_ambiguity, camera_to_tag_distance_meters):
        if not math.isfinite(camera_to_tag_distance_meters):
            return False
        if camera_to_tag_distance_meters < cls.K_MIN_CAMERA_TO_TAG_DISTANCE_METERS:
            return False
        return cls.is_ambiguity_acceptable(pose_ambiguity)

    @classmethod
    def is_ambiguity_acceptable(cls, pose_ambiguity):
        return pose_ambiguity == cls.K_UNKNOWN_POSE_AMBIGUITY or (
            pose_ambiguity >= 0.0 and pose_ambiguity <= cls.K_MAX_POSE_AMBIGUITY
        )

    @classmethod
    def is_consistent_with_estimate(cls, estimated_heading_deg, measured_heading_deg, translation_delta):
        if not math.isfinite(translation_delta):
            return False
        if translation_delta > cls.K_MAX_VISION_TRANSLATION_METERS:
            return False
        heading_delta = abs(wrap_degrees(measured_heading_deg - estimated_heading_deg))
        return math.isfinite(heading_delta) and heading_delta <= cls.K_MAX_VISION_HEADING_DEGREES

    @classmethod
    def measurement_std_devs(cls, targets_used, pose_ambiguity, distance, multi_tag):
        ambiguity = 0.0 if pose_ambiguity < 0.0 else max(0.0, pose_ambiguity)
        distance = max(0.0, distance)
        quality = (1.0 + cls.K_AMBIGUITY_PENALTY * ambiguity) * (
            1.0 + distance / cls.K_DISTANCE_PENALTY_METERS
        )
        tag_strength = cls.K_MULTI_TAG_BONUS if (multi_tag or targets_used >= 2) else 1.0
        scale = quality * tag_strength
        pos = cls.K_BASE_POSITION_STD_DEV_METERS * scale
        return (pos, pos, cls.K_BASE_HEADING_STD_DEV_RADIANS * scale)


class SimulatedGyro:
    """Python mirror of SimulatedGyro.java."""

    def __init__(self, drift_deg_per_second, noise_std_deg_per_second, seed):
        self.drift = drift_deg_per_second
        self.noise = max(0.0, noise_std_deg_per_second)
        self.rng = random.Random(seed)
        self.angle_degrees = 0.0
        self.rate_degrees_per_second = 0.0

    def update(self, true_yaw_rate_deg_per_second, dt):
        self.rate_degrees_per_second = (
            true_yaw_rate_deg_per_second
            + self.drift
            + self.rng.gauss(0.0, self.noise)
        )
        self.angle_degrees += self.rate_degrees_per_second * dt

    def reset_to(self, degrees):
        self.angle_degrees = degrees
        self.rate_degrees_per_second = 0.0

    @property
    def angle_radians(self):
        return math.radians(self.angle_degrees)


# ---------------------------------------------------------------------------
# Closed-loop harness (the DriveSubsystem pipeline, wired end to end)
# ---------------------------------------------------------------------------


@dataclass
class Step:
    time: float
    truth: Pose2d
    estimate: Pose2d


class Harness:
    AMBIGUITY = 0.05
    TAG_DISTANCE = 2.0
    TARGETS_USED = 2

    def __init__(self, gyro_drift=0.02, initial_pose=Pose2d(1.0, 2.0, math.radians(30.0))):
        self.truth = initial_pose
        self.gyro = SimulatedGyro(gyro_drift, 0.02, 987)
        self.gyro.reset_to(math.degrees(initial_pose.theta))
        self.estimator = PoseEstimator(initial_pose, self.gyro.angle_radians)
        self.fusion = VisionFusion()
        self.time = 0.0
        self.next_vision_time = 0.1
        self.seeded = True
        self.history = [(0.0, initial_pose)]
        self.vision_noise = random.Random(4242)

    # -- physics + sensors --------------------------------------------------

    def truth_at(self, t):
        samples = self.history
        if t <= samples[0][0]:
            return samples[0][1]
        if t >= samples[-1][0]:
            return samples[-1][1]
        for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
            if t0 <= t <= t1:
                alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return Pose2d.interpolate(p0, p1, alpha)
        return samples[-1][1]

    def step(self, vx, vy, omega_deg, vision_enabled=True):
        self.truth = self.truth.exp(vx * DT, vy * DT, math.radians(omega_deg) * DT)
        self.gyro.update(omega_deg, DT)
        self.time += DT
        self.history.append((self.time, self.truth))

        estimate = self.estimator.update_with_time(
            self.time,
            self.gyro.angle_radians,
            vx * DT,
            vy * DT,
        )

        if vision_enabled:
            while self.time >= self.next_vision_time:
                self.fuse_vision_frame(self.next_vision_time - VISION_LATENCY)
                self.next_vision_time += VISION_PERIOD
        return Step(self.time, self.truth, estimate)

    # -- vision fusion (exact DriveSubsystem policy) ------------------------

    def fuse_vision_frame(self, capture_time):
        truth_at_capture = self.truth_at(capture_time)
        multi_tag = self.TARGETS_USED >= 2
        heading_noise = 0.8 if multi_tag else 2.0
        measurement = Pose2d(
            truth_at_capture.x + self.vision_noise.gauss(0.0, 0.03),
            truth_at_capture.y + self.vision_noise.gauss(0.0, 0.03),
            wrap_radians(
                truth_at_capture.theta
                + math.radians(self.vision_noise.gauss(0.0, heading_noise))
            ),
        )

        if not self.fusion.should_process_frame("sim-cam", capture_time):
            return
        if not VisionFusion.is_frame_acceptable(self.AMBIGUITY, self.TAG_DISTANCE):
            return
        if not self.seeded:
            self.estimator.reset_position(self.gyro.angle_radians, measurement)
            self.seeded = True
            return
        if not VisionFusion.is_consistent_with_estimate(
            math.degrees(self.estimator.pose.theta),
            math.degrees(measurement.theta),
            self.estimator.pose.distance_to(measurement),
        ):
            return
        stddevs = VisionFusion.measurement_std_devs(
            self.TARGETS_USED, self.AMBIGUITY, self.TAG_DISTANCE, multi_tag
        )
        self.estimator.add_vision_measurement(measurement, capture_time, stddevs)


def heading_error_degrees(step):
    return abs(wrap_degrees(math.degrees(step.estimate.theta - step.truth.theta)))


# ---------------------------------------------------------------------------
# Scenario tests
# ---------------------------------------------------------------------------


class AutoStartHeadingTest(unittest.TestCase):
    """The heading bug and its root-cause fix: what an odometry reset means."""

    PATH_START = Pose2d(4.17, 0.69, math.radians(0.0))  # "2m and turn" start

    def _sit_at_alliance_default(self):
        harness = Harness()
        # Blue alliance default heading is 180 deg (see DriveSubsystem).
        harness.truth = Pose2d(0.0, 0.0, math.radians(180.0))
        harness.gyro.reset_to(180.0)
        harness.estimator = PoseEstimator(harness.truth, harness.gyro.angle_radians)
        harness.history = [(0.0, harness.truth)]
        return harness

    def test_authored_start_rotation_is_authoritative(self):
        harness = self._sit_at_alliance_default()

        # resetOdometry honors the FULL authored pose (the fix): estimator and
        # simulated ground truth both land on the path start pose.
        harness.estimator.reset_position(self.gyro_angle(harness), self.PATH_START)
        harness.truth = self.PATH_START
        harness.history.append((harness.time, harness.truth))

        # The path's first waypoint holds heading 0 - rotation error must be ~0.
        for _ in range(25):
            step = harness.step(0.0, 0.0, 0.0)
        self.assertLess(abs(wrap_degrees(math.degrees(step.estimate.theta))), 1.0)
        self.assertLess(heading_error_degrees(step), 1.0)

    def test_freezing_heading_across_the_reset_is_what_caused_the_spin(self):
        # The removed workaround discarded the authored rotation and kept the
        # pre-auto heading (the 180 deg alliance default). PathPlanner then saw
        # a 180 deg rotation error against the trajectory and spun the robot.
        harness = self._sit_at_alliance_default()
        frozen_heading = harness.estimator.pose.theta
        rotation_error = abs(
            wrap_degrees(math.degrees(self.PATH_START.theta - frozen_heading))
        )
        self.assertAlmostEqual(rotation_error, 180.0, delta=1e-6)

        # With the fix the error is zero instead of a half-turn.
        harness.estimator.reset_position(self.gyro_angle(harness), self.PATH_START)
        rotation_error_fixed = abs(
            wrap_degrees(math.degrees(self.PATH_START.theta - harness.estimator.pose.theta))
        )
        self.assertAlmostEqual(rotation_error_fixed, 0.0, delta=1e-6)

    @staticmethod
    def gyro_angle(harness):
        return harness.gyro.angle_radians


class HeadingFusionScenariosTest(unittest.TestCase):
    """Accuracy, drift rejection and smoothness during movement and turns."""

    def test_heading_stays_accurate_through_drive_arc_and_turn(self):
        harness = Harness(gyro_drift=0.02)
        steps = []
        for i in range(150):
            if i < 50:
                steps.append(harness.step(1.0, 0.0, 30.0))  # arc
            elif i < 100:
                steps.append(harness.step(0.0, 0.0, 90.0))  # in-place turn
            else:
                steps.append(harness.step(0.0, 1.0, -45.0))  # strafe + turn

        errors = [heading_error_degrees(s) for s in steps]
        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        self.assertLess(max(errors), 3.0, f"heading error peaked at {max(errors):.2f} deg")
        self.assertLess(rms, 1.5, f"heading RMS error {rms:.2f} deg")

    def test_vision_rejects_gyro_drift_after_an_outage(self):
        harness = Harness(gyro_drift=0.5)  # exaggerated so the outage is visible
        for _ in range(50):
            last = harness.step(0.5, 0.0, 10.0)
        self.assertLess(heading_error_degrees(last), 2.0)

        # 5 s vision outage while the gyro drifts.
        for _ in range(250):
            last = harness.step(0.5, 0.0, 0.0, vision_enabled=False)
        self.assertGreater(heading_error_degrees(last), 0.5, "drift must accumulate")

        # Vision returns: fusion must pull the heading back to truth.
        for _ in range(50):
            last = harness.step(0.5, 0.0, 0.0)
        self.assertLess(
            heading_error_degrees(last), 2.0, "heading did not recover after vision returned"
        )

    def test_heading_updates_smoothly_without_jumps(self):
        harness = Harness(gyro_drift=0.02)
        previous = None
        max_excess_step = 0.0
        for i in range(150):
            omega = 60.0 if i < 75 else -120.0
            step = harness.step(1.0 if i < 75 else 0.0, 0.0, omega)
            if previous is not None:
                estimate_step = abs(
                    wrap_degrees(math.degrees(step.estimate.theta - previous.estimate.theta))
                )
                truth_step = abs(
                    wrap_degrees(math.degrees(step.truth.theta - previous.truth.theta))
                )
                # Changes the real motion does not explain come from the
                # estimator/vision pipeline and must never be a snap.
                max_excess_step = max(max_excess_step, estimate_step - truth_step)
            previous = step
        self.assertLess(
            max_excess_step, 2.0, f"heading jumped {max_excess_step:.2f} deg in one loop"
        )

    def test_duplicated_frame_timestamps_are_never_fused_twice(self):
        harness = Harness()
        for _ in range(10):
            harness.step(0.0, 0.0, 0.0, vision_enabled=False)
        capture = harness.time - VISION_LATENCY

        # Same capture handed to the pipeline twice: only the first may be fused.
        before_first = harness.estimator.pose
        harness.fuse_vision_frame(capture)
        after_first = harness.estimator.pose
        harness.fuse_vision_frame(capture)
        after_second = harness.estimator.pose

        self.assertNotEqual(before_first, after_first, "first copy of the frame must be fused")
        self.assertEqual(after_first, after_second, "duplicate frame must not move the pose again")

    def test_vision_is_latency_compensated_while_turning(self):
        # Turn in place at 90 deg/s: a measurement applied 30 ms late would be
        # wrong by 2.7 deg of heading every frame. Capture-stamped fusion keeps
        # the signed error unbiased.
        harness = Harness(gyro_drift=0.0)
        errors = []
        for _ in range(150):
            step = harness.step(0.0, 0.0, 90.0)
            errors.append(wrap_degrees(math.degrees(step.estimate.theta - step.truth.theta)))
        mean_error = sum(errors) / len(errors)
        self.assertLess(abs(mean_error), 1.0, f"mean heading error {mean_error:.2f} deg")
        self.assertLess(max(abs(e) for e in errors), 3.0)


class SeedingTest(unittest.TestCase):
    def test_first_good_frame_seeds_and_later_frames_fuse(self):
        harness = Harness()
        harness.seeded = False
        harness.estimator = PoseEstimator(Pose2d(0.0, 0.0, math.radians(180.0)), math.radians(180.0))
        harness.truth = Pose2d(4.0, 3.0, math.radians(42.0))
        harness.gyro.reset_to(42.0)
        harness.history = [(0.0, harness.truth)]
        harness.time = 0.2

        harness.fuse_vision_frame(0.2 - VISION_LATENCY)
        self.assertTrue(harness.seeded)
        self.assertLess(
            abs(wrap_degrees(math.degrees(harness.estimator.pose.theta - harness.truth.theta))),
            5.0,
            "seed must establish the vision pose",
        )

        # A wildly wrong frame afterwards must be rejected by the outlier gate.
        bogus = Pose2d(harness.truth.x + 4.0, harness.truth.y, math.radians(120.0))
        self.assertFalse(
            VisionFusion.is_consistent_with_estimate(
                math.degrees(harness.estimator.pose.theta),
                math.degrees(bogus.theta),
                harness.estimator.pose.distance_to(bogus),
            )
        )


class PolicyMirroringTest(unittest.TestCase):
    """Keep this harness locked to the Java policy constants."""

    def _java_source(self, name):
        return (ROOT / "src/main/java/frc/robot" / name).read_text()

    def test_vision_fusion_constants_match_java(self):
        java = self._java_source("subsystems/VisionFusion.java")
        mapping = {
            "K_MAX_POSE_AMBIGUITY": r"kMaxPoseAmbiguity\s*=\s*([0-9.eE+-]+)",
            "K_UNKNOWN_POSE_AMBIGUITY": r"kUnknownPoseAmbiguity\s*=\s*([0-9.eE+-]+)",
            "K_MIN_CAMERA_TO_TAG_DISTANCE_METERS": r"kMinCameraToTagDistanceMeters\s*=\s*([0-9.eE+-]+)",
            "K_MAX_VISION_TRANSLATION_METERS": r"kMaxVisionTranslationMeters\s*=\s*([0-9.eE+-]+)",
            "K_MAX_VISION_HEADING_DEGREES": r"kMaxVisionHeadingDegrees\s*=\s*([0-9.eE+-]+)",
        }
        for attr, pattern in mapping.items():
            match = re.search(pattern, java)
            self.assertIsNotNone(match, f"constant for {attr} not found in VisionFusion.java")
            self.assertAlmostEqual(
                float(match.group(1)), getattr(VisionFusion, attr), places=9, msg=attr
            )

        match = re.search(r"kBaseHeadingStdDevRadians\s*=\s*Math.toRadians\(([0-9.eE+-]+)\)", java)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(
            math.radians(float(match.group(1))),
            VisionFusion.K_BASE_HEADING_STD_DEV_RADIANS,
            places=9,
        )
        match = re.search(r"kBasePositionStdDevMeters\s*=\s*([0-9.eE+-]+)", java)
        self.assertAlmostEqual(
            float(match.group(1)), VisionFusion.K_BASE_POSITION_STD_DEV_METERS, places=9
        )
        match = re.search(r"kAmbiguityPenalty\s*=\s*([0-9.eE+-]+)", java)
        self.assertAlmostEqual(float(match.group(1)), VisionFusion.K_AMBIGUITY_PENALTY, places=9)
        match = re.search(r"kDistancePenaltyMeters\s*=\s*([0-9.eE+-]+)", java)
        self.assertAlmostEqual(
            float(match.group(1)), VisionFusion.K_DISTANCE_PENALTY_METERS, places=9
        )
        match = re.search(r"kMultiTagBonus\s*=\s*([0-9.eE+-]+)", java)
        self.assertAlmostEqual(float(match.group(1)), VisionFusion.K_MULTI_TAG_BONUS, places=9)

    def test_simulated_gyro_formula_matches_java(self):
        java = self._java_source("subsystems/SimulatedGyro.java")
        self.assertIn("m_random.nextGaussian() * m_noiseStdDevDegreesPerSecond", java)
        self.assertIn("m_angleDegrees += m_rateDegreesPerSecond * dtSeconds", java)

    def test_reset_odometry_keeps_the_authored_rotation(self):
        java = self._java_source("subsystems/DriveSubsystem.java")
        # The workaround ("its authored rotation must not replace ...") must
        # stay gone, and the sim ground truth must take the full pose.
        self.assertNotIn("authored rotation must not replace", java)
        self.assertRegex(java, r"m_simRobotPose = pose;")


if __name__ == "__main__":
    unittest.main()
