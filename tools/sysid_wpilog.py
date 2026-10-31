#!/usr/bin/env python3
"""Identify feedforward gains (kS, kV, kA) from a real WPILib SysId ``.wpilog``.

This is the missing last mile between the robot and this repository's
simulation/controller constants: the ``SysIdSupport`` routines (and the
SmartDashboard / Auto-Chooser entries they publish) write a normal WPILib data
log, ``FRC_<date>_<time>.wpilog``, into the roboRIO's ``/home/lvuser/logs``
folder (or ``logs/`` in a simulation run). Everything needed to identify the
mechanism is inside that file:

* ``sysid-test-state-<mechanism>`` - a string entry that changes to
  ``quasistatic-forward`` / ``quasistatic-reverse`` / ``dynamic-forward`` /
  ``dynamic-reverse`` / ``none`` once per 20 ms loop.
* ``voltage-<motor>-<mechanism>``, ``linear-velocity-...``,
  ``linear-position-...``, ``angular-velocity-...``,
  ``angular-position-...`` - one ``DoubleLogEntry`` per logged motor (WPILib
  >= 2024 naming; the older ``sysid/<motor>/voltage`` nesting is also read).

WPILib's desktop SysId tool can open those logs, but it needs a Windows/macOS
GUI, a manual data-file conversion, and it reports a single mechanism at a time.
This script is pure standard-library Python, so it runs in CI or over SSH and it
handles every mechanism the robot logs, both the four-part "recommended" fit and
an exact discrete (ZOH) fit as a cross-check, plus per-motor spread and data
quality warnings.

Usage
-----
    # What is in the log? (mechanisms, tests, motors, durations, units)
    python3 tools/sysid_wpilog.py info logs/FRC_20260418_152525.wpilog

    # Fit every mechanism and print a readable report
    python3 tools/sysid_wpilog.py fit logs/FRC_20260418_152525.wpilog

    # One mechanism, machine-readable, plus a Markdown report and CSV dumps
    python3 tools/sysid_wpilog.py fit logs/FRC_....wpilog \\
        --mechanism drive-linear --json gainreport.json \\
        --markdown SYSID_RESULTS.md --csv sysid_csv

    # Paste-ready Java constants for Constants.java / Configs.java
    python3 tools/sysid_wpilog.py fit logs/FRC_....wpilog --java

    # No log handy? Generate a synthetic one from this repo's physics model and
    # run the whole pipeline on it end-to-end.
    python3 tools/sysid_wpilog.py demo /tmp/synthetic_sysid.wpilog
    python3 tools/sysid_wpilog.py fit /tmp/synthetic_sysid.wpilog

Two fits are reported for every mechanism. The recommended one is the exact
zero-order-hold model ``v[k+1] = ad*v[k] + bd*V[k] + cd*sgn(v[k])``, mapped back
to continuous-time gains with ``a_c = ln(ad)/dt``; it never differentiates the
velocity, so encoder noise cannot collapse kA. The second is the model WPILib
SysId's GUI reports, ``V = kS*|sgn|v + kV*v + kA*a`` over the quasistatic tests
plus the acceleration windows of the dynamic tests, printed for comparison (and
used as the fallback when the log is too noisy for the exact fit).

Exit status: 0 on success, 1 if no SysId data was found (or all requested
mechanisms were missing), 2 on usage/IO errors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wpilog import WpiLog, WpiLogError, WpiLogWriter, read_wpilog  # noqa: E402

# ---------------------------------------------------------------------------
# Naming / units
# ---------------------------------------------------------------------------

VOLTAGE = "voltage"
LINEAR_VELOCITY = "linear-velocity"
LINEAR_POSITION = "linear-position"
ANGULAR_VELOCITY = "angular-velocity"
ANGULAR_POSITION = "angular-position"
LINEAR_ACCELERATION = "linear-acceleration"
ANGULAR_ACCELERATION = "angular-acceleration"
CURRENT = "current"

#: Ordered longest-first so ``linear-velocity-...`` wins over ``...`` prefixes.
SYSID_FIELDS: Tuple[str, ...] = (
    LINEAR_ACCELERATION,
    ANGULAR_ACCELERATION,
    LINEAR_VELOCITY,
    LINEAR_POSITION,
    ANGULAR_VELOCITY,
    ANGULAR_POSITION,
    VOLTAGE,
    CURRENT,
)

STATE_PREFIX = "sysid-test-state-"
LEGACY_STATE_ENTRY = "sysid/state"

TEST_ORDER = (
    "quasistatic-forward",
    "quasistatic-reverse",
    "dynamic-forward",
    "dynamic-reverse",
)


def normalize_state(value: object) -> Optional[str]:
    """Map a logged SysId state string onto a canonical test name.

    Handles the hyphenated WPILib strings (``quasistatic-forward``), the older
    camelCase variants (``quasistaticForward``) and case/spacing noise.
    """
    if not isinstance(value, str):
        return None
    text = value.strip().lower().replace("_", "-").replace(" ", "-")
    if text in ("", "none", "null", "idle", "disabled"):
        return None
    quasistatic = "quasi" in text
    dynamic = "dyn" in text
    forward = "fwd" in text or "for" in text or text.endswith("+")
    reverse = "rev" in text or text.endswith("-")
    if quasistatic and forward:
        return "quasistatic-forward"
    if quasistatic and reverse:
        return "quasistatic-reverse"
    if dynamic and forward:
        return "dynamic-forward"
    if dynamic and reverse:
        return "dynamic-reverse"
    return None


def is_quasistatic(test: str) -> bool:
    return test.startswith("quasistatic")


def is_forward(test: str) -> bool:
    return test.endswith("forward")


# ---------------------------------------------------------------------------
# Mechanism catalogue: expectations from this repository's constants
# ---------------------------------------------------------------------------

ROTATIONS_PER_SEC = "RotationsPerSecond"
RPM = "RPM"
RADIANS_PER_SEC = "RadiansPerSecond"
DEGREES_PER_SEC = "DegreesPerSecond"
METERS_PER_SEC = "MetersPerSecond"


def velocity_to_unit(value: float, from_unit: str, to_unit: str) -> float:
    """Convert a rotational/linear velocity between the units we log."""
    per_rotation = {
        ROTATIONS_PER_SEC: 1.0,
        RPM: 60.0,
        RADIANS_PER_SEC: 2.0 * math.pi,
        DEGREES_PER_SEC: 360.0,
    }
    if from_unit == to_unit:
        return value
    if from_unit in per_rotation and to_unit in per_rotation:
        return value / per_rotation[from_unit] * per_rotation[to_unit]
    raise ValueError(f"cannot convert {from_unit} to {to_unit}")


@dataclass(frozen=True)
class MechanismInfo:
    """What we expect for a mechanism, straight from the robot's constants."""

    key: str
    label: str
    #: Free speed in ``natural_unit`` at 12 V.
    free_speed: float
    #: Physics/analysis unit for this mechanism's output.
    natural_unit: str
    #: How to turn the identified gains into robot code.
    java_hint: str
    #: Unit the robot's SysId routine actually logs (defaults to natural_unit).
    log_unit: str = ""

    @property
    def logged_unit(self) -> str:
        return self.log_unit or self.natural_unit


MECHANISMS: Dict[str, MechanismInfo] = {
    "drive-linear": MechanismInfo(
        "drive-linear",
        "Drivetrain, linear (4x NEO drive motors)",
        4.4606,  # NEO 5676 RPM / 5.077:1 * 0.0762*pi m
        METERS_PER_SEC,
        "SimpleMotorFeedforward<Distance> / Configs driving kV: V per (m/s),"
        " kA in V per (m/s^2); PathPlanner uses the same units.",
    ),
    "drive-angular": MechanismInfo(
        "drive-angular",
        "Drivetrain, angular (chassis yaw / MOI)",
        595.4,  # 4.4606 m/s / hypot(21,26.5)/2 in -> 10.39 rad/s
        DEGREES_PER_SEC,
        "kV in V per (deg/s), kA in V per (deg/s^2). Used for the rotation"
        " feedforward in PP holonomic control.",
    ),
    "drive-steer": MechanismInfo(
        "drive-steer",
        "Drivetrain, steering (4x NEO 550 azimuth)",
        3.949,  # NEO 550 11000 RPM / 46.42:1
        ROTATIONS_PER_SEC,
        "kV in V per (rot/s) of module azimuth. REVLib needs V per encoder"
        " velocity unit - multiply by 60 for RPM-based configurations.",
    ),
    "shooter-flywheels": MechanismInfo(
        "shooter-flywheels",
        "Shooter flywheels (front + back NEO)",
        94.6,  # NEO 5676 RPM = 94.6 rot/s, 1:1
        ROTATIONS_PER_SEC,
        "REVLib closedLoop.feedForward.kV takes V per converted velocity unit:"
        " if the encoder reports RPM, use the RPM gains directly.",
        RPM,
    ),
    "shooter-feeder": MechanismInfo(
        "shooter-feeder",
        "Shooter feeder (Kraken X60)",
        100.0,  # Kraken X60 6000 RPM = 100 rot/s, 1:1
        ROTATIONS_PER_SEC,
        "Phoenix 6 Slot0 kS/kV/kA are V, V/(rot/s), V/(rot/s^2) 1:1.",
    ),
    "intake-deploy": MechanismInfo(
        "intake-deploy",
        "Intake deploy arm (NEO Vortex, 212.4:1)",
        0.5323,  # NEO Vortex 6784 RPM / 60 / 212.4
        ROTATIONS_PER_SEC,
        "Arm: kV in V/(rot/s) of arm output; also fit kG from the gravity term"
        " (cos(theta)); check the sign of kS against the gravity droop.",
    ),
    "intake-roller": MechanismInfo(
        "intake-roller",
        "Intake roller (Kraken X60)",
        100.0,  # Kraken X60 6000 RPM = 100 rot/s, 1:1
        ROTATIONS_PER_SEC,
        "Phoenix 6 Slot0 gains 1:1 (V, V/(rot/s), V/(rot/s^2)).",
    ),
    "intake-conveyor": MechanismInfo(
        "intake-conveyor",
        "Intake conveyor (NEO)",
        94.6,  # NEO 5676 RPM = 94.6 rot/s, 1:1
        ROTATIONS_PER_SEC,
        "REVLib closedLoop.feedForward.kV in V per encoder velocity unit (RPM"
        " here).",
        RPM,
    ),
}


# ---------------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------------


@dataclass
class MotorSeries:
    """One motor's logged SysId channels, straight from the log."""

    motor: str
    mechanism: str
    voltage: List[Tuple[int, float]] = field(default_factory=list)
    velocity: List[Tuple[int, float]] = field(default_factory=list)
    position: List[Tuple[int, float]] = field(default_factory=list)
    voltage_unit: str = ""
    velocity_unit: str = ""
    position_unit: str = ""
    velocity_field: str = ""


@dataclass
class Segment:
    """One contiguous SysId test (one of the four parts)."""

    test: str
    start_us: int
    end_us: int
    samples: Dict[str, List[Sample]] = field(default_factory=dict)

    @property
    def duration_seconds(self) -> float:
        return (self.end_us - self.start_us) / 1e6


@dataclass(frozen=True)
class Sample:
    """A single 20 ms frame for one motor."""

    time_s: float
    voltage: float
    velocity: float
    position: float
    acceleration: Optional[float] = None


@dataclass
class MechanismData:
    motors: Dict[str, MotorSeries] = field(default_factory=dict)
    state_samples: List[Tuple[int, str]] = field(default_factory=list)


def _strip_field(name: str) -> Optional[Tuple[str, str]]:
    for sysid_field in SYSID_FIELDS:
        if name.startswith(sysid_field + "-"):
            return sysid_field, name[len(sysid_field) + 1 :]
    return None


def extract_mechanisms(log: WpiLog) -> Dict[str, MechanismData]:
    """Group the log's SysId entries by mechanism."""
    mechanisms: Dict[str, MechanismData] = {}

    def bucket(name: str) -> MechanismData:
        return mechanisms.setdefault(name, MechanismData())

    # 1) State entries tell us the mechanism names.
    known: set = set()
    for entry in log:
        name = entry.name
        if name.startswith(STATE_PREFIX):
            mechanism = name[len(STATE_PREFIX) :]
            known.add(mechanism)
            state_bucket = bucket(mechanism)
            state_bucket.state_samples = [
                (timestamp, str(value))
                for timestamp, value in log.values(entry)
                if isinstance(value, str)
            ]
        elif name == LEGACY_STATE_ENTRY:  # WPILib <= 2023 layout
            state_bucket = bucket("sysid")
            state_bucket.state_samples = [
                (timestamp, str(value))
                for timestamp, value in log.values(entry)
                if isinstance(value, str)
            ]

    # 2) Numeric entries: FIELD-MOTOR-MECHANISM (2024+) or sysid/MOTOR/FIELD.
    for entry in log:
        name = entry.name
        if entry.type not in ("double", "float", "int64"):
            continue

        mechanism: Optional[str] = None
        motor: Optional[str] = None
        sysid_field: Optional[str] = None

        if name.startswith("sysid/") and name.count("/") == 2:
            _, motor, sysid_field = name.split("/")
            if sysid_field not in SYSID_FIELDS:
                continue
            mechanism = "sysid"
        else:
            stripped = _strip_field(name)
            if stripped is None:
                continue
            sysid_field, remainder = stripped
            # Longest known mechanism suffix wins (mechanism names contain '-').
            # Fall back to this repo's mechanism catalogue so a log whose state
            # entry was never written (or dropped) is still grouped correctly.
            for candidate in sorted(known | set(MECHANISMS), key=len, reverse=True):
                if remainder.endswith("-" + candidate) and len(remainder) > len(candidate) + 1:
                    mechanism = candidate
                    motor = remainder[: -(len(candidate) + 1)]
                    break
            if mechanism is None:
                # Unknown naming scheme (hand-written logger): keep the whole
                # remainder as the motor name and treat it as one mechanism.
                mechanism = "sysid"
                motor = remainder

        if not motor:
            continue
        series = bucket(mechanism).motors.setdefault(
            motor, MotorSeries(motor=motor, mechanism=mechanism)
        )
        samples = [
            (timestamp, float(value))
            for timestamp, value in log.values(entry)
            if isinstance(value, (int, float))
        ]
        if sysid_field == VOLTAGE:
            series.voltage = samples
            series.voltage_unit = entry.unit
        elif sysid_field in (LINEAR_VELOCITY, ANGULAR_VELOCITY):
            series.velocity = samples
            series.velocity_unit = entry.unit
            series.velocity_field = sysid_field
        elif sysid_field in (LINEAR_POSITION, ANGULAR_POSITION):
            series.position = samples
            series.position_unit = entry.unit

    return {name: data for name, data in mechanisms.items() if data.motors}


def build_segments(data: MechanismData, nominal_dt: float = 0.02) -> List[Segment]:
    """Split a mechanism's motors into the four SysId test windows."""
    windows: List[Tuple[str, int, int]] = []
    current_test: Optional[str] = None
    start_us = 0

    for timestamp, raw in data.state_samples:
        test = normalize_state(raw)
        if test != current_test:
            if current_test is not None:
                windows.append((current_test, start_us, timestamp))
            current_test = test
            start_us = timestamp
    if current_test is not None:
        last_timestamp = max(
            (samples[-1][0] for series in data.motors.values() for samples in
             (series.voltage, series.velocity, series.position) if samples),
            default=start_us,
        )
        windows.append((current_test, start_us, last_timestamp))

    if not windows and data.state_samples == []:
        # No state entry at all (older logs or a stripped file): one window.
        last_timestamp = max(
            (
                samples[-1][0]
                for series in data.motors.values()
                for samples in (series.voltage, series.velocity)
                if samples
            ),
            default=0,
        )
        first_timestamp = min(
            (
                samples[0][0]
                for series in data.motors.values()
                for samples in (series.voltage, series.velocity)
                if samples
            ),
            default=0,
        )
        windows.append(("unknown", first_timestamp, last_timestamp))

    segments: List[Segment] = []
    for test, window_start, window_end in windows:
        if window_end - window_start < 100_000:  # < 100 ms: not a real test
            continue
        segment = Segment(test=test, start_us=window_start, end_us=window_end)
        for motor, series in data.motors.items():
            segment.samples[motor] = _resample(series, window_start, window_end, nominal_dt)
        segments.append(segment)
    return [segment for segment in segments if any(segment.samples.values())]


def _hold(samples: Sequence[Tuple[int, float]], timestamps: Sequence[int], max_stale_us: int = 100_000):
    """Zero-order-hold lookup of ``samples`` at each timestamp."""
    values: List[Optional[float]] = []
    index = 0
    last_value: Optional[float] = None
    last_timestamp: Optional[int] = None
    for timestamp in timestamps:
        while index < len(samples) and samples[index][0] <= timestamp:
            last_timestamp, last_value = samples[index]
            index += 1
        if last_timestamp is None or timestamp - last_timestamp > max_stale_us:
            values.append(None)
        else:
            values.append(last_value)
    return values


def _differentiate(samples: Sequence[Tuple[int, float]]) -> List[Tuple[int, float]]:
    """Central-difference derivative of a sampled signal (position -> velocity)."""
    if len(samples) < 3:
        return []
    out: List[Tuple[int, float]] = []
    for i in range(1, len(samples) - 1):
        dt = (samples[i + 1][0] - samples[i - 1][0]) / 1e6
        if dt <= 0:
            continue
        out.append(
            (samples[i][0], (samples[i + 1][1] - samples[i - 1][1]) / dt)
        )
    return out


def _resample(
    series: MotorSeries, start_us: int, end_us: int, nominal_dt: float
) -> List[Sample]:
    """Build one time-aligned frame per control loop inside a test window."""
    if not series.voltage or not (series.velocity or series.position):
        return []
    times = [timestamp for timestamp, _ in series.voltage if start_us <= timestamp <= end_us]
    if len(times) < 5:
        return []

    voltages = dict(series.voltage)
    volt_values = [voltages[t] for t in times]
    velocity_samples = series.velocity
    if not velocity_samples and series.position:
        # Logs that only carry position: differentiate it so there is still a
        # velocity signal to identify against.
        velocity_samples = _differentiate(series.position)
    velocities = _hold(velocity_samples, times)
    positions = _hold(series.position, times) if series.position else [None] * len(times)

    raw: List[Sample] = []
    for timestamp, voltage, velocity, position in zip(times, volt_values, velocities, positions):
        if velocity is None:
            continue
        raw.append(
            Sample(
                time_s=(timestamp - start_us) / 1e6,
                voltage=float(voltage),
                velocity=float(velocity),
                position=float(position) if position is not None else math.nan,
            )
        )
    if len(raw) < 5:
        return []

    # Dropped log frames are kept (dropping them would silently stitch two
    # distant moments together); fit_zoh() rejects pairs whose spacing is not
    # near the nominal loop period, and acceleration uses the true dt.
    accelerations = _local_accelerations(raw)
    return [
        Sample(
            time_s=sample.time_s,
            voltage=sample.voltage,
            velocity=sample.velocity,
            position=sample.position,
            acceleration=acceleration,
        )
        for sample, acceleration in zip(raw, accelerations)
    ]


def _local_accelerations(samples: Sequence[Sample]) -> List[Optional[float]]:
    """Acceleration from a centered finite difference on the velocity signal.

    A 3-point central difference is second-order accurate and, unlike any wider
    smoothing window, does not smear the fast voltage-step transient that
    carries the kA information (a NEO flywheel's mechanical time constant can
    be only a few control loops). The price is noise amplification, which is
    why the exact-ZOH fit below is used as the primary estimate and the
    simple fit's kA is only trusted when it agrees.
    """
    out: List[Optional[float]] = []
    for index, _ in enumerate(samples):
        acceleration: Optional[float] = None
        if 0 < index < len(samples) - 1:
            dt = samples[index + 1].time_s - samples[index - 1].time_s
            if dt > 1e-6:
                acceleration = (
                    samples[index + 1].velocity - samples[index - 1].velocity
                ) / dt
        out.append(acceleration)
    return out


# ---------------------------------------------------------------------------
# Least squares
# ---------------------------------------------------------------------------


def solve_linear_system(matrix: List[List[float]], rhs: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting (no numpy in this repo)."""
    n = len(rhs)
    augmented = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    for column in range(n):
        pivot = max(range(column, n), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("singular normal-equation matrix")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for j in range(column, n + 1):
            augmented[column][j] /= divisor
        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor == 0.0:
                continue
            for j in range(column, n + 1):
                augmented[row][j] -= factor * augmented[column][j]
    return [augmented[i][n] for i in range(n)]


@dataclass
class FitResult:
    """Coefficients plus goodness-of-fit for one least-squares model."""

    coefficients: List[float]
    names: List[str]
    r_squared: float
    rmse: float
    samples: int
    #: Unit of ``rmse`` ("V" for a voltage fit, the velocity unit for ZOH).
    rmse_unit: str = "V"

    def get(self, name: str) -> Optional[float]:
        if name not in self.names:
            return None
        return self.coefficients[self.names.index(name)]

    def as_dict(self) -> Dict[str, object]:
        out: Dict[str, object] = dict(zip(self.names, self.coefficients))
        out["r_squared"] = self.r_squared
        out["rmse"] = self.rmse
        out["rmse_unit"] = self.rmse_unit
        out["samples"] = self.samples
        return out


def ols(features: List[List[float]], targets: List[float], names: List[str]) -> FitResult:
    """Ordinary least squares via the normal equations, with R^2 and RMSE."""
    if len(features) < len(names) + 2:
        raise ValueError(f"need at least {len(names) + 2} samples, got {len(features)}")
    p = len(names)
    xtx = [[0.0] * p for _ in range(p)]
    xty = [0.0] * p
    for row, target in zip(features, targets):
        for i in range(p):
            xty[i] += row[i] * target
            for j in range(p):
                xtx[i][j] += row[i] * row[j]
    coefficients = solve_linear_system(xtx, xty)

    mean_y = sum(targets) / len(targets)
    ss_res = 0.0
    ss_tot = 0.0
    for row, target in zip(features, targets):
        predicted = sum(c * x for c, x in zip(coefficients, row))
        ss_res += (target - predicted) ** 2
        ss_tot += (target - mean_y) ** 2
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = math.sqrt(ss_res / len(targets))
    return FitResult(coefficients, names, r_squared, rmse, len(targets))


def _sign(velocity: float, voltage: float) -> float:
    """Direction of motion, falling back to commanded voltage at standstill."""
    if abs(velocity) > 1e-6:
        return 1.0 if velocity > 0 else -1.0
    return 1.0 if voltage >= 0 else -1.0


def fit_quasistatic(segments: Sequence[Segment]) -> Optional[FitResult]:
    """``V = kS*sgn(v) + kV*v`` over the quasistatic tests (SysId's velocity fit)."""
    rows: List[List[float]] = []
    targets: List[float] = []
    for segment in segments:
        if not is_quasistatic(segment.test):
            continue
        for samples in segment.samples.values():
            for sample in samples:
                rows.append([_sign(sample.velocity, sample.voltage), sample.velocity])
                targets.append(sample.voltage)
    try:
        return ols(rows, targets, ["kS", "kV"])
    except ValueError:
        return None


def _dynamic_windows(
    segments: Sequence[Segment], accel_threshold: float, velocity_threshold: float
) -> List[Sample]:
    """Samples from the dynamic tests that carry a meaningful acceleration."""
    dynamic_samples: List[Sample] = []
    for segment in segments:
        if is_quasistatic(segment.test):
            continue
        for samples in segment.samples.values():
            dynamic_samples.extend(samples)
    accelerations = [
        abs(s.acceleration) for s in dynamic_samples if s.acceleration is not None
    ]
    velocities = [abs(s.velocity) for s in dynamic_samples]
    if not accelerations or not velocities:
        return []
    accel_cutoff = accel_threshold * max(accelerations)
    velocity_cutoff = velocity_threshold * max(velocities)
    return [
        sample
        for sample in dynamic_samples
        if sample.acceleration is not None
        and abs(sample.acceleration) >= accel_cutoff
        and abs(sample.velocity) >= velocity_cutoff
    ]


def fit_combined(
    segments: Sequence[Segment], accel_threshold: float = 0.2, velocity_threshold: float = 0.2
) -> Optional[FitResult]:
    """WPILib SysId's ``simple`` model over all four tests.

    Quasistatic rows contribute with ``a = 0`` (their acceleration is negligible
    and dominated by encoder noise); dynamic rows are restricted to their
    acceleration windows, matching what the SysId GUI reports as the
    "recommended" gains.
    """
    rows: List[List[float]] = []
    targets: List[float] = []
    for segment in segments:
        if not is_quasistatic(segment.test):
            continue
        for samples in segment.samples.values():
            for sample in samples:
                # A quasistatic ramp is not acceleration-free: ignoring its
                # (small) acceleration biases kS upward by ~50% at 1 V/s ramps.
                rows.append(
                    [
                        _sign(sample.velocity, sample.voltage),
                        sample.velocity,
                        sample.acceleration or 0.0,
                    ]
                )
                targets.append(sample.voltage)
    for sample in _dynamic_windows(segments, accel_threshold, velocity_threshold):
        rows.append(
            [
                _sign(sample.velocity, sample.voltage),
                sample.velocity,
                sample.acceleration or 0.0,
            ]
        )
        targets.append(sample.voltage)
    try:
        return ols(rows, targets, ["kS", "kV", "kA"])
    except ValueError:
        return fit_quasistatic(segments)


def fit_zoh(segments: Sequence[Segment], dt: float = 0.02) -> Optional[FitResult]:
    """Exact zero-order-hold fit, immune to finite-difference accel noise.

    Fits ``v[k+1] = ad*v[k] + bd*V[k] + cd*sgn(v[k])`` and maps the discrete
    coefficients back to ``(kS, kV, kA)`` through the matrix logarithm
    ``a_c = ln(ad)/dt``, giving a continuous-time equivalent that should agree
    with :func:`fit_combined` when the log is clean.
    """
    rows: List[List[float]] = []
    targets: List[float] = []
    for segment in segments:
        for samples in segment.samples.values():
            for previous, current in zip(samples, samples[1:]):
                step = current.time_s - previous.time_s
                if step <= 1e-6 or step > max(3.0 * dt, 0.075):
                    continue
                if abs(previous.velocity) < 1e-5 and abs(previous.voltage) < 1e-5:
                    continue
                rows.append(
                    [
                        previous.velocity,
                        previous.voltage,
                        _sign(previous.velocity, previous.voltage),
                    ]
                )
                targets.append(current.velocity)
    if len(rows) < 20:
        return None
    try:
        discrete = ols(rows, targets, ["ad", "bd", "cd"])
    except ValueError:
        return None
    ad, bd, cd = discrete.coefficients
    if ad <= 0.0 or abs(ad - 1.0) < 1e-9 or abs(bd) < 1e-12:
        return None
    a_c = math.log(ad) / dt
    scale = a_c / (ad - 1.0)
    b_c = bd * scale
    c_c = cd * scale
    return FitResult(
        coefficients=[-c_c / b_c, -a_c / b_c, 1.0 / b_c],
        names=["kS", "kV", "kA"],
        r_squared=discrete.r_squared,
        rmse=discrete.rmse,
        samples=discrete.samples,
        rmse_unit="velocity unit",
    )


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


@dataclass
class MotorFit:
    motor: str
    velocity_unit: str
    voltage_unit: str
    samples: int
    combined: Optional[FitResult] = None
    quasistatic: Optional[FitResult] = None
    zoh: Optional[FitResult] = None
    segments: Dict[str, int] = field(default_factory=dict)
    max_abs_voltage: float = 0.0
    max_abs_velocity: float = 0.0
    min_velocity: float = 0.0
    saturated_fraction: float = 0.0


@dataclass
class MechanismAnalysis:
    mechanism: str
    label: str
    motors: Dict[str, MotorFit] = field(default_factory=dict)
    pooled: Optional[FitResult] = None
    pooled_quasistatic: Optional[FitResult] = None
    pooled_zoh: Optional[FitResult] = None
    tests_found: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    velocity_unit: str = ""
    info: Optional[MechanismInfo] = None

    #: Which fit ``recommended`` came from: "zoh", "simple" or "" (no fit).
    recommended_source: str = ""

    @property
    def recommended(self) -> Dict[str, Optional[float]]:
        fit = self.recommended_fit
        return {
            "kS": fit.get("kS") if fit else None,
            "kV": fit.get("kV") if fit else None,
            "kA": fit.get("kA") if fit else None,
        }

    @property
    def recommended_fit(self) -> Optional[FitResult]:
        if self.recommended_source == "zoh":
            return self.pooled_zoh
        if self.recommended_source == "simple":
            return self.pooled
        return self.pooled


def _zoh_is_trustworthy(analysis: "MechanismAnalysis") -> bool:
    """The ZOH fit is exact when the log is clean; reject it when it is not.

    Judged on its own physical plausibility (positive kA/kV, low velocity
    residual relative to the tested speed range, kV near the free-speed
    expectation) rather than by agreement with the simple fit, because on a
    noisy log the *simple* fit's kA collapses toward zero while the ZOH fit
    stays close to the truth.
    """
    zoh = analysis.pooled_zoh
    if zoh is None:
        return False
    k_s, k_v, k_a = zoh.get("kS"), zoh.get("kV"), zoh.get("kA")
    if k_s is None or k_v is None or k_a is None:
        return False
    if not all(math.isfinite(value) for value in (k_s, k_v, k_a)):
        return False
    if k_a <= 0.0 or k_v <= 0.0:
        return False
    if zoh.r_squared < 0.90:
        return False

    span = max(
        (motor.max_abs_velocity for motor in analysis.motors.values()), default=0.0
    )
    if span > 0 and zoh.rmse > 0.10 * span:
        return False

    theory = _theoretical_kv(analysis)
    if theory and not 0.3 <= k_v / theory <= 3.0:
        return False
    return True


def _saturation_fraction(samples: Sequence[Sample], nominal_volts: float = 12.0) -> float:
    if not samples:
        return 0.0
    limit = 0.98 * nominal_volts
    saturated = sum(1 for sample in samples if abs(sample.voltage) >= limit)
    return saturated / len(samples)


def analyze_mechanism(
    mechanism: str,
    data: MechanismData,
    nominal_dt: float = 0.02,
    accel_threshold: float = 0.2,
    velocity_threshold: float = 0.2,
) -> Optional[MechanismAnalysis]:
    segments = build_segments(data, nominal_dt)
    if not segments:
        return None

    analysis = MechanismAnalysis(
        mechanism=mechanism,
        label=MECHANISMS[mechanism].label if mechanism in MECHANISMS else mechanism,
        info=MECHANISMS.get(mechanism),
        tests_found=[segment.test for segment in segments],
    )

    for motor, series in sorted(data.motors.items()):
        motor_segments = [
            replace(segment, samples={motor: segment.samples[motor]})
            for segment in segments
            if segment.samples.get(motor)
        ]
        samples = [sample for segment in motor_segments for sample in segment.samples[motor]]
        if len(samples) < 25:
            analysis.warnings.append(
                f"{motor}: only {len(samples)} usable samples - skipped"
            )
            continue
        fit = MotorFit(
            motor=motor,
            velocity_unit=series.velocity_unit,
            voltage_unit=series.voltage_unit,
            samples=len(samples),
            combined=fit_combined(motor_segments, accel_threshold, velocity_threshold),
            quasistatic=fit_quasistatic(motor_segments),
            zoh=fit_zoh(motor_segments, nominal_dt),
            segments={segment.test: len(segment.samples[motor]) for segment in motor_segments},
            max_abs_voltage=max(abs(sample.voltage) for sample in samples),
            max_abs_velocity=max(abs(sample.velocity) for sample in samples),
            min_velocity=min(sample.velocity for sample in samples),
            saturated_fraction=_saturation_fraction(samples),
        )
        analysis.motors[motor] = fit
        analysis.velocity_unit = analysis.velocity_unit or series.velocity_unit

    if not analysis.motors:
        return None

    analysis.pooled = fit_combined(segments, accel_threshold, velocity_threshold)
    analysis.pooled_quasistatic = fit_quasistatic(segments)
    analysis.pooled_zoh = fit_zoh(segments, nominal_dt)
    if _zoh_is_trustworthy(analysis):
        analysis.recommended_source = "zoh"
    elif analysis.pooled is not None:
        analysis.recommended_source = "simple"
    analysis.warnings.extend(_quality_warnings(analysis, segments))
    return analysis


def _quality_warnings(analysis: MechanismAnalysis, segments: Sequence[Segment]) -> List[str]:
    warnings: List[str] = []
    tests = set(analysis.tests_found)

    missing = [test for test in TEST_ORDER if test not in tests]
    if "unknown" in tests:
        warnings.append(
            "no 'sysid-test-state-*' entry found: treated the whole log as one test,"
            " so kA and per-test checks are unreliable"
        )
    elif missing:
        warnings.append("missing test legs: " + ", ".join(missing))

    durations = defaultdict(float)
    for segment in segments:
        durations[segment.test] = max(durations[segment.test], segment.duration_seconds)
    for test, duration in sorted(durations.items()):
        if test != "unknown" and duration < 1.5:
            warnings.append(f"{test}: only {duration:.2f} s of data (ramp may be truncated)")

    fit = analysis.pooled
    if fit is None:
        warnings.append("not enough data for a 3-parameter fit (need kS, kV, kA)")
        return warnings

    if fit.r_squared < 0.90:
        warnings.append(
            f"poor fit: R^2 = {fit.r_squared:.3f} (< 0.90); check for wheel slip,"
            " belt slip, saturated voltage or a slipping encoder"
        )
    if (fit.get("kA") or 0.0) <= 0.0:
        warnings.append(
            f"kA = {fit.get('kA'):.4g} is not positive - the acceleration term is"
            " not identifiable from these tests (dynamic legs too short or too noisy)"
        )
    if abs(fit.get("kS") or 0.0) > 3.0:
        warnings.append(
            f"|kS| = {abs(fit.get('kS')):.3f} V is large (> 3 V); check for a"
            " mechanical bind or a very stiff gearbox"
        )
    if analysis.pooled_zoh is not None and analysis.recommended_source != "zoh":
        warnings.append(
            "the exact ZOH fit was rejected as implausible (noisy or irregular"
            " velocity data); the recommended gains come from the simple fit, whose"
            " kA is fragile - re-run the routine on a cleaner surface/battery"
        )

    if analysis.pooled_zoh is not None:
        zoh = analysis.pooled_zoh
        zoh_kv = zoh.get("kV")
        combined_kv = fit.get("kV")
        if zoh_kv and combined_kv and combined_kv != 0:
            ratio = zoh_kv / combined_kv
            if not 0.90 <= ratio <= 1.11:
                warnings.append(
                    f"kV differs {abs(ratio - 1) * 100:.0f}% between the simple fit"
                    f" ({combined_kv:.4g}) and the exact ZOH fit ({zoh_kv:.4g});"
                    " velocity quantization or noise is biasing the"
                    " finite-difference acceleration"
                )
        zoh_ka = zoh.get("kA") or 0.0
        simple_ka = fit.get("kA") or 0.0
        if zoh_ka > 0 and simple_ka < 0.5 * zoh_ka and analysis.recommended_source == "zoh":
            warnings.append(
                f"the simple fit's kA ({simple_ka:.4g}) is far below the ZOH fit's"
                f" ({zoh_ka:.4g}): the finite-difference acceleration is"
                " noise-dominated. Use the recommended ZOH gains - or, to match the"
                " SysId GUI exactly, average several runs of cleaner data."
            )
        span = max(
            (motor.max_abs_velocity for motor in analysis.motors.values()), default=0.0
        )
        if span > 0 and zoh.rmse > 0.02 * span:
            warnings.append(
                f"velocity residual is {zoh.rmse / span * 100:.1f}% of the tested"
                " speed range: check encoder noise/quantization, belt slop, or a"
                " slipping wheel before trusting kA"
            )

    for motor in analysis.motors.values():
        if motor.saturated_fraction > 0.10:
            warnings.append(
                f"{motor.motor}: {motor.saturated_fraction * 100:.0f}% of samples at"
                " > 11.8 V - gains are biased by battery sag/clipping"
            )

    kvs = [
        motor.combined.get("kV")
        for motor in analysis.motors.values()
        if motor.combined is not None
    ]
    kvs = [kv for kv in kvs if kv is not None]
    if len(kvs) >= 2:
        mean_kv = sum(kvs) / len(kvs)
        spread = (max(kvs) - min(kvs)) / mean_kv if mean_kv else 0.0
        if spread > 0.10:
            warnings.append(
                f"per-motor kV spread is {spread * 100:.0f}% (> 10%): the motors do not"
                " share the load evenly (dragging wheel, different gearing, or a bad"
                " encoder)"
            )

    if analysis.pooled_quasistatic is not None and fit.get("kS") is not None:
        qs_ks = analysis.pooled_quasistatic.get("kS") or 0.0
        # The quasistatic-only model ignores acceleration, so it is expected to
        # differ a little; only a large gap means the log itself is suspect.
        if abs(qs_ks - (fit.get("kS") or 0.0)) > max(0.5, 0.5 * abs(qs_ks)):
            warnings.append(
                f"kS from the quasistatic-only fit ({qs_ks:.3f} V) disagrees with the"
                f" combined fit ({fit.get('kS'):.3f} V); check that the quasistatic"
                " legs were not truncated and that the mechanism was not binding"
            )
    return warnings


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _fmt(value: Optional[float], digits: int = 4) -> str:
    if value is None:
        return "n/a"
    if value == 0:
        return "0"
    if abs(value) >= 1000 or abs(value) < 1e-3:
        return f"{value:.{digits}g}"
    return f"{value:.{digits}f}"


def info_report(log: WpiLog, path: str) -> str:
    lines = [
        f"Log: {path}",
        f"  format version: 0x{log.version:04x}"
        + ("  (truncated - read stopped at the last complete record)" if log.truncated else ""),
        f"  entries: {len(log.entries)}   records: {sum(len(v) for v in log.samples.values())}",
    ]
    if log.entries:
        first = min(
            (samples[0][0] for samples in log.samples.values() if samples), default=0
        )
        last = max(
            (samples[-1][0] for samples in log.samples.values() if samples), default=0
        )
        lines.append(f"  duration: {(last - first) / 1e6:.2f} s")

    mechanisms = extract_mechanisms(log)
    if not mechanisms:
        lines.append("")
        lines.append("No WPILib SysId entries found.")
        candidates = [entry for entry in log if "sysid" in entry.name.lower()]
        if candidates:
            lines.append("Entries mentioning 'sysid':")
            lines.extend(f"  {entry.name} ({entry.type})" for entry in candidates)
        else:
            lines.append(
                "Tip: re-run a SysId routine from the 'SysId Mechanism' chooser; each"
                " test writes sysid-test-state-<mechanism> plus per-motor voltage /"
                " position / velocity entries."
            )
        return "\n".join(lines)

    for name, data in sorted(mechanisms.items()):
        segments = build_segments(data)
        info = MECHANISMS.get(name)
        lines.append("")
        lines.append(f"Mechanism: {name}" + (f"  ({info.label})" if info else ""))
        lines.append(f"  motors: {', '.join(sorted(data.motors)) or '(none)'}")
        if segments:
            lines.append("  tests:")
            for segment in segments:
                lines.append(
                    f"    {segment.test:<22} {segment.duration_seconds:6.2f} s  "
                    f"{max((len(v) for v in segment.samples.values()), default=0)} frames"
                )
        else:
            lines.append("  tests: (no state entry - whole log would be used as one test)")
        units = sorted(
            {
                f"{series.velocity_unit or '?'}"
                for series in data.motors.values()
            }
        )
        lines.append(f"  velocity units: {', '.join(units)}")
    return "\n".join(lines)


def analysis_to_dict(analysis: MechanismAnalysis) -> Dict[str, object]:
    def fit_dict(fit: Optional[FitResult]) -> Optional[Dict[str, object]]:
        return fit.as_dict() if fit else None

    return {
        "mechanism": analysis.mechanism,
        "label": analysis.label,
        "velocity_unit": analysis.velocity_unit,
        "tests": analysis.tests_found,
        "recommended": analysis.recommended,
        "recommended_source": analysis.recommended_source,
        "combined_fit": fit_dict(analysis.pooled),
        "quasistatic_fit": fit_dict(analysis.pooled_quasistatic),
        "zoh_fit": fit_dict(analysis.pooled_zoh),
        "theoretical_kV": _theoretical_kv(analysis),
        "motors": {
            motor: {
                "velocity_unit": motor_fit.velocity_unit,
                "samples": motor_fit.samples,
                "segments": motor_fit.segments,
                "combined_fit": fit_dict(motor_fit.combined),
                "quasistatic_fit": fit_dict(motor_fit.quasistatic),
                "zoh_fit": fit_dict(motor_fit.zoh),
                "max_abs_voltage": motor_fit.max_abs_voltage,
                "max_abs_velocity": motor_fit.max_abs_velocity,
                "min_velocity": motor_fit.min_velocity,
                "saturated_fraction": motor_fit.saturated_fraction,
            }
            for motor, motor_fit in sorted(analysis.motors.items())
        },
        "warnings": analysis.warnings,
    }


def _theoretical_kv(analysis: MechanismAnalysis) -> Optional[float]:
    """12 V divided by free speed, expressed in the log's velocity unit."""
    info = analysis.info
    unit = analysis.velocity_unit
    if info is None or not unit:
        return None
    try:
        free_speed = velocity_to_unit(info.free_speed, info.natural_unit, unit)
    except ValueError:
        return None
    return 12.0 / free_speed if free_speed else None


def _velocity_unit_label(unit: str) -> str:
    return {
        METERS_PER_SEC: "m/s",
        ROTATIONS_PER_SEC: "rot/s",
        RPM: "RPM",
        RADIANS_PER_SEC: "rad/s",
        DEGREES_PER_SEC: "deg/s",
    }.get(unit, unit or "?")


def _acceleration_unit_label(unit: str) -> str:
    label = _velocity_unit_label(unit)
    if label in ("m/s", "rad/s", "deg/s"):
        return label[:-2] + "/s^2"
    return f"{label}/s"


def text_report(analyses: Sequence[MechanismAnalysis], path: str) -> str:
    lines: List[str] = [f"SysId identification - {path}"]
    for analysis in analyses:
        unit = _velocity_unit_label(analysis.velocity_unit)
        lines.append("")
        lines.append("=" * 78)
        lines.append(f"{analysis.mechanism}  -  {analysis.label}")
        lines.append("=" * 78)
        lines.append(f"tests: {', '.join(analysis.tests_found) or 'none'}")
        lines.append(f"velocity unit: {unit}")

        recommended = analysis.recommended
        source = "exact ZOH model" if analysis.recommended_source == "zoh" else "simple model"
        if analysis.recommended_fit is not None:
            fit = analysis.recommended_fit
            lines.append("")
            lines.append(
                f"recommended ({source}):  kS = {_fmt(recommended['kS'])} V"
                f"   kV = {_fmt(recommended['kV'])} V/({unit})"
                f"   kA = {_fmt(recommended['kA'])} V/({_acceleration_unit_label(analysis.velocity_unit)})"
            )
            lines.append(
                f"    fit quality: R^2 = {fit.r_squared:.4f}"
                f"   RMSE = {_fmt(fit.rmse, 3)} {fit.rmse_unit}   n = {fit.samples}"
            )
        if analysis.pooled is not None and analysis.recommended_source != "simple":
            simple = analysis.pooled
            lines.append(
                f"simple fit (WPILib-SysId-comparable):  kS = {_fmt(simple.get('kS'))} V"
                f"   kV = {_fmt(simple.get('kV'))} V/({unit})"
                f"   kA = {_fmt(simple.get('kA'))} V/({_acceleration_unit_label(analysis.velocity_unit)})"
                f"   (R^2 = {simple.r_squared:.4f})"
            )
        if analysis.pooled_quasistatic:
            qs = analysis.pooled_quasistatic
            lines.append(
                f"quasistatic-only (velocity) fit:       kS = {_fmt(qs.get('kS'))} V"
                f"   kV = {_fmt(qs.get('kV'))} V/({unit})   (R^2 = {qs.r_squared:.4f})"
            )
        if analysis.pooled_zoh and analysis.recommended_source != "zoh":
            zoh = analysis.pooled_zoh
            lines.append(
                f"exact ZOH cross-check:                 kS = {_fmt(zoh.get('kS'))} V"
                f"   kV = {_fmt(zoh.get('kV'))} V/({unit})"
                f"   kA = {_fmt(zoh.get('kA'))} V/({_acceleration_unit_label(analysis.velocity_unit)})"
                f"   (R^2 = {zoh.r_squared:.4f})"
            )
        theory = _theoretical_kv(analysis)
        if theory is not None:
            measured = recommended.get("kV")
            delta = (
                f"  ({measured / theory - 1:+.1%} vs. free-speed model)"
                if measured
                else ""
            )
            lines.append(
                f"free-speed expectation (12 V / max):    kV = {_fmt(theory)} V/({unit}){delta}"
            )

        if analysis.motors:
            lines.append("")
            lines.append("per-motor:")
            header = (
                f"    {'motor':<20}{'samples':>8}"
                f"{'kS':>10}{'kV':>12}{'kA':>12}{'R^2':>8}{'vmax':>10}{'sat':>7}"
            )
            lines.append(header)
            for motor, motor_fit in sorted(analysis.motors.items()):
                combined = motor_fit.combined
                lines.append(
                    f"    {motor:<20}{motor_fit.samples:>8}"
                    f"{_fmt(combined.get('kS') if combined else None, 3):>10}"
                    f"{_fmt(combined.get('kV') if combined else None, 4):>12}"
                    f"{_fmt(combined.get('kA') if combined else None, 4):>12}"
                    f"{(f'{combined.r_squared:.3f}' if combined else 'n/a'):>8}"
                    f"{_fmt(motor_fit.max_abs_velocity, 3):>10}"
                    f"{motor_fit.saturated_fraction * 100:>6.0f}%"
                )
        if analysis.warnings:
            lines.append("")
            lines.append("warnings:")
            for warning in analysis.warnings:
                lines.append(f"  ! {warning}")
        if analysis.info is not None:
            lines.append("")
            lines.append(f"next step: {analysis.info.java_hint}")
    return "\n".join(lines)


def java_report(analyses: Sequence[MechanismAnalysis]) -> str:
    lines = [
        "// ---------------------------------------------------------------------------",
        "// SysId feedforward gains identified from a .wpilog",
        "// Generated by tools/sysid_wpilog.py - paste into the matching constants.",
        "// ---------------------------------------------------------------------------",
    ]
    for analysis in analyses:
        recommended = analysis.recommended
        if recommended.get("kS") is None:
            continue
        unit = _velocity_unit_label(analysis.velocity_unit)
        lines.append("")
        lines.append(f"// {analysis.label}  (velocity unit: {unit})")
        fit = analysis.recommended_fit
        if fit is not None:
            lines.append(
                f"// from the {analysis.recommended_source.upper()} fit:"
                f" R^2 = {fit.r_squared:.4f}, RMSE = {_fmt(fit.rmse, 3)}"
                f" {fit.rmse_unit}, {fit.samples} samples"
            )
        lines.append(f"public static final double k{_camel(analysis.mechanism)}KS = {recommended['kS']:.6f}; // V")
        lines.append(
            f"public static final double k{_camel(analysis.mechanism)}KV = {recommended['kV']:.6f};"
            f" // V / ({unit})"
        )
        if recommended.get("kA") is not None:
            lines.append(
                f"public static final double k{_camel(analysis.mechanism)}KA ="
                f" {recommended['kA']:.6f};"
                f" // V / ({_acceleration_unit_label(analysis.velocity_unit)})"
            )
        if analysis.info is not None:
            lines.append(f"// {analysis.info.java_hint}")
    return "\n".join(lines)


def _camel(mechanism: str) -> str:
    parts = mechanism.replace("_", "-").split("-")
    return "".join(part.capitalize() for part in parts)


def markdown_report(
    analyses: Sequence[MechanismAnalysis], path: str, source_file: str
) -> str:
    lines = [
        "# SysId results",
        "",
        f"Source log: `{source_file}`  ",
        "Generated by `python3 tools/sysid_wpilog.py fit ... --markdown`.",
        "",
        "| Mechanism | Fit | kS (V) | kV | kA | R² | n | Velocity unit | Theory kV |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for analysis in analyses:
        fit = analysis.recommended_fit
        recommended = analysis.recommended
        unit = _velocity_unit_label(analysis.velocity_unit)
        if fit is None:
            lines.append(f"| `{analysis.mechanism}` | - | - | - | - | - | 0 | {unit} | - |")
            continue
        source = "ZOH" if analysis.recommended_source == "zoh" else "simple"
        lines.append(
            f"| `{analysis.mechanism}` | {source} | {_fmt(recommended['kS'])} |"
            f" {_fmt(recommended['kV'])} | {_fmt(recommended['kA'])} |"
            f" {fit.r_squared:.4f} | {fit.samples} | {unit} |"
            f" {_fmt(_theoretical_kv(analysis))} |"
        )

    lines.append("")
    lines.append("## Per-motor detail")
    for analysis in analyses:
        lines.append("")
        lines.append(f"### `{analysis.mechanism}` - {analysis.label}")
        lines.append("")
        lines.append("| Motor | samples | kS (V) | kV | kA | R² | v_max | saturated |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for motor, motor_fit in sorted(analysis.motors.items()):
            combined = motor_fit.combined
            lines.append(
                f"| `{motor}` | {motor_fit.samples} |"
                f" {_fmt(combined.get('kS') if combined else None, 3)} |"
                f" {_fmt(combined.get('kV') if combined else None)} |"
                f" {_fmt(combined.get('kA') if combined else None)} |"
                f" {(f'{combined.r_squared:.4f}' if combined else 'n/a')} |"
                f" {_fmt(motor_fit.max_abs_velocity, 3)} |"
                f" {motor_fit.saturated_fraction * 100:.0f}% |"
            )
        if analysis.warnings:
            lines.append("")
            for warning in analysis.warnings:
                lines.append(f"- :warning: {warning}")
        if analysis.info is not None:
            lines.append("")
            lines.append(f"*Next step:* {analysis.info.java_hint}")

    lines.append("")
    lines.append("## Java constants")
    lines.append("")
    lines.append("```java")
    lines.append(java_report(analyses))
    lines.append("```")
    return "\n".join(lines)


def write_csv(analyses: Sequence[MechanismAnalysis], segments_by_mechanism, directory: Path) -> List[Path]:
    directory.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []
    for analysis in analyses:
        segments = segments_by_mechanism[analysis.mechanism]
        for motor in sorted(analysis.motors):
            rows = [
                sample
                for segment in segments
                for sample in segment.samples.get(motor, [])
            ]
            if not rows:
                continue
            output = directory / f"{analysis.mechanism}-{motor}.csv".replace(" ", "_")
            with output.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["time_s", "voltage_V", "velocity", "position", "acceleration"])
                for sample in rows:
                    writer.writerow(
                        [
                            f"{sample.time_s:.4f}",
                            f"{sample.voltage:.6f}",
                            f"{sample.velocity:.6f}",
                            f"{sample.position:.6f}" if not math.isnan(sample.position) else "",
                            "" if sample.acceleration is None else f"{sample.acceleration:.6f}",
                        ]
                    )
            written.append(output)
    return written


# ---------------------------------------------------------------------------
# Synthetic demo log (exercises the whole pipeline without a robot)
# ---------------------------------------------------------------------------

#: First-order plant parameters used to synthesize a believable log.
DEMO_PLANTS = {
    "drive-linear": dict(
        motors=["drive-front-left", "drive-front-right"],
        kv=2.69,  # V / (m/s), a loaded 65 kg chassis on NEOs
        ka=0.55,  # V / (m/s^2)
        ks=0.18,  # V
    ),
    "shooter-flywheels": dict(
        motors=["shooter-front", "shooter-back"],
        kv=12.0 / (5676.0 / 60.0),  # V / (rot/s) for a 1:1 NEO flywheel
        ka=0.048,  # V / (rot/s^2)  (~0.38 s mechanical time constant)
        ks=0.02,  # V
    ),
}


def build_demo_log(
    path: Path,
    mechanisms: Sequence[str] = ("drive-linear", "shooter-flywheels"),
    dt: float = 0.02,
    seed: int = 7,
    speed_noise_fraction: float = 0.002,
) -> Path:
    """Write a synthetic 4-part SysId log with known ground-truth gains.

    Each motor is simulated with the very model the fitter inverts
    (``dv/dt = (V - kS*sgn(v) - kV*v) / kA``) using exact zero-order hold, so a
    correct pipeline must recover ``DEMO_PLANTS`` almost exactly. Small
    per-motor gain variation and velocity measurement noise are added so the
    quality checks and per-motor spread report something realistic.
    """
    import random

    rng = random.Random(seed)
    writer = WpiLogWriter()
    timestamp_us = 0

    for mechanism in mechanisms:
        plant = DEMO_PLANTS[mechanism]
        info = MECHANISMS[mechanism]
        unit = info.logged_unit
        is_linear = unit == METERS_PER_SEC
        # Gains in the logged unit (e.g. V/RPM rather than V/(rot/s)).
        unit_scale = velocity_to_unit(1.0, info.natural_unit, unit)
        kv = plant["kv"] / unit_scale
        ka = plant["ka"] / unit_scale
        ks = plant["ks"]
        free_speed_logged = velocity_to_unit(
            info.free_speed, info.natural_unit, unit
        )
        motor_gains = {
            motor: 1.0 + rng.uniform(-0.02, 0.02) for motor in plant["motors"]
        }
        noise_sigma = speed_noise_fraction * free_speed_logged

        voltage_entries: Dict[str, int] = {}
        velocity_entries: Dict[str, int] = {}
        position_entries: Dict[str, int] = {}
        for motor in plant["motors"]:
            voltage_entries[motor] = writer.start_entry(
                f"voltage-{motor}-{mechanism}", "double", "Volts"
            )
            velocity_entries[motor] = writer.start_entry(
                ("linear-velocity-" if is_linear else "angular-velocity-")
                + f"{motor}-{mechanism}",
                "double",
                unit,
            )
            position_entries[motor] = writer.start_entry(
                ("linear-position-" if is_linear else "angular-position-")
                + f"{motor}-{mechanism}",
                "double",
                "Meters" if is_linear else "Rotations",
            )
        state_entry = writer.start_entry(f"sysid-test-state-{mechanism}", "string", "")

        for test in TEST_ORDER:
            dynamic = not is_quasistatic(test)
            direction = 1.0 if is_forward(test) else -1.0
            velocity = 0.0
            position = 0.0
            duration = 6.0 if dynamic else 4.0
            peak_volts = 6.0 if dynamic else 3.0
            ramp_volts_per_s = peak_volts / 3.0
            steps = int(round(duration / dt))
            for step in range(steps):
                time_s = step * dt
                volts = direction * (
                    peak_volts if dynamic else min(peak_volts, ramp_volts_per_s * time_s)
                )
                writer.append_string(state_entry, timestamp_us, test)
                for motor in plant["motors"]:
                    gain = motor_gains[motor]
                    writer.append_double(voltage_entries[motor], timestamp_us, volts)
                    writer.append_double(
                        velocity_entries[motor],
                        timestamp_us,
                        velocity * gain + rng.gauss(0.0, noise_sigma),
                    )
                    writer.append_double(
                        position_entries[motor], timestamp_us, position * gain
                    )
                # Exact ZOH integration of the standard SysId model.
                target = (volts - ks * math.copysign(1.0, velocity)) / kv
                decay = math.exp(-dt * kv / ka)
                velocity = target + (velocity - target) * decay
                position += velocity * dt
                timestamp_us += int(round(dt * 1e6))
            writer.append_string(state_entry, timestamp_us, "none")
            timestamp_us += int(1.0 / dt) * int(dt * 1e6)  # 1 s settle at 50 Hz

    return writer.write(path)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _load(path: str) -> WpiLog:
    try:
        return read_wpilog(path)
    except FileNotFoundError:
        raise SystemExit(f"error: no such file: {path}")
    except WpiLogError as exc:
        raise SystemExit(f"error: {exc}")
    except OSError as exc:
        raise SystemExit(f"error: cannot read {path}: {exc}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="sysid_wpilog.py",
        description="Identify SysId feedforward gains (kS, kV, kA) from a WPILib .wpilog.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  sysid_wpilog.py info logs/FRC_20260418_152525.wpilog\n"
            "  sysid_wpilog.py fit logs/FRC_20260418_152525.wpilog --java\n"
            "  sysid_wpilog.py fit logs/FRC_....wpilog --mechanism drive-linear \\\n"
            "      --json gains.json --markdown SYSID_RESULTS.md --csv sysid_csv\n"
            "  sysid_wpilog.py demo /tmp/synthetic_sysid.wpilog\n"
            "\n"
            "See the module docstring (or tools/README.md) for the log format"
            " this reads."
        ),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    info_parser = subparsers.add_parser("info", help="list mechanisms, tests, and units in a log")
    info_parser.add_argument("log", help="path to the .wpilog file")

    fit_parser = subparsers.add_parser("fit", help="identify kS/kV/kA and report")
    fit_parser.add_argument("log", help="path to the .wpilog file")
    fit_parser.add_argument(
        "--mechanism",
        action="append",
        default=None,
        help="only analyze this mechanism (repeatable, e.g. --mechanism drive-linear)",
    )
    fit_parser.add_argument("--json", metavar="PATH", help="write a JSON report")
    fit_parser.add_argument("--markdown", metavar="PATH", help="write a Markdown report")
    fit_parser.add_argument("--csv", metavar="DIR", help="write per-motor sample CSVs")
    fit_parser.add_argument("--java", action="store_true", help="print paste-ready Java constants")
    fit_parser.add_argument("--quiet", action="store_true", help="suppress the text report")
    fit_parser.add_argument("--dt", type=float, default=0.02, help="nominal loop period (default 0.02 s)")
    fit_parser.add_argument(
        "--accel-threshold", type=float, default=0.2,
        help="dynamic windows must exceed this fraction of peak |accel| (default 0.2)",
    )
    fit_parser.add_argument(
        "--velocity-threshold", type=float, default=0.2,
        help="dynamic windows must exceed this fraction of peak |velocity| (default 0.2)",
    )
    fit_parser.add_argument(
        "--fail-on-warning", action="store_true", help="exit 3 if any quality warning is raised"
    )

    demo_parser = subparsers.add_parser("demo", help="write a synthetic .wpilog with known gains")
    demo_parser.add_argument("output", help="path to write (e.g. /tmp/synthetic_sysid.wpilog)")
    demo_parser.add_argument(
        "--mechanism", action="append", default=None, choices=sorted(DEMO_PLANTS),
        help="mechanism(s) to include (default: drive-linear and shooter-flywheels)",
    )

    args = parser.parse_args(argv)

    if args.command == "demo":
        mechanisms = args.mechanism or list(DEMO_PLANTS)
        output = build_demo_log(Path(args.output), mechanisms)
        print(f"wrote synthetic SysId log: {output}")
        for mechanism in mechanisms:
            plant = DEMO_PLANTS[mechanism]
            info = MECHANISMS[mechanism]
            scale = velocity_to_unit(1.0, info.natural_unit, info.logged_unit)
            print(
                f"  {mechanism}: ground truth kS={plant['ks']} V,"
                f" kV={plant['kv'] / scale:.6g} V/({_velocity_unit_label(info.logged_unit)}),"
                f" kA={plant['ka'] / scale:.6g}"
                f" V/({_acceleration_unit_label(info.logged_unit)})"
            )
        return 0

    log = _load(args.log)
    mechanisms = extract_mechanisms(log)
    if not mechanisms:
        print(info_report(log, args.log))
        return 1

    if args.command == "info":
        print(info_report(log, args.log))
        return 0

    selected = args.mechanism or sorted(mechanisms)
    unknown = [name for name in selected if name not in mechanisms]
    if unknown:
        print(
            f"warning: no SysId data for {', '.join(unknown)} in this log"
            f" (found: {', '.join(sorted(mechanisms))})",
            file=sys.stderr,
        )

    analyses: List[MechanismAnalysis] = []
    segments_by_mechanism = {}
    for name in selected:
        if name not in mechanisms:
            continue
        analysis = analyze_mechanism(
            name, mechanisms[name], args.dt, args.accel_threshold, args.velocity_threshold
        )
        if analysis is None:
            continue
        analyses.append(analysis)
        segments_by_mechanism[name] = build_segments(mechanisms[name], args.dt)

    if not analyses:
        print("error: no usable SysId data found for the requested mechanism(s)", file=sys.stderr)
        return 1

    if not args.quiet:
        print(text_report(analyses, args.log))
    if args.java:
        print()
        print(java_report(analyses))
    if args.json:
        payload = {
            "source": str(Path(args.log).resolve()),
            "truncated": log.truncated,
            "mechanisms": [analysis_to_dict(analysis) for analysis in analyses],
        }
        Path(args.json).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nwrote {args.json}")
    if args.markdown:
        Path(args.markdown).write_text(
            markdown_report(analyses, args.log, args.log), encoding="utf-8"
        )
        print(f"wrote {args.markdown}")
    if args.csv:
        written = write_csv(analyses, segments_by_mechanism, Path(args.csv))
        print(f"wrote {len(written)} CSV file(s) to {args.csv}")

    warnings = sum(len(analysis.warnings) for analysis in analyses)
    if args.fail_on_warning and warnings:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
