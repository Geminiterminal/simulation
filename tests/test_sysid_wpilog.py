"""Tests for the .wpilog reader and the SysId gain identification tool.

Run with:  python3 -m unittest discover -s tests -v

The synthetic logs are written by helpers in this file (independent of the
tool's own ``build_demo_log``) so the extraction and fitting code is exercised
against data it did not create. One test additionally hand-assembles the raw
bytes of a WPILib data log to pin the binary format itself, rather than only
round-tripping through our own writer.
"""

import contextlib
import gzip
import io
import json
import math
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

import sysid_wpilog as sysid  # noqa: E402
import wpilog  # noqa: E402

DT = 0.02


# ---------------------------------------------------------------------------
# Fixture builders
# ---------------------------------------------------------------------------


def _inner_string(text):
    encoded = text.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _record(entry_id, timestamp_us, payload):
    def varint(value):
        length = max(1, (value.bit_length() + 7) // 8)
        return value.to_bytes(length, "little")

    entry_bytes = varint(entry_id)
    size_bytes = varint(len(payload))
    time_bytes = varint(timestamp_us)
    header = bytes(
        [
            (len(entry_bytes) - 1)
            | ((len(size_bytes) - 1) << 2)
            | ((len(time_bytes) - 1) << 4)
        ]
    )
    return header + entry_bytes + size_bytes + time_bytes + payload


def hand_built_log():
    """A WPILib data log assembled byte-by-byte from the published spec."""
    data = bytearray(b"WPILOG")
    data += struct.pack("<H", 0x0100)
    data += struct.pack("<I", 2) + b"{}"
    # Start control record for entry 5.
    start = (
        bytes([0])
        + struct.pack("<I", 5)
        + _inner_string("/Shooter/Front RPM")
        + _inner_string("double")
        + _inner_string("RPM")
    )
    data += _record(0, 0, start)
    # Two data records, 20 ms apart.
    data += _record(5, 1_000_000, struct.pack("<d", 1234.5))
    data += _record(5, 1_020_000, struct.pack("<d", 1500.25))
    return bytes(data)


def build_sysid_log(
    path,
    mechanism="drive-linear",
    motors=("drive-front-left", "drive-front-right"),
    ks=0.18,
    kv=2.69,
    ka=0.55,
    unit="MetersPerSecond",
    tests=sysid.TEST_ORDER,
    naming="flat",
    noise=0.0,
    dt=DT,
    quasistatic_seconds=2.5,
    dynamic_seconds=3.0,
    write_velocity=True,
    gains_spread=0.0,
):
    """Write a synthetic SysId log with a known ground-truth plant."""
    writer = wpilog.WpiLogWriter()
    timestamp_us = 0
    is_linear = unit == "MetersPerSecond"
    voltage_entries = {}
    velocity_entries = {}
    position_entries = {}
    for index, motor in enumerate(motors):
        if naming == "flat":
            volt_name = f"voltage-{motor}-{mechanism}"
            vel_name = ("linear-velocity-" if is_linear else "angular-velocity-") + f"{motor}-{mechanism}"
            pos_name = ("linear-position-" if is_linear else "angular-position-") + f"{motor}-{mechanism}"
        else:  # WPILib <= 2023 layout
            volt_name = f"sysid/{motor}/voltage"
            vel_name = f"sysid/{motor}/" + ("linear-velocity" if is_linear else "angular-velocity")
            pos_name = f"sysid/{motor}/" + ("linear-position" if is_linear else "angular-position")
        voltage_entries[motor] = writer.start_entry(volt_name, "double", "Volts")
        if write_velocity:
            velocity_entries[motor] = writer.start_entry(vel_name, "double", unit)
        position_entries[motor] = writer.start_entry(
            pos_name, "double", "Meters" if is_linear else "Rotations"
        )
    state_entry = writer.start_entry(
        "sysid-test-state-" + mechanism if naming == "flat" else "sysid/state",
        "string",
        "",
    )

    gains = {
        motor: 1.0 + (gains_spread * (index - (len(motors) - 1) / 2.0))
        for index, motor in enumerate(motors)
    }

    for test in tests:
        dynamic = not sysid.is_quasistatic(test)
        direction = 1.0 if sysid.is_forward(test) else -1.0
        duration = dynamic_seconds if dynamic else quasistatic_seconds
        peak_volts = 6.0 if dynamic else 2.0
        velocity = 0.0
        position = 0.0
        steps = int(round(duration / dt))
        for step in range(steps):
            time_s = step * dt
            volts = direction * (peak_volts if dynamic else min(peak_volts, time_s))
            writer.append_string(state_entry, timestamp_us, test)
            for motor in motors:
                writer.append_double(voltage_entries[motor], timestamp_us, volts)
                if write_velocity:
                    noise_value = noise * math.sin(step * 1.7 + len(motor)) if noise else 0.0
                    writer.append_double(
                        velocity_entries[motor],
                        timestamp_us,
                        velocity * gains[motor] + noise_value,
                    )
                writer.append_double(position_entries[motor], timestamp_us, position)
            target = (volts - ks * math.copysign(1.0, velocity)) / kv
            velocity = target + (velocity - target) * math.exp(-dt * kv / ka)
            position += velocity * dt
            timestamp_us += int(round(dt * 1e6))
        writer.append_string(state_entry, timestamp_us, "none")
        timestamp_us += int(1.0 * 1e6)

    return writer.write(path)


class TempDirTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()


# ---------------------------------------------------------------------------
# Binary format
# ---------------------------------------------------------------------------


class WpiLogFormatTest(TempDirTestCase):
    def test_hand_built_log_matches_the_published_layout(self):
        log = wpilog.parse_wpilog(hand_built_log(), source="hand-built")
        self.assertEqual(0x0100, log.version)
        self.assertEqual("{}", log.extra_header)
        self.assertEqual(1, len(log.entries))
        entry = log.entry("/Shooter/Front RPM")
        self.assertIsNotNone(entry)
        self.assertEqual("double", entry.type)
        self.assertEqual("RPM", entry.unit)

        values = log["/Shooter/Front RPM"]
        self.assertEqual(2, len(values))
        self.assertEqual(1_000_000, values[0][0])
        self.assertAlmostEqual(1234.5, values[0][1], places=9)
        self.assertAlmostEqual(1500.25, values[1][1], places=9)
        self.assertFalse(log.truncated)

    def test_writer_roundtrip_for_every_scalar_type(self):
        writer = wpilog.WpiLogWriter(extra_header='{"source":"test"}')
        double_id = writer.start_entry("d", "double", "Volts")
        float_id = writer.start_entry("f", "float", "")
        int_id = writer.start_entry("i", "int64", "")
        bool_id = writer.start_entry("b", "boolean", "")
        string_id = writer.start_entry("s", "string", "")
        raw_id = writer.start_entry("r", "raw", "")
        writer.append_double(double_id, 10, 1.25)
        writer.append_float(float_id, 20, 2.5)
        writer.append_int(int_id, 30, -7)
        writer.append_bool(bool_id, 40, True)
        writer.append_string(string_id, 50, "quasistatic-forward")
        writer._write_record(raw_id, 60, b"\x01\x02")
        path = writer.write(self.tmp / "roundtrip.wpilog")

        log = wpilog.read_wpilog(path)
        self.assertEqual('{"source":"test"}', log.extra_header)
        self.assertAlmostEqual(1.25, log["d"][0][1], places=9)
        self.assertAlmostEqual(2.5, log["f"][0][1], places=5)
        self.assertEqual(-7, log["i"][0][1])
        self.assertIs(True, log["b"][0][1])
        self.assertEqual("quasistatic-forward", log["s"][0][1])
        self.assertEqual(b"\x01\x02", log["r"][0][1])

    def test_truncated_log_returns_the_complete_records(self):
        data = hand_built_log()
        truncated = data[:-3]  # chop the last double in half
        log = wpilog.parse_wpilog(truncated, source="cut")
        self.assertTrue(log.truncated)
        self.assertEqual(1, len(log["/Shooter/Front RPM"]))

    def test_gzip_and_plain_paths_are_both_supported(self):
        plain = self.tmp / "plain.wpilog"
        plain.write_bytes(hand_built_log())
        compressed = self.tmp / "plain.wpilog.gz"
        compressed.write_bytes(gzip.compress(hand_built_log()))
        self.assertEqual(
            wpilog.read_wpilog(plain)["/Shooter/Front RPM"][0][1],
            wpilog.read_wpilog(compressed)["/Shooter/Front RPM"][0][1],
        )

    def test_git_lfs_pointer_and_garbage_are_rejected_with_useful_errors(self):
        lfs = self.tmp / "lfs.wpilog"
        lfs.write_text(
            "version https://git-lfs.github.com/spec/v1\n"
            "oid sha256:abc\nsize 123\n"
        )
        with self.assertRaises(wpilog.WpiLogError) as context:
            wpilog.read_wpilog(lfs)
        self.assertIn("Git LFS", str(context.exception))

        junk = self.tmp / "junk.wpilog"
        junk.write_bytes(b"not a log at all")
        with self.assertRaises(wpilog.WpiLogError) as context:
            wpilog.read_wpilog(junk)
        self.assertIn("WPILOG", str(context.exception))

    def test_empty_file_is_an_error(self):
        empty = self.tmp / "empty.wpilog"
        empty.write_bytes(b"")
        with self.assertRaises(wpilog.WpiLogError):
            wpilog.read_wpilog(empty)

    def test_struct_metadata_is_not_mistaken_for_a_unit(self):
        entry = wpilog.WpiLogEntry(1, "x", "struct:Foo", '{"schema":"...","type":"Foo"}')
        self.assertEqual("", entry.unit)
        self.assertEqual("Volts", wpilog.WpiLogEntry(2, "v", "double", "Volts").unit)


# ---------------------------------------------------------------------------
# SysId extraction and segmentation
# ---------------------------------------------------------------------------


class SysIdExtractionTest(TempDirTestCase):
    def test_flat_2024_naming_is_grouped_per_mechanism(self):
        path = build_sysid_log(self.tmp / "flat.wpilog")
        log = wpilog.read_wpilog(path)
        mechanisms = sysid.extract_mechanisms(log)
        self.assertIn("drive-linear", mechanisms)
        self.assertEqual(
            {"drive-front-left", "drive-front-right"},
            set(mechanisms["drive-linear"].motors),
        )
        series = mechanisms["drive-linear"].motors["drive-front-left"]
        self.assertEqual("MetersPerSecond", series.velocity_unit)
        self.assertEqual("Volts", series.voltage_unit)
        self.assertEqual("linear-velocity", series.velocity_field)

    def test_legacy_2023_nested_naming_is_grouped_too(self):
        path = build_sysid_log(self.tmp / "legacy.wpilog", naming="legacy")
        log = wpilog.read_wpilog(path)
        mechanisms = sysid.extract_mechanisms(log)
        self.assertIn("sysid", mechanisms)
        self.assertIn("drive-front-left", mechanisms["sysid"].motors)
        analysis = sysid.analyze_mechanism("sysid", mechanisms["sysid"])
        self.assertIsNotNone(analysis)
        self.assertAlmostEqual(2.69, analysis.recommended["kV"], delta=0.15)

    def test_four_part_routine_is_segmented_in_order(self):
        path = build_sysid_log(self.tmp / "segments.wpilog")
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        segments = sysid.build_segments(data)
        self.assertEqual(list(sysid.TEST_ORDER), [segment.test for segment in segments])
        for segment in segments:
            self.assertGreater(segment.duration_seconds, 2.0)
            self.assertGreater(len(segment.samples["drive-front-left"]), 100)

    def test_state_strings_are_normalized_across_wpilib_spellings(self):
        cases = {
            "quasistatic-forward": "quasistatic-forward",
            "quasistaticForward": "quasistatic-forward",
            "QUASISTATIC_REVERSE": "quasistatic-reverse",
            "dynamic-forward": "dynamic-forward",
            "dynamicReverse": "dynamic-reverse",
            "none": None,
            "": None,
            "garbage": None,
        }
        for raw, expected in cases.items():
            self.assertEqual(expected, sysid.normalize_state(raw), raw)

    def test_missing_state_entry_falls_back_to_one_window_with_a_warning(self):
        path = build_sysid_log(self.tmp / "nostate.wpilog")
        log = wpilog.read_wpilog(path)
        # Simulate an older / stripped log by dropping the state entry.
        for entry_id, entry in list(log.entries.items()):
            if entry.name.startswith("sysid-test-state-"):
                del log.entries[entry_id]
        data = sysid.extract_mechanisms(log)["drive-linear"]
        segments = sysid.build_segments(data)
        self.assertEqual(["unknown"], [segment.test for segment in segments])
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertTrue(
            any("no 'sysid-test-state" in warning for warning in analysis.warnings),
            analysis.warnings,
        )

    def test_position_only_log_still_identifies(self):
        path = build_sysid_log(self.tmp / "posonly.wpilog", write_velocity=False)
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertIsNotNone(analysis)
        # Differentiating position is noisier than reading velocity, but the
        # scale must still be right.
        self.assertAlmostEqual(2.69, analysis.recommended["kV"], delta=0.35)


# ---------------------------------------------------------------------------
# Identification accuracy
# ---------------------------------------------------------------------------


class IdentificationTest(TempDirTestCase):
    def test_linear_drive_gains_are_recovered(self):
        path = build_sysid_log(self.tmp / "drive.wpilog", ks=0.18, kv=2.69, ka=0.55)
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertEqual("zoh", analysis.recommended_source)
        self.assertAlmostEqual(0.18, analysis.recommended["kS"], delta=0.03)
        self.assertAlmostEqual(2.69, analysis.recommended["kV"], delta=0.08)
        self.assertAlmostEqual(0.55, analysis.recommended["kA"], delta=0.06)
        self.assertGreater(analysis.recommended_fit.r_squared, 0.999)
        self.assertEqual([], analysis.warnings)

    def test_angular_flywheel_gains_use_the_logged_units(self):
        # Same plant expressed in RPM: kV = 2.69 m/s -> 1234 RPM equivalent.
        kv_rpm = 0.002114
        path = build_sysid_log(
            self.tmp / "shooter.wpilog",
            mechanism="shooter-flywheels",
            motors=("shooter-front", "shooter-back"),
            ks=0.02,
            kv=kv_rpm,
            ka=0.0008,
            unit="RPM",
        )
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["shooter-flywheels"]
        analysis = sysid.analyze_mechanism("shooter-flywheels", data)
        self.assertAlmostEqual(kv_rpm, analysis.recommended["kV"], delta=kv_rpm * 0.05)
        self.assertAlmostEqual(0.02, analysis.recommended["kS"], delta=0.01)
        self.assertEqual("RPM", analysis.velocity_unit)
        # 12 V / 5676 RPM is the free-speed expectation for a 1:1 NEO flywheel.
        self.assertAlmostEqual(12.0 / 5676.0, sysid._theoretical_kv(analysis), delta=1e-6)

    def test_theory_conversion_respects_rot_per_second_vs_rpm(self):
        self.assertAlmostEqual(
            5676.0,
            sysid.velocity_to_unit(94.6, sysid.ROTATIONS_PER_SEC, sysid.RPM),
            delta=1e-6,
        )
        self.assertAlmostEqual(
            2.0 * math.pi * 94.6,
            sysid.velocity_to_unit(94.6, sysid.ROTATIONS_PER_SEC, sysid.RADIANS_PER_SEC),
            delta=1e-6,
        )
        with self.assertRaises(ValueError):
            sysid.velocity_to_unit(1.0, sysid.METERS_PER_SEC, sysid.RPM)

    def test_noisy_log_still_fits_but_raises_quality_warnings(self):
        path = build_sysid_log(
            self.tmp / "noisy.wpilog", kv=2.69, ka=0.55, noise=0.35
        )
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertAlmostEqual(2.69, analysis.recommended["kV"], delta=0.5)
        self.assertTrue(analysis.warnings, "noisy data should trigger quality warnings")
        self.assertTrue(
            any("velocity residual" in warning for warning in analysis.warnings),
            analysis.warnings,
        )

    def test_mismatched_motors_raise_a_spread_warning(self):
        path = build_sysid_log(self.tmp / "spread.wpilog", gains_spread=0.15)
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertTrue(
            any("per-motor kV spread" in warning for warning in analysis.warnings),
            analysis.warnings,
        )

    def test_missing_test_legs_are_reported(self):
        path = build_sysid_log(
            self.tmp / "partial.wpilog",
            tests=("quasistatic-forward", "quasistatic-reverse"),
        )
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertTrue(
            any("missing test legs" in warning for warning in analysis.warnings),
            analysis.warnings,
        )
        # The quasistatic ramp still carries acceleration information, so a
        # three-parameter fit remains possible - it is just less trustworthy.
        self.assertAlmostEqual(2.69, analysis.recommended["kV"], delta=0.25)

    def test_short_tests_are_flagged(self):
        path = build_sysid_log(
            self.tmp / "short.wpilog",
            quasistatic_seconds=0.6,
            dynamic_seconds=0.6,
        )
        data = sysid.extract_mechanisms(wpilog.read_wpilog(path))["drive-linear"]
        analysis = sysid.analyze_mechanism("drive-linear", data)
        self.assertTrue(
            any("ramp may be truncated" in warning for warning in analysis.warnings),
            analysis.warnings,
        )


# ---------------------------------------------------------------------------
# Reports and CLI
# ---------------------------------------------------------------------------


class ReportingTest(TempDirTestCase):
    def setUp(self):
        super().setUp()
        self.log_path = build_sysid_log(self.tmp / "report.wpilog")

    def _analysis(self):
        data = sysid.extract_mechanisms(wpilog.read_wpilog(self.log_path))["drive-linear"]
        return sysid.analyze_mechanism("drive-linear", data)

    def test_text_report_contains_the_identified_gains(self):
        text = sysid.text_report([self._analysis()], "report.wpilog")
        self.assertIn("drive-linear", text)
        self.assertIn("kV = ", text)
        self.assertIn("recommended", text)

    def test_java_report_emits_paste_ready_constants(self):
        java = sysid.java_report([self._analysis()])
        self.assertIn("kDriveLinearKS", java)
        self.assertIn("kDriveLinearKV", java)
        self.assertIn("kDriveLinearKA", java)
        self.assertIn("public static final double", java)

    def test_json_markdown_and_csv_outputs(self):
        out_json = self.tmp / "out.json"
        out_md = self.tmp / "out.md"
        csv_dir = self.tmp / "csv"
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            code = sysid.main(
                [
                    "fit",
                    str(self.log_path),
                    "--json",
                    str(out_json),
                    "--markdown",
                    str(out_md),
                    "--csv",
                    str(csv_dir),
                    "--java",
                    "--quiet",
                ]
            )
        self.assertEqual(0, code)
        payload = json.loads(out_json.read_text())
        self.assertEqual("drive-linear", payload["mechanisms"][0]["mechanism"])
        self.assertEqual("zoh", payload["mechanisms"][0]["recommended_source"])
        self.assertAlmostEqual(
            2.69, payload["mechanisms"][0]["recommended"]["kV"], delta=0.1
        )
        markdown = out_md.read_text()
        self.assertIn("| Mechanism | Fit |", markdown)
        self.assertIn("`drive-linear`", markdown)
        self.assertIn("kDriveLinearKV", markdown)
        csv_files = sorted(csv_dir.glob("*.csv"))
        self.assertEqual(2, len(csv_files))
        header = csv_files[0].read_text().splitlines()[0]
        self.assertEqual("time_s,voltage_V,velocity,position,acceleration", header)
        self.assertIn("kDriveLinearKV", stdout.getvalue())

    def test_info_lists_mechanisms_and_tests(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            code = sysid.main(["info", str(self.log_path)])
        self.assertEqual(0, code)
        output = stdout.getvalue()
        self.assertIn("Mechanism: drive-linear", output)
        self.assertIn("quasistatic-forward", output)
        self.assertIn("MetersPerSecond", output)

    def test_log_without_sysid_data_exits_nonzero_and_explains(self):
        writer = wpilog.WpiLogWriter()
        entry = writer.start_entry("Robot/BatteryVoltage", "double", "Volts")
        writer.append_double(entry, 1000, 12.4)
        path = writer.write(self.tmp / "nosysid.wpilog")
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            code = sysid.main(["info", str(path)])
        self.assertEqual(1, code)
        self.assertIn("No WPILib SysId entries found", stdout.getvalue())

    def test_unknown_mechanism_exits_nonzero(self):
        stderr = io.StringIO()
        stdout = io.StringIO()
        with contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(stdout):
            code = sysid.main(
                ["fit", str(self.log_path), "--mechanism", "nope", "--quiet"]
            )
        self.assertEqual(1, code)
        self.assertIn("no SysId data for nope", stderr.getvalue())

    def test_fail_on_warning_exit_code(self):
        path = build_sysid_log(
            self.tmp / "warn.wpilog",
            tests=("quasistatic-forward", "quasistatic-reverse"),
        )
        with contextlib.redirect_stdout(io.StringIO()):
            code = sysid.main(["fit", str(path), "--fail-on-warning", "--quiet"])
        self.assertEqual(3, code)

    def test_demo_command_writes_a_log_that_fits(self):
        output = self.tmp / "demo.wpilog"
        with contextlib.redirect_stdout(io.StringIO()):
            code = sysid.main(["demo", str(output)])
        self.assertEqual(0, code)
        self.assertTrue(output.exists())
        mechanisms = sysid.extract_mechanisms(wpilog.read_wpilog(output))
        self.assertEqual({"drive-linear", "shooter-flywheels"}, set(mechanisms))
        drive = sysid.analyze_mechanism("drive-linear", mechanisms["drive-linear"])
        self.assertAlmostEqual(2.69, drive.recommended["kV"], delta=0.08)

    def test_missing_file_exits_cleanly(self):
        with self.assertRaises(SystemExit) as context:
            sysid.main(["info", str(self.tmp / "does-not-exist.wpilog")])
        self.assertIn("no such file", str(context.exception))


class RealLogValidationTest(unittest.TestCase):
    """Optional smoke test against a real log on this machine.

    Point ``SYSID_WPILOG_FIXTURE`` at any ``FRC_*.wpilog`` (robot, sim, or a
    fresh download) to confirm the reader handles the file WPILib actually
    wrote - including all the NT/SmartDashboard entries around the SysId data.
    """

    @unittest.skipUnless(
        os.environ.get("SYSID_WPILOG_FIXTURE"),
        "set SYSID_WPILOG_FIXTURE=/path/to/FRC_....wpilog to run",
    )
    def test_reads_a_real_wpilib_log(self):
        log = wpilog.read_wpilog(os.environ["SYSID_WPILOG_FIXTURE"])
        self.assertGreater(len(log.entries), 0)
        self.assertTrue(any(log.values(entry) for entry in log))
        # And the tool must survive whatever it finds (or does not find) there.
        sysid.extract_mechanisms(log)


if __name__ == "__main__":
    unittest.main()
