import test from "node:test";
import assert from "node:assert/strict";
import {
  PRESETS,
  calculatePositionFeedbackGains,
  calculateVelocityFeedbackGains,
  presetsEqual,
} from "../js/analysis/feedback.js";

test("REV Brushless Encoder Port preset maps to the SysId UI fields", () => {
  const p = PRESETS["REV Brushless Encoder Port"];
  // "Max Controller Output" shows outputConversionFactor * 12
  assert.ok(Math.abs(p.outputConversionFactor * 12 - 1.0) < 1e-12);
  assert.equal(p.velocityTimeFactor, 60.0); // "Velocity Denominator Units (s)"
  assert.equal(p.periodMs, 1); // "Controller Period (ms)"
  assert.equal(p.normalized, false); // "Time-Normalized?" unchecked
  assert.equal(p.delayMs, 112); // "Measurement Delay (ms)"
});

test("CTRE Phoenix 5 preset values", () => {
  const p = PRESETS["CTRE Phoenix 5"];
  assert.ok(Math.abs(p.outputConversionFactor * 12 - 1023) < 1e-9);
  assert.equal(p.velocityTimeFactor, 0.1);
  assert.equal(p.delayMs, 81.5);
});

test("velocity feedback gains are finite and positive for a healthy plant", () => {
  const preset = PRESETS["REV Brushless Encoder Port"];
  const lqr = { qp: 0.1, qv: 1.3, r: 7 };
  const { Kp, Kd } = calculateVelocityFeedbackGains(preset, lqr, 2.25, 0.3);
  assert.ok(Number.isFinite(Kp) && Kp > 0, `Kp=${Kp}`);
  assert.equal(Kd, 0);
});

test("velocity Kp scales with the output conversion and velocity time factor", () => {
  const lqr = { qp: 0.1, qv: 1.3, r: 7 };
  const base = { outputConversionFactor: 1, velocityTimeFactor: 1, periodMs: 1, normalized: true, delayMs: 10 };
  const scaled = { ...base, outputConversionFactor: 1 / 12, velocityTimeFactor: 60 };
  const k1 = calculateVelocityFeedbackGains(base, lqr, 2.25, 0.3);
  const k2 = calculateVelocityFeedbackGains(scaled, lqr, 2.25, 0.3);
  // Identical plants and latency; only the unit conversions differ (×720).
  assert.ok(Math.abs(k1.Kp / k2.Kp - 720) < 1e-6, `${k1.Kp / k2.Kp}`);
});

test("degenerate Ka yields zero gains", () => {
  const preset = PRESETS.Default;
  const lqr = { qp: 0.1, qv: 1.3, r: 7 };
  assert.deepEqual(calculateVelocityFeedbackGains(preset, lqr, 2.25, 0), { Kp: 0, Kd: 0 });
});

test("position feedback gains produce both Kp and Kd", () => {
  const preset = PRESETS.WPILib;
  const lqr = { qp: 0.1, qv: 1.3, r: 7 };
  const { Kp, Kd } = calculatePositionFeedbackGains(preset, lqr, 2.25, 0.3);
  assert.ok(Number.isFinite(Kp) && Kp > 0, `Kp=${Kp}`);
  assert.ok(Number.isFinite(Kd) && Kd > 0, `Kd=${Kd}`);
});

test("presetsEqual detects modifications", () => {
  const a = { ...PRESETS.Default };
  assert.ok(presetsEqual(a, PRESETS.Default));
  a.delayMs = 5;
  assert.ok(!presetsEqual(a, PRESETS.Default));
});
