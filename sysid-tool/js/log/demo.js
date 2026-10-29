/**
 * Demo dataset generator: synthesizes a "Drivetrain linear" SysId log with the
 * same entry layout this repo's DriveSubsystem produces
 * (sysid-test-state-drive-linear plus per-module voltage/position/velocity
 * channels), using a known ground-truth plant so the recovered gains can be
 * sanity-checked (Ks = 0.10 V, Kv = 2.25 V·s/m, Ka = 0.30 V·s²/m).
 */

import { SimpleMotorSim } from "../analysis/sim.js";

const TRUE_KS = 0.10;
const TRUE_KV = 2.25;
const TRUE_KA = 0.30;

const MODULES = ["drive-front-left", "drive-front-right", "drive-rear-left", "drive-rear-right"];

// Deterministic PRNG so the demo is reproducible.
function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Generates the demo log model.
 *
 * @returns {{name: string, numRecords: number, numEntries: number, entries: Array}}
 */
export function generateDemoLog() {
  const rand = mulberry32(0x53594449);
  const gaussian = () => {
    const u = Math.max(rand(), 1e-12);
    const v = rand();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };

  // Four tests: quasistatic ramps at 1 V/s (0→4 V) and dynamic 7 V steps,
  // matching SysIdRoutine defaults (ramp 1 V/s, step 7 V).
  const tests = [
    { state: "quasistatic-forward", startT: 142.2, samples: 208, kind: "ramp", volts: 4 },
    { state: "quasistatic-reverse", startT: 147.6, samples: 211, kind: "ramp", volts: -4 },
    { state: "dynamic-forward", startT: 152.7, samples: 78, kind: "step", volts: 7 },
    { state: "dynamic-reverse", startT: 158.1, samples: 74, kind: "step", volts: -7 },
  ];

  // Sample times: 20 ms loop with jitter, like a real robot loop.
  const sampleTimes = []; // {t, testIndex}
  for (let ti = 0; ti < tests.length; ti++) {
    const test = tests[ti];
    let t = test.startT;
    for (let i = 0; i < test.samples; i++) {
      sampleTimes.push({ t, testIndex: ti });
      t += 0.02 + gaussian() * 0.0022;
    }
  }

  // Simulate the true plant once over the whole timeline. Within a test the
  // recorded voltage is zero-order-held over the following sample interval
  // (matching the analysis model); between tests the mechanism sits at 0 V.
  const sim = new SimpleMotorSim(TRUE_KS, TRUE_KV, TRUE_KA);
  const truth = []; // {t, voltage, position, velocity}
  let prevT = null;
  let prevCmd = 0;
  let prevTestIndex = -1;
  for (const { t, testIndex } of sampleTimes) {
    const test = tests[testIndex];
    const elapsed = t - test.startT;
    const commanded =
      test.kind === "ramp" ? Math.sign(test.volts) * Math.min(1.0 * elapsed, Math.abs(test.volts)) : test.volts;
    if (prevT !== null) {
      if (testIndex === prevTestIndex) {
        sim.update(prevCmd, t - prevT); // ZOH of the previous sample's voltage
      } else {
        sim.update(0, t - prevT); // idle gap between tests
      }
    }
    truth.push({ t, voltage: commanded, position: sim.getPosition(), velocity: sim.getVelocity() });
    prevT = t;
    prevCmd = commanded;
    prevTestIndex = testIndex;
  }

  // Per-module measured channels with encoder noise/quantization.
  const entries = [];
  for (const module of MODULES) {
    const noiseScale = 0.85 + rand() * 0.3;
    const voltage = [];
    const position = [];
    const velocity = [];
    for (const pt of truth) {
      voltage.push({ t: pt.t, v: pt.voltage + gaussian() * 0.004 });
      position.push({
        t: pt.t,
        v: Math.round((pt.position + gaussian() * 0.0009 * noiseScale) * 2000) / 2000,
      });
      velocity.push({ t: pt.t, v: pt.velocity + gaussian() * 0.011 * noiseScale });
    }
    entries.push({ name: `voltage-${module}-drive-linear`, type: "double", values: voltage });
    entries.push({ name: `position-${module}-drive-linear`, type: "double", values: position });
    entries.push({ name: `velocity-${module}-drive-linear`, type: "double", values: velocity });
  }

  // Test state string stream.
  const stateValues = [];
  for (const { t, testIndex } of sampleTimes) {
    stateValues.push({ t, v: tests[testIndex].state });
  }
  entries.push({
    name: "sysid-test-state-drive-linear",
    type: "string",
    values: stateValues,
  });

  // Background telemetry so the Log Loader tree looks like a real capture.
  const fill = (name, type, fn) => {
    const values = [];
    for (let t = 140; t < 162; t += 1) values.push({ t, v: fn(t) });
    entries.push({ name, type, values });
  };
  fill("Battery/Voltage", "double", () => 12.4 + gaussian() * 0.05);
  fill("Battery/Current", "double", (t) => 18 + 12 * Math.sin(t) + gaussian());
  fill("NT/Robot/Mode", "string", (t) => (t < 142 ? "Disabled" : "Autonomous"));
  fill("NT/AdvantageKit/Real Outputs", "boolean", () => true);
  entries.push({
    name: "console",
    type: "string",
    values: [
      { t: 140, v: "Starting SysId routine drive-linear" },
      { t: 142.2, v: "quasistatic-forward" },
      { t: 147.6, v: "quasistatic-reverse" },
      { t: 152.7, v: "dynamic-forward" },
      { t: 158.1, v: "dynamic-reverse" },
      { t: 161, v: "SysId complete" },
    ],
  });
  entries.push({
    name: "messages",
    type: "string",
    values: [
      { t: 140, v: "{\"message\":\"SysId armed\",\"level\":\"info\"}" },
      { t: 161, v: "{\"message\":\"SysId finished\",\"level\":\"info\"}" },
    ],
  });

  const numRecords = entries.reduce((n, e) => n + e.values.length, 0);
  return {
    name: "Drivetrain linear",
    numRecords,
    numEntries: entries.length,
    entries,
    demoTruth: { Ks: TRUE_KS, Kv: TRUE_KV, Ka: TRUE_KA },
  };
}
