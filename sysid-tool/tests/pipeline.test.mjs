import test from "node:test";
import assert from "node:assert/strict";
import { SimpleMotorSim, buildPlotData } from "../js/analysis/sim.js";
import { calculateFeedforwardGains, ANALYSIS_TYPES } from "../js/analysis/feedforward.js";
import { prepareData, runAnalysis } from "../js/analysis/run.js";
import { generateDemoLog } from "../js/log/demo.js";
import { extractSysIdData, buildTestRuns, parseCsv, parseJson } from "../js/log/parse.js";

const TRUE = { Ks: 0.1, Kv: 2.25, Ka: 0.3 };

/** Generates clean synthetic four-test data from the known plant. */
function syntheticRuns({ noise = 0 } = {}) {
  const rand = (() => {
    let s = 42;
    return () => {
      s = (s * 1664525 + 1013904223) >>> 0;
      return s / 2 ** 32;
    };
  })();
  const g = () => (rand() - 0.5) * noise;

  const makeTest = (kind, volts, samples, startT) => {
    const sim = new SimpleMotorSim(TRUE.Ks, TRUE.Kv, TRUE.Ka);
    const voltage = [];
    const position = [];
    const velocity = [];
    let t = startT;
    let prevT = null;
    let prevCmd = 0;
    for (let i = 0; i < samples; i++) {
      const commanded =
        kind === "ramp" ? Math.sign(volts) * Math.min(1.0 * (t - startT), Math.abs(volts)) : volts;
      if (prevT !== null) sim.update(prevCmd, t - prevT); // ZOH of previous sample
      voltage.push({ t, v: commanded + g() });
      position.push({ t, v: sim.getPosition() + g() * 0.01 });
      velocity.push({ t, v: sim.getVelocity() + g() });
      prevT = t;
      prevCmd = commanded;
      t += 0.02;
    }
    return { voltage, position, velocity };
  };

  return {
    "quasistatic-forward": makeTest("ramp", 4, 200, 10),
    "quasistatic-reverse": makeTest("ramp", -4, 200, 20),
    "dynamic-forward": makeTest("step", 7, 100, 30),
    "dynamic-reverse": makeTest("step", -7, 100, 40),
  };
}

function defaultSettings(overrides = {}) {
  return {
    velocityThreshold: 0,
    medianWindow: 1,
    stepTestDuration: 0.5,
    type: "Simple",
    preset: {
      outputConversionFactor: 1 / 12,
      velocityTimeFactor: 60,
      periodMs: 1,
      normalized: false,
      delayMs: 112,
    },
    lqr: { qp: 0.1, qv: 1.3, r: 7 },
    loopType: "velocity",
    direction: 0,
    pointSize: 1.25,
    ...overrides,
  };
}

test("feedforward gains recover the true plant on clean data", () => {
  const settings = defaultSettings();
  const prepared = prepareData(syntheticRuns(), settings);
  const ff = calculateFeedforwardGains(prepared.filteredDataset, ANALYSIS_TYPES.Simple);
  assert.ok(Math.abs(ff.gains.Ks - TRUE.Ks) < 0.05, `Ks=${ff.gains.Ks}`);
  assert.ok(Math.abs(ff.gains.Kv - TRUE.Kv) < 0.06, `Kv=${ff.gains.Kv}`);
  assert.ok(Math.abs(ff.gains.Ka - TRUE.Ka) < 0.05, `Ka=${ff.gains.Ka}`);
  assert.ok(ff.rSquared > 0.85, `rSquared=${ff.rSquared}`);
});

test("full pipeline: sim statistics and delays are sane", () => {
  const settings = defaultSettings();
  const result = runAnalysis(syntheticRuns(), settings);
  assert.ok(result.plots.simRSquared > 0.95, `simR2=${result.plots.simRSquared}`);
  assert.ok(result.plots.simRMSE < 0.15, `simRMSE=${result.plots.simRMSE}`);
  assert.ok(result.positionDelayMs >= 0);
  assert.ok(result.velocityDelayMs >= 0);
  assert.ok(Math.abs(result.timescaleMs - (TRUE.Ka / TRUE.Kv) * 1000) < 20);
  assert.ok(result.fb.Kp > 0 && Number.isFinite(result.fb.Kp));
});

test("buildPlotData produces four diagnostic plot datasets", () => {
  const settings = defaultSettings();
  const prepared = prepareData(syntheticRuns(), settings);
  const ff = calculateFeedforwardGains(prepared.filteredDataset, ANALYSIS_TYPES.Simple);
  const plots = buildPlotData(
    prepared.rawDataset,
    prepared.filteredDataset,
    ff.gains,
    prepared.startTimes,
    ANALYSIS_TYPES.Simple,
    0
  );
  assert.ok(plots.quasistatic.raw.length > 10);
  assert.ok(plots.quasistatic.filtered.length > 10);
  assert.ok(plots.quasistatic.sim.length > 10);
  assert.ok(plots.dynamic.filtered.length > 10);
  assert.ok(plots.regression.points.length > 10);
  assert.equal(plots.regression.fit.length, 2);
  assert.ok(plots.timesteps.points.length > 10);
  // Fit line slope is −Kv/Ka
  const [p0, p1] = plots.regression.fit;
  const slope = (p1[1] - p0[1]) / (p1[0] - p0[0]);
  assert.ok(Math.abs(slope - -ff.gains.Kv / ff.gains.Ka) < 1e-9);
});

test("demo log end-to-end recovers ground-truth gains", () => {
  const log = generateDemoLog();
  const sysid = extractSysIdData(log);
  assert.equal(sysid.groups.length, 4);
  assert.equal(sysid.runs.length, 4);

  const runs = buildTestRuns(sysid, "average", {});
  const settings = defaultSettings({ velocityThreshold: null, stepTestDuration: 0 });
  const result = runAnalysis(runs, settings);
  assert.ok(Math.abs(result.ff.gains.Ks - TRUE.Ks) < 0.08, `Ks=${result.ff.gains.Ks}`);
  assert.ok(Math.abs(result.ff.gains.Kv - TRUE.Kv) < 0.12, `Kv=${result.ff.gains.Kv}`);
  assert.ok(Math.abs(result.ff.gains.Ka - TRUE.Ka) < 0.08, `Ka=${result.ff.gains.Ka}`);
});

test("CSV parsing round-trips record data", () => {
  const csv = [
    "timestamp,voltage,position,velocity,state",
    "0.00,0,0,0,quasistatic-forward",
    "0.02,0.5,0.01,0.5,quasistatic-forward",
    "0.04,1.0,0.03,0.9,quasistatic-forward",
    "0.06,7.0,0.05,1.9,dynamic-forward",
    "0.08,7.0,0.09,2.7,dynamic-forward",
  ].join("\n");
  const log = parseCsv(csv, "test");
  assert.equal(log.entries.length, 4);
  const sysid = extractSysIdData(log);
  assert.equal(sysid.groups.length, 1);
  assert.equal(sysid.runs.length, 2);
});

test("JSON parsing supports record arrays", () => {
  const records = [
    { time: 0, voltage: 0, position: 0, velocity: 0, state: "dynamic-forward" },
    { time: 0.02, voltage: 7, position: 0.02, velocity: 2, state: "dynamic-forward" },
    { time: 0.04, voltage: 7, position: 0.06, velocity: 2.8, state: "dynamic-forward" },
    { time: 0.06, voltage: -7, position: 0.1, velocity: -1, state: "dynamic-reverse" },
    { time: 0.08, voltage: -7, position: 0.08, velocity: -2, state: "dynamic-reverse" },
  ];
  const log = parseJson(JSON.stringify(records), "test");
  const sysid = extractSysIdData(log);
  assert.equal(sysid.runs.length, 2);
  assert.equal(sysid.runs[0].state, "dynamic-forward");
  assert.equal(sysid.runs[1].state, "dynamic-reverse");
});
