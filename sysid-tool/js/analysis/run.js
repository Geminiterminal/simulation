/**
 * Analysis orchestration, mirroring WPILib SysId's AnalysisManager + Analyzer
 * state flow: prepare data → feedforward gains → feedback gains → plot data.
 */

import {
  accelFilter,
  copyPoints,
  convertToPrepared,
  initialTrimAndFilter,
  InvalidDataError,
  NoDynamicDataError,
  NoQuasistaticDataError,
} from "./prepare.js";
import { getMeanTimeDelta } from "./filters.js";
import { ANALYSIS_TYPES, calculateFeedforwardGains } from "./feedforward.js";
import {
  calculatePositionFeedbackGains,
  calculateVelocityFeedbackGains,
} from "./feedback.js";
import { buildPlotData } from "./sim.js";

export { InvalidDataError, NoDynamicDataError, NoQuasistaticDataError };

/**
 * @param {Record<string, {voltage: [], position: [], velocity: []}} testRuns
 * @param {{velocityThreshold: number|null, medianWindow: number, stepTestDuration: number}} settings
 */
export function prepareData(testRuns, settings) {
  const datasets = {};
  for (const [state, run] of Object.entries(testRuns)) {
    const prepared = convertToPrepared(run);
    if (prepared.length < 2) {
      throw new InvalidDataError(
        `${state} data has ${prepared.length} samples and at least 2 are required.`
      );
    }
    datasets[state] = {
      original: copyPoints(prepared),
      raw: copyPoints(prepared),
      filtered: copyPoints(prepared),
    };
  }

  const required = [
    "quasistatic-forward",
    "quasistatic-reverse",
    "dynamic-forward",
    "dynamic-reverse",
  ];
  const missing = required.filter((r) => !datasets[r]);
  if (missing.length > 0) {
    throw new InvalidDataError(`The following tests were not detected: ${missing.join(", ")}.`);
  }

  const trim = initialTrimAndFilter(datasets, settings);
  accelFilter(datasets);

  const collect = (key) => ({
    slowForward: datasets["quasistatic-forward"][key],
    slowBackward: datasets["quasistatic-reverse"][key],
    fastForward: datasets["dynamic-forward"][key],
    fastBackward: datasets["dynamic-reverse"][key],
  });

  const rawDataset = collect("raw");
  const filteredDataset = collect("filtered");

  const startTimes = [
    rawDataset.slowForward[0]?.timestamp ?? 0,
    rawDataset.slowBackward[0]?.timestamp ?? 0,
    rawDataset.fastForward[0]?.timestamp ?? 0,
    rawDataset.fastBackward[0]?.timestamp ?? 0,
  ];

  return {
    rawDataset,
    filteredDataset,
    originalDataset: collect("original"),
    startTimes,
    ...trim,
    meanDt: getMeanTimeDelta(filteredDataset),
  };
}

/**
 * Runs the full analysis pipeline.
 *
 * @param {Record<string, {voltage: [], position: [], velocity: []}} testRuns
 * @param {object} settings {velocityThreshold, medianWindow, stepTestDuration, type, preset, lqr, loopType, direction}
 * @param {{useMeasuredDelay?: boolean}} [options] When true (default) the
 *   feedback preset's measurement delay is overwritten with the measured
 *   delay, mirroring Analyzer::UpdateFeedforwardGains.
 * @returns full analysis result
 */
export function runAnalysis(testRuns, settings, options = {}) {
  const useMeasuredDelay = options.useMeasuredDelay !== false;
  // Step 1: prepare (may auto-update velocityThreshold / stepTestDuration in settings).
  const prepared = prepareData(testRuns, settings);

  // Step 2: feedforward gains from the filtered datasets.
  const type = ANALYSIS_TYPES[settings.type] ?? ANALYSIS_TYPES.Simple;
  const ff = calculateFeedforwardGains(prepared.filteredDataset, type);

  // Step 3: measurement delays → feedback preset delay (mirrors
  // Analyzer::UpdateFeedforwardGains: delay is overwritten with the measured
  // delay for the active loop type).
  const avg = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0);
  const positionDelayMs = Math.max(0, avg(prepared.positionDelays) * 1000);
  const velocityDelayMs = Math.max(0, avg(prepared.velocityDelays) * 1000);
  const preset = { ...settings.preset };
  if (useMeasuredDelay) {
    preset.delayMs =
      settings.loopType === "position" ? positionDelayMs : velocityDelayMs;
  }

  // Step 4: feedback gains.
  const fb =
    settings.loopType === "position"
      ? calculatePositionFeedbackGains(preset, settings.lqr, ff.gains.Kv, ff.gains.Ka)
      : calculateVelocityFeedbackGains(preset, settings.lqr, ff.gains.Kv, ff.gains.Ka);

  // Step 5: diagnostic plot data.
  const plots = buildPlotData(
    prepared.rawDataset,
    prepared.filteredDataset,
    ff.gains,
    prepared.startTimes,
    type,
    settings.direction
  );

  return {
    prepared,
    ff,
    fb,
    preset,
    plots,
    positionDelayMs,
    velocityDelayMs,
    timescaleMs: (ff.gains.Ka / ff.gains.Kv) * 1000,
  };
}
