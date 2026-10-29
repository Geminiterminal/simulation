/**
 * Data preparation pipeline, mirroring WPILib SysId's AnalysisManager +
 * FilteringUtils (ConvertToPrepared, InitialTrimAndFilter, TrimStepVoltageData,
 * AccelFilter).
 *
 * A "dataset" holds PreparedData points:
 *   { timestamp, voltage, position, velocity, dt, acceleration }
 */

import {
  applyMedianFilter,
  getMaxSpeed,
  getMeanTimeDelta,
  getNoiseFloor,
  NOISE_MEAN_WINDOW,
  prepareAcceleration,
  sgn,
} from "./filters.js";

export class InvalidDataError extends Error {}
export class NoQuasistaticDataError extends Error {
  constructor() {
    super(
      "Quasistatic test trimming removed all data. Please adjust your velocity threshold and double check your units and test data to make sure that the robot is reporting reasonable values."
    );
  }
}
export class NoDynamicDataError extends Error {
  constructor() {
    super(
      "Dynamic test trimming removed all data. Please adjust your test duration and double check your units and test data to make sure that the robot is reporting reasonable values."
    );
  }
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

/** Linearly interpolates a {t, v} series at the given time. */
function lerpAt(series, time) {
  let next = series.findIndex((e) => e.t > time);
  if (next === -1) next = series.length - 1;
  if (next === 0) next = 1;
  const prev = next - 1;
  const a = series[prev];
  const b = series[Math.min(next, series.length - 1)];
  const span = b.t - a.t;
  return span === 0 ? a.v : lerp(a.v, b.v, (time - a.t) / span);
}

/**
 * Converts one contiguous run of raw voltage/position/velocity samples into
 * PreparedData, interpolating position and velocity onto the voltage
 * timestamps (equivalent to SysId's ConvertToPrepared).
 */
export function convertToPrepared(run) {
  const prepared = [];
  const { voltage, position, velocity } = run;
  for (let i = 0; i < voltage.length - 1; i++) {
    prepared.push({
      timestamp: voltage[i].t,
      voltage: voltage[i].v,
      position: lerpAt(position, voltage[i].t),
      velocity: lerpAt(velocity, voltage[i].t),
      dt: voltage[i + 1].t - voltage[i].t,
      acceleration: 0,
    });
  }
  return prepared;
}

/**
 * Trims step (dynamic) voltage data to discard points before the maximum
 * acceleration and after the (user-specified or default) test duration.
 * Mirrors SysId's TrimStepVoltageData.
 *
 * @returns {{minStepTime, positionDelay, velocityDelay}}
 */
export function trimStepVoltageData(data, settings, minStepTime, maxStepTime) {
  const voltageBegins = data.find((d) => Math.abs(d.voltage) > 0);
  if (!voltageBegins) {
    throw new NoDynamicDataError();
  }
  const firstTimestamp = voltageBegins.timestamp;
  const firstPosition = voltageBegins.position;

  const motionBegins = data.find(
    (d) => Math.abs(d.position - firstPosition) > settings.velocityThreshold * d.dt
  );
  const positionDelay = motionBegins ? motionBegins.timestamp - firstTimestamp : 0;

  let maxAccel = data[0];
  for (const d of data) {
    if (sgn(d.velocity) * d.acceleration > sgn(maxAccel.velocity) * maxAccel.acceleration) {
      maxAccel = d;
    }
  }

  // Current limiting can delay onset of peak acceleration: find the first
  // acceleration near (>= 80% of) the max.
  const accelBegins = data.find(
    (d) =>
      sgn(d.velocity) * d.acceleration >
      0.8 * sgn(maxAccel.velocity) * maxAccel.acceleration
  );

  let velocityDelay = 0;
  if (accelBegins) {
    velocityDelay = accelBegins.timestamp - firstTimestamp;
    // Trim data before max acceleration
    const idx = data.indexOf(maxAccel);
    data.splice(0, idx);
  }

  minStepTime = Math.min(data[0].timestamp - firstTimestamp, minStepTime);

  // If step test duration not yet specified, calculate default: time from the
  // (new) start until 90% of the maximum speed is exceeded.
  if (settings.stepTestDuration === 0) {
    const maxSpeed = getMaxSpeed(data, (pt) => pt.velocity);
    const endIt = data.find((d) => Math.abs(d.velocity) > maxSpeed * 0.9);
    if (endIt) {
      settings.stepTestDuration = Math.min(
        endIt.timestamp - data[0].timestamp + minStepTime,
        maxStepTime
      );
    }
  }

  const maxIt = data.find(
    (d) => d.timestamp - data[0].timestamp > settings.stepTestDuration
  );
  if (maxIt) {
    data.splice(data.indexOf(maxIt), data.length - data.indexOf(maxIt));
  }

  return { minStepTime, positionDelay, velocityDelay };
}

/**
 * Trims quasistatic data, applies the median filter to the velocity data of
 * the filtered copies, computes acceleration, and trims the dynamic tests.
 *
 * @param {Record<string, {original: [], raw: [], filtered: []}>} datasets
 * @param {{velocityThreshold: number|null, medianWindow: number, stepTestDuration: number}} settings
 */
export function initialTrimAndFilter(datasets, settings) {
  const positionDelays = [];
  const velocityDelays = [];
  let minStepTime = 0;
  let maxStepTime = 0;

  // Maximum duration across the dynamic tests.
  for (const [name, ds] of Object.entries(datasets)) {
    if (name.includes("dynamic") && ds.raw.length > 0) {
      const duration = ds.raw[ds.raw.length - 1].timestamp - ds.raw[0].timestamp;
      if (duration > maxStepTime) maxStepTime = duration;
    }
  }

  // Auto velocity threshold: smallest quasistatic velocity noise floor.
  if (settings.velocityThreshold == null) {
    let threshold = Infinity;
    for (const [name, ds] of Object.entries(datasets)) {
      if (name.includes("quasistatic") && ds.filtered.length > 0) {
        threshold = Math.min(
          threshold,
          getNoiseFloor(ds.filtered, NOISE_MEAN_WINDOW, (pt) => pt.velocity)
        );
      }
    }
    settings.velocityThreshold = Number.isFinite(threshold) ? threshold : 0;
  }

  for (const [name, ds] of Object.entries(datasets)) {
    const copies = [ds.raw, ds.filtered];

    // Trim quasistatic data: drop zero-voltage and sub-threshold velocity points.
    if (name.includes("quasistatic")) {
      for (const copy of copies) {
        const kept = copy.filter(
          (pt) =>
            !(Math.abs(pt.voltage) <= 0 || Math.abs(pt.velocity) < settings.velocityThreshold)
        );
        copy.length = 0;
        copy.push(...kept);
        if (copy.length === 0) throw new NoQuasistaticDataError();
      }
    }

    // Median-filter the filtered copy.
    if (settings.medianWindow > 1 && ds.filtered.length >= settings.medianWindow) {
      applyMedianFilter(ds.filtered, settings.medianWindow);
    }

    for (const copy of copies) prepareAcceleration(copy);

    // Trim filtered dynamic data and align the raw copy to its start.
    if (name.includes("dynamic")) {
      const result = trimStepVoltageData(
        ds.filtered,
        settings,
        minStepTime,
        maxStepTime
      );
      minStepTime = result.minStepTime;
      positionDelays.push(result.positionDelay);
      velocityDelays.push(result.velocityDelay);

      const startTime = ds.filtered[0].timestamp;
      const rawStart = ds.raw.findIndex((pt) => pt.timestamp === startTime);
      if (rawStart > 0) ds.raw.splice(0, rawStart);
      if (ds.raw.length === 0) throw new NoDynamicDataError();
    }
  }

  return { positionDelays, velocityDelays, minStepTime, maxStepTime };
}

/** Removes all points with acceleration === 0 (SysId's AccelFilter). */
export function accelFilter(datasets) {
  for (const ds of Object.values(datasets)) {
    for (const copy of [ds.raw, ds.filtered]) {
      const kept = copy.filter((pt) => pt.acceleration !== 0);
      copy.length = 0;
      copy.push(...kept);
      if (copy.length === 0) {
        throw new InvalidDataError("Acceleration filtering has removed all data.");
      }
    }
  }
}

/** Deep-copies a PreparedData array. */
export function copyPoints(points) {
  return points.map((p) => ({ ...p }));
}

export { getMeanTimeDelta };
