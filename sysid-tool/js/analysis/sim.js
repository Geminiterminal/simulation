/**
 * Time-domain simulation and diagnostic-plot data generation, mirroring
 * WPILib SysId's SimpleMotorSim and AnalyzerPlot::SetData.
 */

import { getMeanTimeDelta, sgn } from "./filters.js";

/**
 * Simulation of a simple motor mechanism driven by feedforward gains:
 *
 *   dx/dt = [0 1; 0 −Kv/Ka] x + [0; 1/Ka] u + [0; −Ks/Ka] sgn(v)
 */
export class SimpleMotorSim {
  constructor(Ks, Kv, Ka, initialPosition = 0, initialVelocity = 0) {
    this.Ks = Ks;
    this.Kv = Kv;
    this.Ka = Ka;
    this.reset(initialPosition, initialVelocity);
  }

  reset(position = 0, velocity = 0) {
    this.x = position;
    this.v = velocity;
  }

  getVelocity() {
    return this.v;
  }

  getPosition() {
    return this.x;
  }

  /** Simulates the affine system forward dt seconds with a constant voltage. */
  update(voltage, dt) {
    const alpha = this.Kv / this.Ka; // −A(1,1)
    const b = 1 / this.Ka;
    const effective = voltage - this.Ks * sgn(this.v); // u + z, z = −Ks·sgn(v)

    let phi; // ∫ e^{−ατ} dτ terms
    let e;
    if (Math.abs(alpha) < 1e-12) {
      e = 1;
      phi = dt;
    } else {
      e = Math.exp(-alpha * dt);
      phi = (1 - e) / alpha;
    }

    // x_{k+1} = Ad x_k + Bd (u + z)
    const vNext = e * this.v + b * phi * effective;
    const xNext =
      this.x +
      (Math.abs(alpha) < 1e-12
        ? this.v * dt + (b * effective * dt * dt) / 2
        : this.v * phi + b * effective * (dt - phi));
    this.x = xNext;
    this.v = vNext;
  }
}

/**
 * Runs the simulated model over one dataset, accumulating velocity
 * squared-error statistics (mirrors PopulateTimeDomainSim).
 *
 * @returns {{series: number[][], simSquaredErrorSum: number, squaredVariationSum: number, timeSeriesPoints: number}}
 */
export function populateTimeDomainSim(data, startTimes, model, stats) {
  const series = [];
  if (data.length === 0) return series;

  const startTime = data[0].timestamp;
  series.push([startTime, data[0].velocity]);
  model.reset(data[0].position, data[0].velocity);
  let t = 0;

  for (let i = 1; i < data.length; i++) {
    const now = data[i];
    const pre = data[i - 1];
    t += now.timestamp - pre.timestamp;

    if (startTimes.includes(now.timestamp)) {
      // Start of a new test: reset the model.
      model.reset(now.position, now.velocity);
      continue;
    }

    model.update(pre.voltage, now.timestamp - pre.timestamp);
    series.push([startTime + t, model.getVelocity()]);
    stats.simSquaredErrorSum += (now.velocity - model.getVelocity()) ** 2;
    stats.squaredVariationSum += now.velocity ** 2;
    stats.timeSeriesPoints++;
  }
  return series;
}

/**
 * Builds all diagnostic plot series + sim statistics from the prepared
 * datasets, replicating AnalyzerPlot::SetData.
 *
 * @param {{slowForward, slowBackward, fastForward, fastBackward}} rawData
 * @param {{slowForward, slowBackward, fastForward, fastBackward}} filteredData
 * @param {{Ks, Kv, Ka, Kg, offset}} gains
 * @param {number[]} startTimes
 * @param {{name: string}} type
 * @param {number} direction 0 = forward, 1 = reverse/backward
 */
export function buildPlotData(rawData, filteredData, gains, startTimes, type, direction) {
  const { Ks, Kv, Ka, Kg } = gains;

  const concat = (...arrays) => {
    const out = [].concat(...arrays.map((a) => a.map((p) => ({ ...p }))));
    out.sort((a, b) => a.timestamp - b.timestamp);
    return out;
  };

  const slow = concat(filteredData.slowForward, filteredData.slowBackward);
  const fast = concat(filteredData.fastForward, filteredData.fastBackward);
  const rawSlow = concat(rawData.slowForward, rawData.slowBackward);
  const rawFast = concat(rawData.fastForward, rawData.fastBackward);

  const slowDir = direction === 0 ? filteredData.slowForward : filteredData.slowBackward;
  const fastDir = direction === 0 ? filteredData.fastForward : filteredData.fastBackward;
  const rawSlowDir = direction === 0 ? rawData.slowForward : rawData.slowBackward;
  const rawFastDir = direction === 0 ? rawData.fastForward : rawData.fastBackward;

  const toPoints = (arr) => arr.map((p) => [p.timestamp, p.velocity]);

  const timestepPoints = [];
  const collectTimesteps = (arr) => {
    for (let i = 1; i < arr.length; i++) {
      if (arr[i].dt > 0 && !startTimes.includes(arr[i].timestamp)) {
        timestepPoints.push([arr[i].timestamp, arr[i].dt * 1000]);
      }
    }
  };

  const quasistatic = {
    raw: toPoints(rawSlowDir),
    filtered: toPoints(slowDir),
    sim: [],
  };
  const dynamic = {
    raw: toPoints(rawFastDir),
    filtered: toPoints(fastDir),
    sim: [],
  };

  collectTimesteps(slowDir);
  collectTimesteps(fastDir);

  // Simulated time-domain data over both directions (stats use all data).
  const stats = { simSquaredErrorSum: 0, squaredVariationSum: 0, timeSeriesPoints: 0 };
  if (type.name !== "Arm" && type.name !== "Elevator") {
    quasistatic.sim = populateTimeDomainSim(
      rawSlow,
      startTimes,
      new SimpleMotorSim(Ks, Kv, Ka),
      stats
    );
    dynamic.sim = populateTimeDomainSim(
      rawFast,
      startTimes,
      new SimpleMotorSim(Ks, Kv, Ka),
      stats
    );
  } else {
    // Elevator/arm share the simple-motor backbone for the velocity plots;
    // gravity terms are folded into the regression plots below.
    quasistatic.sim = populateTimeDomainSim(
      rawSlow,
      startTimes,
      new SimpleMotorSim(Ks, Kv, Ka),
      stats
    );
    dynamic.sim = populateTimeDomainSim(
      rawFast,
      startTimes,
      new SimpleMotorSim(Ks, Kv, Ka),
      stats
    );
  }

  // Acceleration vs. velocity regression points and fit line.
  const velocities = [...slow, ...fast].map((p) => p.velocity);
  const minVel = Math.min(...velocities);
  const maxVel = Math.max(...velocities);
  const fitLine = [
    [minVel, (-Kv / Ka) * minVel],
    [maxVel, (-Kv / Ka) * maxVel],
  ];
  const regressionPoints = [];
  for (const pt of [...slow, ...fast]) {
    let accelPortion =
      pt.acceleration - (1 / Ka) * pt.voltage + sgn(pt.velocity) * (Ks / Ka);
    if (type.name === "Elevator") accelPortion -= Kg / Ka;
    else if (type.name === "Arm") accelPortion -= (Kg / Ka) * pt.cos;
    regressionPoints.push([pt.velocity, accelPortion]);
  }

  // Timestep fit line at the mean dt.
  const meanDt = getMeanTimeDelta([...slow, ...fast]);
  const minTime = Math.min(slow[0].timestamp, fast[0].timestamp);
  const maxTime = Math.max(
    slow[slow.length - 1].timestamp,
    fast[fast.length - 1].timestamp
  );
  const timestepFit = [
    [minTime, meanDt * 1000],
    [maxTime, meanDt * 1000],
  ];

  // Sim velocity statistics (formula mirrors AnalyzerPlot::SetData).
  const simRMSE = Math.sqrt(stats.simSquaredErrorSum / Math.max(1, stats.timeSeriesPoints));
  const simRSquared =
    1 - simRMSE / Math.sqrt(stats.squaredVariationSum / Math.max(1, stats.timeSeriesPoints));

  return {
    quasistatic,
    dynamic,
    regression: { points: regressionPoints, fit: fitLine },
    timesteps: { points: timestepPoints, fit: timestepFit },
    simRMSE,
    simRSquared,
    meanDtMs: meanDt * 1000,
  };
}
