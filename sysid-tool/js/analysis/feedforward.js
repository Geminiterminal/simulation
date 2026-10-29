/**
 * Feedforward gain calculation, mirroring WPILib SysId's FeedforwardAnalysis.
 *
 * Regresses acceleration as the dependent variable:
 *
 *   Simple / Drivetrain:      a = α·v + β·V + γ·sgn(v)
 *   Elevator:                 a = α·v + β·V + γ·sgn(v) + δ
 *   Arm:                      a = α·v + β·V + γ·sgn(v) + δ·cos(θ) + ε·sin(θ)
 *
 * Gains: Ks = −γ/β, Kv = −α/β, Ka = 1/β (Kg and offset for elevator/arm).
 */

import { ols } from "./ols.js";
import { sgn } from "./filters.js";

/** System types, matching SysId's AnalysisType. */
export const ANALYSIS_TYPES = {
  Simple: { independentVariables: 3, name: "Simple" },
  Elevator: { independentVariables: 4, name: "Elevator" },
  Arm: { independentVariables: 5, name: "Arm" },
};

function populateOLSData(data, type, X, y, rowOffset) {
  for (let sample = 0; sample < data.length; sample++) {
    const pt = data[sample];
    const row = X[rowOffset + sample];
    row[0] = pt.velocity;
    row[1] = pt.voltage;
    row[2] = sgn(pt.velocity);
    if (type.name === "Elevator") {
      row[3] = 1.0;
    } else if (type.name === "Arm") {
      row[3] = pt.cos;
      row[4] = pt.sin;
    }
    y[rowOffset + sample] = pt.acceleration;
  }
}

/**
 * Calculates feedforward gains from the four datasets.
 *
 * @param {{slowForward: [], slowBackward: [], fastForward: [], fastBackward: []}} storage
 * @param {{independentVariables: number, name: string}} type
 * @returns {{gains: {Ks, Kv, Ka, Kg, offset}, rSquared: number, rmse: number, warnings: string[]}}
 */
export function calculateFeedforwardGains(storage, type) {
  const { slowForward, slowBackward, fastForward, fastBackward } = storage;
  const size =
    slowForward.length + slowBackward.length + fastForward.length + fastBackward.length;
  const p = type.independentVariables;
  if (size < p + 1) {
    throw new Error(
      "Insufficient samples to compute gains. Make sure all four tests are present and the robot moved."
    );
  }

  const X = Array.from({ length: size }, () => new Array(p).fill(0));
  const y = new Array(size).fill(0);

  let rowOffset = 0;
  for (const dataset of [slowForward, slowBackward, fastForward, fastBackward]) {
    populateOLSData(dataset, type, X, y, rowOffset);
    rowOffset += dataset.length;
  }

  const result = ols(X, y);
  const [alpha, beta, gamma] = result.coeffs;

  const warnings = [];
  if (!Number.isFinite(beta) || Math.abs(beta) < 1e-12) {
    throw new Error("Insufficient samples to compute Ka (degenerate OLS fit).");
  }

  const gains = {
    Ks: -gamma / beta,
    Kv: -alpha / beta,
    Ka: 1 / beta,
    Kg: 0,
    offset: 0,
  };

  if (type.name === "Elevator") {
    const delta = result.coeffs[3];
    gains.Kg = -delta / beta;
  } else if (type.name === "Arm") {
    const delta = result.coeffs[3];
    const epsilon = result.coeffs[4];
    gains.Kg = Math.hypot(delta, epsilon) / beta;
    gains.offset = Math.atan2(epsilon, -delta);
  }

  // Sanity checks mirroring AnalysisManager::CalculateFeedforward.
  if (gains.Ks < 0) {
    warnings.push(`Calculated Ks gain of: ${gains.Ks.toFixed(3)} is erroneous! Ks should be >= 0.`);
  }
  if (gains.Kv < 0) {
    warnings.push(`Calculated Kv gain of: ${gains.Kv.toFixed(3)} is erroneous! Kv should be >= 0.`);
  }
  if (gains.Ka <= 0) {
    warnings.push(`Calculated Ka gain of: ${gains.Ka.toFixed(3)} is erroneous! Ka should be > 0.`);
  }
  if ((type.name === "Elevator" || type.name === "Arm") && gains.Kg < 0) {
    warnings.push(`Calculated Kg gain of: ${gains.Kg.toFixed(3)} is erroneous! Kg should be >= 0.`);
  }

  return { gains, rSquared: result.rSquared, rmse: result.rmse, warnings };
}
