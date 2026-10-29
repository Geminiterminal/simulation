/**
 * Feedback gain calculation via LQR, mirroring WPILib SysId's
 * FeedbackAnalysis + FeedbackControllerPreset and wpimath's
 * LinearQuadraticRegulator (DARE + LatencyCompensate).
 *
 * The system is discretized at the controller period, a discrete algebraic
 * Riccati equation is solved for the optimal gain, and the gain is then
 * latency-compensated for the measurement delay.
 */

/** Feedback controller presets (periods and delays in milliseconds). */
export const PRESETS = {
  Default: { outputConversionFactor: 1.0, velocityTimeFactor: 1.0, periodMs: 20, normalized: true, delayMs: 0 },
  WPILib: { outputConversionFactor: 1.0, velocityTimeFactor: 1.0, periodMs: 20, normalized: true, delayMs: 0 },
  "CTRE Phoenix 5": { outputConversionFactor: 1023.0 / 12.0, velocityTimeFactor: 0.1, periodMs: 1, normalized: false, delayMs: 81.5 },
  "CTRE Phoenix 6": { outputConversionFactor: 1.0, velocityTimeFactor: 1.0, periodMs: 1, normalized: true, delayMs: 1 },
  "REV Brushless Encoder Port": { outputConversionFactor: 1.0 / 12.0, velocityTimeFactor: 60.0, periodMs: 1, normalized: false, delayMs: 112 },
  "REV Brushed Encoder Port": { outputConversionFactor: 1.0 / 12.0, velocityTimeFactor: 60.0, periodMs: 1, normalized: false, delayMs: 81.5 },
  "REV Data Port": { outputConversionFactor: 1.0 / 12.0, velocityTimeFactor: 60.0, periodMs: 1, normalized: false, delayMs: 81.5 },
  Venom: { outputConversionFactor: 4096.0 / 12.0, velocityTimeFactor: 60.0, periodMs: 1, normalized: false, delayMs: 5 },
};

export const PRESET_NAMES = Object.keys(PRESETS);

export function clonePreset(preset) {
  return { ...preset };
}

export function presetsEqual(a, b) {
  return (
    a.outputConversionFactor === b.outputConversionFactor &&
    a.velocityTimeFactor === b.velocityTimeFactor &&
    a.periodMs === b.periodMs &&
    a.normalized === b.normalized &&
    a.delayMs === b.delayMs
  );
}

// ---------------------------------------------------------------------------
// Small dense matrix helpers (1×1 and 2×2 are all we need).
// ---------------------------------------------------------------------------

function matExp2x2(A, dt) {
  // A = [[0, 1], [0, a]] for the elevator plant; fall back to series for safety.
  const a = A[1][1];
  const e = Math.exp(a * dt);
  const phi = Math.abs(a) < 1e-12 ? dt : (e - 1) / a;
  return [
    [1, phi],
    [0, e],
  ];
}

/** Discretizes ẋ = Ax + Bu (exact ZOH) for the 2-state elevator plant. */
function discretizeElevator(A, B, dt) {
  const a = A[1][1];
  const b = B[1][0];
  const e = Math.exp(a * dt);
  // phi = (e^{a·dt} − 1)/a  (→ dt as a → 0)
  const phi = Math.abs(a) < 1e-12 ? dt : (e - 1) / a;
  const Ad = [
    [1, phi],
    [0, e],
  ];
  // Bd = ∫₀^dt e^{Aτ}B dτ
  const upper = Math.abs(a) < 1e-12 ? (b * dt * dt) / 2 : (b / a) * (phi - dt);
  const lower = b * phi;
  return { Ad, Bd: [[upper], [lower]] };
}

/** Discretizes the 1-state flywheel plant (exact ZOH). */
function discretizeFlywheel(a, b, dt) {
  const e = Math.exp(a * dt);
  const Ad = [[e]];
  const Bd = [[Math.abs(a) < 1e-12 ? b * dt : (b * (e - 1)) / a]];
  return { Ad, Bd };
}

/** Iteratively solves the DARE for 1- or 2-state systems. */
function dare(Ad, Bd, Q, R) {
  const n = Ad.length;
  let P = Q.map((row) => [...row]);
  for (let iter = 0; iter < 5000; iter++) {
    // S = AdᵀPAd − AdᵀPBd(R + BdᵀPBd)⁻¹BdᵀPAd + Q
    const BdPBd = matMul(matMul(transpose(Bd), P), Bd)[0][0] + R[0][0];
    const BdPA = matMul(matMul(transpose(Bd), P), Ad); // 1×n
    const AdPA = matMul(matMul(transpose(Ad), P), Ad); // n×n
    // S = AdᵀPAd − (AdᵀPBd)(R + BdᵀPBd)⁻¹(BdᵀPAd) + Q
    const adj = AdPA.map((row, i) =>
      row.map(
        (val, j) => val - (BdPA[0][i] * BdPA[0][j]) / BdPBd + (i === j ? Q[i][j] : 0)
      )
    );
    let maxDelta = 0;
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        maxDelta = Math.max(maxDelta, Math.abs(adj[i][j] - P[i][j]));
      }
    }
    P = adj;
    if (maxDelta < 1e-12) break;
  }
  return P;
}

function transpose(M) {
  return M[0].map((_, j) => M.map((row) => row[j]));
}

function matMul(A, B) {
  const n = A.length;
  const m = B[0].length;
  const k = B.length;
  const out = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < m; j++) {
      for (let t = 0; t < k; t++) out[i][j] += A[i][t] * B[t][j];
    }
  }
  return out;
}

function matPow(M, exponent) {
  const n = M.length;
  let result = Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))
  );
  let base = M.map((row) => [...row]);
  let e = Math.max(0, Math.round(exponent));
  while (e > 0) {
    if (e & 1) result = matMul(result, base);
    base = matMul(base, base);
    e >>= 1;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Velocity (flywheel) loop gains, mirroring CalculateVelocityFeedbackGains.
 *
 * @param {object} preset FeedbackControllerPreset-like {outputConversionFactor, velocityTimeFactor, periodMs, normalized, delayMs}
 * @param {{qp: number, qv: number, r: number}} lqr LQR tolerances.
 * @param {number} Kv Velocity feedforward gain.
 * @param {number} Ka Acceleration feedforward gain.
 * @param {number} [encFactor=1] Output-unit → encoder-unit conversion.
 * @returns {{Kp: number, Kd: number}}
 */
export function calculateVelocityFeedbackGains(preset, lqr, Kv, Ka, encFactor = 1.0) {
  if (!Number.isFinite(Kv) || !Number.isFinite(Ka) || Math.abs(Ka) < 1e-7) {
    return { Kp: 0, Kd: 0 };
  }
  const a = -Kv / Ka;
  const b = 1 / Ka;
  const dt = preset.periodMs / 1000;
  const { Ad, Bd } = discretizeFlywheel(a, b, dt);
  const P = dare(Ad, Bd, [[lqr.qv]], [[lqr.r]]);
  let K = (Bd[0][0] * P[0][0] * Ad[0][0]) / (lqr.r + Bd[0][0] * P[0][0] * Bd[0][0]);

  // Latency compensation: K = K · (Ad − Bd·K)^(delay/dt)
  const cl = [[Ad[0][0] - Bd[0][0] * K]];
  const powered = matPow(cl, preset.delayMs / preset.periodMs);
  K = K * powered[0][0];

  return {
    Kp: (K * preset.outputConversionFactor) / (preset.velocityTimeFactor * encFactor),
    Kd: 0,
  };
}

/**
 * Position (elevator-style) loop gains, mirroring CalculatePositionFeedbackGains.
 *
 * @returns {{Kp: number, Kd: number}}
 */
export function calculatePositionFeedbackGains(preset, lqr, Kv, Ka) {
  if (!Number.isFinite(Kv) || !Number.isFinite(Ka)) {
    return { Kp: 0, Kd: 0 };
  }
  const dt = preset.periodMs / 1000;

  // If acceleration for position control requires no effort, velocity becomes
  // an input (1-state model).
  if (Math.abs(Ka) < 1e-7) {
    const { Ad, Bd } = discretizeFlywheel(0, 1, dt);
    const P = dare(Ad, Bd, [[lqr.qp]], [[lqr.r]]);
    let K = (Bd[0][0] * P[0][0] * Ad[0][0]) / (lqr.r + Bd[0][0] * P[0][0] * Bd[0][0]);
    const cl = [[Ad[0][0] - Bd[0][0] * K]];
    K *= matPow(cl, preset.delayMs / preset.periodMs)[0][0];
    return { Kp: Kv * K * preset.outputConversionFactor, Kd: 0 };
  }

  const A = [
    [0, 1],
    [0, -Kv / Ka],
  ];
  const B = [[0], [1 / Ka]];
  const { Ad, Bd } = discretizeElevator(A, B, dt);
  const P = dare(
    Ad,
    Bd,
    [
      [lqr.qp, 0],
      [0, lqr.qv],
    ],
    [[lqr.r]]
  );

  // K = (R + BdᵀSBd)⁻¹ BdᵀSAd  (1×2)
  const BdPBd = matMul(matMul(transpose(Bd), P), Bd)[0][0] + lqr.r;
  const BdPA = matMul(matMul(transpose(Bd), P), Ad); // 1×2
  let K = [[BdPA[0][0] / BdPBd, BdPA[0][1] / BdPBd]];

  // Latency compensation: K = K · (Ad − Bd·K)^(delay/dt)
  const cl = matMul2x2Diff(Ad, Bd, K);
  const powered = matPow(cl, preset.delayMs / preset.periodMs);
  K = matMul(K, powered);

  return {
    Kp: K[0][0] * preset.outputConversionFactor,
    Kd:
      (K[0][1] * preset.outputConversionFactor) /
      (preset.normalized ? 1 : preset.periodMs / 1000),
  };
}

function matMul2x2Diff(Ad, Bd, K) {
  // cl = Ad − Bd·K (2×2)
  const BK = matMul(Bd, K);
  return Ad.map((row, i) => row.map((val, j) => val - BK[i][j]));
}
