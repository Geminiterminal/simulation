import test from "node:test";
import assert from "node:assert/strict";
import { ols, solveLinearSystem } from "../js/analysis/ols.js";

test("solveLinearSystem solves a 2x2 system", () => {
  const x = solveLinearSystem(
    [
      [2, 1],
      [1, 3],
    ],
    [5, 10]
  );
  assert.ok(Math.abs(x[0] - 1) < 1e-12);
  assert.ok(Math.abs(x[1] - 3) < 1e-12);
});

test("ols recovers exact coefficients on noise-free data", () => {
  // y = 2a − 3b + 0.5
  const X = [];
  const y = [];
  for (let i = 0; i < 20; i++) {
    const a = i * 0.37 - 3;
    const b = Math.sin(i) * 2;
    X.push([a, b, 1]);
    y.push(2 * a - 3 * b + 0.5);
  }
  const result = ols(X, y);
  assert.ok(Math.abs(result.coeffs[0] - 2) < 1e-9);
  assert.ok(Math.abs(result.coeffs[1] + 3) < 1e-9);
  assert.ok(Math.abs(result.coeffs[2] - 0.5) < 1e-9);
  assert.ok(Math.abs(result.rSquared - 1) < 1e-9);
  assert.ok(result.rmse < 1e-9);
});

test("ols computes adjusted R² and RMSE", () => {
  // Simple 1-predictor regression with known residuals.
  const X = [[1], [2], [3], [4]];
  const y = [2.1, 3.9, 6.2, 7.8]; // ≈ 2x
  const result = ols(X, y);
  const n = 4;
  const p = 1;
  // Manual computation
  const beta = result.coeffs[0];
  let sse = 0;
  let mean = y.reduce((a, b) => a + b, 0) / n;
  let ssto = 0;
  for (let i = 0; i < n; i++) {
    sse += (y[i] - beta * X[i][0]) ** 2;
    ssto += (y[i] - mean) ** 2;
  }
  const r2 = 1 - sse / ssto;
  const adj = 1 - (1 - r2) * ((n - 1) / (n - p - 1));
  assert.ok(Math.abs(result.rSquared - adj) < 1e-12);
  assert.ok(Math.abs(result.rmse - Math.sqrt(sse / n)) < 1e-12);
});

test("ols throws on singular normal matrix", () => {
  assert.throws(() =>
    ols(
      [
        [1, 2],
        [1, 2],
      ],
      [3, 3]
    )
  );
});
