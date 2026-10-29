/**
 * Ordinary least squares multiple regression, mirroring WPILib SysId's OLS
 * (tools/sysid .../analysis/OLS.cpp).
 *
 *   y = Xβ + ε   →   β̂ = (XᵀX)⁻¹Xᵀy
 *
 * XᵀX is symmetric positive definite; we solve with Gaussian elimination and
 * partial pivoting (no external linear-algebra dependency).
 */

/** Solves matrix·x = rhs. Throws on a singular matrix. */
export function solveLinearSystem(matrix, rhs) {
  const n = rhs.length;
  const aug = matrix.map((row, i) => [...row, rhs[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
    }
    [aug[col], aug[pivot]] = [aug[pivot], aug[col]];
    const denom = aug[col][col];
    if (Math.abs(denom) < 1e-15) {
      throw new Error("Singular OLS normal matrix");
    }
    for (let j = col; j <= n; j++) aug[col][j] /= denom;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = aug[r][col];
      for (let j = col; j <= n; j++) aug[r][j] -= factor * aug[col][j];
    }
  }
  return aug.map((row) => row[n]);
}

/**
 * Performs OLS on the provided data.
 *
 * @param {number[][]} X Independent data in y = Xβ (n rows × p columns).
 * @param {number[]} y Dependent data.
 * @returns {{coeffs: number[], rSquared: number, rmse: number}} Regression
 *   coefficients plus the *adjusted* R² (as displayed by SysId) and the RMSE.
 */
export function ols(X, y) {
  const n = X.length;
  const p = X[0].length;
  if (y.length !== n) throw new Error("OLS: X and y size mismatch");

  // Normal equations: XᵀX β = Xᵀy
  const xtx = Array.from({ length: p }, () => new Array(p).fill(0));
  const xty = new Array(p).fill(0);
  for (let i = 0; i < n; i++) {
    const row = X[i];
    for (let a = 0; a < p; a++) {
      xty[a] += row[a] * y[i];
      for (let b = 0; b < p; b++) xtx[a][b] += row[a] * row[b];
    }
  }
  const coeffs = solveLinearSystem(xtx, xty);

  // SSE = (y − Xβ)ᵀ(y − Xβ)
  let sse = 0;
  for (let i = 0; i < n; i++) {
    let fit = 0;
    for (let a = 0; a < p; a++) fit += X[i][a] * coeffs[a];
    sse += (y[i] - fit) ** 2;
  }

  // SSTO = yᵀy − (Σy)²/n
  let sumY = 0;
  let sumYY = 0;
  for (let i = 0; i < n; i++) {
    sumY += y[i];
    sumYY += y[i] * y[i];
  }
  const ssto = sumYY - (sumY * sumY) / n;

  const rSquared = 1 - sse / ssto;
  // Adjusted R²: 1 − (1 − R²)(n − 1)/(n − p − 1)
  const adjRSquared = 1 - (1 - rSquared) * ((n - 1) / (n - p - 1));
  const rmse = Math.sqrt(sse / n);

  return { coeffs, rSquared: adjRSquared, rmse };
}
