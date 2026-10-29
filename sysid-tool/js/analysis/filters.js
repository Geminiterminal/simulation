/**
 * Filtering utilities, mirroring WPILib SysId's FilteringUtils.
 */

export const NOISE_MEAN_WINDOW = 9;

export function sgn(x) {
  return x < 0 ? -1 : 1;
}

/** Sliding median filter matching wpi::math::MedianFilter behavior. */
class MedianFilter {
  constructor(window) {
    this.window = Math.max(1, Math.floor(window));
    this.buf = [];
  }

  calculate(x) {
    this.buf.push(x);
    if (this.buf.length > this.window) this.buf.shift();
    const sorted = [...this.buf].sort((a, b) => a - b);
    const n = sorted.length;
    return n % 2 === 1 ? sorted[(n - 1) / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2;
  }
}

/**
 * Reduces noise in velocity data by applying a median filter with the same
 * phase handling as SysId (filter pre-loaded with the first value, outputs
 * shifted by (window-1)/2, tail flushed with the last value).
 */
export function applyMedianFilter(data, window) {
  if (data.length < window) {
    throw new Error(
      `Not enough data to run Median Filter which has a window size of ${window}.`
    );
  }
  const filter = new MedianFilter(window);
  const shift = Math.floor((window - 1) / 2);
  for (let i = 0; i < window; i++) filter.calculate(data[0].velocity);
  const velocities = data.map((d) => d.velocity);
  for (let i = shift; i < data.length; i++) {
    data[i - shift].velocity = filter.calculate(velocities[i]);
  }
  for (let i = data.length - shift; i < data.length; i++) {
    data[i].velocity = filter.calculate(velocities[data.length - 1]);
  }
}

/**
 * Computes acceleration from velocity with a 3-sample central finite
 * difference at the mean time delta, matching SysId's PrepareMechData.
 *
 * acceleration[0]   = (v[1] − v[0]) / (2h)
 * acceleration[i]   = (v[i+1] − v[i−1]) / (2h)
 * acceleration[n-1] = 0
 */
export function prepareAcceleration(data) {
  const WINDOW = 3;
  if (data.length < WINDOW) {
    throw new Error(
      `Not enough data to run Acceleration Calculation which has a window size of ${WINDOW}.`
    );
  }
  const h = getMeanTimeDelta(data);
  const v = data.map((d) => d.velocity);
  data[0].acceleration = (v[1] - v[0]) / (2 * h);
  for (let i = 1; i < data.length - 1; i++) {
    data[i].acceleration = (v[i + 1] - v[i - 1]) / (2 * h);
  }
  data[data.length - 1].acceleration = 0;
}

/**
 * Expected noise used as the floor of the velocity trim: standard deviation
 * of each point from a moving average of the series.
 */
export function getNoiseFloor(data, window, accessor) {
  let sum = 0;
  const step = Math.floor(window / 2);
  const buf = [];
  let bufSum = 0;
  for (let i = 0; i < data.length; i++) {
    const x = accessor(data[i]);
    buf.push(x);
    bufSum += x;
    if (buf.length > window) bufSum -= buf.shift();
    const mean = bufSum / buf.length;
    if (i >= step) {
      sum += (accessor(data[i - step]) - mean) ** 2;
    }
  }
  return Math.sqrt(sum / (data.length - step));
}

export function getMaxSpeed(data, accessor) {
  let max = 0;
  for (const pt of data) max = Math.max(max, Math.abs(accessor(pt)));
  return max;
}

/** Mean time delta, ignoring nonpositive and >500 ms deltas.
 *  Accepts an array of PreparedData or a Storage-like object. */
export function getMeanTimeDelta(data) {
  const points = Array.isArray(data)
    ? data
    : [
        ...(data.slowForward ?? []),
        ...(data.slowBackward ?? []),
        ...(data.fastForward ?? []),
        ...(data.fastBackward ?? []),
      ];
  const dts = points.map((d) => d.dt).filter((dt) => dt > 0 && dt < 0.5);
  if (dts.length === 0) return 0.02;
  return dts.reduce((a, b) => a + b, 0) / dts.length;
}
