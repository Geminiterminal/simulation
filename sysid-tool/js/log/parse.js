/**
 * Turns parsed log data (from .wpilog, .json, or .csv) into the SysId test
 * model used by the analyzer: per-test runs of voltage/position/velocity
 * sample streams, matching WPILib SysIdRoutineLog conventions
 * (voltage-<motor>-<mechanism>, position-..., velocity-...,
 * sysid-test-state-<mechanism>).
 */

import { lerp } from "./util.js";

export const TEST_STATES = [
  "quasistatic-forward",
  "quasistatic-reverse",
  "dynamic-forward",
  "dynamic-reverse",
];

function normalizeState(value) {
  const v = String(value).toLowerCase().trim();
  if (v.includes("quasistatic") || v === "quasi") {
    return v.includes("reverse") || v.includes("backward") || v.includes("back")
      ? "quasistatic-reverse"
      : "quasistatic-forward";
  }
  if (v.includes("dynamic") || v === "step") {
    return v.includes("reverse") || v.includes("backward") || v.includes("back")
      ? "dynamic-reverse"
      : "dynamic-forward";
  }
  return null;
}

function interpAt(series, t) {
  if (series.length === 0) return 0;
  if (t <= series[0].t) return series[0].v;
  const last = series[series.length - 1];
  if (t >= last.t) return last.v;
  let lo = 0;
  let hi = series.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (series[mid].t <= t) lo = mid;
    else hi = mid;
  }
  const a = series[lo];
  const b = series[hi];
  return b.t === a.t ? a.v : lerp(a.v, b.v, (t - a.t) / (b.t - a.t));
}

/**
 * Extracts SysId test data from a parsed log.
 *
 * @param {{entries: Array<{name, type, values}>}} log
 * @returns {{
 *   mechanism: string|null,
 *   stateEntry: string|null,
 *   groups: Array<{id, label, voltage, position, velocity}>,
 *   runs: Array<{state, run, startT, endT, voltCount, posCount, velCount}>,
 * }}
 */
export function extractSysIdData(log) {
  const entries = log.entries;

  // Test-state entry (string), e.g. "sysid-test-state-drive-linear"
  const stateEntries = entries.filter(
    (e) => e.type === "string" && e.name.toLowerCase().includes("test-state")
  );
  const stateEntry = stateEntries[0] ?? null;

  // Signal entries, e.g. "voltage-drive-front-left-drive-linear"
  const bySuffix = new Map();
  for (const e of entries) {
    if (e.type !== "double" && e.type !== "float") continue;
    const lower = e.name.toLowerCase();
    for (const kind of ["voltage", "position", "velocity"]) {
      if (lower.startsWith(kind + "-")) {
        const suffix = e.name.slice(kind.length + 1);
        if (!bySuffix.has(suffix)) {
          bySuffix.set(suffix, { id: suffix, voltage: [], position: [], velocity: [] });
        }
        const values = e.values.map(({ t, v }) => ({ t, v: Number(v) }));
        values.sort((a, b) => a.t - b.t);
        bySuffix.get(suffix)[kind] = values;
        break;
      }
    }
  }

  // Split the state stream into contiguous runs.
  const runs = [];
  if (stateEntry) {
    let current = null;
    for (const { t, v } of stateEntry.values) {
      const state = normalizeState(v);
      if (state && current && current.state === state) {
        current.endT = t;
      } else if (state) {
        if (current) runs.push(current);
        current = { state, run: 0, startT: t, endT: t };
      } else {
        if (current) runs.push(current);
        current = null;
      }
    }
    if (current) runs.push(current);

    // Number runs within each state.
    const counters = {};
    for (const run of runs) {
      counters[run.state] = (counters[run.state] ?? 0) + 1;
      run.run = counters[run.state];
    }
  }

  // Build groups with sample counts per run.
  const groups = [];
  for (const [suffix, signals] of bySuffix) {
    if (signals.voltage.length < 2 || signals.position.length < 2 || signals.velocity.length < 2) {
      continue;
    }
    groups.push({
      id: suffix,
      label: suffix.replace(/-[^-]+$/, ""), // drop mechanism suffix for display
      voltage: signals.voltage,
      position: signals.position,
      velocity: signals.velocity,
    });
  }
  groups.sort((a, b) => a.id.localeCompare(b.id));

  // Per-run sample counts (samples within the run's time window).
  for (const run of runs) {
    const inWindow = (series) =>
      series.filter(({ t }) => t >= run.startT - 1e-9 && t <= run.endT + 1e-9).length;
    run.voltCount = groups.length > 0 ? inWindow(groups[0].voltage) : 0;
    run.posCount = groups.length > 0 ? inWindow(groups[0].position) : 0;
    run.velCount = groups.length > 0 ? inWindow(groups[0].velocity) : 0;
  }

  const mechanism = stateEntry
    ? stateEntry.name.replace(/^.*sysid-test-state-?/i, "") || null
    : null;

  return { mechanism, stateEntry: stateEntry?.name ?? null, groups, runs };
}

/**
 * Builds per-test raw sample streams for analysis from extracted SysId data.
 *
 * @param {ReturnType<typeof extractSysIdData>} sysid
 * @param {string|"average"} groupId Which signal group (motor) to use.
 * @param {Record<string, number>} runSelection Map of state → run index (1-based).
 * @returns {Record<string, {voltage: [], position: [], velocity: []}>}
 */
export function buildTestRuns(sysid, groupId, runSelection) {
  const out = {};
  const groups =
    groupId === "average"
      ? [averageGroup(sysid.groups)]
      : sysid.groups.filter((g) => g.id === groupId);
  const group = groups[0];
  if (!group) return out;

  for (const state of TEST_STATES) {
    const runs = sysid.runs.filter((r) => r.state === state);
    const runIndex = (runSelection[state] ?? 1) - 1;
    const run = runs[Math.min(runIndex, runs.length - 1)];
    if (!run) continue;

    const clip = (series) =>
      series
        .filter(({ t }) => t >= run.startT - 1e-9 && t <= run.endT + 1e-9)
        .map(({ t, v }) => ({ t, v }));
    out[state] = {
      voltage: clip(group.voltage),
      position: clip(group.position),
      velocity: clip(group.velocity),
    };
  }
  return out;
}

/** Averages all motor groups onto the first group's timestamps. */
function averageGroup(groups) {
  if (groups.length === 1) return groups[0];
  const base = groups[0];
  const avgSeries = (kind) =>
    base[kind].map(({ t }) => {
      let sum = 0;
      for (const g of groups) sum += interpAt(g[kind], t);
      return { t, v: sum / groups.length };
    });

  return {
    id: "average",
    label: "Average",
    voltage: avgSeries("voltage"),
    position: avgSeries("position"),
    velocity: avgSeries("velocity"),
  };
}

// ---------------------------------------------------------------------------
// Fallback formats
// ---------------------------------------------------------------------------

function pickKey(keys, candidates) {
  for (const c of candidates) {
    const found = keys.find((k) => k.toLowerCase() === c);
    if (found) return found;
  }
  for (const c of candidates) {
    const found = keys.find((k) => k.toLowerCase().includes(c));
    if (found) return found;
  }
  return null;
}

/** Normalizes record-style data ({t, voltage, position, velocity, state}...) into a log. */
export function recordsToLog(records, name) {
  const tKey = pickKey(Object.keys(records[0]), ["timestamp", "time", "t", "seconds"]);
  const vKey = pickKey(Object.keys(records[0]), ["voltage", "volt", "volts", "v"]);
  const pKey = pickKey(Object.keys(records[0]), ["position", "pos", "distance", "p"]);
  const velKey = pickKey(Object.keys(records[0]), ["velocity", "vel", "speed"]);
  const sKey = pickKey(Object.keys(records[0]), ["state", "test-state", "test_state", "teststate"]);

  if (!tKey || !vKey || !pKey || !velKey) {
    throw new Error(
      "Could not identify timestamp/voltage/position/velocity columns in the file"
    );
  }

  const sorted = [...records].sort((a, b) => Number(a[tKey]) - Number(b[tKey]));
  const values = (key) =>
    sorted.map((r) => ({ t: Number(r[tKey]), v: Number(r[key]) }));

  const entries = [
    { name: "voltage-sysid-drive-linear", type: "double", values: values(vKey) },
    { name: "position-sysid-drive-linear", type: "double", values: values(pKey) },
    { name: "velocity-sysid-drive-linear", type: "double", values: values(velKey) },
  ];
  if (sKey) {
    entries.push({
      name: "sysid-test-state-drive-linear",
      type: "string",
      values: sorted.map((r) => ({ t: Number(r[tKey]), v: String(r[sKey]) })),
    });
  }

  return {
    name,
    numRecords: sorted.length,
    numEntries: entries.length,
    entries,
  };
}

/** Parses legacy SysId JSON ({tests: {"quasistatic-forward": [[V, pos, vel, t]...]}}). */
export function parseSysIdJson(obj, name) {
  const tests = obj.tests ?? obj;
  const records = [];
  for (const [state, rows] of Object.entries(tests)) {
    if (!Array.isArray(rows)) continue;
    for (const row of rows) {
      if (!Array.isArray(row) || row.length < 4) continue;
      records.push({
        timestamp: row[3],
        voltage: row[0],
        position: row[1],
        velocity: row[2],
        state,
      });
    }
  }
  if (records.length === 0) throw new Error("No SysId test rows found in JSON");
  return recordsToLog(records, name);
}

/** Parses CSV text with fuzzy-matched columns. */
export function parseCsv(text, name) {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  if (lines.length < 2) throw new Error("CSV file has no data rows");
  const headers = lines[0].split(",").map((h) => h.trim().replace(/^"|"$/g, ""));
  const records = [];
  for (let i = 1; i < lines.length; i++) {
    const cells = lines[i].split(",");
    const row = {};
    headers.forEach((h, j) => (row[h] = (cells[j] ?? "").trim().replace(/^"|"$/g, "")));
    records.push(row);
  }
  return recordsToLog(records, name);
}

/** Parses JSON text: SysId-style, records array, or columnar object. */
export function parseJson(text, name) {
  const obj = JSON.parse(text);
  if (Array.isArray(obj)) {
    return recordsToLog(obj, name);
  }
  if (obj.tests) {
    return parseSysIdJson(obj, name);
  }
  // Columnar: {timestamp: [...], voltage: [...], ...}
  const keys = Object.keys(obj);
  const tKey = pickKey(keys, ["timestamp", "time", "t", "seconds"]);
  if (tKey && Array.isArray(obj[tKey])) {
    const n = obj[tKey].length;
    const records = [];
    for (let i = 0; i < n; i++) {
      const row = {};
      for (const k of keys) {
        if (Array.isArray(obj[k])) row[k] = obj[k][i];
      }
      records.push(row);
    }
    return recordsToLog(records, name);
  }
  throw new Error("Unrecognized JSON layout");
}
