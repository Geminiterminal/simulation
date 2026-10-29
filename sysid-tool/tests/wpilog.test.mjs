import test from "node:test";
import assert from "node:assert/strict";
import { parseWpilog, writeWpilog } from "../js/log/wpilog.js";
import { generateDemoLog } from "../js/log/demo.js";
import { extractSysIdData } from "../js/log/parse.js";

test("wpilog round-trips names, types, timestamps and values", () => {
  const entries = [
    {
      name: "sysid-test-state-drive-linear",
      type: "string",
      values: [
        { t: 1.5, v: "quasistatic-forward" },
        { t: 1.52, v: "dynamic-forward" },
      ],
    },
    {
      name: "voltage-drive-front-left-drive-linear",
      type: "double",
      values: [
        { t: 1.5, v: 0.25 },
        { t: 1.52, v: 7.0 },
      ],
    },
    {
      name: "NT/Robot/Mode",
      type: "string",
      values: [{ t: 0.5, v: "Autonomous" }],
    },
  ];
  const buffer = writeWpilog(entries, { generator: "test" });
  const parsed = parseWpilog(buffer);

  assert.equal(parsed.numEntries, 3);
  assert.deepEqual(parsed.extraHeader, { generator: "test" });

  const state = parsed.entries.find((e) => e.name === "sysid-test-state-drive-linear");
  assert.equal(state.type, "string");
  assert.equal(state.values.length, 2);
  assert.equal(state.values[0].v, "quasistatic-forward");
  assert.ok(Math.abs(state.values[0].t - 1.5) < 1e-6);

  const volt = parsed.entries.find((e) => e.name === "voltage-drive-front-left-drive-linear");
  assert.equal(volt.type, "double");
  assert.ok(Math.abs(volt.values[1].v - 7.0) < 1e-12);
});

test("wpilog rejects invalid files", () => {
  assert.throws(() => parseWpilog(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]).buffer));
});

test("demo log through a wpilog round trip still extracts SysId data", () => {
  const log = generateDemoLog();
  // Round trip only the SysId channels (plus state) to keep the test quick.
  const wanted = log.entries.filter(
    (e) =>
      e.name.startsWith("voltage-") ||
      e.name.startsWith("position-") ||
      e.name.startsWith("velocity-") ||
      e.name.startsWith("sysid-test-state-")
  );
  const buffer = writeWpilog(wanted);
  const parsed = parseWpilog(buffer);
  const sysid = extractSysIdData(parsed);

  assert.equal(sysid.groups.length, 4);
  assert.equal(sysid.runs.length, 4);
  assert.equal(sysid.mechanism, "drive-linear");
  const states = sysid.runs.map((r) => r.state);
  assert.deepEqual(states, [
    "quasistatic-forward",
    "quasistatic-reverse",
    "dynamic-forward",
    "dynamic-reverse",
  ]);
  for (const run of sysid.runs) {
    assert.ok(run.voltCount > 50, `${run.state} voltCount=${run.voltCount}`);
  }
});
