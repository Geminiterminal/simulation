/**
 * UI smoke test: boots the full application in jsdom with the demo log and
 * verifies that the panels populate and the analysis outputs land in the DOM.
 */
import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));

test("UI boots, loads the demo log and displays analysis outputs", async (t) => {
  let JSDOM;
  try {
    ({ JSDOM } = await import("jsdom"));
  } catch {
    t.skip("jsdom not installed — run npm install");
    return;
  }

  const html = readFileSync(join(here, "..", "index.html"), "utf8");
  const dom = new JSDOM(html, { url: "http://localhost/" });
  const { window } = dom;

  // --- Browser API stubs that jsdom lacks -----------------------------------
  const mockCtx = new Proxy(
    {},
    {
      get: (target, prop) => {
        if (prop === "canvas") return {};
        return typeof prop === "string" ? () => mockCtx : undefined;
      },
      set: () => true,
    }
  );
  window.HTMLCanvasElement.prototype.getContext = () => mockCtx;
  window.ResizeObserver = class {
    observe() {}
    unobserve() {}
    disconnect() {}
  };
  if (!window.matchMedia) {
    window.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {} });
  }

  global.window = window;
  global.document = window.document;
  global.getComputedStyle = window.getComputedStyle.bind(window);
  global.localStorage = window.localStorage;
  global.HTMLElement = window.HTMLElement;
  global.ResizeObserver = window.ResizeObserver;

  // Boot the application (init() runs on import and loads the demo log).
  await import("../js/main.js");
  // Let any queued work finish.
  await new Promise((resolve) => setTimeout(resolve, 50));

  const $ = (id) => window.document.getElementById(id);

  // Log Loader summary shows the demo dataset.
  assert.match($("logSummary").textContent, /Drivetrain linear/);
  assert.match($("logSummary").textContent, /records/);

  // Entry tree contains the SysId channels.
  const treeText = $("entryTree").textContent;
  assert.match(treeText, /sysid-test-state-drive-linear/);
  assert.match(treeText, /drive-front-left/);
  assert.match(treeText, /Battery/);

  // Data Selector lists all four test states with run sample counts.
  const ds = $("dsList").textContent;
  for (const s of [
    "quasistatic-forward",
    "quasistatic-reverse",
    "dynamic-forward",
    "dynamic-reverse",
  ]) {
    assert.match(ds, new RegExp(`Test State: ${s}`));
    assert.match(ds, /samples: \d+ Volt \d+ Pos \d+ Vel/);
  }

  // Analyzer shows recovered gains (format: 5 significant digits).
  assert.ok(Number($("ffKs").value) > 0, `Ks=${$("ffKs").value}`);
  assert.ok(Math.abs(Number($("ffKv").value) - 2.25) < 0.15, `Kv=${$("ffKv").value}`);
  assert.ok(Math.abs(Number($("ffKa").value) - 0.3) < 0.15, `Ka=${$("ffKa").value}`);
  assert.ok(Number($("ffTimescale").value) > 0);

  // Feedback panel: REV preset fields.
  assert.equal($("fbPreset").value, "REV Brushless Encoder Port");
  assert.ok(Math.abs(Number($("fbMaxOut").value) - 1.0) < 1e-6);
  assert.ok(Math.abs(Number($("fbVelDenom").value) - 60) < 1e-6);
  assert.ok(Number($("fbKp").value) > 0, `Kp=${$("fbKp").value}`);

  // Stats bar is populated and in a sane range.
  assert.ok(Number($("statAccelR2").value) > 0.5, `accel R2=${$("statAccelR2").value}`);
  assert.ok(Number($("statSimR2").value) > 0.9, `sim R2=${$("statSimR2").value}`);
  assert.ok(Number($("statSimRmse").value) > 0);

  // Program log recorded the boot + analysis messages.
  assert.match($("programLogBody").textContent, /demo log/i);
  assert.match($("programLogBody").textContent, /Analysis updated/);

  // Theme toggle flips dark → light and updates the plots palette path.
  assert.equal(window.document.documentElement.getAttribute("data-theme"), "dark");
  $("themeToggle").click();
  assert.equal(window.document.documentElement.getAttribute("data-theme"), "light");
  $("themeToggle").click();
  assert.equal(window.document.documentElement.getAttribute("data-theme"), "dark");
});
