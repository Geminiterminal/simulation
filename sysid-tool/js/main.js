/**
 * System Identification — application shell.
 *
 * UI layout mirrors the WPILib SysId tool (Log Loader / Data Selector /
 * Analyzer / Diagnostic Plots / Program Log); the analysis math lives in
 * ./analysis/* and matches the upstream implementation.
 */

import { parseWpilog } from "./log/wpilog.js";
import {
  TEST_STATES,
  buildTestRuns,
  extractSysIdData,
  parseCsv,
  parseJson,
} from "./log/parse.js";
import { generateDemoLog } from "./log/demo.js";
import { runAnalysis } from "./analysis/run.js";
import {
  PRESETS,
  PRESET_NAMES,
  clonePreset,
  presetsEqual,
} from "./analysis/feedback.js";
import { Plot } from "./plots.js";

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

const state = {
  log: null,
  sysid: null,
  groupId: "average",
  runSelection: {},
  unit: "Meters",
  logUnit: "Meters",
  result: null,
  lqrConfigured: false,
  settings: {
    velocityThreshold: null,
    medianWindow: 1,
    stepTestDuration: 0,
    type: "Simple",
    presetName: "REV Brushless Encoder Port",
    preset: clonePreset(PRESETS["REV Brushless Encoder Port"]),
    lqr: { qp: 1, qv: 1.5, r: 7 },
    loopType: "velocity",
    direction: 0,
    pointSize: 1.25,
  },
};

const $ = (id) => document.getElementById(id);

const fmt5 = (x) => {
  if (!Number.isFinite(x)) return "—";
  return Number(x.toPrecision(5)).toString();
};

const ABBREVIATIONS = {
  Meters: "m",
  Feet: "ft",
  Inches: "in",
  Radians: "rad",
  Degrees: "deg",
  Rotations: "rot",
};

// ---------------------------------------------------------------------------
// Program log
// ---------------------------------------------------------------------------

function programLog(message, level = "info") {
  const body = $("programLogBody");
  const time = new Date().toLocaleTimeString();
  const tag = level === "error" ? "ERROR" : level === "warn" ? " WARN" : " INFO";
  body.textContent += `[${time}] ${tag}  ${message}\n`;
  body.scrollTop = body.scrollHeight;
}

// ---------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------

function applyTheme(theme) {
  document.documentElement.setAttribute("data-theme", theme);
  $("themeToggle").textContent = theme === "dark" ? "🌙" : "☀";
  localStorage.setItem("sysid-theme", theme);
  // Redraw plots with the new palette.
  updatePlots();
}

function initTheme() {
  const stored = localStorage.getItem("sysid-theme");
  const preferred =
    window.matchMedia && window.matchMedia("(prefers-color-scheme: light)").matches
      ? "light"
      : "dark";
  applyTheme(stored ?? preferred);
}

// ---------------------------------------------------------------------------
// Tooltip
// ---------------------------------------------------------------------------

function initTooltips() {
  const tooltip = $("tooltip");
  document.addEventListener("mouseover", (e) => {
    const target = e.target.closest("[data-tip]");
    if (!target) return;
    tooltip.textContent = target.getAttribute("data-tip");
    tooltip.style.display = "block";
  });
  document.addEventListener("mousemove", (e) => {
    if (tooltip.style.display !== "block") return;
    const pad = 14;
    let x = e.clientX + pad;
    let y = e.clientY + pad;
    const rect = tooltip.getBoundingClientRect();
    if (x + rect.width > window.innerWidth - 8) x = e.clientX - rect.width - pad;
    if (y + rect.height > window.innerHeight - 8) y = e.clientY - rect.height - pad;
    tooltip.style.left = `${x}px`;
    tooltip.style.top = `${y}px`;
  });
  document.addEventListener("mouseout", (e) => {
    if (e.target.closest("[data-tip]")) tooltip.style.display = "none";
  });
}

// ---------------------------------------------------------------------------
// Menus, panels, modals
// ---------------------------------------------------------------------------

function initMenus() {
  const menubar = $("menubar");
  menubar.addEventListener("click", (e) => {
    const title = e.target.closest(".menu-title");
    document.querySelectorAll(".menu").forEach((m) => {
      if (!title || m !== title.parentElement) m.classList.remove("open");
    });
    if (title) title.parentElement.classList.toggle("open");
  });
  document.addEventListener("click", (e) => {
    if (!e.target.closest(".menu")) {
      document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
    }
  });

  document.querySelectorAll(".menu-dropdown button").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
      const action = btn.dataset.action;
      const panel = btn.dataset.panel;
      if (panel) {
        const el = $(panel);
        el.hidden = !el.hidden;
        btn.textContent = `${el.hidden ? "☐" : "☑"} ${btn.textContent.slice(2)}`;
        return;
      }
      handleAction(action);
    });
  });
}

function handleAction(action) {
  switch (action) {
    case "open-log":
      $("logFileInput").click();
      break;
    case "load-demo":
      loadDemo();
      break;
    case "reset-workspace":
      state.result = null;
      state.lqrConfigured = false;
      state.settings.velocityThreshold = null;
      state.settings.stepTestDuration = 0;
      state.settings.medianWindow = 1;
      clearOutputs();
      programLog("Workspace reset. SysId is currently in theoretical analysis mode.");
      break;
    case "toggle-theme":
      applyTheme(
        document.documentElement.getAttribute("data-theme") === "dark" ? "light" : "dark"
      );
      break;
    case "toggle-program-log":
      $("programLog").hidden = !$("programLog").hidden;
      break;
    case "refit-plots":
      updatePlots(true);
      break;
    case "about":
      showModal(
        "About",
        `<p><b>System Identification</b> — browser-based SysId analysis for FRC
        mechanisms (drivetrain linear/angular/steer, elevators, arms, flywheels).</p>
        <p>The analysis pipeline (OLS feedforward regression, median filtering,
        step-test trimming, LQR feedback gains, and the diagnostic plots) is a
        faithful port of the WPILib SysId tool's math. Load a
        <code>.wpilog</code> captured with <code>DataLogManager</code> and a
        WPILib <code>SysIdRoutine</code>, or try the built-in demo log.</p>
        <p>Dark and light modes are available from the 🌙 button in the title
        bar or <i>View → Toggle Dark / Light Mode</i>.</p>`
      );
      break;
    case "docs-guide":
      showModal(
        "User Guide",
        `<h3>1. Capture a log</h3>
        <p>Run your SysId routine (all four tests: quasistatic & dynamic, forward
        & reverse) on the robot. WPILib's <code>SysIdRoutine</code> logs
        <code>sysid-test-state-*</code> and per-motor
        <code>voltage-/position-/velocity-*</code> entries into the datalog.</p>
        <h3>2. Open the log</h3>
        <p>Use <i>Workspace → Open Data Log…</i> and pick the
        <code>.wpilog</code> file (JSON and CSV are also accepted). The Log
        Loader shows the file's entries; the Data Selector detects the test
        states and runs.</p>
        <h3>3. Analyze</h3>
        <p>Press <b>Ok</b> in the Data Selector. Tweak <i>Window Size</i>,
        <i>Velocity Threshold</i> and <i>Test Duration</i> until the fit looks
        good in the Diagnostic Plots. Check the R²/RMSE readouts.</p>
        <h3>4. Feedback gains</h3>
        <p>Pick the <i>Gain Preset</i> that matches your motor controller, set
        the loop type, and read off Kp/Kd. Adjust <i>Max Velocity Error</i> and
        <i>Max Control Effort</i> to trade setpoint tracking against control effort.</p>
        <h3>Plots</h3>
        <p>Scroll to zoom, drag to pan, double-click to refit.</p>`
      );
      break;
    case "docs-math":
      showModal(
        "Math Reference",
        `<h3>Feedforward</h3>
        <p>Acceleration is regressed against velocity, voltage and a friction
        sign term with ordinary least squares:</p>
        <p><code>a = α·v + β·V + γ·sgn(v) [+ δ (+ ε)]</code></p>
        <p><code>Ks = −γ/β, Kv = −α/β, Ka = 1/β</code></p>
        <p>The displayed <i>Acceleration R²</i> is the adjusted R² of that fit;
        <i>Acceleration RMSE</i> is √(SSE/n).</p>
        <h3>Simulation</h3>
        <p>The Simulation traces integrate the identified model
        <code>dx/dt = [0 1; 0 −Kv/Ka]·x + [0; 1/Ka]·u + [0; −Ks/Ka]·sgn(v)</code>
        over the recorded voltages. Sim velocity R²/RMSE compare that simulated
        velocity against the raw measurements.</p>
        <h3>Feedback</h3>
        <p>Gains come from a discrete LQR on the identified plant, discretized at
        the controller period and latency-compensated for the measurement delay:
        <code>K = K·(Ad − Bd·K)^(delay/dt)</code>. Velocity loops use a
        1-state flywheel model; position loops a 2-state (position, velocity)
        model with Kp = K(0) and Kd = K(1).</p>
        <h3>Delays</h3>
        <p><i>Position Measurement Delay</i> = time from first applied voltage to
        first detected motion in the step tests. <i>Velocity Measurement
        Delay</i> = time from first applied voltage to the onset of peak
        acceleration.</p>`
      );
      break;
    default:
      break;
  }
}

function showModal(title, html) {
  $("modalTitle").textContent = title;
  $("modalBody").innerHTML = html;
  $("modalBackdrop").hidden = false;
}

function initModal() {
  $("modalClose").addEventListener("click", () => ($("modalBackdrop").hidden = true));
  $("modalBackdrop").addEventListener("click", (e) => {
    if (e.target === $("modalBackdrop")) $("modalBackdrop").hidden = true;
  });
}

function initPanels() {
  document.querySelectorAll("[data-collapse]").forEach((caret) => {
    caret.addEventListener("click", () => {
      const body = $(caret.dataset.collapse);
      body.classList.toggle("collapsed");
      caret.textContent = body.classList.contains("collapsed") ? "▶" : "▼";
    });
  });
  document.querySelectorAll(".panel-controls .panel-btn").forEach((btn) => {
    if (btn.textContent === "✕") {
      btn.addEventListener("click", () => {
        const panel = btn.closest(".panel");
        panel.hidden = true;
        // Reflect in Widgets menu.
        document.querySelectorAll(`[data-panel="${panel.id}"]`).forEach((m) => {
          m.textContent = `☐ ${m.textContent.slice(2)}`;
        });
      });
    } else if (btn.textContent === "—") {
      btn.addEventListener("click", () => {
        const panel = btn.closest(".panel");
        const caret = panel.querySelector("[data-collapse]");
        if (caret) caret.click();
      });
    }
  });

  $("programLogBar").addEventListener("click", (e) => {
    if (e.target.id === "programLogClose") return;
    $("programLog").classList.toggle("open");
    $("programLogToggle").textContent = $("programLog").classList.contains("open")
      ? "▼"
      : "▶";
  });
  $("programLogClose").addEventListener("click", (e) => {
    e.stopPropagation();
    $("programLog").hidden = true;
  });
}

// ---------------------------------------------------------------------------
// Log loading
// ---------------------------------------------------------------------------

function loadDemo() {
  const log = generateDemoLog();
  ingestLog(log);
  programLog(
    `Loaded demo log "Drivetrain linear" (${log.numRecords.toLocaleString()} records, ${log.numEntries} entries). Ground truth: Ks=0.10, Kv=2.25, Ka=0.30.`
  );
}

async function loadFile(file) {
  const name = file.name.replace(/\.[^.]+$/, "");
  try {
    if (file.name.toLowerCase().endsWith(".wpilog")) {
      const buffer = await file.arrayBuffer();
      const parsed = parseWpilog(buffer);
      ingestLog({ name, ...parsed });
    } else if (file.name.toLowerCase().endsWith(".csv")) {
      ingestLog(parseCsv(await file.text(), name));
    } else {
      ingestLog(parseJson(await file.text(), name));
    }
    programLog(`Loaded ${file.name} (${state.log.numRecords.toLocaleString()} records, ${state.log.numEntries} entries).`);
  } catch (err) {
    programLog(`Failed to load ${file.name}: ${err.message}`, "error");
    showModal("Error", `<p>Could not load <b>${file.name}</b>:</p><p><code>${err.message}</code></p>`);
  }
}

function ingestLog(log) {
  state.log = log;
  state.sysid = extractSysIdData(log);
  state.groupId = "average";
  state.runSelection = {};
  state.settings.velocityThreshold = null;
  state.settings.stepTestDuration = 0;
  state.lqrConfigured = false;

  $("logSummary").textContent =
    `${log.name}\n${log.numRecords.toLocaleString()} records, ${log.numEntries} entries`;

  renderEntryTree();
  renderDataSelector();

  if (state.sysid.groups.length > 0 && state.sysid.runs.length > 0) {
    analyze({ firstPass: true });
  } else {
    programLog("No SysId test-state / voltage / position / velocity entries were detected in this log.", "warn");
    $("dsList").innerHTML = `<div class="muted">No SysId test data detected in this log.</div>`;
    clearOutputs();
  }
}

// ---------------------------------------------------------------------------
// Log Loader entry tree
// ---------------------------------------------------------------------------

function renderEntryTree() {
  const tree = $("entryTree");
  tree.innerHTML = "";
  if (!state.log) return;

  const filter = $("logFilter").value.trim().toLowerCase();
  const nodes = { name: "", children: new Map(), entry: null };

  for (const entry of state.log.entries) {
    if (!["double", "float", "string", "boolean", "int64"].includes(entry.type)) continue;
    if (filter && !entry.name.toLowerCase().includes(filter)) continue;

    // Split on first ":" (unless the prefix contains "/"), then on "/".
    let parts = [];
    const colon = entry.name.indexOf(":");
    if (colon > 0 && !entry.name.slice(0, colon).includes("/")) {
      parts.push(entry.name.slice(0, colon));
      parts.push(...entry.name.slice(colon + 1).split("/"));
    } else {
      parts = entry.name.split("/").filter((p) => p.length > 0);
    }
    if (parts.length === 0) continue;

    let node = nodes;
    for (const part of parts.slice(0, -1)) {
      if (!node.children.has(part)) {
        node.children.set(part, { name: part, children: new Map(), entry: null });
      }
      node = node.children.get(part);
    }
    const leaf = parts[parts.length - 1];
    if (!node.children.has(leaf)) {
      node.children.set(leaf, { name: leaf, children: new Map(), entry: null });
    }
    node.children.get(leaf).entry = entry;
  }

  const renderNode = (node, depth) => {
    for (const child of node.children.values()) {
      const hasChildren = child.children.size > 0;
      const row = document.createElement("div");
      row.className = "entry-row";
      const nameSpan = document.createElement("span");
      nameSpan.className = "name";
      if (hasChildren) {
        const caret = document.createElement("span");
        caret.className = "caret";
        caret.textContent = "▶";
        nameSpan.appendChild(caret);
        nameSpan.appendChild(document.createTextNode(child.name));
      } else {
        nameSpan.style.paddingLeft = "12px";
        nameSpan.textContent = child.name;
        nameSpan.title = child.entry?.name ?? child.name;
      }
      const typeSpan = document.createElement("span");
      typeSpan.className = "type";
      typeSpan.textContent = child.entry?.type ?? "";
      row.appendChild(nameSpan);
      row.appendChild(typeSpan);
      tree.appendChild(row);

      if (hasChildren) {
        const kids = document.createElement("div");
        kids.className = "entry-children hidden";
        tree.appendChild(kids);
        const container = kids;
        const renderInto = (parent, target) => {
          for (const c of parent.children.values()) {
            const cHasKids = c.children.size > 0;
            const r = document.createElement("div");
            r.className = "entry-row";
            const n = document.createElement("span");
            n.className = "name";
            if (cHasKids) {
              const cr = document.createElement("span");
              cr.className = "caret";
              cr.textContent = "▶";
              n.appendChild(cr);
              n.appendChild(document.createTextNode(c.name));
            } else {
              n.style.paddingLeft = "12px";
              n.textContent = c.name;
              n.title = c.entry?.name ?? c.name;
            }
            const t = document.createElement("span");
            t.className = "type";
            t.textContent = c.entry?.type ?? "";
            r.appendChild(n);
            r.appendChild(t);
            target.appendChild(r);
            if (cHasKids) {
              const sub = document.createElement("div");
              sub.className = "entry-children hidden";
              target.appendChild(sub);
              n.querySelector(".caret").addEventListener("click", (ev) => {
                ev.stopPropagation();
                sub.classList.toggle("hidden");
                n.querySelector(".caret").textContent = sub.classList.contains("hidden")
                  ? "▶"
                  : "▼";
              });
              renderInto(c, sub);
            }
          }
        };
        renderInto(child, container);
        nameSpan.querySelector(".caret").addEventListener("click", (ev) => {
          ev.stopPropagation();
          container.classList.toggle("hidden");
          nameSpan.querySelector(".caret").textContent = container.classList.contains("hidden")
            ? "▶"
            : "▼";
        });
      }
    }
  };
  renderNode(nodes, 0);
}

// ---------------------------------------------------------------------------
// Data Selector
// ---------------------------------------------------------------------------

function renderDataSelector() {
  const list = $("dsList");
  list.innerHTML = "";
  const sysid = state.sysid;

  if (!sysid || sysid.runs.length === 0) {
    list.innerHTML = `<div class="muted">Load a data log to select test states.</div>`;
    return;
  }

  for (const stateName of TEST_STATES) {
    const runs = sysid.runs.filter((r) => r.state === stateName);
    const head = document.createElement("div");
    head.className = "ds-state";
    head.textContent = `Test State: ${stateName}`;
    list.appendChild(head);

    if (runs.length === 0) {
      const miss = document.createElement("div");
      miss.className = "ds-run muted";
      miss.textContent = "not detected";
      list.appendChild(miss);
      continue;
    }

    runs.forEach((run, i) => {
      const line = document.createElement("div");
      line.className = "ds-run";
      const sel = document.createElement("select");
      sel.className = "input select";
      runs.forEach((_, j) => {
        const opt = document.createElement("option");
        opt.value = String(j + 1);
        opt.textContent = `Run ${j + 1}`;
        sel.appendChild(opt);
      });
      sel.value = String(state.runSelection[stateName] ?? 1);
      sel.addEventListener("change", () => {
        state.runSelection[stateName] = Number(sel.value);
      });
      if (i === (state.runSelection[stateName] ?? 1) - 1) {
        line.appendChild(sel);
      }
      const label = document.createElement("span");
      label.textContent = `samples: ${run.voltCount} Volt ${run.posCount} Pos ${run.velCount} Vel`;
      if (i === (state.runSelection[stateName] ?? 1) - 1) {
        line.appendChild(label);
      } else {
        label.textContent = `Run ${i + 1} samples: ${run.voltCount} Volt ${run.posCount} Pos ${run.velCount} Vel`;
        label.style.paddingLeft = "0";
        line.appendChild(label);
        line.style.cursor = "pointer";
        line.addEventListener("click", () => {
          state.runSelection[stateName] = i + 1;
          renderDataSelector();
        });
      }
      list.appendChild(line);
    });
  }

  // Motor selector
  const motorSel = $("dsMotor");
  motorSel.innerHTML = "";
  const avgOpt = document.createElement("option");
  avgOpt.value = "average";
  avgOpt.textContent = "All (average)";
  motorSel.appendChild(avgOpt);
  for (const g of sysid.groups) {
    const opt = document.createElement("option");
    opt.value = g.id;
    opt.textContent = g.label;
    motorSel.appendChild(opt);
  }
  motorSel.value = state.groupId;
}

// ---------------------------------------------------------------------------
// Analysis
// ---------------------------------------------------------------------------

function clearOutputs() {
  for (const id of [
    "ffKs",
    "ffKv",
    "ffKa",
    "ffTimescale",
    "ffPosDelay",
    "ffVelDelay",
    "fbKp",
    "fbKd",
    "statAccelR2",
    "statAccelRmse",
    "statSimR2",
    "statSimRmse",
  ]) {
    $(id).value = "";
  }
  updatePlots(true);
}

function analyze({ firstPass = false, feedbackOnly = false } = {}) {
  if (!state.sysid || state.sysid.groups.length === 0) return;

  if (feedbackOnly && state.result) {
    updateFeedbackFromSettings();
    return;
  }

  try {
    const testRuns = buildTestRuns(state.sysid, state.groupId, state.runSelection);
    const result = runAnalysis(testRuns, state.settings);
    state.result = result;

    // On first analysis of a fresh dataset, configure LQR defaults like SysId
    // (qp = 1/10 unit; qv = 1/4 of the max speed at 12 V).
    if (firstPass || !state.lqrConfigured) {
      const Ks = result.ff.gains.Ks;
      const Kv = result.ff.gains.Kv;
      if (Number.isFinite(Ks) && Number.isFinite(Kv) && Kv !== 0) {
        state.settings.lqr.qp = 0.1;
        state.settings.lqr.qv = 0.25 * (12.0 - Ks) / Kv;
        state.lqrConfigured = true;
        // Recompute feedback with the configured LQR tolerances.
        const testRuns2 = buildTestRuns(state.sysid, state.groupId, state.runSelection);
        Object.assign(state, { result: runAnalysis(testRuns2, state.settings) });
      }
    }

    for (const w of state.result.ff.warnings) programLog(w, "warn");
    // Full analysis adopts the measured delay (mirrors Analyzer::UpdateFeedforwardGains).
    state.settings.preset.delayMs = state.result.preset.delayMs;
    programLog(
      `Analysis updated: Ks=${fmt5(state.result.ff.gains.Ks)} Kv=${fmt5(state.result.ff.gains.Kv)} Ka=${fmt5(state.result.ff.gains.Ka)} ` +
        `(accel R²=${fmt5(state.result.ff.rSquared)}, sim R²=${fmt5(state.result.plots.simRSquared)})`
    );
    updateAnalyzerFields();
    updatePlots(true);
  } catch (err) {
    programLog(`Analysis failed: ${err.message}`, "error");
    showModal("Exception Caught!", `<p>${err.message}</p>`);
    clearOutputs();
  }
}

function updateFeedbackFromSettings() {
  // Recompute only the feedback gains (preset edits, LQR edits, loop type).
  // The user-specified measurement delay is preserved.
  if (!state.result) return;
  try {
    const testRuns = buildTestRuns(state.sysid, state.groupId, state.runSelection);
    const result = runAnalysis(testRuns, state.settings, { useMeasuredDelay: false });
    state.result = result;
    updateAnalyzerFields();
  } catch (err) {
    programLog(`Feedback update failed: ${err.message}`, "error");
  }
}

function updateAnalyzerFields() {
  const r = state.result;
  if (!r) return;

  $("anaUnits").textContent = state.unit;
  $("anaType").textContent = state.settings.type;

  $("ffKs").value = fmt5(r.ff.gains.Ks);
  $("ffKv").value = fmt5(r.ff.gains.Kv);
  $("ffKa").value = fmt5(r.ff.gains.Ka);
  $("ffTimescale").value = fmt5(r.timescaleMs);
  $("ffPosDelay").value = fmt5(r.positionDelayMs);
  $("ffVelDelay").value = fmt5(r.velocityDelayMs);

  $("ffWindow").value = state.settings.medianWindow;
  $("ffThreshold").value = state.settings.velocityThreshold ?? 0;
  $("ffDuration").value = fmt5(state.settings.stepTestDuration);

  // Feedback panel
  $("fbPreset").value = state.settings.presetName;
  $("fbMaxOut").value = fmt5(state.settings.preset.outputConversionFactor * 12);
  $("fbVelDenom").value = fmt5(state.settings.preset.velocityTimeFactor);
  $("fbPeriod").value = fmt5(state.settings.preset.periodMs);
  $("fbNormalized").checked = state.settings.preset.normalized;
  $("fbDelay").value = fmt5(state.settings.preset.delayMs);
  $("fbModified").textContent = presetsEqual(
    state.settings.preset,
    PRESETS[state.settings.presetName]
  )
    ? ""
    : "(modified)";

  $("fbLoopType").value = state.settings.loopType;
  $("fbKp").value = fmt5(r.fb.Kp);
  $("fbKd").value = fmt5(r.fb.Kd);
  $("fbQp").value = fmt5(state.settings.lqr.qp);
  $("fbQv").value = fmt5(state.settings.lqr.qv);
  $("fbR").value = fmt5(state.settings.lqr.r);
  $("fbQpField").style.display = state.settings.loopType === "position" ? "" : "none";
  const abbr = ABBREVIATIONS[state.unit] ?? state.unit;
  $("fbQpLabel").textContent = `Max Position Error (${abbr})`;
  $("fbQvLabel").textContent = `Max Velocity Error (${abbr}/s)`;

  // Stats
  $("statAccelR2").value = fmt5(r.ff.rSquared);
  $("statAccelRmse").value = fmt5(r.ff.rmse);
  $("statSimR2").value = fmt5(r.plots.simRSquared);
  $("statSimRmse").value = fmt5(r.plots.simRMSE);
}

// ---------------------------------------------------------------------------
// Plots
// ---------------------------------------------------------------------------

const plots = {};

function initPlots() {
  plots.quasistatic = new Plot($("plotQuasistatic"), {
    title: "Quasistatic Velocity vs. Time",
    xLabel: "Time (s)",
    yLabel: "Velocity (m/s)",
  });
  plots.dynamic = new Plot($("plotDynamic"), {
    title: "Dynamic Velocity vs. Time",
    xLabel: "Time (s)",
    yLabel: "Velocity (m/s)",
  });
  plots.accel = new Plot($("plotAccel"), {
    title: "Acceleration vs. Velocity",
    xLabel: "Velocity (m/s)",
    yLabel: "Velocity-Portion Accel (m/s²)",
  });
  plots.timesteps = new Plot($("plotTimesteps"), {
    title: "Timesteps vs. Time",
    xLabel: "Time (s)",
    yLabel: "Timestep duration (ms)",
    yLock: [0, 50],
  });
}

function seriesColors() {
  const style = getComputedStyle(document.documentElement);
  return {
    raw: style.getPropertyValue("--raw-color").trim() || "#59a8ff",
    filtered: style.getPropertyValue("--filtered-color").trim() || "#ff9d45",
    sim: style.getPropertyValue("--sim-color").trim() || "#4ade80",
    fit: style.getPropertyValue("--fit-color").trim() || "#ff9d45",
  };
}

function updatePlots(refit = false) {
  if (!plots.quasistatic) return; // plots not initialized yet
  const abbr = ABBREVIATIONS[state.unit] ?? state.unit;
  plots.quasistatic.opts.yLabel = `Velocity (${abbr}/s)`;
  plots.dynamic.opts.yLabel = `Velocity (${abbr}/s)`;
  plots.accel.opts.xLabel = `Velocity (${abbr}/s)`;
  plots.accel.opts.yLabel = `Velocity-Portion Accel (${abbr}/s²)`;

  const size = state.settings.pointSize;
  const colors = seriesColors();
  const r = state.result;

  const build = (data) => [
    { name: "Raw Data", points: data.raw, color: colors.raw, kind: "scatter", size },
    { name: "Filtered Data", points: data.filtered, color: colors.filtered, kind: "scatter", size },
    { name: "Simulation", points: data.sim, color: colors.sim, kind: "line", width: 1.5 },
  ];

  if (r) {
    plots.quasistatic.setSeries(build(r.plots.quasistatic));
    plots.dynamic.setSeries(build(r.plots.dynamic));
    plots.accel.setSeries([
      { name: "Filtered Data", points: r.plots.regression.points, color: colors.raw, kind: "scatter", size },
      { name: "Fit", points: r.plots.regression.fit, color: colors.fit, kind: "line", width: 1.5 },
    ]);
    plots.timesteps.setSeries([
      { name: "Filtered Data", points: r.plots.timesteps.points, color: colors.raw, kind: "scatter", size },
      { name: "Fit", points: r.plots.timesteps.fit, color: colors.fit, kind: "line", width: 1.5 },
    ]);
  } else {
    for (const p of Object.values(plots)) p.setSeries([]);
  }
  if (refit) {
    for (const p of Object.values(plots)) {
      p.limits = null;
      p.draw();
    }
  }
}

// ---------------------------------------------------------------------------
// Analyzer input wiring
// ---------------------------------------------------------------------------

function bindAnalyzerInputs() {
  const rerun = () => analyze();
  const rerunFeedback = () => analyze({ feedbackOnly: true });

  $("ffWindow").addEventListener("change", (e) => {
    state.settings.medianWindow = Math.min(15, Math.max(1, Math.round(Number(e.target.value)) || 1));
    rerun();
  });
  $("ffThreshold").addEventListener("change", (e) => {
    state.settings.velocityThreshold = Math.max(0, Number(e.target.value) || 0);
    rerun();
  });
  $("ffDuration").addEventListener("change", (e) => {
    state.settings.stepTestDuration = Math.max(0, Number(e.target.value) || 0);
    rerun();
  });

  // Preset selects
  const presetSel = $("fbPreset");
  for (const name of PRESET_NAMES) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    presetSel.appendChild(opt);
  }
  presetSel.value = state.settings.presetName;
  presetSel.addEventListener("change", () => {
    state.settings.presetName = presetSel.value;
    state.settings.preset = clonePreset(PRESETS[presetSel.value]);
    state.settings.loopType = "velocity";
    rerunFeedback();
  });

  $("fbMaxOut").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.preset.outputConversionFactor = v / 12.0;
      rerunFeedback();
    }
  });
  $("fbVelDenom").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.preset.velocityTimeFactor = v;
      rerunFeedback();
    }
  });
  $("fbPeriod").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.preset.periodMs = v;
      rerunFeedback();
    }
  });
  $("fbNormalized").addEventListener("change", (e) => {
    state.settings.preset.normalized = e.target.checked;
    rerunFeedback();
  });
  $("fbDelay").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v >= 0) {
      state.settings.preset.delayMs = v;
      rerunFeedback();
    }
  });
  $("fbLoopType").addEventListener("change", (e) => {
    state.settings.loopType = e.target.value;
    // SysId adopts the measured delay for the newly selected loop type.
    if (state.result) {
      state.settings.preset.delayMs =
        state.settings.loopType === "position"
          ? state.result.positionDelayMs
          : state.result.velocityDelayMs;
    }
    rerunFeedback();
    updateAnalyzerFields();
  });
  $("fbQp").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.lqr.qp = v;
      rerunFeedback();
    }
  });
  $("fbQv").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.lqr.qv = v;
      rerunFeedback();
    }
  });
  $("fbR").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v > 0) {
      state.settings.lqr.r = v;
      rerunFeedback();
    }
  });

  $("anaResetBtn").addEventListener("click", () => {
    state.settings.velocityThreshold = null;
    state.settings.stepTestDuration = 0;
    state.settings.medianWindow = 1;
    state.lqrConfigured = false;
    state.result = null;
    clearOutputs();
    programLog("Analyzer reset.");
  });

  $("overrideUnitsBtn").addEventListener("click", () => {
    showModal(
      "Override Units",
      `<p>Select the distance/angle unit for axis labels and gain units:</p>
       <select class="input select" id="unitOverrideSel">
         ${Object.keys(ABBREVIATIONS)
           .map((u) => `<option ${u === state.unit ? "selected" : ""}>${u}</option>`)
           .join("")}
       </select>
       <br/><button class="btn btn-accent" id="unitOverrideApply">Apply</button>`
    );
    $("unitOverrideApply").addEventListener("click", () => {
      state.unit = $("unitOverrideSel").value;
      $("anaUnits").textContent = state.unit;
      updateAnalyzerFields();
      updatePlots(true);
      $("modalBackdrop").hidden = true;
      programLog(`Units overridden to ${state.unit}.`);
    });
  });
  $("resetUnitsBtn").addEventListener("click", () => {
    state.unit = state.logUnit;
    $("anaUnits").textContent = state.unit;
    updateAnalyzerFields();
    updatePlots(true);
    programLog(`Units reset to log units (${state.unit}).`);
  });

  // Plots toolbar
  $("pointSize").value = state.settings.pointSize;
  $("pointSize").addEventListener("change", (e) => {
    const v = Number(e.target.value);
    if (v >= 1 && v <= 4) {
      state.settings.pointSize = v;
      updatePlots();
    }
  });
  $("directionSel").addEventListener("change", (e) => {
    state.settings.direction = Number(e.target.value);
    if (state.result) analyze();
    else updatePlots();
  });

  // Data Selector
  $("dsMotor").addEventListener("change", (e) => {
    state.groupId = e.target.value;
  });
  $("dsUnits").addEventListener("change", (e) => {
    state.unit = e.target.value;
  });
  $("dsType").addEventListener("change", (e) => {
    state.settings.type = e.target.value;
  });
  $("dsOkBtn").addEventListener("click", () => analyze());
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

function init() {
  initTooltips();
  initMenus();
  initModal();
  initPanels();
  initPlots();
  bindAnalyzerInputs();
  initTheme();

  $("themeToggle").addEventListener("click", () =>
    applyTheme(
      document.documentElement.getAttribute("data-theme") === "dark" ? "light" : "dark"
    )
  );
  $("openLogBtn").addEventListener("click", () => $("logFileInput").click());
  $("demoLogBtn").addEventListener("click", loadDemo);
  $("logFileInput").addEventListener("change", (e) => {
    if (e.target.files.length > 0) loadFile(e.target.files[0]);
    e.target.value = "";
  });
  $("logFilter").addEventListener("input", renderEntryTree);

  // Drag & drop onto the window loads a log file.
  window.addEventListener("dragover", (e) => e.preventDefault());
  window.addEventListener("drop", (e) => {
    e.preventDefault();
    if (e.dataTransfer.files.length > 0) loadFile(e.dataTransfer.files[0]);
  });

  programLog("System Identification started. SysId is currently in theoretical analysis mode.");
  loadDemo();
}

init();
