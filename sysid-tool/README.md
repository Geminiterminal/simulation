# System Identification (SysId) Tool

A browser-based system-identification tool for FRC mechanisms — a working,
shareable version of the classic SysId desktop application with **dark and
light modes**.

## Features

- **Log Loader** — open real robot logs:
  - `.wpilog` (WPILib DataLog, e.g. from `DataLogManager`) — full binary parser
  - `.json` (SysId legacy format, record arrays, or columnar)
  - `.csv` (fuzzy-matched `timestamp/voltage/position/velocity/state` columns)
  - Entry tree with filter, drag-and-drop onto the window works too
- **Data Selector** — detects the four SysId test states
  (`quasistatic-forward/reverse`, `dynamic-forward/reverse`) and their runs with
  per-signal sample counts, lets you pick the motor (or average all) and run.
- **Analyzer**
  - *Feedforward*: Ks, Kv, Ka by OLS acceleration regression (Simple / Elevator /
    Arm models), median window size, velocity threshold, test duration trim,
    response timescale, measured position/velocity delays
  - *Feedback*: LQR-tuned Kp/Kd for position or velocity loops with controller
    gain presets (WPILib, CTRE Phoenix 5/6, REV Brushless/Brushed/Data Port,
    Venom), max controller output, velocity denominator units, controller
    period, time-normalized gains, measurement-delay latency compensation
- **Diagnostic Plots** — Quasistatic/Dynamic Velocity vs. Time (raw, filtered and
  simulated), Acceleration vs. Velocity, Timesteps vs. Time. Scroll to zoom,
  drag to pan, double-click to fit.
- **Statistics** — Acceleration R²/RMSE and Sim velocity R²/RMSE with
  explanatory tooltips.
- **Program Log** — timestamped analysis messages.
- **Dark & light mode** — 🌙 button in the title bar, *View → Toggle Dark /
  Light Mode*, persisted in `localStorage` (defaults to your OS preference).

The analysis math (OLS regression + adjusted R², median filtering, step-test
trimming and delay estimation, exact-ZOH motor simulation, discrete LQR with
latency compensation, controller presets) is a faithful port of the
[WPILib SysId tool](https://github.com/wpilibsuite/allwpilib/tree/main/tools/sysid),
so results match the desktop tool.

## Running

No build step — it is a static web app:

```bash
cd sysid-tool
python3 -m http.server 8000     # or: npm run serve
# open http://localhost:8000
```

A demo dataset ("Drivetrain linear", matching this repo's
`SysIdRoutine("drive-linear")` capture layout) loads automatically so the tool
is usable immediately. Click **Open data log file…** to analyze your own
`.wpilog`.

## Tests

```bash
cd sysid-tool
npm install        # dev-only dependency: jsdom (for the UI smoke test)
npm test           # node --test tests/*.test.mjs
```

The suite covers the OLS solver, feedforward recovery on synthetic plants,
feedback gain scaling/presets, the `.wpilog` round trip, JSON/CSV parsing, and a
full jsdom UI boot smoke test.

## Tips

- The demo log's ground truth is Ks = 0.10 V, Kv = 2.25 V·s/m, Ka = 0.30 V·s²/m —
  a successful analysis recovers ≈ those values.
- *Test Duration* trims each dynamic (step) test; the default keeps the data up
  to 90% of the maximum speed.
- For best feedback gains, set **Gain Preset** to match the motor controller
  that will run the loop; the measured delay is adopted automatically after
  each analysis (shown as “(modified)” when it differs from the preset).
