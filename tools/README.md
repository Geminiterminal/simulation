# `tools/` — offline analysis of robot logs

Two standard-library-only Python modules:

| File | What it does |
|------|--------------|
| `wpilog.py` | Reader **and** writer for the WPILib `.wpilog` (DataLog) binary format, v1.0/v1.1. No numpy, no WPILib, no vendor libraries. Handles truncated logs, gzip, Git-LFS pointers, control/metadata records, every scalar and array type. |
| `sysid_wpilog.py` | Finds the `SysIdRoutine` data in a log, splits it into the four characterization tests, and identifies `kS`, `kV`, `kA` for every mechanism. Prints a text report, and optionally JSON / Markdown / CSV / paste-ready Java. |

Both are covered by `tests/test_sysid_wpilog.py` (`python3 -m unittest discover -s tests`).

---

## Why

`SysIdSupport.java` already runs the full four-part WPILib characterization for
eight mechanisms and writes per-motor `voltage-*`, `linear-velocity-*`,
`linear-position-*`, `angular-velocity-*` and `angular-position-*` entries plus a
`sysid-test-state-<mechanism>` marker into the normal data log. That log is the
only output the routine produces — the gains have to come from somewhere else.

WPILib's desktop SysId tool can consume those logs, but it needs a GUI, it
converts one mechanism at a time, and it is not available in CI or over SSH.
This tool reads the log directly:

```bash
python3 tools/sysid_wpilog.py info logs/FRC_20260418_152525.wpilog
python3 tools/sysid_wpilog.py fit  logs/FRC_20260418_152525.wpilog
```

## Where the log is

| Run | Location |
|-----|----------|
| Real robot | roboRIO `/home/lvuser/logs/` (or a USB stick plugged into the roboRIO — logs are copied there automatically) |
| Simulation | `logs/` in the project directory (already git-ignored) |
| Driver Station laptop | AdvantageScope / the Driver Station "Download Logs" button |

Files are named `FRC_<yyyymmdd>_<hhmmss>.wpilog`. A single log usually contains
several mechanisms if you ran more than one routine; `info` lists them all.

## Reading the report

```
recommended (exact ZOH model):  kS = 0.1814 V   kV = 2.7190 V/(m/s)   kA = 0.5477 V/(m/s^2)
    fit quality: R^2 = 0.9999   RMSE = 0.012 velocity unit   n = 1992
simple fit (WPILib-SysId-comparable):  kS = 0.1629 V   kV = 2.7739 ...
quasistatic-only (velocity) fit:       kS = 0.3205 V   kV = 2.7144 ...
free-speed expectation (12 V / max):   kV = 2.6902 V/(m/s)  (+1.1% vs. free-speed model)
```

* **recommended** — normally the *exact zero-order-hold* fit: it fits
  `v[k+1] = ad·v[k] + bd·V[k] + cd·sgn(v[k])` over every 20 ms frame and maps
  the discrete coefficients back to continuous-time `kS`, `kV`, `kA`
  (`a_c = ln(ad)/Δt`). This does not touch a finite-difference acceleration, so
  encoder noise cannot collapse `kA`.
* **simple fit** — the same model WPILib SysId's GUI reports:
  `V = kS·sgn(v) + kV·v + kA·a` with `a` from a central difference, quasistatic
  rows plus the dynamic tests' acceleration windows. Kept for comparison; on
  noisy logs its `kA` is biased low (because `a` is the noisy regressor).
* **quasistatic-only** — `V = kS·sgn(v) + kV·v` on the two ramp tests. Useful as
  an independent sanity check of `kV`.
* **free-speed expectation** — `12 V / free speed` in the *logged* unit, from
  the constants in `Constants.java` (`NEO 5676 RPM / 5.077:1`, etc.). A real
  `kV` should land within a few tens of percent of it. A large negative number
  in that column means the log's velocity unit is not what the entry metadata
  claims.
* **per-motor table** — one fit per logged motor. `kV` differing by more than
  ~10 % between wheels of the same mechanism means a dragging wheel, mismatched
  gearing or a bad encoder, not a tuning problem.

### Units matter — the numbers go straight into the controller config

The tool never converts your data: gains come out in **whatever unit the log
entry contains** (shown in the report, and in the `--java` snippet's comments).

| Mechanism | Logged velocity | Use the gains as |
|---|---|---|
| `drive-linear` | m/s | `SimpleMotorFeedforward<Distance>` / `Configs` drive `kV`: V per (m/s) |
| `drive-angular` | deg/s | V per (deg/s) |
| `drive-steer` | rot/s | V per (rot/s); multiply by 60 before entering an RPM-based REVLib config |
| `shooter-flywheels` | RPM | REVLib `closedLoop.feedForward.kV` (V per encoder velocity unit) |
| `shooter-feeder` | rot/s | Phoenix 6 `Slot0` 1:1 |
| `intake-deploy` | rot/s of the arm joint | V per (arm rot/s); fit `kG` from the gravity term separately |
| `intake-roller` | rot/s | Phoenix 6 `Slot0` 1:1 |
| `intake-conveyor` | RPM | REVLib `closedLoop.feedForward.kV` |

## Applying the gains

```bash
python3 tools/sysid_wpilog.py fit logs/FRC_....wpilog --mechanism shooter-flywheels --java
```

prints, for example:

```java
// Shooter flywheels (front + back NEO)  (velocity unit: RPM)
// from the ZOH fit: R^2 = 0.9999, RMSE = 15.4 velocity unit, 1992 samples
public static final double kShooterFlywheelsKS = 0.032190; // V
public static final double kShooterFlywheelsKV = 0.002110; // V / (RPM)
public static final double kShooterFlywheelsKA = 0.000800; // V / (RPM/s)
```

Paste into the matching place:

* `Constants.java` → add the identified numbers next to the current estimate.
* `Configs.java` / `Shooter.java` / `Intake.java` → the `closedLoop.feedForward.kV(...)`
  calls (REVLib 2026 expects **volts** per converted velocity unit, which is
  exactly what this emits — see `PATH_ACCURACY_AUDIT.md` item 2).
* `SimulationConstants` → `kA` converts to an inertia (`J = kA·kt·G / R…`); at
  minimum use it to sanity-check `kShooterFlywheelMoiKgMetersSquared` and the
  other MOI estimates documented in `SIMULATION.md`.

`--markdown SYSID_RESULTS.md` produces a table plus the Java block that can be
committed as the record of a tuning session; `--json` is the machine-readable
version (`recommended_source` says which fit was used).

## Quality warnings

The report ends with a `warnings:` list. Worth acting on:

| Warning | Meaning |
|---|---|
| `missing test legs` | Only part of the four-test routine is in the log — re-run it. |
| `ramp may be truncated` | A test is shorter than 1.5 s; the quasistatic ramp/step did not finish. |
| `poor fit: R^2 < 0.90` | Wheel slip, belt slip, binding, or a saturated battery. |
| `velocity residual is N% of the tested speed range` | Encoder noise/quantization or mechanical slop — `kA` is not trustworthy. |
| `the simple fit's kA is far below the ZOH fit's` | Finite-difference acceleration is noise-dominated; use the recommended values. |
| `the exact ZOH fit was rejected` | The log is too noisy/irregular for the exact fit; the fallback gains are rough. Re-run on carpet, on a fresh battery, with charged wheels. |
| `per-motor kV spread` | The motors are not sharing load; fix mechanics before tuning. |
| `N% of samples at > 11.8 V` | Voltage clipping — charge the battery and re-run. |
| `kA is not positive` | The acceleration term is not identifiable (dynamic legs missing or too short). |

## Reproducing without a robot

`demo` synthesizes a spec-compliant log from a known plant so the whole pipeline
can be exercised (and used as a regression fixture):

```bash
python3 tools/sysid_wpilog.py demo /tmp/synthetic.wpilog
python3 tools/sysid_wpilog.py fit  /tmp/synthetic.wpilog --java
```

The synthetic plant uses `kS = 0.18 V`, `kV = 2.69 V/(m/s)`, `kA = 0.55 V/(m/s²)`
for the drivetrain and a 1:1 NEO flywheel for the shooter; the recovered gains
land within ~1 % of those numbers.

## Notes on the file format

`wpilog.py` implements the layout written by `edu.wpi.first.util.datalog.DataLog`
(and read by `DataLogReader`) in WPILib 2024-2026:

```
"WPILOG" | uint16 version | uint32 extra-header length | extra header
record: uint8 header (entry/size/timestamp field widths) | entry id | size |
        timestamp (µs) | payload
control record (entry 0): 0 = start (name/type/metadata), 1 = finish, 2 = metadata
entry types: boolean, int64, float, double, string, *[] arrays, raw
```

It has been checked against a log written by WPILib itself (73 entries,
1 499 records, structs, NT metadata, non-empty extra header), and tolerates the
usual field problems: a log cut off mid-record (roboRIO power loss) stops
cleanly at the last complete record and reports `truncated`, and a Git-LFS
pointer or a non-log file gets a clear error instead of garbage.

To run that check against a real log of your own:

```bash
SYSID_WPILOG_FIXTURE=/path/to/FRC_....wpilog python3 -m unittest tests.test_sysid_wpilog -v
```
