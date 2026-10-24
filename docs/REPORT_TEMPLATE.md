# VPS Drift Test — Results Report

**Date(s):**
**Run by:**
**Jetson / camera serials:**
**Repo commit under test:** `git -C ~/ros2_ws/src/vps_uav_benchmarking rev-parse --short HEAD` →
**OpenVINS version / commit:**
**Orbbec SDK ROS 2 version:**

---

## Stage 0 — Bring-up

| Check | Result | Notes |
|---|---|---|
| Power mode MAXN_SUPER (`nvpmodel -m 2`, **not** `-m 0`) + `jetson_clocks` | ☐ |  |
| Jetson clock synced from ground PC (`sync_jetson_clock.sh`) | ☐ |  |
| `check_setup.sh` exit code | 0 / ___ |  |
| left IR rate | ___ Hz |  |
| right IR rate | ___ Hz |  |
| IMU `/camera/gyro_accel/sample` rate | ___ Hz | target 500 (~493 measured) |
| `/ov_msckf/odomimu` publishing | ☐ |  |
| left IR resolution | ___ × ___ | expect 848×480 |
| camera↔system clock offset | ___ s | expect < 0.5 |
| `ros2 bag info` shows all raw topics | ☐ | paste output below |

```
<paste check_setup.sh output>
<paste ros2 bag info output>
```

---

## Stage 1 — Axis fix verification

**Mounting:** matches reference photo ☐   Orientation notes:

### Static test (`stage1_static`)
| Metric | Value | Pass? |
|---|---|---|
| endpoint error | ___ m | few cm → ☐ |
| max \|z − z_start\| | ___ m | few cm → ☐ |
| altitude plot flat | | ☐ |

Attach: `stage1_static_trajectory.png`

### Hand-translation test (`stage1_axes`)
| Physical move | Estimate Δ reported (x, y, z) | Same direction? | Magnitude ≈ 1 m? |
|---|---|---|---|
| +X forward 1 m |  | ☐ | ☐ |
| +Y left 1 m |  | ☐ | ☐ |
| +Z up 1 m |  | ☐ | ☐ |

**Any sign flip / wrong axis / wrong scale?**  YES / NO  →  if YES, stop and report.

---

## Stage 2 — Handheld closed-loop drift

**Loop perimeter:** ___ m   **Laps per run:** ___   **path_length used:** ___ m
**Ground truth:** tape-and-loop / mocap

| Run | duration (s) | integrated path (m) | endpoint err (m) | err z (m) | max \|z−z0\| (m) | drift % |
|---|---|---|---|---|---|---|
| stage2_run1 |  |  |  |  |  |  |
| stage2_run2 |  |  |  |  |  |  |
| stage2_run3 |  |  |  |  |  |  |
| stage2_run4 |  |  |  |  |  |  |
| stage2_run5 |  |  |  |  |  |  |

(paste from `stage2_summary.csv`)

**Median drift:** ___ %   **Max drift:** ___ %   **Z bounded on all runs?** ☐
**Gate (low single-digit %, repeatable, no Z blow-up):** PASS / FAIL

Attach: `stage2_summary.csv`, `drift_summary.png`, per-run `*_trajectory.png`

---

## Stage 3 — Flight test  *(only after owner approval)*

**Approved by / date:**
**Reference:** flight-controller EKF (GPS) / RTK   **Analysis:** `scripts/analysis/` (see its README)
Example of a filled-in analysis: [`flight_tests/2026-10-10/`](flight_tests/2026-10-10/).

| Flight (session folder) | `.bin` log | profile (height, max speed, shape) | ground / light | OpenVINS ran (s of s) | error > 10 m after (m flown) | ATE anchored RMSE (m) | true drift % | height RMSE (m) | landing offset (m) |
|---|---|---|---|---|---|---|---|---|---|
| flight01 |  |  |  |  |  |  |  |  |  |
| flight02 |  |  |  |  |  |  |  |  |  |
| flight03 |  |  |  |  |  |  |  |  |  |
| flight04 |  |  |  |  |  |  |  |  |  |
| flight05 |  |  |  |  |  |  |  |  |  |

**OpenVINS stopped / crashed in any flight?** (time, `flight_status` message, core dump?)
**Z bounded?** ☐   **Verdict vs previous flights:**

Attach: per-flight `tN_metrics.txt` + `tN_report.png`; ground photos; `session.txt` of each flight

---

## Bags archived at

(path / drive — keep all raw bags for reprocessing)

## Open issues / anomalies observed
