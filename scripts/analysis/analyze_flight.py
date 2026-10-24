#!/usr/bin/env python3
"""OpenVINS vs ArduPilot telemetry — flight metrics.

usage: analyze_flight.py <name>       (expects <name>_bag.npz, <name>_log.npz from
                                       extract_bag.py / extract_bin.py)

Steps
  1. Time sync: cross-correlate low-passed |gyro| (camera IMU vs FC IMU).
     Rotation-invariant, so camera mounting does not matter.
  2. Mounting: Kabsch fit of the two gyro vector streams -> R_body_imu.
  3. Frame link (independent of the trajectory): VIO world -> ENU yaw from
     FC attitude x mounting x VIO attitude, averaged over the pre-takeoff part.
     Anchored at arming: VIO position at arm == reference position at arm.
     This is how VIO would actually be used live, so its error is the honest one.
  4. Metrics (reference = EKF3 position XKF1, GPS+IMU+baro fused):
     #1 ATE (anchored, and best-fit 4-DoF for comparison)
     #2 true drift % (end error / true path length)
     #4 scale (VIO path / true path; early-segment sim3 scale)
     #5 per-axis error, esp. Z
     #9 time / distance to divergence (2, 5, 10 m)
     #12 vibration: FC VIBE + same metric on the camera IMU
"""
import sys, json, numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as Rot
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

name = sys.argv[1]
B = np.load(f"{name}_bag.npz"); L = np.load(f"{name}_log.npz", allow_pickle=True)
out, M = [], {}
def say(s=""): print(s); out.append(s)

def lp(x, fc, fs):
    b, a = butter(2, fc / (fs / 2)); return filtfilt(b, a, x, axis=0)

# ---------- 1. time sync ----------
ci, fi = B["imu"], L["IMU"]
T = np.arange(max(ci[0, 0], fi[0, 0]), min(ci[-1, 0], fi[-1, 0]), 0.02)
cam_f, fc_f = lp(ci[:, 1:4], 1.5, 1 / np.median(np.diff(ci[:, 0]))), lp(fi[:, 1:4], 1.5, 1 / np.median(np.diff(fi[:, 0])))
interp3 = lambda t, src, val: np.array([np.interp(t, src, val[:, k]) for k in range(3)]).T
F = interp3(T, fi[:, 0], fc_f); fm = np.linalg.norm(F, axis=1); fm -= fm.mean()
best = (-2, 0)
for lag in np.arange(-3, 3, 0.005):
    c = np.linalg.norm(interp3(T + lag, ci[:, 0], cam_f), axis=1); c -= c.mean()
    r = c @ fm / np.linalg.norm(c) / np.linalg.norm(fm)
    if r > best[0]: best = (r, lag)
corr, lag = best
M["sync_corr"], M["bag_minus_gps_s"] = round(corr, 3), round(lag, 3)

# ---------- 2. mounting ----------
C = interp3(T + lag, ci[:, 0], cam_f)
U_, S_, Vt = np.linalg.svd(C.T @ F); d = np.sign(np.linalg.det(Vt.T @ U_.T))
R_b_i = Vt.T @ np.diag([1, 1, d]) @ U_.T
mount_tilt = np.degrees(np.arccos(np.clip(R_b_i[2, 2], -1, 1)))

# ---------- data on a common clock (GPS time) ----------
P = B["pose"].copy(); P[:, 0] -= lag; P[:, 1] -= lag
X = L["XKF1"]
ref_t, ref = X[:, 0], np.c_[X[:, 2], X[:, 1], -X[:, 3]]          # ENU
ref_spd = np.linalg.norm(X[:, 4:6], axis=1)
ev = L["events"]
t_arm = X[0, 0]
dis = [e[0] for e in ev if e[1] == "ARM" and "'ArmState': 0" in e[2]]
t_dis = dis[0] if dis else X[-1, 0]
u0 = np.median(ref[ref_t < t_arm + 2, 2])
up = np.where((ref_t > t_arm) & (ref[:, 2] > u0 + 0.5))[0]
t_to = ref_t[up[0]] if len(up) else t_arm
land = np.where((ref_t > t_to + 5) & (ref[:, 2] < u0 + 0.5))[0]
t_ld = min(ref_t[land[0]], t_dis) if len(land) else t_dis
win = (P[:, 0] >= t_arm) & (P[:, 0] <= t_dis)
tv, pv = P[win, 0], P[win, 2:5]
gaps = np.diff(P[:, 0]); big = np.where(gaps > 1.0)[0]
cov_note = (f"OpenVINS output covers {tv[0]-t_arm:.1f}s .. {tv[-1]-t_arm:.1f}s after arming "
            f"(flight ends {t_dis-t_arm:.1f}s)"
            + (f"; STOPPED {t_dis-tv[-1]:.0f}s before disarm" if t_dis - tv[-1] > 2 else "")
            + (f"; gaps >1 s: " + ", ".join(f"{P[k,0]-t_arm:.1f}s ({gaps[k]:.1f}s)" for k in big[:5]) if len(big) else ""))
pr = interp3(tv, ref_t, ref)

# ---------- 3. frame link (attitude based) ----------
A = L["ATT"]; R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
def link(t):
    a = [np.interp(t, A[:, 0], A[:, k]) for k in (1, 2, 3)]
    R_n_b = Rot.from_euler("ZYX", [a[2], a[1], a[0]], degrees=True).as_matrix()
    i = min(np.searchsorted(P[:, 0], t), len(P) - 1)
    return R_enu_ned @ R_n_b @ R_b_i @ Rot.from_quat(P[i, 5:9]).as_matrix().T
yaw_of = lambda R: np.degrees(np.arctan2(R[1, 0], R[0, 0]))
pre = np.arange(t_arm, max(t_to, t_arm + 1), 0.2)
yaw0 = np.degrees(np.angle(np.mean(np.exp(1j * np.radians([yaw_of(link(t)) for t in pre])))))
Rz = lambda y: Rot.from_euler("z", y, degrees=True).as_matrix()
tt = np.arange(t_arm, min(t_dis, P[-1, 0]), 0.5)          # only where VIO exists
yaw_track = np.array([yaw_of(link(t)) for t in tt])
heading_drift = (yaw_track - yaw0 + 180) % 360 - 180
tilt_track = np.array([np.degrees(np.arccos(np.clip(link(t)[2, 2], -1, 1))) for t in tt])

R0 = Rz(yaw0); t0 = pr[0] - R0 @ pv[0]
pa = pv @ R0.T + t0                                   # anchored VIO in ENU
err = pa - pr; en = np.linalg.norm(err, axis=1)

# best-fit 4-DoF (yaw + translation) over the flight, standard-ATE style
def fit4(a, b):
    ac, bc = a - a.mean(0), b - b.mean(0)
    H = ac[:, :2].T @ bc[:, :2]; y = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = Rz(np.degrees(y)); return R, b.mean(0) - R @ a.mean(0), np.degrees(y)
Rb, tb, yawb = fit4(pv, pr); eb = np.linalg.norm(pv @ Rb.T + tb - pr, axis=1)

# ---------- 4. metrics ----------
def path(t, p, dt=0.2):
    g = np.arange(t[0], t[-1], dt); q = interp3(g, t, p)
    return np.linalg.norm(np.diff(q, axis=0), axis=1).sum()
air = (tv >= t_to) & (tv <= t_ld)
L_ref, L_vio = path(tv, pr), path(tv, pv)
end_err = en[-1]
M.update(ate_anchored_rmse=np.sqrt((en ** 2).mean()), ate_anchored_max=en.max(),
         ate_bestfit_rmse=np.sqrt((eb ** 2).mean()), ate_bestfit_max=eb.max(),
         end_err=end_err, path_ref=L_ref, path_vio=L_vio,
         drift_true_pct=100 * end_err / L_ref, drift_old_pct=100 * end_err / L_vio,
         scale_path=L_vio / L_ref)
div = {}
for th in (2, 5, 10):
    k = np.where(air & (en > th))[0]
    if len(k):
        tk = tv[k[0]]; m = (tv >= t_to) & (tv <= tk)
        div[th] = (tk - t_to, path(tv[m], pr[m]) if m.sum() > 2 else 0.0)
early_end = t_to + (div[5][0] if 5 in div else 15)
m = (tv >= t_to) & (tv <= early_end)
if m.sum() > 10:
    a, b = pv[m] - pv[m].mean(0), pr[m] - pr[m].mean(0)
    Re, _, _ = fit4(pv[m], pr[m]); M["scale_early_sim3"] = np.sum(b * (a @ Re.T)) / np.sum(a * a)
    M["scale_early_path"] = path(tv[m], pv[m]) / max(path(tv[m], pr[m]), 1e-6)

# vibration (ArduPilot VIBE style: RMS of 5 Hz high-passed accel)
V = L["VIBE"]; Vf = V[(V[:, 0] >= t_to) & (V[:, 0] <= t_ld)]
fs_c = 1 / np.median(np.diff(ci[:, 0])); b_, a_ = butter(2, 5 / (fs_c / 2), "high")
hp = filtfilt(b_, a_, ci[:, 4:7], axis=0)
w = int(fs_c)                                        # 1 s windows
nwin = len(hp) // w
cam_vibe_t = ci[: nwin * w, 0].reshape(nwin, w).mean(1) - lag
cam_vibe = np.sqrt((hp[: nwin * w] ** 2).reshape(nwin, w, 3).mean(1))
cf = (cam_vibe_t >= t_to) & (cam_vibe_t <= t_ld)
acc_mag = np.linalg.norm(ci[:, 4:7], axis=1)

sig3 = 3 * np.sqrt(P[win, 9] + P[win, 10] + P[win, 11])
rate_air = air.sum() / max(min(t_ld, tv[-1]) - t_to, 1e-6)

# ---------- report ----------
base = lambda t: f"{t - t_arm:6.1f}s"
say(f"=== {name}: OpenVINS vs flight-controller reference ===")
say(f"Reference: EKF3 (XKF1) position. GPS {int(L['GPS'][:,2].min())}-{int(L['GPS'][:,2].max())} sats, "
    f"HAcc {L['GPA'][:,1].min():.1f}-{L['GPA'][:,1].max():.1f} m, VAcc {L['GPA'][:,2].min():.1f}-{L['GPA'][:,2].max():.1f} m "
    f"-> errors below ~2-3 m are not resolvable.")
say(f"Time sync: gyro correlation {corr:.3f}; bag clock = GPS time {lag:+.3f} s")
say(f"Camera mounting (from gyros): camera IMU z-axis {mount_tilt:.1f} deg from body-down -> camera faces DOWN")
say(f"Timeline (from arming): takeoff {base(t_to)}, landing {base(t_ld)}, disarm {base(t_dis)}")
say(cov_note + "  -> metrics below use only the covered part" if t_dis - tv[-1] > 2 else cov_note)
L_full = path(ref_t[(ref_t>=t_arm)&(ref_t<=t_dis)], ref[(ref_t>=t_arm)&(ref_t<=t_dis)])
covered = t_dis - tv[-1] <= 2
say(f"Flight: true path {L_full:.0f} m whole flight ({L_ref:.0f} m while OpenVINS ran), max speed {ref_spd.max():.1f} m/s, mean airborne speed "
    f"{ref_spd[(ref_t>=t_to)&(ref_t<=t_ld)].mean():.1f} m/s, height ~{np.percentile(ref[(ref_t>=t_to)&(ref_t<=t_ld),2]-u0,50):.1f} m")
say("")
say("#1 ATE (absolute trajectory error)")
say(f"   anchored at arming (live-use, honest) : RMSE {M['ate_anchored_rmse']:.1f} m, max {M['ate_anchored_max']:.1f} m")
say(f"   best-fit 4-DoF over flight (optimistic): RMSE {M['ate_bestfit_rmse']:.1f} m, max {M['ate_bestfit_max']:.1f} m")
say(f"   frame-link yaw {yaw0:.1f} deg vs best-fit yaw {yawb:.1f} deg")
say("#2 Drift")
say(("   end-point error" if covered else "   error at LAST OpenVINS output (not a closed loop)") + f" {end_err:.1f} m  /  true path {L_ref:.0f} m  =  {M['drift_true_pct']:.1f} %   "
    f"(old method, VIO's own path {L_vio:.0f} m: {M['drift_old_pct']:.1f} %)")
say("#4 Scale")
say(f"   whole flight: VIO path / true path = {M['scale_path']:.2f}")
if "scale_early_sim3" in M:
    say(f"   early flight (takeoff -> {early_end - t_to:.0f}s): sim3 scale {M['scale_early_sim3']:.2f}, path ratio {M['scale_early_path']:.2f}")
say("#5 Per-axis error (anchored)          RMSE     max     at end")
for k, ax in enumerate(["East", "North", "Up (Z)"]):
    say(f"   {ax:8s}                          {np.sqrt((err[:,k]**2).mean()):6.1f}  {np.abs(err[:,k]).max():6.1f}  {err[-1,k]:+7.1f} m")
if tv[-1] >= t_ld - 1: say(f"   VIO height at landing: {pv[np.searchsorted(tv,t_ld)-1,2]-pv[0,2]:+.1f} m (true ~0)")
say("#9 Divergence (anchored error first exceeds threshold, after takeoff)")
for th in (2, 5, 10):
    say(f"   > {th:2d} m : " + (f"{div[th][0]:5.1f} s after takeoff, after {div[th][1]:5.0f} m flown" if th in div else "never"))
say("#12 Vibration (RMS of >5 Hz accel, m/s^2; ArduPilot guide: <30 good, >60 bad)")
say(f"   FC IMU   (VIBE)  : mean X/Y/Z {Vf[:,1].mean():.1f}/{Vf[:,2].mean():.1f}/{Vf[:,3].mean():.1f}, max {Vf[:,1:4].max():.1f}, clip count {int(V[-1,4]-V[0,4])}")
say(f"   camera IMU       : mean X/Y/Z {cam_vibe[cf,0].mean():.1f}/{cam_vibe[cf,1].mean():.1f}/{cam_vibe[cf,2].mean():.1f}, max {cam_vibe[cf].max():.1f}"
    f"   (on ground before arming: {cam_vibe[cam_vibe_t<t_arm].mean():.2f})")
say(f"   camera accel |a| max {acc_mag.max():.1f} m/s^2 ({acc_mag.max()/9.81:.1f} g)")
say("Also noted")
say(f"   heading drift vs FC: max {np.abs(heading_drift).max():.1f} deg, end {heading_drift[-1]:+.1f} deg; tilt agreement max {tilt_track.max():.1f} deg")
say(f"   OpenVINS output rate in air {rate_air:.1f} Hz (camera 30 Hz)")
say(f"   error inside OpenVINS's own 3-sigma: {100*np.mean(en < sig3):.0f}% of the flight (consistent filter ~99%)")

M.update(yaw_link=yaw0, yaw_bestfit=yawb, t_takeoff=t_to - t_arm, t_land=t_ld - t_arm,
         divergence={str(k): v for k, v in div.items()}, rate_air=rate_air,
         vibe_fc_mean=Vf[:, 1:4].mean(0).tolist(), vibe_cam_mean=cam_vibe[cf].mean(0).tolist(),
         heading_drift_max=float(np.abs(heading_drift).max()), within_3sigma=float(np.mean(en < sig3)))
np.savez(f"{name}_series.npz", t=tv - t_to, err=en, err_xyz=err, air=air, pa=pa, pr=pr,
         dist=np.r_[0, np.cumsum(np.linalg.norm(np.diff(pr, axis=0), axis=1))],
         spd=np.interp(tv, ref_t, ref_spd), hgt=pr[:, 2] - u0,
         yawrate=np.interp(tv, A[:, 0], np.degrees(np.unwrap(np.radians(A[:, 3]))), ) )
open(f"{name}_metrics.txt", "w").write("\n".join(out) + "\n")
json.dump({k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in M.items()},
          open(f"{name}_metrics.json", "w"), indent=1, default=float)

# ---------- plots ----------
C1, C2, MUTED, INK, SURF = "#2a78d6", "#eb6834", "#8a8984", "#0b0b0b", "#fcfcfb"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": SURF, "axes.facecolor": SURF})
fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
rel = lambda t: t - t_to
a = ax[0, 0]
a.plot(pr[:, 0], pr[:, 1], color=C1, lw=2, label="True path (flight controller)")
a.plot(pa[:, 0], pa[:, 1], color=C2, lw=2, label="OpenVINS")
a.plot(*pr[0, :2], "o", ms=9, color=INK, mec=SURF, mew=2); a.annotate("start", pr[0, :2], xytext=(8, 8), textcoords="offset points", color=INK)
a.set_aspect("equal", "datalim"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)"); a.set_title("Top view", loc="left", color=INK)
a.legend(frameon=False, loc="best")
a = ax[0, 1]
a.plot(rel(tv), en, color=C2, lw=2, label="OpenVINS position error")
for th in (2, 5, 10): a.axhline(th, color=MUTED, lw=0.8, ls="--"); a.text(rel(tv[-1]), th, f" {th} m", va="center", color=MUTED, fontsize=8)
a.set_yscale("log"); a.set_ylim(0.05, max(en.max() * 1.5, 20)); a.set_xlabel("Time since takeoff (s)"); a.set_ylabel("Error (m, log scale)")
a.set_title("Position error vs time", loc="left", color=INK); a.axvline(rel(t_ld), color=MUTED, lw=0.8); a.text(rel(t_ld), a.get_ylim()[1], " landed", va="top", color=MUTED, fontsize=8)
a = ax[1, 0]
a.plot(rel(tv), pr[:, 2] - pr[0, 2], color=C1, lw=2, label="True height")
a.plot(rel(tv), pa[:, 2] - pr[0, 2], color=C2, lw=2, label="OpenVINS height")
a.set_xlabel("Time since takeoff (s)"); a.set_ylabel("Height change from arming (m)"); a.set_title("Height (Z)", loc="left", color=INK); a.legend(frameon=False)
a = ax[1, 1]
a.plot(rel(V[:, 0]), np.linalg.norm(V[:, 1:4], axis=1), color=C1, lw=2, label="Flight-controller IMU")
a.plot(rel(cam_vibe_t), np.linalg.norm(cam_vibe, axis=1), color=C2, lw=2, label="Camera IMU")
a.axhline(30, color=MUTED, lw=0.8, ls="--"); a.text(rel(cam_vibe_t[-1]), 30, " 30", va="center", color=MUTED, fontsize=8)
a.set_xlabel("Time since takeoff (s)"); a.set_ylabel("Vibration, |RMS >5 Hz accel| (m/s²)"); a.set_title("Vibration", loc="left", color=INK); a.legend(frameon=False)
for a in ax.flat: a.axvspan(rel(t_to), rel(t_ld), color="#f0efec", zorder=0) if a is not ax[0, 0] else None
fig.suptitle(f"{name} — OpenVINS vs flight controller", x=0.06, ha="left", color=INK, fontsize=13)
fig.tight_layout(); fig.savefig(f"{name}_report.png", dpi=130)
say(f"\nsaved {name}_metrics.txt, {name}_metrics.json, {name}_report.png")
