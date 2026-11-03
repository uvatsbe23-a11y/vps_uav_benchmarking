#!/usr/bin/env python3
"""Compass simulation (1 Nov 2026). Runs analyze_flight.py's time sync, mounting fit and frame link, then
rebuilds the OpenVINS track with the flight controller's heading applied to every step (what a compass
connected to the camera system would provide). Output: <name>_compass.npz with
tv (s after take-off), pc (compass-corrected track), pr (reference), pa (anchored OpenVINS), yaw.
usage: compass_simulation.py tN   (needs tN_bag.npz and tN_log.npz, as for analyze_flight.py)
Add a fixed compass error afterwards by rotating pc about the take-off point."""

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


# compass-fusion proxy: VIO increments rotated by the FC-derived heading at each instant
yaw_at = np.array([yaw_of(link(t)) for t in tv])
P2 = np.zeros((len(tv), 2)); P2[0] = pr[0, :2]
for k in range(len(tv) - 1):
    c, s_ = np.cos(np.radians(yaw_at[k])), np.sin(np.radians(yaw_at[k]))
    dv = pv[k + 1, :2] - pv[k, :2]
    P2[k + 1] = P2[k] + np.array([c * dv[0] - s_ * dv[1], s_ * dv[0] + c * dv[1]])
np.savez(f"{name}_compass.npz", tv=tv - t_to, pc=P2, pr=pr, pa=pa, yaw=yaw_at - yaw0)
print(name, "ok")
