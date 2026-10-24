#!/usr/bin/env python3
"""Which flight conditions make OpenVINS drift fastest / slowest?

All four flights are cut into 5 s airborne windows. For each window:
  outcome  local drift rate (m/s) = | R_now * dp_vio - dp_ref | / 5 s
           R_now = VIO->ENU rotation from the attitude link at the window start,
           so heading error accumulated EARLIER is removed: the window is judged
           only on the error it adds itself.
  inputs   speed, accel, height, yaw rate, camera gyro, camera vibration,
           image brightness / sharpness / corners / optical flow / track ratio,
           GPS accuracy (reference quality).
'healthy' windows = accumulated error still < 20 m (what starts drift);
the rest are after OpenVINS was already lost.
Needs tN_bag.npz, tN_log.npz, tN_img.npy, tN_metrics.json (from the other scripts).
"""
import json, numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as Rot
from scipy.stats import spearmanr
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
W = 5.0
rows = []
for n in ["t1", "t2", "t3", "t4"]:
    B = np.load(f"{n}_bag.npz"); L = np.load(f"{n}_log.npz", allow_pickle=True)
    M = json.load(open(f"{n}_metrics.json")); lag = M["bag_minus_gps_s"]
    S = np.load(f"{n}_series.npz")
    P = B["pose"].copy(); P[:, 0] -= lag
    ci = B["imu"].copy(); ci[:, 0] -= lag
    I = np.load(f"{n}_img.npy"); I[:, 0] -= lag
    X, A, G = L["XKF1"], L["ATT"], L["GPA"]
    ref_t, ref = X[:, 0], np.c_[X[:, 2], X[:, 1], -X[:, 3]]
    vel = np.c_[X[:, 5], X[:, 4], -X[:, 6]]
    t_arm = X[0, 0]; t_to = t_arm + M["t_takeoff"]; t_ld = t_arm + M["t_land"]
    u0 = np.median(ref[ref_t < t_arm + 2, 2])
    # mounting + attitude link (same method as analyze_flight.py)
    def lp(x, fc, fs): b, a = butter(2, fc / (fs / 2)); return filtfilt(b, a, x, axis=0)
    fi = L["IMU"]; T = np.arange(max(ci[0, 0], fi[0, 0]), min(ci[-1, 0], fi[-1, 0]), 0.02)
    i3 = lambda t, s, v: np.array([np.interp(t, s, v[:, k]) for k in range(3)]).T
    C = i3(T, ci[:, 0], lp(ci[:, 1:4], 1.5, 493)); F = i3(T, fi[:, 0], lp(fi[:, 1:4], 1.5, 25))
    U, _, Vt = np.linalg.svd(C.T @ F); d = np.sign(np.linalg.det(Vt.T @ U.T)); R_b_i = Vt.T @ np.diag([1, 1, d]) @ U.T
    def link(t):
        a = [np.interp(t, A[:, 0], A[:, k]) for k in (1, 2, 3)]
        R_n_b = Rot.from_euler("ZYX", [a[2], a[1], a[0]], degrees=True).as_matrix()
        i = min(np.searchsorted(P[:, 0], t), len(P) - 1)
        R = R_enu_ned @ R_n_b @ R_b_i @ Rot.from_quat(P[i, 5:9]).as_matrix().T
        y = np.arctan2(R[1, 0], R[0, 0]); return Rot.from_euler("z", y).as_matrix()
    # camera vibration (5 Hz high-pass accel RMS) and |gyro|
    b_, a_ = butter(2, 5 / (493 / 2), "high"); hp = filtfilt(b_, a_, ci[:, 4:7], axis=0)
    yaw_u = np.degrees(np.unwrap(np.radians(A[:, 3])))
    t_end = min(t_ld, P[-1, 0])
    for w0 in np.arange(t_to + 2, t_end - W, W):      # skip the first 2 s of takeoff
        w1 = w0 + W
        dv = np.interp(w1, P[:, 0], P[:, 2:5].T[0]), np.interp(w1, P[:, 0], P[:, 3]), np.interp(w1, P[:, 0], P[:, 4])
        dv = np.array(dv) - np.array([np.interp(w0, P[:, 0], P[:, k]) for k in (2, 3, 4)])
        dr = i3(np.array([w1]), ref_t, ref)[0] - i3(np.array([w0]), ref_t, ref)[0]
        e = link(w0) @ dv - dr
        m = (ref_t >= w0) & (ref_t < w1); mi = (ci[:, 0] >= w0) & (ci[:, 0] < w1); mg = (I[:, 0] >= w0) & (I[:, 0] < w1)
        ma = (A[:, 0] >= w0) & (A[:, 0] < w1)
        spd = np.linalg.norm(vel[m, :2], axis=1)
        acc = np.linalg.norm(np.diff(vel[m], axis=0), axis=1) / np.diff(ref_t[m]) if m.sum() > 2 else [np.nan]
        cum = np.interp(w0 - t_to, S["t"], S["err"])
        rows.append(dict(
            flight=n, t=w0 - t_to, cum=cum, healthy=cum < 20,
            drift=np.linalg.norm(e) / W, drift_h=np.linalg.norm(e[:2]) / W, drift_v=abs(e[2]) / W,
            dist=np.linalg.norm(dr[:2]),
            speed=spd.mean(), speed_max=spd.max(), accel=np.mean(acc), vspeed=np.abs(vel[m, 2]).mean(),
            height=np.mean(ref[m, 2] - u0),
            yawrate=np.abs(np.diff(yaw_u[ma]) / np.diff(A[ma, 0])).mean() if ma.sum() > 2 else np.nan,
            cam_gyro=np.linalg.norm(ci[mi, 1:4], axis=1).mean(),
            vibe=np.linalg.norm(np.sqrt((hp[mi] ** 2).mean(0))),
            bright=np.nanmean(I[mg, 1]), sat=np.nanmean(I[mg, 2]), sharp=np.nanmean(I[mg, 4]),
            corners=np.nanmean(I[mg, 5]), flow=np.nanmedian(I[mg, 6]), track=np.nanmean(I[mg, 7]),
            gps_hacc=np.interp(w0 + W / 2, G[:, 0], G[:, 1])))
keys = ["speed", "speed_max", "accel", "vspeed", "height", "yawrate", "cam_gyro", "vibe",
        "bright", "sharp", "corners", "flow", "track", "gps_hacc"]
labels = dict(speed="speed (m/s)", speed_max="max speed (m/s)", accel="acceleration (m/s²)", vspeed="climb/descent (m/s)",
              height="height (m)", yawrate="yaw rate (°/s)", cam_gyro="camera rotation (rad/s)", vibe="camera vibration (m/s²)",
              bright="image brightness", sharp="image sharpness", corners="texture (corners/img)", flow="image motion (px/frame)",
              track="feature track ratio", gps_hacc="GPS accuracy (m)")
D = {k: np.array([r[k] for r in rows]) for k in rows[0]}
np.savez("conditions_windows.npz", **D)
out = []
def say(s=""): print(s); out.append(s)

H = D["healthy"]
say(f"{len(rows)} windows of {W:.0f} s ({H.sum()} healthy: accumulated error < 20 m; {(~H).sum()} after OpenVINS was already lost)")
for n in ["t1", "t2", "t3", "t4"]:
    k = (D["flight"] == n)
    say(f"  {n}: {k.sum():3d} windows, healthy {(k & H).sum():3d}, median local drift healthy {np.median(D['drift'][k & H]) if (k&H).any() else float('nan'):.2f} m/s, lost {np.median(D['drift'][k & ~H]) if (k&~H).any() else float('nan'):.2f} m/s")
say("\nRank correlation of local drift rate with each condition (Spearman rho; |rho|>0.3 notable)")
say(f"  {'condition':28s} {'healthy':>8s}  {'p':>6s}   {'all':>6s}")
corr = {}
for k in keys:
    ok = H & np.isfinite(D[k]); r, p = spearmanr(D[k][ok], D["drift"][ok])
    oka = np.isfinite(D[k]); ra, _ = spearmanr(D[k][oka], D["drift"][oka]); corr[k] = r
    say(f"  {labels[k]:28s} {r:+8.2f}  {p:6.3f}   {ra:+6.2f}")

say("\nLocal drift rate (m/s, median) by condition bins - healthy windows")
def bins(k, edges):
    s = f"  {labels[k]:26s}"
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = H & (D[k] >= lo) & (D[k] < hi)
        s += f" | {lo:g}-{hi:g}: " + (f"{np.median(D['drift'][m]):.2f} (n={m.sum()})" if m.sum() >= 3 else f"n={m.sum()}")
    say(s)
bins("speed", [0, 0.5, 2, 4, 6, 11]); bins("height", [0, 6, 12]); bins("flow", [0, 1, 3, 6, 20, 100])
bins("corners", [0, 300, 600, 900, 3000]); bins("vibe", [0, 8, 12, 16, 40]); bins("yawrate", [0, 2, 5, 10, 90])
bins("accel", [0, 0.3, 0.7, 1.5, 10]); bins("sharp", [0, 80, 120, 160, 1000])

for title, idx in [("WORST", np.argsort(-D["drift"])), ("BEST", np.argsort(D["drift"]))]:
    say(f"\n{title} 10 healthy windows")
    say("  flight   t(s)  drift  speed  hgt  yaw°/s  vibe  corners  flow  sharp  track  gpsHacc")
    c = 0
    for i in idx:
        if not H[i]: continue
        say(f"  {D['flight'][i]:6s} {D['t'][i]:5.0f}  {D['drift'][i]:5.2f}  {D['speed'][i]:5.1f} {D['height'][i]:4.1f}  {D['yawrate'][i]:5.1f}  {D['vibe'][i]:5.1f}  {D['corners'][i]:6.0f}  {D['flow'][i]:5.1f}  {D['sharp'][i]:5.0f}  {D['track'][i]:5.2f}  {D['gps_hacc'][i]:5.1f}")
        c += 1
        if c == 10: break
open("conditions_report.txt", "w").write("\n".join(out) + "\n")

# figure: drift vs the strongest conditions (healthy windows), one panel each
C1, C2, MUTED, INK, SURF = "#2a78d6", "#eb6834", "#8a8984", "#0b0b0b", "#fcfcfb"
FC = {"t1": "#2a78d6", "t2": "#eb6834", "t3": "#1baf7a", "t4": "#eda100"}
plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.grid": True, "grid.color": "#e6e5e0", "axes.spines.top": False, "axes.spines.right": False,
                     "figure.facecolor": SURF, "axes.facecolor": SURF})
top = sorted(keys, key=lambda k: -abs(corr[k]) if np.isfinite(corr[k]) else 0)[:6]
fig, ax = plt.subplots(2, 3, figsize=(14, 8.5))
for a, k in zip(ax.flat, top):
    for n in ["t1", "t2", "t3", "t4"]:
        m = H & (D["flight"] == n)
        a.scatter(D[k][m], D["drift"][m], s=36, color=FC[n], edgecolor=SURF, linewidth=1.5, label=f"flight {n[1]}")
    a.set_xlabel(labels[k]); a.set_ylabel("local drift (m/s)")
    a.set_title(f"{labels[k]}   ρ = {corr[k]:+.2f}", loc="left", color=INK)
ax.flat[0].legend(frameon=False)
fig.suptitle("OpenVINS local drift vs flight conditions (5 s windows, before OpenVINS was lost)", x=0.05, ha="left", color=INK, fontsize=13)
fig.tight_layout(); fig.savefig("conditions_vs_drift.png", dpi=120)
print("\nsaved conditions_report.txt, conditions_vs_drift.png, conditions_windows.npz")
