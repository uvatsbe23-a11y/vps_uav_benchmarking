"""Side-by-side flight video (used for the 2026-11 handover videos in docs/videos/): camera with the corners
OpenVINS uses (left), GPS vs OpenVINS map (right), date heading on top, drift summary at the bottom.
usage: python3 make_flight_video.py worst|best  (ROS 2 sourced; env FF = ffmpeg binary, SP = folder for a check frame).
The CFG paths point at the analysis folders on the ground PC; edit them for other flights. 0.25x speed, 30 fps."""
import sqlite3, glob, json, os, sys, subprocess, numpy as np, cv2
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

H = os.path.expanduser("~")
CFG = {
    "worst": dict(dir=f"{H}/Desktop/flight_tests_2026-10-10/analysis", n="t1", bag="../stage3_flight1_0928/*.db3",
                  date="10 Oct 2026", heading="10 October 2026", label="flight 1", a0=19.5, rot="first_leg", full_vio=False,
                  xlab="Along first leg (m)", bottom="Drift: 72%   |   Final error: 289 m after 399 m flown (72%)",
                  out="oct10_flight1_worst_side_by_side_HQ_slow.mp4"),
    "best": dict(dir=f"{H}/Desktop/flight_tests_2026-11-01/analysis", n="t4", bag="../flight04_*/bag/*.db3",
                 date="1 Nov 2026", heading="1 November 2026", label="flight 4", a0=39.75, rot="far", full_vio=True,
                 xlab="Along route (m)", bottom="Drift: 0.2%   |   Final error: 0.3 m after 163 m flown (0.2%)",
                 out="nov1_flight4_best_side_by_side_HQ_slow.mp4"),
}[sys.argv[1]]
OUT = f"{H}/Desktop/VPS_flight_test_report_2026-11-01/{CFG['out']}"
os.chdir(CFG["dir"]); n = CFG["n"]
P = np.load(f"{n}_bag.npz")["pose"]; t_bag0 = P[0, 0]
M = json.load(open(f"{n}_metrics.json")); L = np.load(f"{n}_log.npz", allow_pickle=True); S = np.load(f"{n}_series.npz")
t_to = L["XKF1"][0, 0] + M["t_takeoff"]; lag = M["bag_minus_gps_s"]
t, pa, pr = S["t"], S["pa"][:, :2], S["pr"][:, :2]; o = pr[0].copy(); pa, pr = pa - o, pr - o
kref = np.argmax(np.hypot(*pr.T)) if CFG["rot"] == "far" else np.searchsorted(t, 14)
ang = np.arctan2(pr[kref, 1], pr[kref, 0]); c, s = np.cos(-ang), np.sin(-ang); R = np.array([[c, -s], [s, c]])
pa, pr = pa @ R.T, pr @ R.T
A0, DUR = CFG["a0"], 5.0; A1 = A0 + DUR; T0 = t_to + lag - t_bag0 + A0
db = sqlite3.connect("file:" + glob.glob(CFG["bag"])[0] + "?mode=ro", uri=True)
tid = dict((nm, i) for i, nm in db.execute("select id,name from topics"))["/camera/left_ir/image_raw"]
rows = db.execute("select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp",
                  (tid, int((t_bag0 + T0 - 0.2) * 1e9), int((t_bag0 + T0 + DUR + 0.2) * 1e9))).fetchall()
ts = np.array([r[0] / 1e9 for r in rows]); fast = cv2.FastFeatureDetector_create(20)

def used(kp, nmax=200, d=10):  # strongest corners, at least d px apart (OpenVINS num_pts / min_px_dist)
    out = []
    for p in sorted(kp, key=lambda q: -q.response):
        x, y = p.pt
        if all((x - a) ** 2 + (y - b) ** 2 >= d * d for a, b in out): out.append((x, y))
        if len(out) >= nmax: break
    return out

K = 2.25; PW, PH = int(848 * K) // 2 * 2, int(480 * K) // 2 * 2; GAP, TOP, BAR = 8, 130, 120
W, Hh = 2 * PW + GAP, TOP + PH + BAR
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb"; GPS_C, VIO_C = "#eb6834", "#2a78d6"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 20, "axes.edgecolor": MUTED, "xtick.color": INK2, "ytick.color": INK2, "axes.labelcolor": INK2})
fig = plt.figure(figsize=(PW / 100, PH / 100), dpi=100, facecolor=SURF); ax = fig.add_axes([0.08, 0.12, 0.9, 0.72])
if CFG["full_vio"]: allp = np.vstack([pr, pa]); pad = 6
else: w = (t >= 0) & (t <= A1); allp = np.vstack([pr[w], pa[w]]); pad = 8
(x0, y0), (x1, y1) = allp.min(0) - pad, allp.max(0) + pad
FPS = 30; N = int(DUR * 4 * FPS)  # 0.25x speed
proc = subprocess.Popen([os.environ["FF"], "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{Hh}", "-r", str(FPS), "-i", "-",
                         "-c:v", "libx264", "-preset", "slow", "-crf", "16", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
font = cv2.FONT_HERSHEY_SIMPLEX
ip = lambda tq, arr: np.array([np.interp(tq, t, arr[:, 0]), np.interp(tq, t, arr[:, 1])])
# fixed top heading and bottom bar
head = np.zeros((TOP, W, 3), np.uint8)
(tw, th), _ = cv2.getTextSize(CFG["heading"], cv2.FONT_HERSHEY_DUPLEX, 2.6, 5)
cv2.putText(head, CFG["heading"], ((W - tw) // 2, (TOP + th) // 2), cv2.FONT_HERSHEY_DUPLEX, 2.6, (255, 255, 255), 5, cv2.LINE_AA)
cv2.line(head, (0, TOP - 2), (W, TOP - 2), (90, 90, 90), 2)
foot = np.zeros((BAR, W, 3), np.uint8)
(w1, _), _ = cv2.getTextSize(CFG["bottom"], font, 1.8, 4)
cv2.putText(foot, CFG["bottom"], ((W - w1) // 2, 75), font, 1.8, (255, 255, 255), 4, cv2.LINE_AA)
cache = {}
for k in range(N):
    tq = t_bag0 + T0 + DUR * k / N; i = int(np.argmin(abs(ts - tq)))
    if i not in cache:
        cache.clear()
        m = deserialize_message(rows[i][1], Image); a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        u = used(fast.detect(a, None))
        big = cv2.cvtColor(cv2.resize(a, (PW, PH), interpolation=cv2.INTER_LANCZOS4), cv2.COLOR_GRAY2BGR)
        for x, y in u: cv2.circle(big, (int(x * PW / a.shape[1]), int(y * PH / a.shape[0])), 6, (0, 255, 0), -1, cv2.LINE_AA)
        tf = ts[i] - lag - t_to
        cv2.rectangle(big, (0, 0), (PW, 76), (0, 0, 0), -1)
        cv2.putText(big, f"{CFG['date']} | t+{ts[i]-t_bag0:5.2f}s | {np.interp(tf,t,S['hgt']):4.1f} m high | {np.interp(tf,t,S['spd']):4.1f} m/s",
                    (18, 52), font, 1.3, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(big, "0.25x speed", (PW - 255, PH - 28), font, 1.2, (255, 255, 255), 3, cv2.LINE_AA)
        cache[i] = big
    big = cache[i]
    tg = tq - lag - t_to; spd = np.interp(tg, t, S["spd"]); hgt = np.interp(tg, t, S["hgt"]); tlab = tq - t_bag0
    j = min(max(np.searchsorted(t, tg), 1), len(t) - 1); gp, vp = ip(tg, pr), ip(tg, pa)
    ax.cla(); ax.set_facecolor(SURF)
    ax.plot(pr[t >= 0, 0], pr[t >= 0, 1], color=GPS_C, lw=2, alpha=0.22)
    if CFG["full_vio"]: ax.plot(pa[:, 0], pa[:, 1], color=VIO_C, lw=2, alpha=0.22)
    ax.plot(np.r_[pr[:j, 0], gp[0]], np.r_[pr[:j, 1], gp[1]], color=GPS_C, lw=4.5, label="GPS")
    ax.plot(np.r_[pa[:j, 0], vp[0]], np.r_[pa[:j, 1], vp[1]], color=VIO_C, lw=4.5, label="OpenVINS (VPS)")
    ax.plot([gp[0], vp[0]], [gp[1], vp[1]], color=INK, lw=2, ls=(0, (2, 2)))
    ax.plot(*gp, "o", ms=18, color=GPS_C, mec=SURF, mew=4, zorder=5); ax.plot(*vp, "o", ms=18, color=VIO_C, mec=SURF, mew=4, zorder=5)
    ax.plot(0, 0, "^", ms=16, color=INK, zorder=6)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel(CFG["xlab"]); ax.set_ylabel("Across (m)"); ax.grid(color=GRID, lw=1.2); ax.set_axisbelow(True)
    for sp in ("top", "right"): ax.spines[sp].set_visible(False)
    ax.legend(loc="lower right" if CFG["full_vio"] else "lower left", frameon=False, ncol=2)
    e = np.linalg.norm(gp - vp); fig.texts.clear()
    fig.text(0.02, 0.92, f"{CFG['date']}, {CFG['label']}  |  t+{tlab:5.2f}s  |  {hgt:4.1f} m high  |  {spd:4.1f} m/s  |  error {e:5.1f} m",
             fontsize=25, weight="bold", color=INK)
    fig.canvas.draw(); mp = np.asarray(fig.canvas.buffer_rgba())[:, :, :3][:, :, ::-1]
    mp = cv2.resize(mp, (PW, PH)) if mp.shape[:2] != (PH, PW) else mp
    mid = np.zeros((PH, W, 3), np.uint8); mid[:, :PW] = big; mid[:, PW:PW + GAP] = 255; mid[:, PW + GAP:] = mp
    frame = np.vstack([head, mid, foot])
    proc.stdin.write(np.ascontiguousarray(frame).tobytes())
    if k == 300: cv2.imwrite(f"{os.environ['SP']}/{sys.argv[1]}_check.png", frame)
proc.stdin.close(); proc.wait()
cap = cv2.VideoCapture(OUT); nf = int(cap.get(7)); f = cap.get(5)
print(sys.argv[1], os.path.getsize(OUT) // 1024, "KB |", nf, "frames @", f, "fps =", nf / f, "s |", int(cap.get(3)), "x", int(cap.get(4)))
