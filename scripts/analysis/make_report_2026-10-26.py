#!/usr/bin/env python3
"""26 Oct 2026 flight report (compared with 10 Oct): 5 HTML pages -> PNG via headless Firefox.
Run from this analysis folder. Horizontal metrics only (height is left out)."""
import os, json, subprocess, numpy as np

HERE = os.getcwd()
A10 = os.path.expanduser("~/Desktop/flight_tests_2026-10-10/analysis")
OUT = os.path.expanduser("~/Desktop/VPS_flight_test_report_2026-10-26")
PROF = os.path.expanduser("~/ffshot_profile")
os.makedirs(OUT, exist_ok=True); os.makedirs(PROF, exist_ok=True)

INK, INK2, MUTED, LINE, SURF, CARD = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#f4f3ef", "#fcfcfb"
REAL, OV = "#3b3a37", "#eb6834"
FC = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#6250d6"]      # palette slots 1-4, 7 (fixed order)
GOOD, WARN, BAD = "#0ca30c", "#c98500", "#d03b3b"
CSS = f"""
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: {SURF}; color: {INK}; font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; width: 1200px; padding: 44px 48px; }}
.kicker {{ font-size: 15px; letter-spacing: .12em; text-transform: uppercase; color: {INK2}; font-weight: 700; }}
h1 {{ font-size: 42px; line-height: 1.12; margin: 6px 0 8px; letter-spacing: -.01em; }}
h2 {{ font-size: 23px; margin: 0 0 12px; }}
.sub {{ font-size: 19px; color: {INK2}; max-width: 1000px; line-height: 1.45; }}
.row {{ display: flex; gap: 18px; }}
.card {{ background: {CARD}; border: 1px solid {LINE}; border-radius: 14px; padding: 22px 24px; flex: 1; }}
.tile .big {{ font-size: 44px; font-weight: 700; line-height: 1; }}
.tile .lab {{ font-size: 16px; color: {INK2}; margin-top: 10px; line-height: 1.4; }}
.sec {{ margin-top: 26px; }}
ul.plain {{ list-style: none; }}
ul.plain li {{ font-size: 17px; line-height: 1.45; padding: 6px 0 6px 30px; position: relative; }}
ul.plain li .ic {{ position: absolute; left: 0; top: 6px; width: 21px; height: 21px; border-radius: 50%; color: #fff; font-size: 13px;
                   font-weight: 700; display: flex; align-items: center; justify-content: center; }}
table {{ border-collapse: collapse; width: 100%; font-size: 16px; }}
th {{ text-align: left; color: {INK2}; font-weight: 700; font-size: 14px; padding: 8px 10px; border-bottom: 2px solid {LINE}; }}
td {{ padding: 9px 10px; border-bottom: 1px solid {LINE}; vertical-align: top; }}
tr.hl td {{ background: #fff4e8; font-weight: 700; }}
.banner {{ background: #fff1ef; border: 2px solid {BAD}; border-radius: 14px; padding: 20px 24px; }}
.banner h2 {{ color: {BAD}; }}
.verdict {{ background: {INK}; color: #fff; border-radius: 14px; padding: 22px 28px; font-size: 20px; line-height: 1.45; }}
.verdict b {{ color: #ffd58a; }}
.small {{ font-size: 15px; color: {INK2}; line-height: 1.45; }}
.pill {{ display: inline-block; font-size: 13px; font-weight: 700; padding: 3px 10px; border-radius: 99px; color: #fff; }}
.foot {{ margin-top: 22px; font-size: 13px; color: {MUTED}; }}
"""
FOOT = ('<div class="foot">VPS benchmarking · Jetson "vajra" + Orbbec Gemini 335L · OpenVINS (same settings on both days) · '
        'reference = flight-controller GPS · horizontal position only</div>')
def ic(k):
    c, s = {"good": (GOOD, "✓"), "bad": (BAD, "✕"), "warn": (WARN, "!"), "info": (INK2, "•")}[k]
    return f'<span class="ic" style="background:{c}">{s}</span>'
def page(name, body, h):
    p = os.path.join(OUT, name + ".html")
    open(p, "w").write(f"<!doctype html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>{body}</body></html>")
    subprocess.run(["timeout", "120", "firefox", "--headless", "--no-remote", "--profile", PROF, "--screenshot",
                    os.path.join(OUT, name + ".png"), f"--window-size=1200,{h}", "file://" + p],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("wrote", os.path.join(OUT, name + ".png"))

# ------------------------------------------------------------------ data
FL = [("10 Oct #1", f"{A10}/t1"), ("10 Oct #2", f"{A10}/t2"), ("10 Oct #3", f"{A10}/t3"), ("10 Oct #4", f"{A10}/t4"), ("26 Oct", f"{HERE}/t5")]
D = {}
for lab, p in FL:
    S = np.load(p + "_series.npz"); m = S["t"] >= 0
    D[lab] = dict(t=S["t"][m], eh=np.linalg.norm(S["err_xyz"][m][:, :2], axis=1),
                  dist=S["dist"][m] - S["dist"][m][0], pr=S["pr"][:, :2], pa=S["pa"][:, :2])

def mini_map(lab, w, h):
    pr, pa = D[lab]["pr"], D[lab]["pa"]; ext = np.vstack([pr, pa])
    lo, hi = ext.min(0), ext.max(0); span = (hi - lo).max() * 1.2 + 1; c = (lo + hi) / 2; sc = min(w, h) / span
    X = lambda p: w / 2 + (p[:, 0] - c[0]) * sc; Y = lambda p: h / 2 - (p[:, 1] - c[1]) * sc
    pts = lambda p: " ".join(f"{x:.1f},{y:.1f}" for x, y in zip(X(p), Y(p)))
    st = max(1, len(pr) // 400)
    s = f'<svg width="{w}" height="{h}"><rect width="{w}" height="{h}" rx="10" fill="#efeee9"/>'
    s += f'<polyline points="{pts(pr[::st])}" fill="none" stroke="{REAL}" stroke-width="3.5" stroke-linejoin="round"/>'
    s += f'<polyline points="{pts(pa[::st])}" fill="none" stroke="{OV}" stroke-width="3.5" stroke-linejoin="round"/>'
    s += f'<circle cx="{X(pr[:1])[0]:.1f}" cy="{Y(pr[:1])[0]:.1f}" r="8" fill="{INK}" stroke="#fff" stroke-width="2.5"/>'
    L = 20 if span < 200 else 50
    s += f'<line x1="{w-24-L*sc:.1f}" y1="{h-20}" x2="{w-24}" y2="{h-20}" stroke="{INK2}" stroke-width="2"/>'
    s += f'<text x="{w-24-L*sc/2:.1f}" y="{h-28}" font-size="13" fill="{INK2}" text-anchor="middle">{L} m</text></svg>'
    return s

def leg_maps(lab, w, h):
    """Out and back legs side by side, same scale: on an out-and-back the return hides the outbound path."""
    S = np.load(f"{HERE}/t5_series.npz"); m = S["t"] >= 0
    pr, pa = S["pr"][m][:, :2], S["pa"][m][:, :2]
    k = int(np.argmax(np.linalg.norm(pr - pr[0], axis=1)))          # turnaround
    ext = np.vstack([pr, pa]); lo, hi = ext.min(0), ext.max(0); span = (hi - lo).max() * 1.15 + 1
    c = (lo + hi) / 2; sc = min(w, h) / span
    X = lambda q: w / 2 + (q[:, 0] - c[0]) * sc; Y = lambda q: h / 2 - (q[:, 1] - c[1]) * sc
    pts = lambda q: " ".join(f"{x:.1f},{y:.1f}" for x, y in zip(X(q), Y(q)))
    def one(i0, i1, title):
        r, a = pr[i0:i1 + 1], pa[i0:i1 + 1]; st = max(1, len(r) // 250)
        s = f'<div style="flex:1"><div style="font-weight:700;margin-bottom:6px">{title}</div><svg width="{w}" height="{h}"><rect width="{w}" height="{h}" rx="10" fill="#efeee9"/>'
        s += f'<polyline points="{pts(r[::st])}" fill="none" stroke="{REAL}" stroke-width="3.5" stroke-linejoin="round"/>'
        s += f'<polyline points="{pts(a[::st])}" fill="none" stroke="{OV}" stroke-width="3.5" stroke-linejoin="round"/>'
        for q, col in [(r, REAL), (a, OV)]:
            s += f'<circle cx="{X(q[:1])[0]:.1f}" cy="{Y(q[:1])[0]:.1f}" r="6" fill="#fff" stroke="{col}" stroke-width="3"/>'
            s += f'<rect x="{X(q[-1:])[0]-6:.1f}" y="{Y(q[-1:])[0]-6:.1f}" width="12" height="12" fill="{col}" stroke="#fff" stroke-width="2"/>'
        s += f'<circle cx="{X(pr[:1])[0]:.1f}" cy="{Y(pr[:1])[0]:.1f}" r="4" fill="{INK}"/>'
        s += f'<line x1="{w-16-20*sc:.1f}" y1="{h-16}" x2="{w-16}" y2="{h-16}" stroke="{INK2}" stroke-width="2"/>'
        s += f'<text x="{w-16-10*sc:.1f}" y="{h-23}" font-size="12" fill="{INK2}" text-anchor="middle">20 m</text></svg></div>'
        return s
    return (f'<div style="display:flex;gap:14px">{one(0, k, "Out: take-off → waypoint (107 m)")}'
            f'{one(k, len(pr) - 1, "Back: waypoint → landing")}</div>')

# ================================================================== PAGE 1: the latest flight
p1 = f"""
<div class="kicker">VPS · Flight test · 26 October 2026</div>
<h1>Latest flight: the best result so far</h1>
<div class="sub">One automatic mission, flown at 11:04–11:06: take-off, a straight ~107 m leg to a waypoint, then
return-to-launch. 245 m flown in 73 s, top speed 10 m/s. The camera system (OpenVINS) was recorded and
compared against the flight controller's GPS position.</div>

<div class="sec row">
  <div class="card tile" style="border-top:5px solid {GOOD}"><div class="big" style="color:{GOOD}">4.7 %</div>
    <div class="lab">horizontal drift: 11.5 m off at landing after 245 m</div></div>
  <div class="card tile" style="border-top:5px solid {GOOD}"><div class="big" style="color:{GOOD}">~10 m</div>
    <div class="lab">off after 100 m, and it <b>stopped growing</b> over the next 145 m</div></div>
  <div class="card tile" style="border-top:5px solid {GOOD}"><div class="big" style="color:{GOOD}">0 stops</div>
    <div class="lab">OpenVINS ran the whole flight: 15 positions/s, no gaps, clean shutdown</div></div>
  <div class="card tile" style="border-top:5px solid {GOOD}"><div class="big" style="color:{GOOD}">1.00</div>
    <div class="lab">scale: distances measured exactly right</div></div>
</div>

<div class="sec row">
  <div class="card" style="flex:1.15"><h2>Seen from above</h2>{leg_maps("26 Oct", 268, 300)}
    <div class="small" style="margin-top:10px"><svg width="30" height="10"><line x1="2" y1="5" x2="28" y2="5" stroke="{REAL}" stroke-width="4"/></svg> real path (flight controller GPS)
    &nbsp; <svg width="30" height="10"><line x1="2" y1="5" x2="28" y2="5" stroke="{OV}" stroke-width="4"/></svg> OpenVINS ·
    ○ start of leg, ■ end of leg, black dot = take-off point. The drone flew back along the same line (within 0.7 m) and
    landed 0.8 m from take-off; OpenVINS ended 10.7 m away.</div></div>
  <div class="card"><h2>How it went</h2><ul class="plain">
    <li>{ic('good')}OpenVINS drew the <b>right shape and the right length</b>: a straight line out and back.</li>
    <li>{ic('warn')}Its line is turned by <b>~6°</b>, so it was ~12 m to the side at the far end, and partly came back on the return.</li>
    <li>{ic('warn')}Almost all the error built up on the <b>fast outbound stretch over plain grass</b> (~9 m/s).</li>
    <li>{ic('good')}Over the textured ground at the far end and on the way back, the error <b>stopped growing</b>.</li>
    <li>{ic('good')}OpenVINS was <b>more honest about its own accuracy</b>: true error inside its uncertainty 41 % of the time (10 Oct: 5–10 %).</li>
  </ul></div>
</div>

<div class="sec banner"><h2>⚠ GPS was being blocked during this test</h2>
<div style="font-size:17px;line-height:1.5">Satellite signals were disturbed by artillery testing nearby. The camera system does not use GPS
and was unaffected, but the <b>GPS position we measure it against was about half as accurate as on 10 Oct</b>,
so part of the error above is GPS error, not camera error. The real drift was likely <b>lower than 4.7 %</b>.
Details on page 4.</div></div>
{FOOT}"""
page("01_latest_flight", p1, 1240)

# ================================================================== PAGE 2: comparison with 10 Oct
rows = [("10 Oct #1", "4.7 m high, fast, rectangle", "71 %", "125 m", "78 m", "full flight", "2.4 m", "356"),
        ("10 Oct #2", "slow straight line", "9.3 %*", "7.8 m", "103 m", "stopped at 78 s", "2.2 m", "259"),
        ("10 Oct #3", "zig-zag + hovers", "75 %*", "33 m", "95 m", "stopped at 91 s", "3.5 m", "172"),
        ("10 Oct #4", "short out and back", "12.9 %", "9.1 m", "91 m", "stopped 4 s early", "3.9 m", "32"),
        ("26 Oct", "straight out and back", "4.7 %", "8.6 m", "102 m, then flat", "full flight", "4.3 m", "400")]
tb = "".join(f"<tr class='{'hl' if r[0]=='26 Oct' else ''}'>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows)
W_, H_, ml, mb, mt, mr = 1040, 360, 56, 46, 24, 170; xmax, ymax = 250, 40
X = lambda v: ml + v / xmax * (W_ - ml - mr); Y = lambda v: H_ - mb - min(v, ymax) / ymax * (H_ - mb - mt)
ch = f'<svg width="{W_}" height="{H_}">'
for yv in range(0, ymax + 1, 10):
    ch += f'<line x1="{ml}" x2="{W_-mr}" y1="{Y(yv)}" y2="{Y(yv)}" stroke="{LINE}"/><text x="{ml-10}" y="{Y(yv)+5}" font-size="13" fill="{MUTED}" text-anchor="end">{yv} m</text>'
for xv in range(0, xmax + 1, 50):
    ch += f'<text x="{X(xv)}" y="{H_-mb+22}" font-size="13" fill="{MUTED}" text-anchor="middle">{xv} m</text>'
ch += f'<text x="{(ml+W_-mr)/2}" y="{H_-4}" font-size="14" fill="{INK2}" text-anchor="middle">distance flown since take-off</text>'
lab_dy = {"10 Oct #1": 0, "10 Oct #2": -16, "10 Oct #3": 0, "10 Oct #4": -4, "26 Oct": 0}
for k, (lab, _) in enumerate(FL):
    d, e = D[lab]["dist"], D[lab]["eh"]; keep = d <= xmax; dd, ee = d[keep], e[keep]
    over = np.where(ee > ymax)[0]; off = len(over) > 0
    if off: j = over[0]; dd, ee = np.r_[dd[:j], dd[j]], np.r_[ee[:j], ymax]
    sw = 5 if lab == "26 Oct" else 2.5
    ch += f'<polyline points="{" ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(dd[::3].tolist()+[dd[-1]], ee[::3].tolist()+[ee[-1]]))}" fill="none" stroke="{FC[k]}" stroke-width="{sw}" stroke-linejoin="round"/>'
    lx, ly = dd[-1], ee[-1]
    txt = {"10 Oct #1": "10 Oct #1 ↑ 289 m", "10 Oct #3": "10 Oct #3 ↑ 105 m"}.get(lab, lab)
    if off:
        ch += f'<polygon points="{X(lx)-7:.1f},{Y(ly)+4:.1f} {X(lx)+7:.1f},{Y(ly)+4:.1f} {X(lx):.1f},{Y(ly)-8:.1f}" fill="{FC[k]}"/>'
        ch += f'<text x="{X(lx)+(12 if lab=="10 Oct #1" else -12):.1f}" y="{Y(ly)+5:.1f}" font-size="14" font-weight="700" fill="{INK}" text-anchor="{"start" if lab=="10 Oct #1" else "end"}">{txt}</text>'
    else:
        ch += f'<circle cx="{X(lx):.1f}" cy="{Y(ly):.1f}" r="6" fill="{FC[k]}" stroke="{CARD}" stroke-width="2"/>'
        ch += f'<text x="{X(lx)+11:.1f}" y="{Y(ly)+5+lab_dy[lab]:.1f}" font-size="14" font-weight="700" fill="{INK}">{txt}</text>'
ch += "</svg>"
p2 = f"""
<div class="kicker">Compared with the 10 October flights</div>
<h1>Same software, much better result</h1>
<div class="sub">OpenVINS settings, calibration and camera mount were identical on both days. Everything that improved
came from how and where we flew, and from the new field routine.</div>

<div class="sec card"><h2>Side by side (horizontal error only)</h2>
<table><tr><th>Flight</th><th>Route</th><th>Drift (end error ÷ path)</th><th>Average error</th><th>10 m off after</th>
<th>OpenVINS ran</th><th>GPS accuracy in flight</th><th>Texture (corners/image)</th></tr>{tb}</table>
<div class="small" style="margin-top:8px">* measured where OpenVINS stopped, not at landing. On an out-and-back route a heading error partly cancels
on the way back; at the far end of its leg the 26 Oct flight was ~12 m off after ~107 m.</div></div>

<div class="sec card"><h2>Error grows with distance flown, then levels off on 26 Oct</h2>{ch}
<div class="small">How far OpenVINS was from the GPS position (horizontal, up to 40 m shown). 26 Oct follows the best 10 Oct flight for the
first 100 m, then stays flat for the next 145 m.</div></div>

<div class="sec card"><h2>What changed between the tests</h2>
<table><tr><th style="width:26%">Change</th><th style="width:30%">10 Oct → 26 Oct</th><th>Why it helped</th></tr>
<tr><td><b>More texture on the ground</b></td><td>32–356 → <b>400</b> corners per image; worst 10 % of frames 5–87 → <b>167</b></td><td>More distinct points to lock on to. The error stopped growing over the textured far end.</td></tr>
<tr><td><b>Simpler route</b></td><td>rectangles, zig-zags, hovers → <b>one straight leg out and back</b></td><td>Few turns, no long hovers over plain ground (where flight #3 failed).</td></tr>
<tr><td><b>Ground moved slower in the image</b></td><td>up to 167 px between frames → <b>13 px</b></td><td>Features stay in view and sharp long enough to be followed (sharpness 46–92 → 133).</td></tr>
<tr><td><b>Shorter time in the air</b></td><td>up to 209 s → <b>73 s</b></td><td>The IMU part of the estimate drifts with time as well as distance.</td></tr>
<tr><td><b>New one-command field routine</b></td><td>OpenVINS stopped in 3 of 4 flights → <b>no stop</b></td><td>Programs now run detached from the Wi-Fi/SSH link, with logs and crash capture.</td></tr>
<tr><td><b>~2 min still before take-off</b></td><td>shorter / varied → <b>~2 min</b></td><td>Time for OpenVINS to measure the IMU's small offsets while the drone is still.</td></tr>
<tr><td><b>Slightly less vibration</b></td><td>camera 9.5–10 → <b>7.6</b> m/s² (average)</td><td>Cleaner IMU readings.</td></tr>
</table></div>
{FOOT}"""
page("02_compared_to_10_oct", p2, 1900)

# ================================================================== PAGE 3: images
p3 = f"""
<div class="kicker">What the camera saw</div>
<h1>Best and worst moments</h1>
<div class="sub">Each frame is labelled with time after take-off, speed, number of trackable corners, and how fast the horizontal
error was growing at that moment.</div>

<div class="sec card"><h2>26 October</h2>
<img src="{HERE}/t5_best_worst.png" style="width:100%;border-radius:8px">
<ul class="plain" style="margin-top:10px">
<li>{ic('bad')}<b>Worst (top):</b> the fast outbound stretch at ~9 m/s. The error grew ~0.9 m every second there, both over uniform grass
(176 corners: from 14 m up it blurs into an even grey) and over more varied ground (638 corners). Speed matters as well as texture;
part of this growth is also the ~6° heading turn, which adds more sideways error per second the faster the drone flies.</li>
<li>{ic('good')}<b>Best (bottom):</b> slow passes over varied ground: the dry channel at the far end (616 corners) and close-up grass on
the descent (1454 corners). The error barely grew.</li>
<li>{ic('info')}No over-exposure; people and their shadows near the take-off point are visible in some frames.</li></ul></div>

<div class="sec card"><h2>10 October, for comparison</h2>
<img src="{A10}/worst_vs_best_frames.png" style="width:100%;border-radius:8px">
<div class="small" style="margin-top:8px">Top: worst moments (fast and low over tyre tracks; bare earth with almost nothing to track). Bottom: best moments.
On 26 Oct even the worst frames show more texture than the bare ground of 10 Oct.</div></div>

<div class="sec card"><h2>Texture and image quality in the air</h2>
<table><tr><th>Flight</th><th>Corners per image (median)</th><th>Worst 10 % of frames</th><th>Frames under 300 corners</th><th>Sharpness</th><th>Ground motion between frames (max)</th></tr>
<tr><td>10 Oct #1</td><td>356</td><td>87</td><td>39 %</td><td>92</td><td>167 px</td></tr>
<tr><td>10 Oct #2</td><td>259</td><td>22</td><td>55 %</td><td>84</td><td>130 px</td></tr>
<tr><td>10 Oct #3</td><td>172</td><td>30</td><td>69 %</td><td>79</td><td>17 px</td></tr>
<tr><td>10 Oct #4</td><td>32</td><td>5</td><td>77 %</td><td>46</td><td>18 px</td></tr>
<tr class="hl"><td>26 Oct</td><td>400</td><td>167</td><td>39 %</td><td>133</td><td>13 px</td></tr></table></div>
{FOOT}"""
page("03_camera_images", p3, 2040)

# ================================================================== PAGE 4: GPS interference
L = np.load(f"{HERE}/t5_log.npz", allow_pickle=True); M = json.load(open(f"{HERE}/t5_metrics.json"))
t_to = L["XKF1"][0, 0] + M["t_takeoff"]; G = L["GPA"]; N = L["GPS"]
gt, ha = G[:, 0] - t_to, G[:, 1]; nt, ns = N[:, 0] - t_to, N[:, 2]
W2, H2, ml2, mb2, mt2, mr2 = 1040, 340, 56, 46, 30, 30; x0, x1, ymax2 = -200, 200, 10
X2 = lambda v: ml2 + (v - x0) / (x1 - x0) * (W2 - ml2 - mr2); Y2 = lambda v: H2 - mb2 - min(v, ymax2) / ymax2 * (H2 - mb2 - mt2)
gc = f'<svg width="{W2}" height="{H2}">'
gc += f'<rect x="{X2(0):.1f}" y="{mt2}" width="{X2(73)-X2(0):.1f}" height="{H2-mb2-mt2}" fill="#e8f4e8"/>'
gc += f'<text x="{(X2(0)+X2(73))/2:.1f}" y="{mt2+18}" font-size="14" font-weight="700" fill="{GOOD}" text-anchor="middle">IN THE AIR</text>'
for yv in range(0, ymax2 + 1, 2):
    gc += f'<line x1="{ml2}" x2="{W2-mr2}" y1="{Y2(yv)}" y2="{Y2(yv)}" stroke="{LINE}"/><text x="{ml2-10}" y="{Y2(yv)+5}" font-size="13" fill="{MUTED}" text-anchor="end">{yv} m</text>'
for xv in range(-200, 201, 50):
    gc += f'<text x="{X2(xv):.1f}" y="{H2-mb2+22}" font-size="13" fill="{MUTED}" text-anchor="middle">{xv:+d} s</text>'
gc += f'<text x="{(ml2+W2-mr2)/2}" y="{H2-4}" font-size="14" fill="{INK2}" text-anchor="middle">time from take-off</text>'
gc += f'<line x1="{ml2}" x2="{W2-mr2}" y1="{Y2(2.3)}" y2="{Y2(2.3)}" stroke="{FC[0]}" stroke-width="2" stroke-dasharray="6 5"/>'
gc += f'<text x="{W2-mr2-6}" y="{Y2(2.3)+18}" font-size="13" font-weight="700" fill="{INK}" text-anchor="end">10 Oct best flights ≈ 2.3 m</text>'
m = np.where((gt >= x0) & (gt <= x1))[0]
for seg in np.split(m, np.where(np.diff(gt[m]) > 5)[0] + 1):
    gc += f'<polyline points="{" ".join(f"{X2(a):.1f},{Y2(b):.1f}" for a, b in zip(gt[seg], ha[seg]))}" fill="none" stroke="{BAD}" stroke-width="3.5"/>'
gc += f'<text x="{X2(-115):.1f}" y="{Y2(1.2):.1f}" font-size="12" fill="{MUTED}" text-anchor="middle">(no GPS log in this gap)</text>'
for xs, txt in [(-150, "7–13 satellites, worsening"), (-30, "12–14"), (35, "12–13"), (96, "6")]:
    gc += f'<text x="{X2(xs):.1f}" y="{Y2(np.interp(xs, gt, ha) if xs > -100 else 8.8)-12:.1f}" font-size="13" font-weight="700" fill="{INK}" text-anchor="middle">{txt}</text>'
for xs, txt in [(80, "GPS Glitch"), (113, "GPS vs AHRS 30 m apart")]:
    gc += f'<line x1="{X2(xs):.1f}" x2="{X2(xs):.1f}" y1="{mt2+26}" y2="{H2-mb2}" stroke="{BAD}" stroke-dasharray="3 4"/>'
    gc += f'<text x="{X2(xs)+5:.1f}" y="{mt2+40 + (16 if xs > 100 else 0)}" font-size="12" fill="{BAD}">{txt}</text>'
gc += "</svg>"
p4 = f"""
<div class="kicker">GPS interference on 26 October</div>
<h1>GPS blocking made our measured error worse</h1>
<div class="sub">Artillery testing nearby was disturbing satellite signals. OpenVINS does not use GPS, so the camera system itself
was not affected. But we judge OpenVINS <b>against</b> the GPS position, and that yardstick was degraded the whole time.</div>

<div class="sec card"><h2>How accurate the GPS said it was (lower is better)</h2>{gc}
<div class="small">Red line: the GPS receiver's own horizontal accuracy estimate. Dashed: typical on 10 Oct.</div></div>

<div class="sec row">
  <div class="card tile" style="border-top:5px solid {BAD}"><div class="big" style="color:{BAD}">8.3 m</div><div class="lab">before take-off the GPS accuracy got <b>worse</b>, from ~5 m to 8.3 m in 30 s (7–13 satellites)</div></div>
  <div class="card tile" style="border-top:5px solid {WARN}"><div class="big" style="color:{WARN}">4.3 m</div><div class="lab">during the flight: about <b>2× worse</b> than 10 Oct's best flights (2.2–2.4 m)</div></div>
  <div class="card tile" style="border-top:5px solid {BAD}"><div class="big" style="color:{BAD}">30 m</div><div class="lab">after landing: satellites dropped from 13 to <b>6</b>, then GPS and the flight controller disagreed by 30 m, with repeated "GPS Glitch" alarms</div></div>
</div>

<div class="sec banner"><h2>What it means for the 4.7 %</h2><ul class="plain">
<li>{ic('bad')}The measured 11.5 m horizontal error includes the GPS's own error: with ±4.3 m uncertainty in flight, <b>up to about a third
of what we measured may be GPS error rather than camera error</b>. A noisy yardstick tends to make the measured error bigger, so with clean GPS the measured drift would likely
have been lower, possibly as low as ~3 %.</li>
<li>{ic('warn')}The clearest sign of blocking came right after landing: a receiver that had locked on to 13 satellites dropped to 6, and the
"GPS Glitch" alarms started 7 s after touchdown. During the flight itself the GPS held 12–13 satellites but stayed about 2× less accurate than on 10 Oct.</li>
<li>{ic('good')}<b>The camera system kept working normally throughout</b>, exactly the situation a GPS-free positioning system is meant for.</li>
</ul></div>
{FOOT}"""
page("04_gps_interference", p4, 1580)

# ================================================================== PAGE 5: conclusion
p5 = f"""
<div class="kicker">Conclusion</div>
<h1>26 October in one page</h1>
<div class="sec card"><ul class="plain" style="font-size:18px">
<li>{ic('good')}<b>Best result so far:</b> 4.7 % horizontal drift over a 245 m out-and-back (10 Oct: 9.3–75 %). About 10 m off after 100 m,
and the error then <b>stopped growing</b> for the rest of the flight.</li>
<li>{ic('good')}<b>No software was changed.</b> The improvement came from flying conditions (more textured ground, a simple straight route,
slower image motion, a short flight) and from the new field routine.</li>
<li>{ic('good')}<b>OpenVINS ran the entire flight without stopping</b>, for the first time on a real flight. On 10 Oct it stopped in 3 of 4 flights.</li>
<li>{ic('good')}Distances were measured exactly right (scale 1.00), and OpenVINS was more honest about its own uncertainty (41 % vs 5–10 %).</li>
<li>{ic('bad')}<b>GPS interference degraded our yardstick</b>: about half as accurate as on 10 Oct during the flight, and much worse before and after it.
Part of the measured error is GPS error; the true drift was likely lower than 4.7 %.</li>
<li>{ic('warn')}The weak spots are unchanged: error builds up when flying fast over plain grass, and OpenVINS's heading slowly turns
(~6° here), which is most of the remaining horizontal error.</li>
<li>{ic('warn')}It is <b>one flight</b> on an out-and-back route, which flatters the end-point number: at the far end of the leg the error was ~12 m after ~107 m.</li>
</ul></div>
<div class="sec verdict">The camera system tracked a fast 245 m flight to within ~10 m, kept running all the way, and was not affected by
the GPS blocking that degraded the satellite positions around it.</div>
{FOOT}"""
page("05_conclusion", p5, 1010)
