#!/usr/bin/env python3
"""Build the 10 Oct 2026 flight-test infographic: 5 HTML pages -> PNG (headless Firefox)."""
import os, subprocess, numpy as np

HERE = os.getcwd()                    # run from the analysis folder holding tN_series.npz, worst_vs_best_frames.png
OUT = os.environ.get("INFOGRAPHIC_OUT", os.path.expanduser("~/Desktop/VPS_flight_test_report_2026-10-10"))
os.makedirs(OUT, exist_ok=True)
os.makedirs(os.path.expanduser("~/ffshot_profile"), exist_ok=True)   # headless-Firefox profile (snap Firefox cannot use hidden dirs)

INK, INK2, MUTED, LINE, SURF, CARD = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#f4f3ef", "#fcfcfb"
REAL, OV = "#3b3a37", "#eb6834"                       # maps: real path / OpenVINS
FC = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]      # flights 1-4 (palette slots 1-4, fixed order)
GOOD, WARN, BAD = "#0ca30c", "#c98500", "#d03b3b"

CSS = f"""
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: {SURF}; color: {INK}; font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; width: 1200px; padding: 44px 48px; }}
.kicker {{ font-size: 15px; letter-spacing: .12em; text-transform: uppercase; color: {INK2}; font-weight: 700; }}
h1 {{ font-size: 44px; line-height: 1.12; margin: 6px 0 8px; letter-spacing: -.01em; }}
h2 {{ font-size: 24px; margin: 0 0 14px; }}
.sub {{ font-size: 19px; color: {INK2}; max-width: 980px; line-height: 1.45; }}
.row {{ display: flex; gap: 18px; }}
.card {{ background: {CARD}; border: 1px solid {LINE}; border-radius: 14px; padding: 22px 24px; flex: 1; }}
.tile .big {{ font-size: 46px; font-weight: 700; line-height: 1; }}
.tile .lab {{ font-size: 16px; color: {INK2}; margin-top: 10px; line-height: 1.4; }}
.sec {{ margin-top: 30px; }}
ul.plain {{ list-style: none; }}
ul.plain li {{ font-size: 17px; line-height: 1.45; padding: 7px 0 7px 30px; position: relative; }}
ul.plain li .ic {{ position: absolute; left: 0; top: 7px; width: 21px; height: 21px; border-radius: 50%;
                   color: #fff; font-size: 13px; font-weight: 700; display: flex; align-items: center; justify-content: center; }}
.verdict {{ margin-top: 30px; background: {INK}; color: #fff; border-radius: 14px; padding: 22px 28px; font-size: 21px; line-height: 1.4; }}
.verdict b {{ color: #ffd58a; }}
.foot {{ margin-top: 24px; font-size: 13px; color: {MUTED}; }}
.pill {{ display: inline-block; font-size: 13px; font-weight: 700; padding: 3px 10px; border-radius: 99px; color: #fff; }}
.small {{ font-size: 15px; color: {INK2}; line-height: 1.45; }}
.num {{ font-weight: 700; }}
"""
def ic(kind):
    c, s = {"good": (GOOD, "✓"), "bad": (BAD, "✕"), "warn": (WARN, "!"), "info": (INK2, "•")}[kind]
    return f'<span class="ic" style="background:{c}">{s}</span>'
def page(name, body, height):
    html = f"<!doctype html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>{body}</body></html>"
    p = os.path.join(OUT, name + ".html"); open(p, "w").write(html)
    png = os.path.join(OUT, name + ".png")
    subprocess.run(["timeout", "120", "firefox", "--headless", "--no-remote", "--profile", os.path.expanduser("~/ffshot_profile"),
                    "--screenshot", png, f"--window-size=1200,{height}", "file://" + p],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("wrote", png)
FOOT = '<div class="foot">VPS benchmarking · Jetson "vajra" + Orbbec Gemini 335L · OpenVINS · reference = flight controller GPS (accurate to ~2–5 m)</div>'

# ---------------------------------------------------------------- data
S = {n: np.load(os.path.join(HERE, f"{n}_series.npz")) for n in ["t1", "t2", "t3", "t4"]}

def mini_map(n, w=500, h=300, clip=None):
    d = S[n]; pr, pa = d["pr"][:, :2], d["pa"][:, :2]
    ext = pr if clip else np.vstack([pr, pa])
    lo, hi = ext.min(0), ext.max(0); span = (hi - lo).max() * (1.25 if not clip else 1.6) + 1
    c = (lo + hi) / 2
    sc = min(w, h) / span
    X = lambda p: w / 2 + (p[:, 0] - c[0]) * sc
    Y = lambda p: h / 2 - (p[:, 1] - c[1]) * sc
    pts = lambda p: " ".join(f"{x:.1f},{y:.1f}" for x, y in zip(X(p), Y(p)))
    step = max(1, len(pr) // 400)
    s = f'<svg width="{w}" height="{h}" viewBox="0 0 {w} {h}"><defs><clipPath id="c{n}"><rect width="{w}" height="{h}" rx="10"/></clipPath></defs>'
    s += f'<rect width="{w}" height="{h}" rx="10" fill="#efeee9"/><g clip-path="url(#c{n})">'
    s += f'<polyline points="{pts(pr[::step])}" fill="none" stroke="{REAL}" stroke-width="3" stroke-linejoin="round"/>'
    s += f'<polyline points="{pts(pa[::step])}" fill="none" stroke="{OV}" stroke-width="3" stroke-linejoin="round"/>'
    s += f'<circle cx="{X(pr[:1])[0]:.1f}" cy="{Y(pr[:1])[0]:.1f}" r="7" fill="{INK}" stroke="#fff" stroke-width="2.5"/></g>'
    # 10 m / 50 m scale bar
    L = 50 if span > 150 else (20 if span > 60 else 10)
    s += f'<line x1="{w-20-L*sc:.1f}" y1="{h-18}" x2="{w-20}" y2="{h-18}" stroke="{INK2}" stroke-width="2"/>'
    s += f'<text x="{w-20-L*sc/2:.1f}" y="{h-25}" font-size="12" fill="{INK2}" text-anchor="middle">{L} m</text>'
    if clip:
        s += f'<text x="14" y="{h-16}" font-size="13" fill="{OV}" font-weight="700">{clip}</text>'
    return s + "</svg>"

# =============================================================== PAGE 1
p1 = f"""
<div class="kicker">VPS · Dynamic flight test · 10 October 2026</div>
<h1>First real flights of the camera-based positioning system</h1>
<div class="sub">Four automatic GPS missions with the camera looking down. We recorded everything and compared what
OpenVINS (the camera-based position estimate) said against where the drone really was.</div>

<div class="sec row">
  <div class="card tile"><div class="big">4</div><div class="lab">flights, all recorded cleanly<br>(24 GB of camera + motion data)</div></div>
  <div class="card tile"><div class="big">1.2 km</div><div class="lab">flown in total,<br>about 9 minutes in the air</div></div>
  <div class="card tile"><div class="big">5–9 m</div><div class="lab">height above ground,<br>camera pointing down</div></div>
  <div class="card tile"><div class="big">10 m/s</div><div class="lab">top speed<br>(about 36 km/h)</div></div>
</div>

<div class="sec"><h2>The headline</h2>
<div class="row">
  <div class="card tile" style="border-top:5px solid {GOOD}"><div class="big" style="color:{GOOD}">0–15 s</div>
    <div class="lab">OpenVINS tracks the drone well at the start: right size, right direction, right tilt.</div></div>
  <div class="card tile" style="border-top:5px solid {WARN}"><div class="big" style="color:{WARN}">50–100 m</div>
    <div class="lab">… then it slowly loses its place. It is about <b>10 m off</b> after 50–100 m of flying.</div></div>
  <div class="card tile" style="border-top:5px solid {BAD}"><div class="big" style="color:{BAD}">3 of 4</div>
    <div class="lab">flights where OpenVINS <b>stopped working mid-flight</b>. Cause not yet known.</div></div>
</div></div>

<div class="sec row">
  <div class="card"><h2>What works</h2><ul class="plain">
    <li>{ic('good')}The whole recording setup: every flight is saved and can be replayed on a laptop.</li>
    <li>{ic('good')}OpenVINS knows which way is up and which way it is facing (within a few degrees).</li>
    <li>{ic('good')}Its sense of distance is right at the start (within ~10%).</li>
    <li>{ic('good')}The Jetson computer keeps up easily, so no faster computer is needed.</li>
  </ul></div>
  <div class="card"><h2>What doesn't (yet)</h2><ul class="plain">
    <li>{ic('bad')}Its position drifts away after 50–100 m.</li>
    <li>{ic('bad')}Height is the weakest: one flight thought it was 255 m underground.</li>
    <li>{ic('bad')}It is over-confident: it thinks it is accurate when it isn't.</li>
    <li>{ic('bad')}It stopped mid-flight in 3 of 4 flights.</li>
  </ul></div>
</div>

<div class="verdict">Verdict: <b>not flight-ready yet</b>, but we now know <b>why</b> it drifts (flying too fast for the
height, and plain ground with nothing to track). Software fixes are already in place, and the next tests are planned.</div>
{FOOT}"""
page("01_overview", p1, 1250)

# =============================================================== PAGE 2
cards = [
 ("t1", "Flight 1", "Low (5 m) and fast (up to 10 m/s)", BAD, "Lost",
  [("10 m off after", "58 m"), ("Ended", "390 m off")], "Full flight",
  "Too fast for the height: the ground rushed past the camera. Lost its place within 15 seconds.", "OpenVINS ran off to 390 m →"),
 ("t2", "Flight 2", "9 m high, slow (up to 4 m/s), straight", GOOD, "Best",
  [("10 m off after", "101 m"), ("Average error", "8 m")], "Stopped at 78 s of 213 s",
  "The easiest flying of the day. Best result, but OpenVINS stopped before the turns.", None),
 ("t3", "Flight 3", "9 m high, up to 7 m/s, zig-zag with hovers", WARN, "Mixed",
  [("10 m off after", "95 m"), ("Ended", "105 m off")], "Stopped at 91 s of 182 s",
  "Good for ~55 s, then drifted while hovering over plain, bare ground. GPS also glitched.", None),
 ("t4", "Flight 4", "9 m high, up to 7 m/s, out and back", WARN, "Fair",
  [("10 m off after", "91 m"), ("Average error", "8 m")], "Full flight (last 4 s missing)",
  "Decent on the way out; lost its way on the return (GPS also glitched there).", None),
]
cc = ""
for n, title, cond, col, tag, stats, cover, take, clip in cards:
    st = "".join(f'<div style="flex:1"><div class="small">{a}</div><div class="num" style="font-size:26px">{b}</div></div>' for a, b in stats)
    cc += f"""<div class="card" style="flex: 0 0 calc(50% - 9px)">
      <div style="display:flex;justify-content:space-between;align-items:center"><h2 style="margin:0">{title}</h2>
      <span class="pill" style="background:{col}">{tag}</span></div>
      <div class="small" style="margin:4px 0 12px">{cond}</div>
      {mini_map(n, 492, 260, clip)}
      <div style="display:flex;gap:12px;margin-top:14px">{st}<div style="flex:1.2"><div class="small">OpenVINS ran</div><div class="num" style="font-size:17px;margin-top:6px">{cover}</div></div></div>
      <div style="font-size:16px;line-height:1.45;margin-top:12px">{take}</div></div>"""
p2 = f"""
<div class="kicker">Flight by flight</div>
<h1>Where the drone really went vs. where OpenVINS thought it went</h1>
<div class="sub" style="margin-bottom:12px">Seen from above. Black dot = take-off.
<svg width="34" height="10"><line x1="2" y1="5" x2="32" y2="5" stroke="{REAL}" stroke-width="4"/></svg> real path (flight controller GPS)
&nbsp;&nbsp;<svg width="34" height="10"><line x1="2" y1="5" x2="32" y2="5" stroke="{OV}" stroke-width="4"/></svg> OpenVINS</div>
<div class="sec" style="display:flex;flex-wrap:wrap;gap:18px">{cc}</div>
<div class="small" style="margin-top:16px">"10 m off after" = distance flown before OpenVINS was 10 m away from the truth. One damaged pre-flight
recording (Jetson lost power before take-off) is not counted.</div>
{FOOT}"""
page("02_flights", p2, 1450)

# =============================================================== PAGE 3
# error vs distance chart
W_, H_, ml, mb, mt, mr = 1040, 340, 56, 44, 22, 200
xmax, ymax = 140, 40
X = lambda v: ml + v / xmax * (W_ - ml - mr)
Y = lambda v: H_ - mb - min(v, ymax) / ymax * (H_ - mb - mt)
ch = f'<svg width="{W_}" height="{H_}">'
for yv in range(0, ymax + 1, 10):
    ch += f'<line x1="{ml}" x2="{W_-mr}" y1="{Y(yv)}" y2="{Y(yv)}" stroke="{LINE}"/><text x="{ml-10}" y="{Y(yv)+5}" font-size="13" fill="{MUTED}" text-anchor="end">{yv} m</text>'
for xv in range(0, xmax + 1, 20):
    ch += f'<text x="{X(xv)}" y="{H_-mb+22}" font-size="13" fill="{MUTED}" text-anchor="middle">{xv} m</text>'
ch += f'<text x="{(ml+W_-mr)/2}" y="{H_-4}" font-size="14" fill="{INK2}" text-anchor="middle">distance flown since take-off</text>'
for k, n in enumerate(["t1", "t2", "t3", "t4"]):
    d = S[n]; m = d["t"] >= 0
    dist = d["dist"][m] - d["dist"][m][0]; e = d["err"][m]
    keep = dist <= xmax; dd, ee = dist[keep], e[keep]
    over = np.where(ee > ymax)[0]
    off_chart = len(over) > 0
    if off_chart:                                  # stop the line where it leaves the chart
        j = over[0]; dd, ee = np.r_[dd[:j], dd[j]], np.r_[ee[:j], ymax]
    ch += f'<polyline points="{" ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(dd[::3].tolist() + [dd[-1]], ee[::3].tolist() + [ee[-1]]))}" fill="none" stroke="{FC[k]}" stroke-width="3" stroke-linejoin="round"/>'
    lx, ly = dd[-1], ee[-1]
    lab = {"t1": "Flight 1 ↑ ends 390 m off", "t2": "Flight 2 (stopped)", "t3": "Flight 3 ↑ ends 105 m off", "t4": "Flight 4"}[n]
    if off_chart:
        ch += f'<polygon points="{X(lx)-7:.1f},{Y(ly)+4:.1f} {X(lx)+7:.1f},{Y(ly)+4:.1f} {X(lx):.1f},{Y(ly)-8:.1f}" fill="{FC[k]}"/>'
        ch += f'<text x="{X(lx):.1f}" y="{Y(ly)+5:.1f}" font-size="14" font-weight="700" fill="{INK}" text-anchor="{"start" if n == "t1" else "end"}" dx="{12 if n == "t1" else -12}">{lab}</text>'
    else:
        ch += f'<circle cx="{X(lx):.1f}" cy="{Y(ly):.1f}" r="5" fill="{FC[k]}" stroke="{CARD}" stroke-width="2"/>'
        ch += f'<text x="{X(lx)+10:.1f}" y="{Y(ly)+5:.1f}" font-size="14" font-weight="700" fill="{INK}">{lab}</text>'
ch += "</svg>"

# speed / height rule
rule = ""
for hgt, vmax in [(5, 1.7), (9, 3.0), (15, 5.0)]:
    wv = vmax / 11 * 640
    rule += f"""<div style="display:flex;align-items:center;gap:14px;margin:9px 0">
      <div style="width:110px;font-size:17px;font-weight:700">{hgt} m high</div>
      <div style="width:640px;height:26px;background:#efeee9;border-radius:6px;position:relative">
        <div style="width:{wv:.0f}px;height:26px;background:{GOOD};border-radius:6px"></div></div>
      <div style="font-size:17px">up to <b>{vmax:g} m/s</b></div></div>"""
rule += f"""<div style="display:flex;align-items:center;gap:14px;margin:9px 0">
      <div style="width:110px;font-size:17px;font-weight:700;color:{BAD}">Flight 1</div>
      <div style="width:640px;height:26px;background:#efeee9;border-radius:6px;position:relative">
        <div style="width:{10.4/11*640:.0f}px;height:26px;background:{BAD};border-radius:6px"></div></div>
      <div style="font-size:17px;color:{BAD}"><b>10 m/s at 5 m</b> (6× too fast)</div></div>"""

p3 = f"""
<div class="kicker">What makes it drift</div>
<h1>Two things throw OpenVINS off</h1>
<div class="sub">We cut the flights into 5-second slices and compared the worst slices with the best ones.</div>

<div class="sec row">
  <div class="card" style="border-top:5px solid {BAD}"><h2>1 · Ground moving too fast in the picture</h2>
    <div style="font-size:17px;line-height:1.5">Flying <b>low and fast</b> (flight 1: 5 m high at 10 m/s) makes the ground streak across the
    camera. Features jumped ~40 pixels between frames, about <b>10× more</b> than in the other flights, and the camera
    could no longer follow them. <b>Worst drift of the day.</b></div></div>
  <div class="card" style="border-top:5px solid {WARN}"><h2>2 · Plain ground with nothing to track</h2>
    <div style="font-size:17px;line-height:1.5">Over <b>smooth, bare earth</b> the camera finds very few distinct spots to lock on to,
    and drifts <b>even when hovering</b>. This is why flight 3 drifted while it was almost standing still.</div></div>
</div>

<div class="sec card"><h2>What the camera saw</h2>
  <img src="{HERE}/worst_vs_best_frames.png" style="width:100%;border-radius:8px">
  <div class="small" style="margin-top:10px">Top: bad moments. Flight 1 has plenty of texture but moves far too fast; the others are plain ground.
  Bottom: good moments. Slow flight at 9 m with tyre tracks or objects in view.</div></div>

<div class="sec card"><h2>Error grows with distance flown, in every flight</h2>{ch}
  <div class="small">How far OpenVINS was from the truth (up to 40 m shown). Over the first 100 m all flights end up 10–23 m off;
  flight 2 looks best mostly because it flew the easiest stretch.</div></div>

<div class="sec card"><h2>Rule of thumb: speed limit by height</h2>
  <div class="small" style="margin-bottom:6px">Every good slice kept speed ÷ height below about 0.35 per second. Estimated from a handful of slices; to be confirmed by flying.</div>
  {rule}</div>

<div class="sec card"><h2>What did <u>not</u> seem to matter</h2><ul class="plain">
  <li>{ic('info')}<b>Vibration:</b> the camera shakes about twice as much as the flight controller, but at 9 m it made no clear difference.</li>
  <li>{ic('info')}<b>Light:</b> no over-exposed images. The infrared camera copes with sunlight.</li>
  <li>{ic('info')}<b>Computer speed:</b> the Jetson kept up every time.</li>
</ul></div>
{FOOT}"""
page("03_what_causes_drift", p3, 2200)

# =============================================================== PAGE 4
def fix(title, before, after, status, scol):
    return f"""<div class="card" style="display:flex;gap:18px;align-items:flex-start;padding:18px 22px">
      <div style="flex:0 0 260px"><div style="font-size:19px;font-weight:700;line-height:1.3">{title}</div>
        <span class="pill" style="background:{scol};margin-top:10px">{status}</span></div>
      <div style="flex:1"><div class="small"><b style="color:{BAD}">Before:</b> {before}</div>
        <div class="small" style="margin-top:6px"><b style="color:{GOOD}">Now:</b> {after}</div></div></div>"""
p4 = f"""
<div class="kicker">Fixed today</div>
<h1>Problems we hit, and what we changed</h1>
<div class="sub">All changes are on the Jetson (saved, not yet uploaded to GitHub).</div>
<div class="sec" style="display:flex;flex-direction:column;gap:14px">
{fix("Jetson forgot the date", "Every restart reset its clock to 1970, which scrambled file dates and made the flight logs hard to match up.",
     "It remembers the time across restarts (tested twice). One command on the laptop sets the exact time.", "✓ Tested", GOOD)}
{fix("One-command flight routine", "Three terminals, many commands, files all named “flight1”, and a frozen screen whenever Wi-Fi dropped.",
     "<b>flight_start → flight_record → flight_stop</b>. Everything keeps running if Wi-Fi drops; each flight gets its own numbered folder.",
     "✓ Installed · camera test pending", WARN)}
{fix("OpenVINS stopping mid-flight", "Stopped in 3 of 4 flights with no record of why.",
     "Its output is logged per flight and a crash recorder is on, so next time we will see exactly what happened.", "✓ Ready", GOOD)}
{fix("OpenVINS crashing on shutdown", "Often crashed or hung when stopped.", "Stops cleanly in 0.1 seconds (3 of 3 tests).", "✓ Tested · camera test pending", WARN)}
{fix("Pre-flight check gave false alarms", "Warned about clock and missing camera topics that were actually fine.", "Fixed, and it now also checks the Jetson's clock.", "✓ Fixed", GOOD)}
</div>
<div class="sec card"><h2>Still to fix (needs hardware)</h2><ul class="plain">
  <li>{ic('bad')}The Jetson <b>lost power once</b> while the drone was being handled, ruining one recording. Its power connector needs securing.</li>
  <li>{ic('warn')}The Jetson's disk has room for only about <b>5–8 more flights</b>.</li>
</ul></div>
{FOOT}"""
page("04_fixed_today", p4, 1130)

# =============================================================== PAGE 5
steps = [("1", "Hover", "60 s at 9 m. Should stay within ~1 m."),
         ("2", "Slow line", "100 m out and back at 2 m/s, 9 m high."),
         ("3", "Faster", "Same line at 4 and 6 m/s."),
         ("4", "Other heights", "Repeat at 5 m and 15 m."),
         ("5", "Missions", "Full routes, inside the safe limits found.")]
lad = "".join(f"""<div style="flex:1;background:{CARD};border:1px solid {LINE};border-radius:12px;padding:16px;margin-top:{(4-i)*16}px">
  <div style="width:34px;height:34px;border-radius:50%;background:{FC[0]};color:#fff;font-weight:700;display:flex;align-items:center;justify-content:center">{a}</div>
  <div style="font-size:18px;font-weight:700;margin-top:10px">{b}</div><div class="small" style="margin-top:4px">{c}</div></div>""" for i, (a, b, c) in enumerate(steps))
def tier(label, col, items):
    return f"""<div class="card" style="border-left:6px solid {col}"><div style="font-size:15px;font-weight:700;color:{col};letter-spacing:.06em;text-transform:uppercase">{label}</div>
    <ul class="plain" style="margin-top:6px">{''.join(f'<li>{ic("info")}{x}</li>' for x in items)}</ul></div>"""
p5 = f"""
<div class="kicker">What's next</div>
<h1>Fix, then test step by step</h1>

<div class="sec card"><h2>Software: try these on the recorded flights first (no flying needed)</h2><ul class="plain">
  <li>{ic('info')}<b>Use every camera frame</b> (it appears to use only every other one today), so the ground moves half as far between looks.</li>
  <li>{ic('info')}<b>Follow faster movement</b> by letting the tracker search a wider area.</li>
  <li>{ic('info')}<b>Boost image contrast</b> so plain ground shows more detail.</li>
  <li>{ic('info')}<b>Ignore the drone's own shadow</b>, which moves with the drone and can confuse the tracking.</li>
  <li>{ic('info')}<b>Replay the flights where it stopped</b>: if it stops at the same moment on the laptop, it's a bug we can fix directly.</li>
</ul></div>

<div class="sec"><h2>Hardware</h2><div style="display:flex;flex-direction:column;gap:14px">
{tier("Before the next flights · cheap", GOOD, ["<b>Clock battery</b> for the Jetson, so it never loses the time",
       "<b>Secure the Jetson's power</b>: locking connector and its own power supply",
       "<b>Soft-mount the camera</b> on vibration dampers (whole camera as one unit)",
       "<b>Height sensor</b> (small downward laser rangefinder), since height was the weakest part"])}
{tier("For accurate testing", WARN, ["<b>RTK GPS</b> (centimetre accuracy) as the yardstick. Today's GPS is only good to 2–5 m, too rough to measure small errors"])}
{tier("Only if software fixes aren't enough", INK2, ["<b>Tilt the camera forward</b> (30–45°) so the ground moves slower in the picture and it sees more detail",
       "<b>Optical-flow sensor</b> as an independent backup for the flight controller",
       "<b>Faster camera frame rate</b>, if the camera supports it"])}
</div></div>

<div class="sec card"><h2>Next flight tests: one change at a time</h2>
  <div style="display:flex;gap:12px;align-items:flex-start">{lad}</div>
  <div class="small" style="margin-top:14px">Every flight: textured ground (tracks, grass edges, objects), the new one-command routine, and the landing spot
  measured against the take-off mark. A new camera calibration is needed after the camera is soft-mounted.</div></div>
{FOOT}"""
page("05_next_steps", p5, 1560)
