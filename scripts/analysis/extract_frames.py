#!/usr/bin/env python3
"""Grab one left-IR frame from the middle of chosen 5 s analysis windows and tile them.
usage: extract_frames.py out.png  name:bag_folder:clock_offset:t_since_takeoff:label  [...]
  name           tN (needs tN_metrics.json + tN_log.npz in the current folder)
  clock_offset   seconds added to bag stamps (0, or the 1970-clock correction)
Example (10 Oct 2026):
  extract_frames.py worst_vs_best_frames.png \\
    "t1:../stage3_flight1_0928:0:17:WORST t1 @17s" "t2:../stage3_flight1_0958:0:17:BEST t2 @17s" ...
Tiles are laid out 3 per row."""
import sys, json, glob, sqlite3, numpy as np, cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
tiles = []
for spec in sys.argv[2:]:
    n, bag, off, t, lab = spec.split(":", 4); off, t = float(off), float(t)
    M = json.load(open(f"{n}_metrics.json")); L = np.load(f"{n}_log.npz", allow_pickle=True)
    t_gps = L["XKF1"][0, 0] + M["t_takeoff"] + t + 2.5                 # middle of the window
    target = t_gps + M["bag_minus_gps_s"] - off                         # raw bag header time
    db = sqlite3.connect("file:" + glob.glob(bag + "/*.db3")[0] + "?mode=ro", uri=True)
    tid = dict((nm, i) for i, nm in db.execute("select id,name from topics"))["/camera/left_ir/image_raw"]
    row = db.execute("select data from messages where topic_id=? and timestamp>=? order by timestamp limit 1",
                     (tid, int((target - 0.3) * 1e9))).fetchone()
    m = deserialize_message(row[0], Image)
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width].copy()
    c = cv2.cvtColor(cv2.resize(a, (424, 240)), cv2.COLOR_GRAY2BGR)
    cv2.rectangle(c, (0, 0), (424, 22), (0, 0, 0), -1)
    cv2.putText(c, lab, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    tiles.append(c)
while len(tiles) % 3: tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(sys.argv[1], np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]))
print("saved", sys.argv[1])
