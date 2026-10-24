#!/usr/bin/env python3
"""Per-frame image-quality metrics from the left IR camera of a bag.
usage: image_metrics.py <bag_folder> <out.npz> [time_offset_s] [step]
Every <step>-th frame (default 3 -> 10 Hz) and the frame right after it:
  mean  : mean brightness (0-255)        sat/dark : % pixels >= 250 / <= 10
  sharp : variance of the Laplacian (blur -> low)
  corners: FAST corners (thr 20, like OpenVINS' detector) - texture available
  flow  : median feature motion to the next frame, px (30 Hz)
  track : fraction of corners tracked to the next frame with a forward-backward check"""
import sys, glob, sqlite3, numpy as np, cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
bag, out = sys.argv[1], sys.argv[2]
off = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
step = int(sys.argv[4]) if len(sys.argv) > 4 else 3
db = sqlite3.connect("file:" + glob.glob(bag + "/*.db3")[0] + "?mode=ro", uri=True)
tid = dict((n, i) for i, n in db.execute("select id,name from topics"))["/camera/left_ir/image_raw"]
fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
def img(d):
    m = deserialize_message(d, Image)
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width]
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + off, a
R, prev, k = [], None, 0
for (d,) in db.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
    if prev is not None:            # prev = sampled frame, this = the next frame
        t, a = prev; _, b = img(d)
        kps = fast.detect(a, None)
        pts = np.float32([p.pt for p in sorted(kps, key=lambda p: -p.response)[:300]]).reshape(-1, 1, 2)
        flow, track = np.nan, np.nan
        if len(pts) >= 10:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None, **lk)
            p0, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None, **lk)
            good = (st[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm((p0 - pts)[:, 0], axis=1) < 1.0)
            track = good.mean()
            if good.sum() >= 5: flow = np.median(np.linalg.norm((p1 - pts)[good, 0], axis=1))
        R.append([t, a.mean(), 100 * (a >= 250).mean(), 100 * (a <= 10).mean(),
                  cv2.Laplacian(a, cv2.CV_64F).var(), len(kps), flow, track])
        prev = None
    elif k % step == 0:
        prev = img(d)
    k += 1
R = np.array(R); np.save(out, R)
print(f"{out}: {len(R)} samples  mean {np.nanmedian(R[:,1]):.0f}  sat {np.nanmedian(R[:,2]):.1f}%  sharp {np.nanmedian(R[:,4]):.0f}  corners {np.nanmedian(R[:,5]):.0f}  flow {np.nanmedian(R[:,6]):.1f}px  track {np.nanmedian(R[:,7]):.2f}")
