#!/usr/bin/env python3
"""Extract reference data from an ArduPilot .bin log into an .npz.
Times are wall-clock (GPS-derived, via pymavlink _timestamp).
usage: extract_bin.py <log.bin> <out.npz>"""
import sys, numpy as np
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
D = {k: [] for k in ["XKF1", "GPS", "GPA", "BARO", "IMU", "VIBE", "ATT", "CTUN", "RCOU"]}
ev, msgs, orgn = [], [], []
while True:
    x = m.recv_match()
    if x is None: break
    t, ts = x.get_type(), x._timestamp
    if t == "XKF1" and x.C == 0:
        D[t].append([ts, x.PN, x.PE, x.PD, x.VN, x.VE, x.VD, x.Roll, x.Pitch, x.Yaw])
    elif t == "GPS" and x.I == 0:
        D[t].append([ts, x.Status, x.NSats, x.HDop, x.Lat, x.Lng, x.Alt, x.Spd, x.VZ])
    elif t == "GPA" and x.I == 0:
        D[t].append([ts, x.HAcc, x.VAcc, x.SAcc])
    elif t == "BARO" and x.I == 0:
        D[t].append([ts, x.Alt])
    elif t == "IMU" and x.I == 0:
        D[t].append([ts, x.GyrX, x.GyrY, x.GyrZ, x.AccX, x.AccY, x.AccZ])
    elif t == "VIBE" and x.IMU == 0:
        D[t].append([ts, x.VibeX, x.VibeY, x.VibeZ, x.Clip])
    elif t == "ATT":
        D[t].append([ts, x.Roll, x.Pitch, x.Yaw])
    elif t == "CTUN":
        D[t].append([ts, x.ThO, x.Alt, x.BAlt])
    elif t in ("ARM", "EV", "MODE"):
        ev.append((ts, t, str(x.to_dict())))
    elif t == "MSG":
        msgs.append((ts, x.Message))
    elif t == "ORGN":
        orgn.append((ts, x.Type, x.Lat, x.Lng, x.Alt))
np.savez(sys.argv[2], **{k: np.array(v) for k, v in D.items()},
         events=np.array(ev, dtype=object), msgs=np.array(msgs, dtype=object), orgn=np.array(orgn))
for k, v in D.items(): print(f"{k}: {len(v)}", end="  ")
print()
for ts, s in msgs: print(f"  MSG {ts:.1f}: {s}")
for e in ev: print("  ", f"{e[0]:.1f}", e[1], e[2][:110])
