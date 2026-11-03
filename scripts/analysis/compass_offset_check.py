#!/usr/bin/env python3
"""Compass offset check (1 Nov 2026).
Compares the flight controller's EKF heading change with its own gyro (earth-frame yaw rate,
integrated) between two log times. The difference is how much GPS corrected the compass in flight,
i.e. roughly the compass error on the ground.
usage: compass_offset_check.py <log.bin> <t_start_s> <t_end_s>   (TimeUS seconds; use arming -> first waypoint,
       and end the window before any fast yaw turn: the 25 Hz IMU log cannot integrate fast turns accurately)"""
import sys, numpy as np
from pymavlink import mavutil
f = mavutil.mavlink_connection(sys.argv[1]); a, b = float(sys.argv[2]), float(sys.argv[3])
Y, G, A = [], [], []
while True:
    m = f.recv_match(type=["XKF1", "IMU", "ATT"])
    if m is None: break
    t = m.TimeUS / 1e6; ty = m.get_type()
    if ty == "XKF1" and m.C == 0: Y.append([t, m.Yaw])
    elif ty == "IMU" and m.I == 0: G.append([t, m.GyrX, m.GyrY, m.GyrZ])
    elif ty == "ATT": A.append([t, m.Roll, m.Pitch])
Y, G, A = map(np.array, (Y, G, A))
k = (G[:, 0] >= a) & (G[:, 0] <= b); t = G[k, 0]
ph = np.radians(np.interp(t, A[:, 0], A[:, 1])); th = np.radians(np.interp(t, A[:, 0], A[:, 2]))
yr = (G[k, 2] * np.sin(ph) + G[k, 3] * np.cos(ph)) / np.cos(th)
yu = np.unwrap(np.radians(Y[:, 1]))
dy = np.degrees(np.interp(b, Y[:, 0], yu) - np.interp(a, Y[:, 0], yu)); g = np.degrees(np.trapezoid(yr, t))
print(f"EKF heading change {dy:+.1f} deg, gyro {g:+.1f} deg, EKF minus gyro {dy-g:+.1f} deg")
