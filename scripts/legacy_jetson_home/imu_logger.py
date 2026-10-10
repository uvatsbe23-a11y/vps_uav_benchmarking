#!/usr/bin/env python3
"""
IMU + GPS CSV logger - reads HIGHRES_IMU and GPS messages from Cube Orange
over USB via pymavlink, logs to CSV with Jetson system timestamps.
Ctrl+C to stop cleanly.
"""

import os
os.environ["MAVLINK20"] = "1"   # MUST be set before importing pymavlink -- fixes
                                 # a known 'NoneType._instances' crash on MAVLink2 traffic

import time
import csv
import signal
import sys
from pymavlink import mavutil

# ---- CONFIG ----
DEVICE = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_2F0034000551333232393436-if00"
BAUD = 115200  # USB ignores baud but pymavlink wants a value
OUTFILE = f"imu_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"

running = True

def handle_sigint(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, handle_sigint)

print(f"Connecting to {DEVICE} ...")
conn = mavutil.mavlink_connection(DEVICE, baud=BAUD)

print("Waiting for heartbeat...")
conn.wait_heartbeat()
print(f"Heartbeat received (sys {conn.target_system}, comp {conn.target_component})")

# ArduPilot does NOT stream most messages by default -- must explicitly request.
# HIGHRES_IMU = msg id 105. Request at TARGET_HZ.
TARGET_HZ = 200
interval_us = int(1_000_000 / TARGET_HZ)

def request_message(msg_id, hz, label):
    interval = int(1_000_000 / hz)
    print(f"Requesting {label} (id {msg_id}) at {hz} Hz ...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval,
        0, 0, 0, 0, 0
    )
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack:
        print(f"  COMMAND_ACK: result={ack.result} (0 = ACCEPTED)")
    else:
        print(f"  WARNING: no COMMAND_ACK for {label} -- request may not have been accepted")

# IMU at 200Hz (matches earlier bench test)
request_message(105, TARGET_HZ, "HIGHRES_IMU")

# GPS at 10Hz -- Here3+ supports this rate fine, and we don't need 200Hz for position
GPS_HZ = 10
request_message(33, GPS_HZ, "GLOBAL_POSITION_INT")   # lat/lon/alt/vel
request_message(24, GPS_HZ, "GPS_RAW_INT")           # fix_type, satellites, hdop -- tells us if RTK locked

GPSFILE = f"gps_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
gps_f = open(GPSFILE, "w", newline="")
gps_writer = csv.writer(gps_f)
gps_writer.writerow(["jetson_timestamp", "mav_time_usec", "lat", "lon", "alt_mm",
                      "relative_alt_mm", "vx", "vy", "vz", "fix_type",
                      "satellites_visible", "hdop"])
gps_count = 0
# cache the latest GPS_RAW_INT fields so we can merge them onto each GLOBAL_POSITION_INT row
latest_fix = {"fix_type": None, "satellites_visible": None, "hdop": None}

count = 0
start_time = time.time()
last_print = start_time

print(f"Logging IMU to {OUTFILE}")
print(f"Logging GPS to {GPSFILE}")
print("Press Ctrl+C to stop")

with open(OUTFILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["jetson_timestamp", "mav_time_usec", "xacc", "yacc", "zacc",
                      "xgyro", "ygyro", "zgyro"])

    while running:
        msg = conn.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue

        mtype = msg.get_type()
        t_now = time.time()

        if mtype == "HIGHRES_IMU":
            writer.writerow([
                t_now,
                msg.time_usec,
                msg.xacc, msg.yacc, msg.zacc,
                msg.xgyro, msg.ygyro, msg.zgyro,
            ])
            count += 1

        elif mtype == "GPS_RAW_INT":
            # cache fix quality, merged onto the next GLOBAL_POSITION_INT row
            latest_fix["fix_type"] = msg.fix_type
            latest_fix["satellites_visible"] = msg.satellites_visible
            latest_fix["hdop"] = msg.eph  # eph = hdop * 100 in GPS_RAW_INT

        elif mtype == "GLOBAL_POSITION_INT":
            gps_writer.writerow([
                t_now,
                msg.time_boot_ms * 1000,  # ms -> us for rough consistency, not exact epoch
                msg.lat, msg.lon, msg.alt, msg.relative_alt,
                msg.vx, msg.vy, msg.vz,
                latest_fix["fix_type"],
                latest_fix["satellites_visible"],
                latest_fix["hdop"],
            ])
            gps_count += 1

        # live rate printout every 2 sec, so you can SEE it's healthy during bench test
        if t_now - last_print >= 2.0:
            elapsed = t_now - start_time
            imu_rate = count / elapsed if elapsed > 0 else 0
            gps_rate = gps_count / elapsed if elapsed > 0 else 0
            fix_str = f"fix={latest_fix['fix_type']} sats={latest_fix['satellites_visible']}"
            print(f"[{elapsed:6.1f}s] IMU: {count:6d} msgs ~{imu_rate:5.1f}Hz | "
                  f"GPS: {gps_count:5d} msgs ~{gps_rate:4.1f}Hz | {fix_str}")
            last_print = t_now

gps_f.close()
elapsed = time.time() - start_time
print(f"\nStopped after {elapsed:.1f}s")
print(f"  IMU: {count} messages (~{count/elapsed:.1f} Hz avg) -> {OUTFILE}")
print(f"  GPS: {gps_count} messages (~{gps_count/elapsed:.1f} Hz avg) -> {GPSFILE}")
print(f"  Last known fix_type={latest_fix['fix_type']} "
      f"(0=no fix,1=no fix,2=2D,3=3D,4=DGPS,5=RTK Float,6=RTK Fixed), "
      f"satellites={latest_fix['satellites_visible']}")