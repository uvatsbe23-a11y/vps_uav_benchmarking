#!/usr/bin/env python3
"""
Quick GPS check - just confirms GLOBAL_POSITION_INT and GPS_RAW_INT
are flowing from the Cube Orange, and prints fix quality live.
Run this OUTDOORS (or near a window) -- GPS won't get a fix indoors.
Ctrl+C to stop.
"""

import os
os.environ["MAVLINK20"] = "1"   # MUST be set before importing pymavlink

import time
from pymavlink import mavutil

DEVICE = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_2F0034000551333232393436-if00"
BAUD = 115200

print(f"Connecting to {DEVICE} ...")
conn = mavutil.mavlink_connection(DEVICE, baud=BAUD)

print("Waiting for heartbeat...")
conn.wait_heartbeat()
print(f"Heartbeat received (sys {conn.target_system}, comp {conn.target_component})")

def request_message(msg_id, hz, label):
    interval = int(1_000_000 / hz)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, interval, 0, 0, 0, 0, 0
    )
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    status = f"result={ack.result}" if ack else "NO ACK"
    print(f"Requested {label} (id {msg_id}) @ {hz}Hz -> {status}")

request_message(33, 5, "GLOBAL_POSITION_INT")
request_message(24, 5, "GPS_RAW_INT")

print("\nListening for 20s -- watching for fix_type and satellite count...\n")

FIX_NAMES = {0: "NO_FIX", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}

t0 = time.time()
seen_pos = False
seen_raw = False

while time.time() - t0 < 20:
    msg = conn.recv_match(blocking=True, timeout=2)
    if msg is None:
        print("...no message in 2s...")
        continue

    mtype = msg.get_type()

    if mtype == "GPS_RAW_INT":
        seen_raw = True
        fix_name = FIX_NAMES.get(msg.fix_type, f"UNKNOWN({msg.fix_type})")
        print(f"[GPS_RAW_INT] fix={fix_name}  sats={msg.satellites_visible}  "
              f"hdop={msg.eph/100:.2f}m  lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}")

    elif mtype == "GLOBAL_POSITION_INT":
        seen_pos = True
        print(f"[GLOBAL_POSITION_INT] lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}  "
              f"alt={msg.alt/1000:.1f}m  rel_alt={msg.relative_alt/1000:.1f}m")

print("\n--- Summary ---")
print(f"GLOBAL_POSITION_INT received: {seen_pos}")
print(f"GPS_RAW_INT received:        {seen_raw}")
if not (seen_pos and seen_raw):
    print("WARNING: one or both message types never arrived. Check GPS wiring/power, "
          "or try running outdoors with clear sky view.")