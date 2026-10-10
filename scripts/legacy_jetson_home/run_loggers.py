#!/usr/bin/env python3
"""
Combined launcher - starts imu_logger.py and cam_logger.py together as
subprocesses, streams both their outputs to this terminal, and stops both
cleanly on a single Ctrl+C.
"""

import subprocess
import signal
import sys
import time

PROCS = []

def handle_sigint(sig, frame):
    print("\n[launcher] Ctrl+C received, stopping both loggers...")
    for name, p in PROCS:
        if p.poll() is None:  # still running
            p.send_signal(signal.SIGINT)

    # give them a moment to flush/close files cleanly
    for name, p in PROCS:
        try:
            p.wait(timeout=5)
            print(f"[launcher] {name} exited cleanly (code {p.returncode})")
        except subprocess.TimeoutExpired:
            print(f"[launcher] {name} did not exit in time, killing it")
            p.kill()

    print("[launcher] Done. Check the CSV file and frames_*/ folder for output.")
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

print("[launcher] Starting imu_logger.py ...")
imu_proc = subprocess.Popen([sys.executable, "imu_logger.py"])
PROCS.append(("imu_logger.py", imu_proc))

time.sleep(1)  # stagger start slightly so terminal output doesn't interleave mid-line

print("[launcher] Starting cam_logger.py ...")
cam_proc = subprocess.Popen([sys.executable, "cam_logger.py"])
PROCS.append(("cam_logger.py", cam_proc))

print("[launcher] Both loggers running. Press Ctrl+C to stop both.\n")

# wait on both; if either dies unexpectedly, say so immediately
while True:
    time.sleep(1)
    for name, p in PROCS:
        ret = p.poll()
        if ret is not None:
            print(f"[launcher] WARNING: {name} exited unexpectedly (code {ret})")
            # let the other one keep running, but flag it loudly
            PROCS.remove((name, p))
    if not PROCS:
        print("[launcher] Both loggers have exited. Stopping.")
        sys.exit(1)