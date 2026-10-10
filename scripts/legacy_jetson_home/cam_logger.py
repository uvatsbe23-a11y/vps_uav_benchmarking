#!/usr/bin/env python3
"""
Camera frame logger - captures from Arducam OV9281 USB camera (/dev/video0),
saves timestamped JPGs at a controlled rate. Ctrl+C to stop cleanly.
"""

import cv2
import time
import os
import signal

# ---- CONFIG ----
DEVICE_INDEX = 0          # /dev/video0
WIDTH, HEIGHT = 1280, 720
TARGET_FPS = 20            # capped well below the 120fps max -- plenty for a presentation,
                            # easier on disk + CPU, less risk of drops during bench test
OUTDIR = f"frames_{time.strftime('%Y%m%d_%H%M%S')}"

running = True

def handle_sigint(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, handle_sigint)

os.makedirs(OUTDIR, exist_ok=True)

cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

if not cap.isOpened():
    print("ERROR: could not open camera. Check /dev/video0 and that nothing else has it open.")
    raise SystemExit(1)

actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Camera opened: {actual_w}x{actual_h}")
print(f"Saving frames to ./{OUTDIR}/  -- press Ctrl+C to stop")

frame_interval = 1.0 / TARGET_FPS
count = 0
dropped = 0
start_time = time.time()
last_capture = 0
last_print = start_time

while running:
    t_now = time.time()
    if t_now - last_capture < frame_interval:
        continue  # busy-wait until next target frame time; simple and reliable enough at 20fps

    ret, frame = cap.read()
    if not ret:
        dropped += 1
        continue

    last_capture = t_now
    fname = os.path.join(OUTDIR, f"{t_now:.6f}.jpg")
    cv2.imwrite(fname, frame)
    count += 1

    if t_now - last_print >= 2.0:
        elapsed = t_now - start_time
        rate = count / elapsed if elapsed > 0 else 0
        print(f"[{elapsed:6.1f}s] {count:5d} frames saved | rate ~{rate:4.1f} fps | dropped reads: {dropped}")
        last_print = t_now

cap.release()
elapsed = time.time() - start_time
print(f"\nStopped. {count} frames saved in {elapsed:.1f}s "
      f"(~{count/elapsed:.1f} fps average), {dropped} dropped reads -> ./{OUTDIR}/")