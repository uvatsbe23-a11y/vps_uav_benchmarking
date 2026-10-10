#!/usr/bin/env bash
# verify_axis_fix.sh — replay stage1_axes through OpenVINS with the current repo
# config and confirm: (1) T_cam_imu fix -> ~1% drift (was 43.7%), (2) the
# shutdown patch -> clean exit (no SIGABRT).
set -u
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
BAG=~/stage1_axes
OUT=/tmp/verify_out
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

echo "=== commits under test ==="
git -C ~/ros2_ws/src/vps_uav_benchmarking log --oneline -2
git -C ~/ros2_ws/src/open_vins log --oneline -1
grep -A3 'cam0:' ~/ros2_ws/src/vps_uav_benchmarking/config/kalibr_imucam_chain.yaml | head -4
echo "  ^ rotation should be ~[0.9999, -0.008, -0.011 ...] (near-identity), NOT [0,-1,0]"
echo

pkill -f run_subscribe_msckf 2>/dev/null; sleep 2
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
rm -rf "$OUT"

setsid ros2 launch vps_uav_benchmarking "$LF" > /tmp/verify_ov.log 2>&1 &
PG=$!
sleep 8
ros2 bag record -o "$OUT" /ov_msckf/poseimu > /tmp/verify_rec.log 2>&1 &
REC=$!
sleep 3
echo "=== replaying $BAG (rate 0.5, ~200 s wall) ==="
timeout 220 ros2 bag play "$BAG" --rate 0.5 --topics $RAW
sleep 3
kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
kill -INT -"$PG" 2>/dev/null; sleep 5; pkill -f run_subscribe_msckf 2>/dev/null

echo
echo "=== RESULT ==="
if grep -qE 'process has died|Assertion|SIGABRT|exit code -6' /tmp/verify_ov.log; then
  echo "  SHUTDOWN: *** CRASHED *** (patch not effective — is ov_msckf rebuilt?)"
else
  echo "  SHUTDOWN: clean (no SIGABRT)  [crash-fix OK]"
fi

DRIFT_TOOL=~/ros2_ws/src/vps_uav_benchmarking/scripts/compute_drift.py
if [ -f "$DRIFT_TOOL" ]; then
  python3 "$DRIFT_TOOL" "$OUT" 2
else
  python3 ~/ros2_ws/src/vps_uav_benchmarking/scripts/flight_analysis.py "$OUT" --odom-topic /ov_msckf/poseimu
fi
echo
echo "  EXPECT: drift ~1.1-1.6 %, endpoint error ~0.1 m over ~9 m path."
echo "  (With the OLD 90-degree T_cam_imu this was 43.7 %.)"
