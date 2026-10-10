#!/usr/bin/env bash
# check_setup.sh - Stage 0 preflight checks for the VPS drift test.
#
# Run this AFTER:
#   1. sudo nvpmodel -m 2 && sudo jetson_clocks      (mode 2 = MAXN_SUPER on this Orin Nano;
#                                                     mode 0 is the throttled 15 W mode)
#   2. source ~/ros2_ws/install/setup.bash
#   3. ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py      (camera)
#   4. ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py (VIO)
#
# Exit code = number of warnings (0 = good to proceed to Stage 1).
set -u

warn=0
ok()  { echo "  [ OK ]  $*"; }
bad() { echo "  [WARN]  $*"; warn=$((warn + 1)); }

IR_L=/camera/left_ir/image_raw
IR_R=/camera/right_ir/image_raw
CI_L=/camera/left_ir/camera_info
IMU=/camera/gyro_accel/sample
ODOM=/ov_msckf/odomimu

echo "== topics present =="
# --no-daemon: the cached daemon list can be stale (seen on vajra: every topic
# reported MISSING while all of them were streaming).
mapfile -t TOPICS < <(ros2 topic list --no-daemon --spin-time 3 2>/dev/null)
for t in "$IR_L" "$IR_R" "$CI_L" "$IMU" "$ODOM"; do
  if printf '%s\n' "${TOPICS[@]}" | grep -qx "$t"; then ok "$t"; else bad "$t MISSING"; fi
done

rate_check() {  # $1=topic  $2=min_hz  $3=label
  local hz
  hz=$(timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
  if [ -z "$hz" ]; then bad "$3: no messages on $1"; return; fi
  if awk -v h="$hz" -v m="$2" 'BEGIN{exit !(h + 0 >= m)}'; then
    ok "$3: ${hz} Hz (>= $2)"
  else
    bad "$3: ${hz} Hz (want >= $2 Hz)"
  fi
}

echo "== stream rates =="
rate_check "$IR_L" 15  "left IR"
rate_check "$IR_R" 15  "right IR"
rate_check "$IMU"  300 "IMU sync accel/gyro"
rate_check "$ODOM" 5   "OpenVINS odom"

echo "== camera_info resolution =="
res=$(timeout 5 ros2 topic echo --once "$CI_L" 2>/dev/null \
      | awk '/^width:/{w=$2} /^height:/{h=$2} END{print w"x"h}')
if [ "$res" = "848x480" ]; then
  ok "left IR $res"
else
  bad "left IR resolution '$res' (kalibr_imucam_chain.yaml expects 848x480)"
fi

echo "== clock offset (camera header stamp vs system time) =="
now=$(date +%s.%N)
cst=$(timeout 5 ros2 topic echo --once "$IR_L" --field header.stamp 2>/dev/null \
      | awk '/^sec:/{s=$2} /^nanosec:/{n=$2} END{printf "%d.%09d", s, n}')
if [ -n "$cst" ] && [ "$cst" != "0.000000000" ]; then
  off=$(awk -v a="$now" -v b="$cst" 'BEGIN{d=a-b; if(d<0)d=-d; printf "%.3f", d}')
  if awk -v d="$off" 'BEGIN{exit !(d < 0.5)}'; then
    ok "offset ${off}s (< 0.5s)"
  else
    bad "offset ${off}s - set the driver to publish host/ROS time (time_domain:=system)"
  fi
else
  bad "could not read a camera header stamp"
fi

echo "== system clock =="
# The Jetson RTC has no battery: after a power cycle the clock is either 1970 or
# the last saved stamp (clock-restore), not the real time.
yr=$(date +%Y)
if [ "$yr" -lt 2026 ]; then
  bad "system clock is $(date) - run sync_jetson_clock.sh on the ground PC"
elif timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes || [ -e /run/clock-synced ]; then
  ok "$(date)  (synced this boot)"
else
  bad "$(date) - restored from last saved stamp, NOT synced this boot (run sync_jetson_clock.sh on the ground PC)"
fi

echo
if [ "$warn" -eq 0 ]; then
  echo "ALL CHECKS PASSED - proceed to Stage 1"
else
  echo "$warn warning(s) - fix these before proceeding"
fi
exit "$warn"
