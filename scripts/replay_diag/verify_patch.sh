#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/verify_patch.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"
kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
for i in 1 2 3; do
  echo; echo "########## verify run $i/3 ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  d="$D/vp_$i"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" > ov.log 2>&1 & PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/poseimu > rec.log 2>&1 & REC=$!
  sleep 3
  timeout 150 ros2 bag play ~/stage1_axes --rate 0.5 --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 5; kill_all
  echo "  SIGABRT/died/assert : $(grep -cE 'process has died|Assertion|SIGABRT|pthread_mutex|exit code -6' ov.log)"
  echo "  exit code line      : $(grep -E 'process has finished cleanly|process has died' ov.log | tail -1 | sed 's/\[.*\] //')"
  echo "  init OK             : $(grep -c 'successful initialization' ov.log)"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "vp_$i"
  cd "$D"
done
kill_all; echo; echo "VERIFY_PATCH DONE"
