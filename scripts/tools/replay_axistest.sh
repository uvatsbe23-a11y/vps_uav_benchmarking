#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
DIAG=~/vps_replay_diag4/axistest_replay; rm -rf "$DIAG"; mkdir -p "$DIAG"
OUT="$DIAG/replay.out"; : > "$OUT"; exec >> "$OUT" 2>&1
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"
DRIFT=~/ros2_ws/src/vps_uav_benchmarking/scripts/compute_drift.py

echo "=== config: T_cam_imu cam0 rotation + key init params ==="
sed -n '3,5p' ~/ros2_ws/src/vps_uav_benchmarking/config/kalibr_imucam_chain.yaml
grep -E '^(init_dyn_use|try_zupt|init_imu_thresh|calib_cam_)' ~/ros2_ws/src/vps_uav_benchmarking/config/estimator_config.yaml

kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }

run(){ local bag="$1" name="$2"
  echo; echo "########## REPLAY $name  ($bag) ##########"
  [ -d "$bag" ] || { echo "  MISSING $bag"; return; }
  ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$DIAG/$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" > ov.log 2>&1 & local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/poseimu /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!
  sleep 3
  timeout 260 ros2 bag play "$bag" --rate 0.5 --topics $RAW
  sleep 4
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 4; kill_all
  echo "  --- init ---"
  grep -nE '\[init\]|successful initialization|failed .*init' ov.log | grep -v '\[TIME\]' | head -18
  echo "  init OK: $(grep -c 'successful initialization' ov.log)   crash: $(grep -cE 'Assertion|process has died|exit code -6' ov.log)"
  echo "  --- first / last filter state ---"
  grep -E 'q_GtoI = .*dist = ' ov.log | head -2 | sed 's/^/    /'
  grep -E 'q_GtoI = .*dist = ' ov.log | tail -2 | sed 's/^/    /'
  grep -E '^bg = .*ba = ' ov.log | tail -1 | sed 's/^/    /'
  echo "  --- drift ---"
  python3 "$DRIFT" "$d/out" 2 2>&1 | grep -E 'Start position|End position|path length|Endpoint drift|Drift as %|Max excursion'
  cd "$DIAG"
}

run ~/axis_test_20260906_220629 t1_220629
run ~/axis_test_v2               t2_v2
run ~/axis_test_20260906_221904  t3_221904
kill_all
echo; echo REPLAY_DONE
