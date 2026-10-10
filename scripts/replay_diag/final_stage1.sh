#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/final_stage1.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

echo "=== repo config in use (T_cam_imu cam0) ==="
sed -n '2,9p' ~/ros2_ws/src/vps_uav_benchmarking/config/kalibr_imucam_chain.yaml
echo "=== init_dyn_use / try_zupt / calib flags ==="
grep -E "^(init_dyn_use|try_zupt|calib_cam_)" ~/ros2_ws/src/vps_uav_benchmarking/config/estimator_config.yaml

kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }

run(){ local bag=$1 label=$2
  echo; echo "########## $label  (bag $bag, repo default config) ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$D/final_$label"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  # no config_path override -> uses the repo's config/estimator_config.yaml (which now has stock T_cam_imu)
  setsid ros2 launch vps_uav_benchmarking "$LF" > ov.log 2>&1 & local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/poseimu /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!
  sleep 3
  timeout 150 ros2 bag play ~/$bag --rate 0.5 --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  crash?      : $(grep -cE 'process has died|Assertion|SIGABRT|exit code -' ov.log)"
  echo "  final intern: $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "$label"
  cd "$D"; }

run stage1_axes        axes_repofix
run stage1_axes_v6sept v6sept_repofix
kill_all; echo; echo "FINAL_STAGE1 DONE"
