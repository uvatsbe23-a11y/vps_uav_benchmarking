#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/multibag.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
# best config so far: dynamic init + calib frozen (keep ZUPT on)
CFG=$D/cfg_mb; rm -rf "$CFG"; cp -r "$SHARE" "$CFG"; F=$CFG/estimator_config.yaml
sed -i -e 's/^init_dyn_use:.*/init_dyn_use: true/' \
       -e 's/^calib_cam_extrinsics:.*/calib_cam_extrinsics: false/' \
       -e 's/^calib_cam_intrinsics:.*/calib_cam_intrinsics: false/' \
       -e 's/^calib_cam_timeoffset:.*/calib_cam_timeoffset: false/' "$F"
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"
kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
run(){ local bag=$1 rate=${2:-1.0} secs=${3:-70}
  echo; echo "########## BAG: $bag  (rate $rate, ~${secs}s) ##########"
  [ -d ~/$bag ] || { echo "  MISSING"; return; }
  ros2 bag info ~/$bag 2>/dev/null | grep -E "Duration|image_raw|gyro" | sed 's/^/  /'
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$D/mb_$bag"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$F" > ov.log 2>&1 & local PG=$!; sleep 8
  ros2 bag record -o out /ov_msckf/odomimu /ov_msckf/poseimu > rec.log 2>&1 & local REC=$!; sleep 3
  timeout "$secs" ros2 bag play ~/$bag --rate "$rate" --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL   : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  num_feats o,n  : $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -6 | tr '\n' ' ')"
  echo "  disparity      : $(grep -oE 'disparity is [0-9.]+,[0-9.]+' ov.log | tail -3 | tr '\n' ' ')"
  echo "  UPDATE-MSCKF   : $(grep -c '\[UPDATE-MSCKF\]' ov.log)   SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)   ZUPT: $(grep -cE 'ZUPT\]: accepted' ov.log)"
  grep -E '\[UPDATE-MSCKF\]' ov.log | head -3 | sed 's/^/    /'
  echo "  final internal : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "${bag}[poseimu]"
  cd "$D"; }
run stage1_static 3.0 90        # 431s static bag, play fast, ~4.5 min of data
run stage1_axes_6sept 0.5 90    # the OTHER 6sept bag
run jetson_stereo_motion_drift_test 0.5 40
run stage1_axes 0.5 90          # the original no-suffix one
kill_all; echo; echo "MULTIBAG DONE"
