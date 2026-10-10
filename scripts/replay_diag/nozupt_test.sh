#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/nozupt_test.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
BAG=~/stage1_axes_v6sept; LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"
mkcfg(){ local n=$1; rm -rf "$D/cfg_nz_$n"; cp -r "$SHARE" "$D/cfg_nz_$n"; local f="$D/cfg_nz_$n/estimator_config.yaml"
  sed -i -e 's/^init_dyn_use:.*/init_dyn_use: true/' -e 's/^try_zupt:.*/try_zupt: false/' \
         -e 's/^calib_cam_extrinsics:.*/calib_cam_extrinsics: false/' -e 's/^calib_cam_intrinsics:.*/calib_cam_intrinsics: false/' \
         -e 's/^calib_cam_timeoffset:.*/calib_cam_timeoffset: false/' "$f"; echo "$f"; }
A=$(mkcfg min)
B=$(mkcfg loose); sed -i -e 's/^track_frequency:.*/track_frequency: 31.0/' -e 's/^fast_threshold:.*/fast_threshold: 12/' \
   -e 's/^num_pts:.*/num_pts: 350/' -e 's/^knn_ratio:.*/knn_ratio: 0.80/' "$B"
echo "=== nz_min vs shipped ==="; diff "$SHARE/estimator_config.yaml" "$A"
kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
run(){ local name=$1 cfg=$2
  echo; echo "########## $name ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  pgrep -f run_subscribe_msckf >/dev/null && { echo "dirty skip"; return; }
  local d="$D/nz_$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$cfg" > ov.log 2>&1 & local PG=$!; sleep 8
  ros2 bag record -o out /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!; sleep 3
  ros2 bag play "$BAG" --rate 0.5 --topics $RAW; sleep 4
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL   : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  num_feats o,n  : $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -5 | tr '\n' ' ')"
  echo "  disparity      : $(grep -oE 'disparity is [0-9.]+,[0-9.]+' ov.log | tail -3 | tr '\n' ' ')"
  echo "  UPDATE-MSCKF   : $(grep -c '\[UPDATE-MSCKF\]' ov.log)   SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)   ZUPT: $(grep -cE 'ZUPT\]: accepted' ov.log)"
  grep -E '\[UPDATE-MSCKF\]' ov.log | head -4 | sed 's/^/    /'
  echo "  final internal : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q odomimu && python3 "$STRICT" out --label "$name" || echo "  no odom"
  cd "$D"; }
run min "$A"
run loose "$B"
kill_all; echo; echo "NOZUPT_TEST DONE"
