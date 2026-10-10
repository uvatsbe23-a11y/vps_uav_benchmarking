#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/pptest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
BAG=~/stage1_axes            # the GENTLE 100s bag (disparity ~0.8px)
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

# --- cfg MONO: max_cameras 1 ---
CM=$D/cfg_mono; rm -rf "$CM"; cp -r "$SHARE" "$CM"
sed -i -e 's/^max_cameras:.*/max_cameras: 1/' -e 's/^use_stereo:.*/use_stereo: false/' \
       -e 's/^init_dyn_use:.*/init_dyn_use: true/' "$CM/estimator_config.yaml"

# --- cfg PP: stereo, but read the pre-processed /pp/* topics (edit kalibr rostopic) ---
CP=$D/cfg_pp; rm -rf "$CP"; cp -r "$SHARE" "$CP"
sed -i -e 's/^init_dyn_use:.*/init_dyn_use: true/' "$CP/estimator_config.yaml"
sed -i 's#/camera/left_ir/image_raw#/pp/left_ir/image_raw#'   "$CP/kalibr_imucam_chain.yaml"
sed -i 's#/camera/right_ir/image_raw#/pp/right_ir/image_raw#' "$CP/kalibr_imucam_chain.yaml"

kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null; pkill -INT -f ir_pp.py 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; pkill -KILL -f ir_pp.py 2>/dev/null; sleep 1; }

run(){ local name=$1 cfg=$2 usepp=$3
  echo; echo "########## $name ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$D/pp_$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  [ "$usepp" = yes ] && { setsid python3 ~/vps_replay_diag4/ir_pp.py > pp.log 2>&1 & sleep 3; }
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$cfg/estimator_config.yaml" > ov.log 2>&1 & local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/odomimu /ov_msckf/poseimu > rec.log 2>&1 & local REC=$!
  sleep 3
  timeout 130 ros2 bag play "$BAG" --rate 0.5 --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  [ "$usepp" = yes ] && echo "  pp republished : $(grep -oE 'republished [0-9]+' pp.log | tail -1)"
  echo "  init OK/FAIL   : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  num_feats o,n  : $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -6 | tr '\n' ' ')"
  echo "  disparity      : $(grep -oE 'disparity is [0-9.]+,[0-9.]+' ov.log | tail -3 | tr '\n' ' ')"
  echo "  UPDATE-MSCKF   : $(grep -c '\[UPDATE-MSCKF\]' ov.log)   SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)   ZUPT: $(grep -cE 'ZUPT\]: accepted' ov.log)"
  grep -E '\[UPDATE-MSCKF\]' ov.log | head -4 | sed 's/^/    /'
  echo "  final internal : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "$name"
  cd "$D"; }

run mono "$CM" no
run pp   "$CP" yes
kill_all; echo; echo "PPTEST DONE"
