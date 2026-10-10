#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4
STRICT="$D/flight_analysis_strict.py"
OUT="$D/histtest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
BAG=~/stage1_axes_v6sept
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

rm -rf "$D"/cfg_hist* "$D"/hist_*
for v in none clahe eqhist_dyn; do cp -r "$SHARE" "$D/cfg_hist_$v"; done
# all three use dynamic init (best initializer for this data) + fix init_max_features
for v in none clahe eqhist_dyn; do
  sed -i 's/^init_dyn_use:.*/init_dyn_use: true/'        "$D/cfg_hist_$v/estimator_config.yaml"
  sed -i 's/^init_max_features:.*/init_max_features: 50/' "$D/cfg_hist_$v/estimator_config.yaml"
done
sed -i 's/^histogram_method:.*/histogram_method: "NONE"/'  "$D/cfg_hist_none/estimator_config.yaml"
sed -i 's/^histogram_method:.*/histogram_method: "CLAHE"/' "$D/cfg_hist_clahe/estimator_config.yaml"
# eqhist_dyn keeps HISTOGRAM (control: isolate histogram vs the init_max_features fix)

kill_all () {
  pkill -INT -f 'run_subscribe_msckf' 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null || break; sleep 0.5; done
  pkill -KILL -f 'run_subscribe_msckf' 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1
}

run () {
  local name="$1" cfg="$2"
  echo; echo "############### $name ###############"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2
  kill_all
  pgrep -f run_subscribe_msckf >/dev/null && { echo "  dirty, skip"; return; }
  local d="$D/hist_$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$cfg/estimator_config.yaml" > ov.log 2>&1 &
  local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/odomimu > rec.log 2>&1 &
  local REC=$!
  sleep 3
  ros2 bag play "$BAG" --rate 0.5 --topics $RAW
  sleep 4
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  hist method     : $(grep -oE 'HISTOGRAM METHOD:.*|histogram.*: (NONE|HISTOGRAM|CLAHE)' ov.log | head -1)"
  echo "  init OK / FAILED: $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  num_features0,1 : $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -3 | tr '\n' ' ')"
  echo "  UPDATE-MSCKF    : $(grep -c '\[UPDATE-MSCKF\]' ov.log)   UPDATE-SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)"
  echo "  ZUPT accepted   : $(grep -cE 'ZUPT\]: accepted' ov.log)"
  grep -E '\[UPDATE-MSCKF\]' ov.log | head -3 | sed 's/^/    /'
  echo "  final internal  : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -E 'odomimu' | sed 's/^/  /'
  ros2 bag info out 2>/dev/null | grep -q odomimu && python3 "$STRICT" out --label "$name"
  cd "$D"
}

run none       "$D/cfg_hist_none"
run clahe      "$D/cfg_hist_clahe"
run eqhist_dyn "$D/cfg_hist_eqhist_dyn"
kill_all
echo; echo "HISTTEST DONE"
