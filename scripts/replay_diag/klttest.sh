#!/usr/bin/env bash
# KLT / calib-frozen sweep on stage1_axes_v6sept.
# Hypothesis: feature tracks die in <1s (num_features0 always ~0), so vision never
# helps. Test freezing online calib + KLT levers we can actually set from yaml.
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4
STRICT="$D/flight_analysis_strict.py"
OUT="$D/klttest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
BAG=~/stage1_axes_v6sept
LF=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

freeze_calib () {  # the requested calib-frozen change
  sed -i 's/^calib_cam_extrinsics:.*/calib_cam_extrinsics: false/' "$1"
  sed -i 's/^calib_cam_intrinsics:.*/calib_cam_intrinsics: false/' "$1"
  sed -i 's/^calib_cam_timeoffset:.*/calib_cam_timeoffset: false/' "$1"
}
mkcfg () {  # $1=name ; remaining handled by caller via sed
  local n="$1"; rm -rf "$D/cfg_klt_$n"; cp -r "$SHARE" "$D/cfg_klt_$n"
  local f="$D/cfg_klt_$n/estimator_config.yaml"
  sed -i 's/^init_dyn_use:.*/init_dyn_use: true/' "$f"    # dynamic init (best on this data)
  freeze_calib "$f"
  echo "$f"
}

A=$(mkcfg frozen)                                                     # just freeze calib
B=$(mkcfg freq30);   sed -i 's/^track_frequency:.*/track_frequency: 31.0/' "$B"
C=$(mkcfg binoc);    sed -i 's/^use_stereo:.*/use_stereo: false/' "$C"  # both cams, no stereo constraint
D_=$(mkcfg loose);   sed -i -e 's/^track_frequency:.*/track_frequency: 31.0/' \
                            -e 's/^fast_threshold:.*/fast_threshold: 10/' \
                            -e 's/^num_pts:.*/num_pts: 400/' \
                            -e 's/^knn_ratio:.*/knn_ratio: 0.85/' \
                            -e 's/^min_px_dist:.*/min_px_dist: 8/' "$D_"
E=$(mkcfg loose_rate1); cp "$D_" "$E"   # same cfg as loose, but played at rate 1.0 below

echo "=== deltas: frozen vs shipped ==="; diff "$SHARE/estimator_config.yaml" "$A"
echo "=== deltas: loose vs frozen ===";   diff "$A" "$D_"

kill_all () {
  pkill -INT -f 'run_subscribe_msckf' 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null || break; sleep 0.5; done
  pkill -KILL -f 'run_subscribe_msckf' 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1
}
run () {
  local name="$1" cfg="$2" rate="${3:-0.5}"
  echo; echo "############### $name  (rate $rate) ###############"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2
  kill_all
  pgrep -f run_subscribe_msckf >/dev/null && { echo "  dirty, skip"; return; }
  local d="$D/klt_$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$cfg" > ov.log 2>&1 &
  local PG=$!; sleep 8
  ros2 bag record -o out /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!
  sleep 3
  ros2 bag play "$BAG" --rate "$rate" --topics $RAW
  sleep 4
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL     : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  num_feats old,new: $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -4 | tr '\n' ' ')"
  echo "  disparity vals   : $(grep -oE 'disparity is [0-9.]+,[0-9.]+' ov.log | tail -3 | tr '\n' ' ')"
  echo "  UPDATE-MSCKF     : $(grep -c '\[UPDATE-MSCKF\]' ov.log)    UPDATE-SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)"
  echo "  ZUPT accepted    : $(grep -cE 'ZUPT\]: accepted' ov.log)"
  grep -E '\[UPDATE-MSCKF\]' ov.log | head -3 | sed 's/^/    /'
  echo "  final internal   : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q odomimu && python3 "$STRICT" out --label "$name" || echo "  (no odom recorded)"
  cd "$D"
}

run frozen      "$A"
run freq30      "$B"
run binoc       "$C"
run loose       "$D_"
run loose_rate1 "$E" 1.0
kill_all
echo; echo "KLTTEST DONE"
