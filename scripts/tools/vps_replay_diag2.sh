#!/usr/bin/env bash
# vps_replay_diag2.sh - clean-room replay diagnostic for the stage1_axes_v6sept bag.
#
# Fixes vs vps_replay_diag.sh:
#   1. Kills leftover estimator nodes RELIABLY (own process group + verify none
#      left + assert /ov_msckf/odomimu has no publisher) so a recording can't be
#      polluted by a zombie / earlier-run filter.
#   2. Plays the bag with --clock and runs the node with use_sim_time:=true
#      (clean TF / RViz; note the OpenVINS filter itself keys off header stamps).
#   3. Drops the "imuonly" arm (max_cameras:0 + init_dyn_use:false cannot even
#      initialise). Instead runs 4 stereo variants to isolate what wrecks it:
#         base          - config exactly as shipped
#         calibfrozen   - calib_cam_{extrinsics,intrinsics,timeoffset}: false
#         nozupt        - try_zupt: false
#         frozen_nozupt - both of the above
#   4. Scores each run with flight_analysis_strict.py, which REJECTS a run whose
#      trajectory has NaN/Inf or whose first pose is >5 m from the origin.
#
# Output: ~/vps_replay_diag2/REPORT.txt  (+ per-run dir with openvins.log & out bag)

set +u
DIAG=~/vps_replay_diag2
STRICT=~/vps_replay_diag2/flight_analysis_strict.py   # copied in by the caller
mkdir -p "$DIAG"
exec > >(tee "$DIAG/REPORT.txt") 2>&1

echo "=== ENV ==="
for s in /opt/ros/*/setup.bash; do [ -f "$s" ] && { source "$s"; echo "sourced $s"; break; }; done
for w in ~/ros2_ws ~/vps_ws ~/colcon_ws ~/dev_ws; do
  [ -f "$w/install/setup.bash" ] && { source "$w/install/setup.bash"; echo "sourced $w"; }
done
echo "ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
command -v ros2 >/dev/null || { echo "!! ros2 not on PATH"; exit 1; }
ros2 pkg prefix ov_msckf >/dev/null || { echo "!! ov_msckf not found"; exit 1; }

BAG=$(dirname "$(find ~ -name metadata.yaml -path '*stage1_axes_v6sept*' 2>/dev/null | head -1)")
echo "BAG = $BAG"
[ -d "$BAG" ] || { echo "!! bag not found"; exit 1; }
[ -f "$STRICT" ] || { echo "!! $STRICT missing (caller should have copied it)"; exit 1; }

SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

# ---------- build the 4 config variants ----------
rm -rf "$DIAG"/cfg_*
for v in base calibfrozen nozupt frozen_nozupt; do
  cp -r "$SHARE" "$DIAG/cfg_$v"
done
patch_calib () {  # freeze online calibration
  local f="$1"
  sed -i 's/^calib_cam_extrinsics:.*/calib_cam_extrinsics: false/' "$f"
  sed -i 's/^calib_cam_intrinsics:.*/calib_cam_intrinsics: false/' "$f"
  sed -i 's/^calib_cam_timeoffset:.*/calib_cam_timeoffset: false/' "$f"
}
patch_nozupt () { sed -i 's/^try_zupt:.*/try_zupt: false/' "$1"; }
patch_calib   "$DIAG/cfg_calibfrozen/estimator_config.yaml"
patch_nozupt  "$DIAG/cfg_nozupt/estimator_config.yaml"
patch_calib   "$DIAG/cfg_frozen_nozupt/estimator_config.yaml"
patch_nozupt  "$DIAG/cfg_frozen_nozupt/estimator_config.yaml"

echo
echo "=== variant config deltas vs shipped ==="
for v in calibfrozen nozupt frozen_nozupt; do
  echo "--- $v ---"
  diff "$SHARE/estimator_config.yaml" "$DIAG/cfg_$v/estimator_config.yaml"
done

# ---------- reliable teardown of any estimator node ----------
kill_estimators () {
  pkill -TERM -f run_subscribe_msckf 2>/dev/null
  for _ in $(seq 1 10); do pgrep -f run_subscribe_msckf >/dev/null || break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null
  sleep 1
}
assert_clean () {
  # no estimator process, and nothing publishing the odom topic
  if pgrep -f run_subscribe_msckf >/dev/null; then
    echo "  !! estimator still running after kill - ABORT"; return 1
  fi
  local pubs
  pubs=$(ros2 topic info /ov_msckf/odomimu 2>/dev/null | awk '/Publisher count/{print $NF}')
  echo "  pre-run publisher count on /ov_msckf/odomimu = ${pubs:-0}"
  [ "${pubs:-0}" = "0" ] || { echo "  !! something is still publishing odom - ABORT"; return 1; }
  return 0
}

run_variant () {
  local name="$1" cfg="$2"
  echo
  echo "########################## RUN: $name ##########################"
  kill_estimators
  assert_clean || { echo "SKIP $name (dirty environment)"; return; }

  local d="$DIAG/$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"

  # estimator in its own session/process-group
  setsid ros2 run ov_msckf run_subscribe_msckf --ros-args \
      -p config_path:="$cfg/estimator_config.yaml" \
      -p use_sim_time:=true \
      -p verbosity:=INFO \
      > openvins.log 2>&1 &
  local ovpg=$!
  sleep 6

  setsid ros2 bag record -o out \
      /ov_msckf/odomimu /ov_msckf/poseimu /ov_msckf/pathimu \
      /ov_msckf/points_msckf /ov_msckf/points_slam \
      > rec.log 2>&1 &
  local recpg=$!
  sleep 2

  echo "  playing bag (rate 0.5, --clock, sensor topics only)..."
  ros2 bag play "$BAG" --rate 0.5 --clock --topics $RAW
  sleep 4

  kill -INT  -"$recpg" 2>/dev/null; sleep 2; kill -KILL -"$recpg" 2>/dev/null
  kill_estimators
  sleep 1

  echo "  ---- init / disparity (cam0,cam1 feature counts) ----"
  grep -E 'compute disp|disparity is|unable to select|Initialized|make_state|dynamic init|We do not have enough' openvins.log | head -25
  echo "  ---- update activity ----"
  local msckf slam zupt oo
  msckf=$(grep -c 'MSCKF update' openvins.log)
  slam=$(grep -c  'SLAM update'  openvins.log)
  zupt=$(grep -c  'ZUPT'         openvins.log)
  oo=$(grep -c 'arrived out of order' openvins.log)
  echo "  MSCKF update lines : $msckf"
  echo "  SLAM  update lines : $slam"
  echo "  ZUPT        lines : $zupt"
  echo "  'out of order' warns: $oo"
  echo "  ---- filter's own final internal state (from log tail) ----"
  grep -E 'q_GtoI = .*dist =' openvins.log | tail -1
  grep -E '^bg = |ba = ' openvins.log | tail -1
  echo "  ---- WARN/ERROR ----"
  grep -nE 'WARN|ERROR|FATAL' openvins.log | grep -v 'arrived out of order' | head -15
  echo "  ---- output bag ----"
  ros2 bag info out 2>/dev/null | grep -E 'Duration|Messages|odomimu'
  echo "  ---- strict trajectory analysis ----"
  python3 "$STRICT" out --path-length 1031.5 --label "$name" || echo "  (strict analyzer rejected this run - see reason above)"
  cd "$DIAG"
}

run_variant base          "$DIAG/cfg_base"
run_variant calibfrozen   "$DIAG/cfg_calibfrozen"
run_variant nozupt        "$DIAG/cfg_nozupt"
run_variant frozen_nozupt "$DIAG/cfg_frozen_nozupt"

echo
echo "########################## REFERENCE: original live run ##########################"
echo "(reads the /ov_msckf/odomimu that was recorded live INTO the bag)"
python3 "$STRICT" "$BAG" --path-length 1031.5 --label "live_v6sept" || true

kill_estimators
echo
echo "DONE -> $DIAG/REPORT.txt"
