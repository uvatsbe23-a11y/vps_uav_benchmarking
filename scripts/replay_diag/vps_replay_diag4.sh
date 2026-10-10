#!/usr/bin/env bash
# vps_replay_diag4.sh - clean-room replay diagnostic, v4.
#
# v4 fixes over v3:
#   * assert_clean uses `pgrep -f run_subscribe_msckf` as the source of truth,
#     NOT `ros2 topic info` (that reads the ros2 daemon cache, which held a
#     phantom publisher from the dead base node -> v3 skipped 3 variants).
#   * estimator launched under `setsid`; torn down by killing the whole process
#     group + pkill by exe name + verified with pgrep. daemon restarted per run.
#   * DRIFT reported as endpoint/integrated (self-consistent) - the 1031 m drone
#     course length does not apply to this bag.
#   * variants chosen to test the "camera contributes nothing" hypothesis:
#       base        - shipped config (static init, init_max_disparity 10)
#       dyninit     - init_dyn_use: true  (dynamic initializer, for handheld/moving start)
#       dispwide    - static init but init_max_disparity: 25
#       dyninit_dbg - dynamic init + verbosity DEBUG (per-frame tracker feature counts)
#
# Output: ~/vps_replay_diag4/REPORT.txt

set +u
export ROS_LOCALHOST_ONLY=1
DIAG=~/vps_replay_diag4
STRICT="$DIAG/flight_analysis_strict.py"
mkdir -p "$DIAG"
exec > >(tee "$DIAG/REPORT.txt") 2>&1

echo "=== ENV ==="
for s in /opt/ros/*/setup.bash; do [ -f "$s" ] && { source "$s"; echo "sourced $s"; break; }; done
for w in ~/ros2_ws ~/vps_ws ~/colcon_ws ~/dev_ws; do
  [ -f "$w/install/setup.bash" ] && { source "$w/install/setup.bash"; echo "sourced $w"; }
done
echo "ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
command -v ros2 >/dev/null || { echo "!! ros2 not on PATH"; exit 1; }

BAG=$(dirname "$(find ~ -name metadata.yaml -path '*stage1_axes_v6sept*' 2>/dev/null | head -1)")
echo "BAG = $BAG"
[ -d "$BAG" ] || { echo "!! bag not found"; exit 1; }
[ -f "$STRICT" ] || { echo "!! $STRICT missing"; exit 1; }

SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
LAUNCHFILE=openvins_gemini335l.launch.py
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

# ---------- build config variants ----------
rm -rf "$DIAG"/cfg_* "$DIAG"/base "$DIAG"/dyninit "$DIAG"/dispwide "$DIAG"/dyninit_dbg
for v in base dyninit dispwide dyninit_dbg; do cp -r "$SHARE" "$DIAG/cfg_$v"; done
sed -i 's/^init_dyn_use:.*/init_dyn_use: true/'          "$DIAG/cfg_dyninit/estimator_config.yaml"
sed -i 's/^init_max_disparity:.*/init_max_disparity: 25.0/' "$DIAG/cfg_dispwide/estimator_config.yaml"
sed -i 's/^init_dyn_use:.*/init_dyn_use: true/'          "$DIAG/cfg_dyninit_dbg/estimator_config.yaml"
sed -i 's/^verbosity:.*/verbosity: "DEBUG"/'            "$DIAG/cfg_dyninit_dbg/estimator_config.yaml"

echo; echo "=== variant deltas vs shipped ==="
for v in dyninit dispwide dyninit_dbg; do
  echo "--- $v ---"; diff "$SHARE/estimator_config.yaml" "$DIAG/cfg_$v/estimator_config.yaml"
done

# ---------- helpers ----------
estimator_running () { pgrep -f 'run_subscribe_msckf' >/dev/null; }
kill_estimators () {
  pkill -INT  -f 'run_subscribe_msckf' 2>/dev/null
  pkill -INT  -f "$LAUNCHFILE"          2>/dev/null
  for _ in $(seq 1 20); do estimator_running || break; sleep 0.5; done
  pkill -KILL -f 'run_subscribe_msckf' 2>/dev/null
  pkill -KILL -f "$LAUNCHFILE"          2>/dev/null
  for _ in $(seq 1 10); do estimator_running || break; sleep 0.5; done
  sleep 1
}
assert_clean () {
  if estimator_running; then
    echo "  !! estimator process still alive after kill:"; pgrep -af run_subscribe_msckf | sed 's/^/     /'
    return 1
  fi
  # camera driver check is process-based (a live Orbbec driver would be a component_container in /camera ns)
  if pgrep -af 'component_container' | grep -q 'camera'; then
    echo "  !! a live camera component_container is running - stop it first"; return 1
  fi
  echo "  env clean (no estimator, no live camera driver)"
  return 0
}

run_variant () {
  local name="$1" cfg="$2"
  echo; echo "########################## RUN: $name ##########################"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2
  kill_estimators
  assert_clean || { echo "SKIP $name (dirty env)"; return; }
  local d="$DIAG/$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"

  setsid ros2 launch vps_uav_benchmarking "$LAUNCHFILE" \
      config_path:="$cfg/estimator_config.yaml" > openvins.log 2>&1 &
  local OVPG=$!
  sleep 8
  echo "  estimator pids: $(pgrep -f run_subscribe_msckf | tr '\n' ' ')   ov topics: $(ros2 topic list --no-daemon 2>/dev/null | grep -c '^/ov_msckf/')"

  ros2 bag record -o out /ov_msckf/odomimu /ov_msckf/poseimu > rec.log 2>&1 &
  local REC=$!
  sleep 3

  echo "  playing bag (rate 0.5)..."
  ros2 bag play "$BAG" --rate 0.5 --topics $RAW
  sleep 4

  kill -INT "$REC" 2>/dev/null; for _ in $(seq 1 12); do kill -0 "$REC" 2>/dev/null || break; sleep 0.5; done; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$OVPG" 2>/dev/null; sleep 3
  kill_estimators

  echo "  ---- init / disparity ----"
  grep -E 'compute disp|disparity is|unable to select|successful initialization|failed .*init|dynamic init|Initialized' openvins.log | head -22
  echo "  ---- counts ----"
  echo "  init OK        : $(grep -c 'successful initialization' openvins.log)"
  echo "  init FAILED    : $(grep -cE 'failed .*init' openvins.log)"
  echo "  UPDATE-MSCKF   : $(grep -c '\[UPDATE-MSCKF\]' openvins.log)"
  echo "  UPDATE-SLAM    : $(grep -c '\[UPDATE-SLAM\]' openvins.log)"
  echo "  ZUPT accepted  : $(grep -cE 'ZUPT\]: accepted' openvins.log)"
  echo "  ---- sample [UPDATE-MSCKF] lines (feature usage) ----"
  grep -E '\[UPDATE-MSCKF\]' openvins.log | head -5 | sed 's/^/    /'
  echo "  ---- tracker feature counts (DEBUG only) ----"
  grep -E '\[TRACK\]|tracked|feats_last|num features|good features|of.*features' openvins.log | grep -viE '\[TIME\]' | head -8 | sed 's/^/    /'
  echo "  ---- filter internal final state ----"
  grep -E 'q_GtoI = .*dist =' openvins.log | tail -1 | sed 's/^/    /'
  echo "  ---- trajectory ----"
  local st; st=$(find "$d" -maxdepth 1 -name 'state_estimate.txt' | head -1)
  if [ -n "$st" ] && [ -s "$st" ]; then
    echo "  state_estimate.txt: $(wc -l < "$st") lines"
    python3 "$STRICT" "$st" --label "${name}[txt]" || true
  fi
  ros2 bag info out 2>/dev/null | grep -E 'Messages|odomimu' | sed 's/^/  /'
  if ros2 bag info out 2>/dev/null | grep -q odomimu; then
    python3 "$STRICT" out --label "${name}[bag]" || true
  fi
  cd "$DIAG"
}

run_variant base        "$DIAG/cfg_base"
run_variant dyninit     "$DIAG/cfg_dyninit"
run_variant dispwide    "$DIAG/cfg_dispwide"
run_variant dyninit_dbg "$DIAG/cfg_dyninit_dbg"

echo; echo "########################## REFERENCE: original live run ##########################"
python3 "$STRICT" "$BAG" --label "live_v6sept" || true

kill_estimators
echo; echo "DONE -> $DIAG/REPORT.txt"
