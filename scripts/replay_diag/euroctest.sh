#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/euroctest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
EUCFG=~/ros2_ws/src/open_vins/config/euroc_mav/estimator_config.yaml
BAG=~/datasets/euroc/V1_01_easy_ros2
kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
d="$D/euroc"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$EUCFG" > ov.log 2>&1 & PG=$!
sleep 8
echo "ov topics: $(ros2 topic list --no-daemon 2>/dev/null | grep -c '^/ov_msckf/')"
ros2 bag record -o out /ov_msckf/odomimu /ov_msckf/poseimu > rec.log 2>&1 & REC=$!
sleep 3
timeout 150 ros2 bag play "$BAG" --rate 1.0
sleep 3
kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
echo "=== EUROC V1_01_easy result ==="
echo "  init OK/FAIL   : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
echo "  num_feats o,n  : $(grep -oE 'compute disp: [0-9]+,[0-9]+' ov.log | tail -8 | tr '\n' ' ')"
echo "  disparity      : $(grep -oE 'disparity is [0-9.]+,[0-9.]+' ov.log | tail -3 | tr '\n' ' ')"
echo "  UPDATE-MSCKF   : $(grep -c '\[UPDATE-MSCKF\]' ov.log)   SLAM: $(grep -c '\[UPDATE-SLAM\]' ov.log)   ZUPT: $(grep -cE 'ZUPT\]: accepted' ov.log)"
grep -E '\[UPDATE-MSCKF\]' ov.log | head -5 | sed 's/^/    /'
echo "  final internal : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
ros2 bag info out 2>/dev/null | grep -E 'Messages|poseimu|odomimu' | sed 's/^/  /'
ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "euroc_V1_01"
kill_all; echo; echo "EUROCTEST DONE"
