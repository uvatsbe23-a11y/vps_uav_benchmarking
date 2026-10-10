#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/stocktest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
STOCK=~/ros2_ws/src/open_vins/config/orbbec_335l_stereo_ir
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

C=$D/cfg_stock; rm -rf "$C"; cp -r "$STOCK" "$C"
# fix line-1 to plain %YAML:1.0 (stock has a trailing comment that can trip cv::FileStorage in some builds - keep as-is actually, OV handles it)
sed -i 's/^verbosity:.*/verbosity: "INFO"/' "$C/estimator_config.yaml"
sed -i 's/^save_total_state:.*/save_total_state: true/' "$C/estimator_config.yaml"
# point kalibr topics at our bag - do it in python to be exact
python3 - "$C/kalibr_imucam_chain.yaml" "$C/kalibr_imu_chain.yaml" <<'PY'
import sys
ic, im = sys.argv[1], sys.argv[2]
t = open(ic).read()
import re
t = re.sub(r'rostopic:\s*\S*cam0\S*', 'rostopic: /camera/left_ir/image_raw', t)
t = re.sub(r'rostopic:\s*/cam0/\S+',  'rostopic: /camera/left_ir/image_raw', t)
t = re.sub(r'rostopic:\s*/cam1/\S+',  'rostopic: /camera/right_ir/image_raw', t)
open(ic,'w').write(t)
u = open(im).read()
u = re.sub(r'rostopic:\s*\S+', 'rostopic: /camera/gyro_accel/sample', u)
open(im,'w').write(u)
print("=== kalibr_imucam rostopics ==="); [print(l) for l in open(ic) if 'rostopic' in l]
print("=== kalibr_imu rostopic ===");     [print(l) for l in open(im) if 'rostopic' in l]
print("=== stock cam0 T_cam_imu ===")
import itertools
lines=open(ic).read().splitlines()
i=lines.index([l for l in lines if 'T_cam_imu' in l][0])
print("\n".join(lines[i:i+5]))
PY

kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
run(){ local bag=$1
  echo; echo "########## STOCK orbbec_335l_stereo_ir  vs  $bag ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$D/stock_$bag"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$C/estimator_config.yaml" > ov.log 2>&1 & local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/poseimu /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!
  sleep 3
  timeout 130 ros2 bag play ~/$bag --rate 0.5 --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  ERRORs      : $(grep -E 'ERROR|FATAL|unable to open' ov.log | head -3)"
  echo "  final intern: $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "stock_$bag"
  cd "$D"; }
run stage1_axes_6sept
run stage1_axes
kill_all; echo; echo "STOCKTEST DONE"
