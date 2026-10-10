#!/usr/bin/env bash
set +u
export ROS_LOCALHOST_ONLY=1
D=~/vps_replay_diag4; STRICT="$D/flight_analysis_strict.py"
OUT="$D/calibtest.out"; : > "$OUT"; exec >> "$OUT" 2>&1
for s in /opt/ros/*/setup.bash; do source "$s"; break; done
source ~/ros2_ws/install/setup.bash 2>/dev/null
LF=openvins_gemini335l.launch.py
VSHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
STOCK=~/ros2_ws/src/open_vins/config/orbbec_335l_stereo_ir
RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"

# --- cfg 1: STOCK orbbec config, but point topics at our bag's topics ---
C1=$D/cfg_ct_stock; rm -rf "$C1"; cp -r "$STOCK" "$C1"
sed -i 's#rostopic:.*left#rostopic: /camera/left_ir/image_raw #; ' "$C1/kalibr_imucam_chain.yaml" 2>/dev/null
sed -i 's#/cam0/image_raw#/camera/left_ir/image_raw#; s#/cam1/image_raw#/camera/right_ir/image_raw#' "$C1/kalibr_imucam_chain.yaml"
sed -i 's#rostopic:.*#rostopic: /camera/gyro_accel/sample#' "$C1/kalibr_imu_chain.yaml"
sed -i 's/^verbosity:.*/verbosity: "INFO"/' "$C1/estimator_config.yaml"

# --- cfg 2: VPS config but ONLY T_cam_imu rotation reverted to stock (near-identity) ---
C2=$D/cfg_ct_revert; rm -rf "$C2"; cp -r "$VSHARE" "$C2"
python3 - "$C2/kalibr_imucam_chain.yaml" "$STOCK/kalibr_imucam_chain.yaml" <<'PY'
import sys,re
tgt,src=sys.argv[1],sys.argv[2]
s=open(src).read(); t=open(tgt).read()
# pull the T_cam_imu 3x3 rotation (first 3 cols of each row) from stock, keep VPS translation
def rows(txt,cam):
    m=re.search(cam+r':\s*\n\s*T_cam_imu:\s*\n((?:\s*-\s*\[[^\]]*\]\s*\n){4})',txt)
    return re.findall(r'-\s*\[([^\]]*)\]',m.group(1))
for cam in ('cam0','cam1'):
    srow=rows(s,cam); trow=rows(t,cam)
    new=[]
    for i in range(4):
        sv=[x.strip() for x in srow[i].split(',')]
        tv=[x.strip() for x in trow[i].split(',')]
        if i<3:
            new.append(f"    - [{sv[0]}, {sv[1]}, {sv[2]}, {tv[3]}]")
        else:
            new.append(f"    - [{tv[0]}, {tv[1]}, {tv[2]}, {tv[3]}]")
    old_block=re.search(cam+r':\s*\n\s*T_cam_imu:\s*\n((?:\s*-\s*\[[^\]]*\]\s*\n){4})',t)
    t=t[:old_block.start(1)]+"\n".join(new)+"\n"+t[old_block.end(1):]
open(tgt,'w').write(t)
print("reverted rotation for cam0,cam1 in",tgt)
PY
sed -i 's/^init_dyn_use:.*/init_dyn_use: true/' "$C2/estimator_config.yaml"
echo "=== cfg_ct_revert kalibr_imucam_chain.yaml ==="; sed -n '1,25p' "$C2/kalibr_imucam_chain.yaml"

kill_all(){ pkill -INT -f run_subscribe_msckf 2>/dev/null; pkill -INT -f "$LF" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f run_subscribe_msckf >/dev/null||break; sleep 0.5; done
  pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f "$LF" 2>/dev/null; sleep 1; }
run(){ local name=$1 cfg=$2 bag=$3
  echo; echo "########## $name  (bag $bag) ##########"
  ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2; kill_all
  local d="$D/ct_$name"; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  setsid ros2 launch vps_uav_benchmarking "$LF" config_path:="$cfg/estimator_config.yaml" > ov.log 2>&1 & local PG=$!
  sleep 8
  ros2 bag record -o out /ov_msckf/poseimu /ov_msckf/odomimu > rec.log 2>&1 & local REC=$!
  sleep 3
  timeout 130 ros2 bag play ~/$bag --rate 0.5 --topics $RAW
  sleep 3
  kill -INT "$REC" 2>/dev/null; sleep 2; kill -KILL "$REC" 2>/dev/null
  kill -INT -"$PG" 2>/dev/null; sleep 3; kill_all
  echo "  init OK/FAIL : $(grep -c 'successful initialization' ov.log) / $(grep -cE 'failed .*init' ov.log)"
  echo "  ERR/WARN     : $(grep -cE 'ERROR|FATAL' ov.log)"
  echo "  final intern : $(grep -E 'dist =' ov.log | tail -1 | sed 's/.*q_GtoI/q_GtoI/')"
  ros2 bag info out 2>/dev/null | grep -q poseimu && python3 "$STRICT" out --odom-topic /ov_msckf/poseimu --label "$name"
  cd "$D"; }

run stock_6sept  "$C1" stage1_axes_6sept
run revert_6sept "$C2" stage1_axes_6sept
run revert_v6sept "$C2" stage1_axes_v6sept
kill_all; echo; echo "CALIBTEST DONE"
