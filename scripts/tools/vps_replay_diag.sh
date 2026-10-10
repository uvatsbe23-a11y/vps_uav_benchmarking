#!/usr/bin/env bash
set +u
mkdir -p ~/vps_replay_diag && cd ~/vps_replay_diag
exec > >(tee ~/vps_replay_diag/REPORT.txt) 2>&1

echo "=== ENV ==="
for s in /opt/ros/*/setup.bash; do [ -f "$s" ] && { source "$s"; echo "sourced $s"; break; }; done
for w in ~/ros2_ws ~/vps_ws ~/colcon_ws ~/dev_ws; do
  [ -f "$w/install/setup.bash" ] && { source "$w/install/setup.bash"; echo "sourced $w"; }
done
echo "ROS_DISTRO=$ROS_DISTRO"
command -v ros2 || { echo "!! ros2 not on PATH - tell me what you normally source and stop here"; exit 1; }
ros2 pkg prefix ov_msckf || { echo "!! ov_msckf not found"; exit 1; }
ros2 pkg prefix vps_uav_benchmarking || { echo "!! vps_uav_benchmarking not found"; exit 1; }

BAG=$(dirname "$(find ~ -name metadata.yaml -path '*stage1_axes_v6sept*' 2>/dev/null | head -1)")
echo "BAG = $BAG"
[ -d "$BAG" ] || { echo "!! bag not found - edit BAG= and rerun"; exit 1; }

SHARE=$(ros2 pkg prefix vps_uav_benchmarking)/share/vps_uav_benchmarking/config
rm -rf cfg_ascfg cfg_imuonly
cp -r "$SHARE" cfg_ascfg
cp -r "$SHARE" cfg_imuonly
sed -i 's/^max_cameras:.*/max_cameras: 0/'   cfg_imuonly/estimator_config.yaml
sed -i 's/^use_stereo:.*/use_stereo: false/' cfg_imuonly/estimator_config.yaml
echo "=== config diff (ascfg -> imuonly) ==="
diff cfg_ascfg/estimator_config.yaml cfg_imuonly/estimator_config.yaml

RAW="/camera/left_ir/image_raw /camera/right_ir/image_raw /camera/left_ir/camera_info /camera/right_ir/camera_info /camera/gyro_accel/sample"
FA="$(ros2 pkg prefix vps_uav_benchmarking)/lib/vps_uav_benchmarking/flight_analysis.py"

run_replay () {
  name="$1"; cfg="$2"
  echo; echo "########################## REPLAY: $name ##########################"
  d=~/vps_replay_diag/$name; rm -rf "$d"; mkdir -p "$d"; cd "$d"
  ros2 run ov_msckf run_subscribe_msckf --ros-args \
      -p config_path:="$cfg/estimator_config.yaml" -p use_sim_time:=false \
      > openvins.log 2>&1 &
  OV=$!
  sleep 6
  ros2 bag record -o out /ov_msckf/odomimu /ov_msckf/poseimu /ov_msckf/pathimu \
      /ov_msckf/points_msckf /ov_msckf/points_slam > rec.log 2>&1 &
  REC=$!
  sleep 2
  ros2 bag play "$BAG" --rate 0.5 --topics $RAW
  sleep 4
  kill $REC 2>/dev/null; sleep 2; kill -9 $REC 2>/dev/null
  kill $OV  2>/dev/null; sleep 3; kill -9 $OV  2>/dev/null
  sleep 1
  echo "---- init lines ----"
  grep -nE 'init|nitializ|waiting for|successful' openvins.log | head -40
  echo "---- update lines (feature counts) ----"
  grep -E 'MSCKF update|SLAM update|ZUPT|for total' openvins.log | head -25
  echo "MSCKF-update lines: $(grep -c 'MSCKF update' openvins.log)"
  echo "SLAM-update  lines: $(grep -c 'SLAM update'  openvins.log)"
  echo "ZUPT lines        : $(grep -c 'ZUPT' openvins.log)"
  echo "---- WARN/ERROR ----"
  grep -nE 'WARN|ERROR|FATAL' openvins.log | head -40
  echo "---- openvins.log tail ----"
  tail -25 openvins.log
  echo "---- bag info: fresh output ----"
  ros2 bag info out
  echo "---- flight_analysis: fresh output ----"
  python3 "$FA" out || true
  cd ~/vps_replay_diag
}

run_replay ascfg   ~/vps_replay_diag/cfg_ascfg
run_replay imuonly ~/vps_replay_diag/cfg_imuonly

echo; echo "########################## ORIGINAL LIVE RUN (reference) ##########################"
python3 "$FA" "$BAG" || true
echo; echo "DONE -> ~/vps_replay_diag/REPORT.txt"
