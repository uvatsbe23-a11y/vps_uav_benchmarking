#!/usr/bin/env bash
# record_bag.sh <output_name>
#
# Records EXACTLY the topic set needed to:
#   (a) measure drift now  (/ov_msckf/odomimu), and
#   (b) replay the bag through OpenVINS later (raw IR + camera_info + raw IMU).
#
# Recording ONLY the odometry output is what made the previous flight test
# impossible to re-analyse (docs/KNOWN_ISSUES.md #2). Always use this script.
set -eu

if [ $# -lt 1 ]; then
  echo "usage: $0 <output_name>   e.g.  $0 stage2_run1"
  exit 1
fi

exec ros2 bag record -o "$1" \
  /camera/left_ir/image_raw    /camera/left_ir/camera_info \
  /camera/right_ir/image_raw   /camera/right_ir/camera_info \
  /camera/gyro_accel/sample \
  /ov_msckf/odomimu  /ov_msckf/poseimu  /ov_msckf/pathimu \
  /tf /tf_static
