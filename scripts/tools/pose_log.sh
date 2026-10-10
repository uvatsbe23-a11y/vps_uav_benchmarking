#!/usr/bin/env bash
# Live monitor for the axis test. Run in a 3rd terminal during the handheld motion.
# Prints ~1 Hz:  t(s)  x  y  z  |dist-from-start|   (metres). Ctrl-C when done, then paste the output.
set +u
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1
python3 - <<'PY'
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
class W(Node):
    def __init__(self):
        super().__init__('pose_log'); self.t0=None; self.p0=None; self.last=-1
        self.create_subscription(PoseWithCovarianceStamped,'/ov_msckf/poseimu',self.cb,20)
        print(f"{'t(s)':>6} {'x':>8} {'y':>8} {'z':>8} {'dist0':>8}")
    def cb(self,m):
        s=m.header.stamp; t=s.sec+s.nanosec*1e-9
        p=m.pose.pose.position; xyz=(p.x,p.y,p.z)
        if self.t0 is None: self.t0=t; self.p0=xyz
        if t-self.t0-self.last>=1.0:
            self.last=t-self.t0
            d=math.dist(xyz,self.p0)
            print(f"{t-self.t0:6.1f} {p.x:8.3f} {p.y:8.3f} {p.z:8.3f} {d:8.3f}")
rclpy.init(); 
try: rclpy.spin(W())
except KeyboardInterrupt: pass
PY
