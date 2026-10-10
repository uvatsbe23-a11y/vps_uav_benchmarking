# flight_common.sh - shared helpers for flight_start / flight_status /
# flight_record / flight_stop.  Sourced, not run.
#
# Every process is started with  setsid nohup ... > <log> < /dev/null  so it is
# NOT attached to the SSH terminal: an SSH/Wi-Fi drop can neither freeze it
# (blocked terminal output) nor kill it (SIGHUP). Output goes to log files in
# the session folder:  ~/flights/<date>/flightNN_HHMMSS/   (~/flights/current
# points at the active one). Processes are tracked by PID files, never pkill -f.

set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_LOCALHOST_ONLY=1      # mandatory on vajra (7 NICs -> DDS discovery fails otherwise)

FLIGHTS="$HOME/flights"
CUR="$FLIGHTS/current"
PKG_LIB="$(ros2 pkg prefix vps_uav_benchmarking)/lib/vps_uav_benchmarking"

red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
yel()   { printf '\033[1;33m%s\033[0m\n' "$*"; }

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }   # $1 = pid file

# start_bg <session_dir> <name> <command...> : detached start, pid -> <name>.pid
start_bg() {
  local dir=$1 name=$2; shift 2
  ( cd "$dir" && ulimit -c unlimited && exec setsid nohup "$@" > "$dir/$name.log" 2>&1 < /dev/null ) &
  echo $! > "$dir/$name.pid"
}

# stop_pid <pidfile> <seconds> : SIGINT, wait, then SIGKILL. Returns 0 if it exited on SIGINT.
stop_pid() {
  local pf=$1 t=$2 p
  alive "$pf" || return 0
  p=$(cat "$pf"); kill -INT "$p" 2>/dev/null
  for _ in $(seq 1 "$t"); do kill -0 "$p" 2>/dev/null || return 0; sleep 1; done
  kill -KILL "$p" 2>/dev/null; sleep 1; return 1
}

ros_running() {   # any of our ROS processes running (pattern trick avoids matching ourselves)
  pgrep -af '[r]os2 launch vps_uav_benchmarking|[r]os2 bag record|[r]un_subscribe_msckf|[c]omponent_container.*camera'
}

clock_state() {   # prints OK / RESTORED / BAD
  if [ "$(date +%Y)" -lt 2026 ]; then echo BAD
  elif timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes || [ -e /run/clock-synced ]; then echo OK
  else echo RESTORED; fi
}

topic_hz() {      # topic_hz <topic> <seconds> -> average rate or empty
  timeout "$2" ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}'
}
