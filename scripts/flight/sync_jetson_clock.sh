#!/usr/bin/env bash
# Set the Jetson's clock from this PC (which is NTP-synced).
#   usage: sync_jetson_clock.sh [host]      host defaults to 192.168.55.1 (USB);
#                                           use 10.42.0.1 over MOTHER_DRONE Wi-Fi
# Asks for the Jetson sudo password. Refuses if OpenVINS / camera / recorder are
# running (a clock jump under a live recording corrupts its timestamps).
host=${1:-192.168.55.1}
pc_epoch=$(date +%s.%N)
ssh -t "vajra@$host" "
  t0=\$(date +%s.%N)
  if pgrep -f '[r]os2 launch|[r]os2 bag record|[r]un_subscribe_msckf' >/dev/null; then
    echo 'REFUSED: ROS processes are running - stop them first (flight_stop).'; exit 1; fi
  sudo -v || exit 1
  # PC time + time elapsed on the Jetson since this session started
  target=\$(awk -v p=$pc_epoch -v a=\$t0 -v b=\$(date +%s.%N) 'BEGIN{printf \"%.3f\", p + (b - a)}')
  sudo date -s @\$target >/dev/null && sudo /usr/local/sbin/clock-restore save && sudo touch /run/clock-synced
  echo \"Jetson clock set: \$(date)\""
echo "PC clock:             $(date)"
