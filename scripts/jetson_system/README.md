# Jetson system setup (vajra)

Copies of system files installed outside the ROS workspace. Install with sudo:

| File | Installs to | Purpose |
|---|---|---|
| `clock-restore` | `/usr/local/sbin/` | Offline clock backstop: never let the clock fall below the last saved time; never save an older time |
| `clock-restore.service` | `/etc/systemd/system/` | Restore early in boot |
| `90-clock-restore.rules` + `clock-restore-rtc@.service` | `/etc/udev/rules.d/`, `/etc/systemd/system/` | Restore again when an RTC registers: the battery-less `nvvrs-pseq-rtc` driver resets the clock to 1970 ~0.3 s after the early restore. Must be a systemd service: udev `RUN+=` cannot set the clock |
| `clock-restore-late.{service,timer}` | `/etc/systemd/system/` | Backstop restore 20 s after boot |
| `clock-restore-save.{service,timer}` | `/etc/systemd/system/` | Save the time every minute (and at shutdown via `clock-restore.service`) |
| `60-core-pattern.conf` | `/etc/sysctl.d/` | Core dumps to `/var/crash/cores/` (dir mode 1777). Also: `systemctl disable apport` and `enabled=0` in `/etc/default/apport`, or apport overrides the pattern |

Then: `systemctl daemon-reload; udevadm control --reload; systemctl enable clock-restore.service clock-restore-late.timer clock-restore-save.timer`.

The restored time is only the last saved time (behind by the downtime). For the
real time run `scripts/flight/sync_jetson_clock.sh` on the ground PC; it also
creates `/run/clock-synced`, which `flight_start` and `check_setup.sh` check.

## Other tools

| File | Installs to | Purpose |
|---|---|---|
| `wifi-mode` | `/usr/local/bin/` | `wifi-mode flight` (MOTHER_DRONE access point, 10.42.0.1) / `wifi-mode internet [SSID]` (joins TU by default) / `wifi-mode status`. Uses Wi-Fi passwords already saved in NetworkManager; none are stored in the script. |

`scripts/tools/`: `pose_log.sh` (print OpenVINS pose once a second), `verify_axis_fix.sh`,
`replay_axistest.sh`, `vps_replay_diag*.sh` (replay helpers from the Jul–Sep diagnosis).
`docs/axis_test_steps.txt`: the Stage-1 handheld axis test procedure (Sep 6).
