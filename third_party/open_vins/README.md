# OpenVINS: local changes on vajra

vajra builds OpenVINS from upstream `rpng/open_vins` at `6948812` (v2.7-20) plus
the patches here (branch `jetson-local-patches` on the Jetson).

| Patch | What |
|---|---|
| `0001-…` | exit without unwinding after shutdown (first shutdown-race fix) |
| `0002-…` | exit immediately on SIGINT/SIGTERM: fixes segfault/abort/hang on stop |

Apply: `cd open_vins && git checkout 6948812 && git am ../vps_uav_benchmarking/third_party/open_vins/patches/*.patch`,
then `colcon build --packages-select ov_msckf`.

`config/`: extra OpenVINS config folders added to the Jetson's open_vins checkout
(not upstream). `orbbec_335l_stereo_ir` is the source of the factory T_cam_imu now used in `config/`.
