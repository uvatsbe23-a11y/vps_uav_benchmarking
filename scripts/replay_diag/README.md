# Replay diagnostic harness (Jul–Sep 2026)

Scripts, OpenVINS config variants (`cfg_*`) and text result summaries (`*.out`,
`REPORT.txt`) from `~/vps_replay_diag4` on vajra. Bags and per-run outputs are
not included (370 MB). Notable variants: `cfg_stock` (Orbbec factory
calibration, the basis of the T_cam_imu revert), `cfg_klt_freq30` (process all
30 Hz frames), `cfg_hist_clahe` (contrast boost), `cfg_klt_loose` (wider KLT).
Paths inside the scripts point at vajra's home directory.
