# Flight videos (handover, Nov 2026)

Side-by-side videos: left, the downward IR camera with the corners OpenVINS uses (green); right, a top-down
map of the GPS path (orange) against OpenVINS (blue), with the current error. Made with
[`scripts/analysis/make_flight_video.py`](../../scripts/analysis/make_flight_video.py). Drift figures are horizontal.

| File | Flight | Shows |
|---|---|---|
| `oct10_flight1_worst_side_by_side_HQ_slow.mp4` | 10 Oct 2026, flight 1 (worst: 289 m after 399 m = 72 %) | 5 s at the first corner, 0.25x speed: 4.7 m high at 10 m/s, OpenVINS loses track despite rich texture (image motion too fast) |
| `nov1_flight4_best_side_by_side_HQ_slow.mp4` | 1 Nov 2026, flight 4 (best: 0.3 m after 163 m = 0.2 %) | 5 s arriving back over take-off, 0.25x speed: error closes to 0.3 m |
| `worst_vs_best_HQ_slow.mp4` | both, worst above best | the two videos above, stacked and synchronised (3104 x 2160) |
| `flight1_side_by_side_t57-67_HQ.mp4` | 1 Nov 2026, flight 1 | 10 s real time on the fast outbound leg, error grows to ~6.5 m from the compass heading error |
