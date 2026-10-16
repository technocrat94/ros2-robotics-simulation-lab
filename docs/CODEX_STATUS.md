The cuRobo-only moving-cube demo ran on `cuda:0` for `30.000 s`: `150/150` replans succeeded (`100%`) and the arm completed `7` goal switches.
Replan mean / p95 / maximum was `89.674 / 97.029 / 99.598 ms`; the overall minimum cuRobo robot-to-cube clearance stayed positive at `0.021935 m`.
Each `0.2 s` cycle advanced `8` x `25 ms` samples; code and JSON/log evidence are under `docs/experiments/curobo-v2/`, and live Viser uses `http://127.0.0.1:8082/`.
