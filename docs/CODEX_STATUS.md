The ROBO-JAZZY home-CPU moving-wall C1 run passed at tested commit `0f9f1c4`: MoveIt stopped and replanned twice before completing the FEM cable pick.
The adapter now paces commands from Newton's reported queue rather than wall time; maximum backlog fell from `96` to `2` commands.
The cable lifted `0.120050 m` and dropped `0.120503 m` after release; `candidate_contact_grasp_pass=true`.
Newton measured minimum robot-wall surface separation `0.045716 m`, with zero loaded, penetrating, or accepted robot-wall contacts.
Transit/contact padding was `0.020/0.000 m`; the wall monitor is active only during transit while Newton collision accounting remains active for the whole run.
Evidence is under `docs/experiments/rigid-obstacles/results/step_c/attempt_20261022_0f9f1c4_moving_wall/`. C2 was not run by user instruction.
