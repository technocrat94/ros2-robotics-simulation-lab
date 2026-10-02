# Jazzy FEM migration evidence

Key artifacts for the verified 2026-10-02 native Jazzy run. The Humble CPU reference remains indexed in `docs/experiments/newton-ros2-bridge/results/`.

| Artifact | Purpose |
|---|---|
| `jazzy_fem_pick_success_2026-10-02.json` | Enriched success record: Humble CPU vs Jazzy `cuda:0`, versions, full Newton configuration, acceptance metrics, and repo-relative logs. |
| `jazzy_fem_pick_result_2026-10-02.json` | Standalone `MOVEIT_GRASP_RESULT` fields, including lift, release drop, bottom z, bilateral contacts, timing, and pass flag. |
| `jazzy_fem_pick_success_2026-10-02.md` | English and Traditional Chinese summary. |
| `jazzy_fem_preview_20261002_160453.log` | Plan-only preview; confirms no named-start detour and complete safe path fractions. |
| `jazzy_fem_pick_20261002_173033.log` | Clean Jazzy MoveIt run: bridge/guard, 100% paths, and `ABSOLUTE POSITION PICK SUCCEEDED`. |
| `jazzy_fem_endpoint_20261002_172956.log` | GPU Newton endpoint run kept alive through release settling; includes `MOVEIT_GRASP_RESULT`. |

Runtime viewer during the run: `http://127.0.0.1:30000/`.
