# Dissertation section mapping

This page is a section-to-component index. It does not restate the dissertation
or extend its claims.

| Dissertation / evidence section | Repository component(s) |
| --- | --- |
| 3.3 Reference Trajectory | `PythonAPI/my_control_project/road_planning/` — route planning, reference construction, Frenet planning, tracking geometry, and reference tracking. |
| 3.4 Error Provider | `PythonAPI/my_control_project/error_providers/` — provider interface, ground-truth errors, and controlled noisy/perception-proxy perturbations. |
| 3.5 Adaptive Speed Planning | `PythonAPI/my_control_project/speed_planning/` — adaptive target-speed planning. `Speed_Planing/` remains a compatibility import path. |
| 3.6 Lateral Controller Implementation | `PythonAPI/my_control_project/control/` — PID, LQR, MPC, shared longitudinal control, and comparison baselines. |
| 3.8 Runtime, Logging and Evaluation | `PythonAPI/my_control_project/experiment/` — runtime, logging, termination, metrics, aggregation, reports, and evaluation workflows. |
| Software Testing | `PythonAPI/my_control_project/tests/` — unit and regression contracts for the project modules and scripts. |

## Supporting entrypoints and configuration

- Entrypoint: `PythonAPI/my_control_project/run_my_control.py`
- Configuration and CLI defaults: `PythonAPI/my_control_project/project_config.py`
- CARLA health entrypoint: `PythonAPI/my_control_project/carla_health.py`
- Runtime setup: `PythonAPI/setup_carla37.ps1`
- Run wrapper: `PythonAPI/my_control_project/scripts/run_my_control.ps1`
- Evaluation matrix: `PythonAPI/my_control_project/scripts/run_controller_evaluation.ps1`
- Module ablation: `PythonAPI/my_control_project/scripts/run_internal_module_ablation_90.ps1`
- Planner/test matrix: `PythonAPI/my_control_project/scripts/run_test_matrix.ps1`
