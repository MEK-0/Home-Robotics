# Validation summary

The current evidence is: **92 deterministic simulation tests; 56 ROS package
tests; one 10-trial full-stack simulated cube benchmark with 9 successful trials
and 1 planning/retiming failure.** These results describe different checks and
should not be read as a single aggregate metric.

## Automated tests

| Layer | Evidence | Result |
| --- | --- | --- |
| Deterministic simulation | `tests/simulation` | 92 passed |
| ROS package tests | `colcon test` | 56 tests; 0 errors; 0 failures; 1 existing skipped |
| Phase 5 Action layer | focused Task Executor suite | 10 passed |

Simulation tests exercise configuration, scene structure, control, reset,
contact, rail constraints, and benchmark utilities without a ROS runtime.
ROS package tests exercise package contracts and component behavior after the
workspace build. The focused Action tests use a fake manipulation adapter; they
do not start MuJoCo, MoveIt, or controllers.

## Runtime integration and manual checks

Runtime integration starts the Phase 2 control stack, `move_group`, static
PlanningScene, dynamic object synchronization, and the manipulation or Action
entry point. It checks end-to-end configuration and runtime interfaces. Manual
RViz checks are visual inspection of the robot, static geometry, and object
synchronization. They are useful checks, but are not automated tests.

## Phase 4 physical simulation benchmark

The cube benchmark ran ten independent full-stack simulated trials to
`surface_left_2`: nine completed successfully; trial 2 failed before motion
during MoveIt TOTG retiming, reporting negative path velocity on a hypothetical
transport trajectory. The failure is retained and is not counted as a grasp or
placement failure.

Successful trials had final position errors from 1.416 to 1.923 mm (mean
1.598 mm). Raw reports and the per-trial table are preserved in
[archive/evidence/validation/phase4](archive/evidence/validation/phase4).

`purple_ball` has one physical grasp/lift/return baseline check. It is not a
named-placement benchmark and does not expand the Phase 5 Action scope.

## Reproduction commands

```bash
cd ~/Home-Robotics
source .venv/bin/activate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/simulation

source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

The test totals above are recorded validation results. Re-running the full stack
requires a clean ROS domain and a graphical environment only when RViz/viewer
inspection is desired.
