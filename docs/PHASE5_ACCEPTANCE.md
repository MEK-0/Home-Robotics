# Phase 5 Acceptance

Start the existing Phase 2, MoveIt, static-scene and dynamic-scene terminals from `docs/DEMO.md`, then:

```bash
ros2 launch home_robotics_task_executor task_executor.launch.py execute:=false
ros2 action send_goal --feedback /home_robotics/pick_and_place home_robotics_interfaces/action/PickAndPlace "{object_id: cube, target_id: surface_left_2}"
```

Planning-only must succeed without motion. For physical execution restart/reset the normal stack, launch with `execute:=true`, and submit the same goal. Expected: Phase 4 verified pick, lift, transport, place and final verification.

Invalid-object: submit `does_not_exist` / `surface_left_2`, expect OBJECT_NOT_FOUND and no motion. Invalid-target: submit `cube` / `does_not_exist`, expect TARGET_NOT_FOUND and no motion. Same-source target `surface_left_1` is INVALID_TARGET.

Busy: while a real goal is active submit a second valid goal; it is rejected by the Action server (single-active-task policy). Cancellation: in planning-only mode submit with `ros2 action send_goal --feedback` then cancel through the client at the pre-delegation boundary; expect CANCELED. Execution cancellation is intentionally rejected; reset the full stack after any interrupted physical run.

Automated coverage: `colcon test --packages-select home_robotics_task_executor` covers generated action headers, task vocabulary and authoritative object/target resolution. Runtime evidence (2026-09-23): planning-only Action succeeded with monotonic feedback; invalid object and target returned OBJECT_NOT_FOUND and TARGET_NOT_FOUND without delegation; planning-only cancellation returned CANCELED; a concurrent second goal was rejected; and execute=true cube to surface_left_2 completed successfully through Phase 4. The Phase 4 result reported 0.00120 m final position error, unique WORLD ownership, and zero Panda2 drift.


## Executed validation — 2026-09-23

The following full stack was started once, then shut down cleanly after the tests:

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
ros2 launch home_robotics_moveit_config move_group.launch.py
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
ros2 launch home_robotics_task_executor task_executor.launch.py execute:=false
```

| Case | Invocation | Observed result |
| --- | --- | --- |
| Planning-only | `cube` to `surface_left_2` | `SUCCEEDED`; feedback 5, 10, 15, 20, 35, 95, 100; no physical command. |
| Empty request | empty `object_id` | `FAILED / INVALID_REQUEST`; no Phase 4 delegation. |
| Invalid object | `does_not_exist` to `surface_left_2` | `FAILED / OBJECT_NOT_FOUND`; no Phase 4 delegation. |
| Invalid target | `cube` to `does_not_exist` | `FAILED / TARGET_NOT_FOUND`; no Phase 4 delegation. |
| Busy | second valid goal while first planning goal active | second Action goal rejected. |
| Cancellation | cancel a planning-only goal after `EXECUTING_PICK` feedback | cancel request accepted; terminal result `CANCELED`. |
| Real execution | restart executor with `execute:=true`; `cube` to `surface_left_2` | `SUCCEEDED` through the Phase 4 pipeline. |

The real-execution YAML report recorded `grasp_verified`, `lift_success`, `transport_success`, `placement_success`, and `retreat_success` as true. It recorded 0.00120055 m final position error, 0.0000771 rad orientation error, one WORLD cube, zero attached cubes, and 2.11e-11 m Panda2 maximum drift.

Regression commands completed after the Action-thread cancellation fix:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/simulation
# 92 passed

cd ros2_ws
python -m colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
# 47 tests, 0 errors, 0 failures, 1 skipped
```
