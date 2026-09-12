# Phase 3.9 — independent cross-robot collision validation

Validated on 2026-09-12. Phase 3.9 passes; **Phase 3 is not complete** and Phase 3.10 final acceptance is still separate.

## Architecture and reproduction

`dual_robot_collision_validation` is a separate C++ executable in `home_robotics_moveit_config`, with its own launch file. It uses only `panda1_manipulator` or `panda2_manipulator` (8 DOF each). The other robot is part of the full RobotState collision geometry. It downloads the live PlanningScene including the ACM and verifies 14 static world objects. Current and target states use both local global PlanningScene collision checks and `/check_state_validity`. The overlap test additionally checks each group scope on both sides. Contacts are printed for invalid states.

Start the existing Phase 2 stack, `move_group.launch.py`, and `planning_scene_environment.launch.py` in the order specified in the project instructions. Confirm `All Phase 2 controllers are active`. In a sourced ROS shell, run these sequentially:

```bash
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=current
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=safe_panda1 execute:=true
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=safe_panda2 execute:=true
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=cross_collision
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=path_collision
ros2 launch home_robotics_moveit_config dual_robot_collision_validation.launch.py mode:=reverse_path_collision
```

`execute` defaults to false. Non-safe modes reject `execute:=true` before constructing MoveGroupInterface; a regression test verifies this guard. A successful plan is checked at every waypoint and interpolated at a maximum 0.005 change per active joint (metres/radians). Each sample includes the unchanged other robot and the static world. Safe samples must also maintain signed rail separation >= 0.7 m. Collision checking has no padding/tolerance relaxation. Joint-bound comparisons permit 1e-6 numerical roundoff without clamping positions or changing model limits.

Safe execution uses MoveGroupInterface → MoveItSimpleControllerManager → FollowJointTrajectory → existing ros2_control → MuJoCo. No direct trajectory action client exists in this node. A `/joint_states` subscription records maximum stationary robot drift during planning/execution, including fingers. Fresh final states are checked for collisions and position errors. The node waits for rail error <= 0.0001 m and arm error <= 0.001 rad (up to 500 samples), and requires stationary drift <= 0.002 in each joint's units.

## Measured results

Home: Panda1 rail **-0.90000000 m**, Panda2 rail **+0.90000000 m**, separation **1.80000000 m**, full state **VALID**, cross-robot contacts **0**.

Target vectors below use `[rail, joint1, joint2, joint3, joint4, joint5, joint6, joint7]`; rail is in metres and arm angles in radians.

| Result | Safe Panda1 | Safe Panda2 |
|---|---|---|
| Target | `[-0.85, 0.10, -0.10, 0.10, -1.50, 0.05, 1.50, -0.70]` | `[0.85, -0.10, 0.10, -0.10, -1.50, -0.05, 1.50, -0.70]` |
| Target full-state validity | VALID | VALID |
| Plan / execution | SUCCESS / SUCCESS | SUCCESS / SUCCESS |
| Planner time | 0.013807 s | 0.017167 s |
| Trajectory waypoints / checked samples | 30 / 38 | 30 / 38 |
| Cross-robot contacts in checked samples/final state | 0 | 0 |
| Final rail error | 0.000099249 m | 0.000099307 m |
| Maximum final arm error | 0.000093294 rad | 0.000084147 rad |
| Other robot maximum joint drift | 0.000000000 | 0.000003000 |

Panda2 started after Panda1 settled: rails -0.85005730 and +0.90000000 m, separation 1.75005730 m. The drift statistic is the maximum of per-joint absolute changes; each constituent retains its metre/radian unit.

## Deterministic invalid state

Only an offline RobotState is changed:

- Panda1: `[0, 0.10, -0.10, 0.10, -1.50, 0.05, 1.50, -0.70]`.
- Panda2: `[0, -0.10, 0.10, -0.10, -1.50, -0.05, 1.50, -0.70]`.
- All four finger variables: `0.04 m`.

Joint bounds are satisfied. Rail separation is **0 m**, below the hardware safety threshold. The state is **INVALID** globally and in both manipulator group scopes, with the following 11 cross-robot contact pairs:

```text
panda1_carriage ↔ panda2_carriage
panda1_link0 ↔ panda2_link0
panda1_link1 ↔ panda2_link1
panda1_link1 ↔ panda2_link2
panda1_link2 ↔ panda2_link1
panda1_link2 ↔ panda2_link2
panda1_link2 ↔ panda2_link3
panda1_link3 ↔ panda2_link2
panda1_link3 ↔ panda2_link3
panda1_link3 ↔ panda2_link4
panda1_link4 ↔ panda2_link3
```

The invalid state was never executed, published to the hardware, or commanded through ros2_control.

## Cross-robot path tests

These are entirely offline planning scenarios. To place the obstacle within the moving robot's limited rail range, only the stationary robot's rail position is replaced in the request's full start RobotState; its arm retains measured values. The live scene and physical robot state are not changed. Full exact start and goal vectors are in the linked logs.

| Scenario | Panda1 active | Panda2 active |
|---|---|---|
| Offline stationary rail | Panda2 = -0.30 m | Panda1 = +0.30 m |
| Moving rail goal | +0.34 m | -0.34 m |
| Arm goal | Unchanged measured arm | Unchanged measured arm |
| Start / goal geometry | VALID / VALID | VALID / VALID |
| Direct-route collision | panda1_carriage ↔ panda2_carriage | panda1_carriage ↔ panda2_carriage |
| Planner outcome | No solution, OMPL TIMED_OUT | No solution, OMPL TIMED_OUT |
| Observed wall time | 10.004925 s | 10.006588 s |
| Returned trajectory points | 0 | 0 |
| Returned waypoint check | N/A — no trajectory | N/A — no trajectory |
| Execution | Prohibited | Prohibited |

The action-level failure code is 99999; the move_group log explicitly confirms the underlying OMPL timeout. Both endpoints are valid, so this is path obstruction rather than goal collision rejection. The fixed carriage geometries cannot exchange rail ordering without overlap. No alternate path was returned and no colliding trajectory was accepted. Sampling is discrete; this work does not claim a continuous swept-volume proof for arbitrary future trajectories.

## Rail safety and ACM

The runtime effective ACM checks **196** Panda1/Panda2 link pairs and rejects any allowed cross-robot pair. The existing SRDF protection test also passes. No ACM entry, collision mesh, environment geometry, controller, hardware code, or Phase 2 architecture was changed.

The hardware still requires **panda2_rail - panda1_rail >= 0.7 m**, independently of MoveIt geometry. Existing regression tests for rail crossing rejection, preservation of the previous target, boundary motion, and the hardware parameter declaration remain green. This task did not send an unsafe command to test the hardware live. Offline overlap/path cases intentionally violate the separation/order rule, demonstrating MoveIt geometric checks without invoking or bypassing the hardware layer. Safe executable trajectories are additionally checked against 0.7 m.

## Regression and limitations

- Simulation: **75 passed** (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/simulation`).
- `python -m colcon build`: **5 packages built**.
- `colcon test` / `colcon test-result --verbose`: **18 tests, 0 errors, 0 failures, 1 skipped**.
- The skipped test is the pre-existing generated-source copyright check in `home_robotics_control`.
- `git diff --check`: passed.
- Test-owned Phase 2 and move_group launch processes were stopped after collecting evidence.
- Existing warnings include unavailable FIFO realtime scheduling, deprecated gripper controllers, MoveIt config inference/default workspace messages, and TOTG mixed revolute/prismatic path-tolerance warning. No controller or timing configuration was changed.
- Initial diagnostic runs exposed approximately machine-scale finger-bound excess and residual rail settling. The validator now explicitly tolerates 1e-6 bounds roundoff and waits for the tighter final rail error before completing a safe test. The final series passes without widening stationary drift tolerance.
- No combined 16-DOF group, simultaneous command, coordinated dual-arm execution, synchronization, pick-and-place, or direct FollowJointTrajectory request was implemented.

Raw final evidence: [home](validation/phase3_9/current.txt), [safe Panda1](validation/phase3_9/safe1.txt), [safe Panda2](validation/phase3_9/safe2.txt), [overlap](validation/phase3_9/cross.txt), [Panda1 path](validation/phase3_9/path1.txt), [Panda2 path](validation/phase3_9/path2.txt), [MoveIt server excerpt](validation/phase3_9/movegroup.txt), [controllers](validation/phase3_9/control-final.txt), [environment](validation/phase3_9/environment.txt), [simulation tests](validation/phase3_9/simulation-tests.txt), [build](validation/phase3_9/build.txt), [ROS test result](validation/phase3_9/ros-results.txt).
