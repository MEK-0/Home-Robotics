# Phase 4.2 — Grasp-pose runtime validation

> Historical subphase validation snapshot. Current transport/place capabilities and closure status are documented in [PHASE4_RELIABILITY](PHASE4_RELIABILITY.md).

**Phase 4.2 — COMPLETE (planning-only). Phase 4 remains incomplete.**

Validated on 2026-09-16 with Panda1, cube, top-down, `execute_pregrasp:=false`, `validate_grasp:=true`, `keep_alive:=false`. Optional execution was not performed. No Phase 4.3 work was started.

## Readiness failure and fix

The historical [failure log](validation/phase4_2/previous_state_failure.txt) shows CurrentStateMonitor listening to `joint_states`, requesting simulation time 306.280 s, but receiving no state (latest timestamp 0) within 10 s. It also records negative elapsed simulation times, indicating clock discontinuities in that earlier runtime. This failure occurred before IK/planning. The evidence does not distinguish an absent publisher from discovery/subscription delivery failure; it does not establish the source of the clock jumps. The executor was already spinning, the topic name was correct, and the launch already enabled `use_sim_time`.

With a single Phase 2 runtime started before MoveIt and scene loaders, the unchanged validation first passed. The readiness fix removes the redundant lazy CurrentStateMonitor dependency from initial state acquisition. The validation reads authoritative `/joint_states` directly into a RobotState, requiring every model variable (including all eight Panda1 manipulator joints, Panda2 and fingers) to be present and finite. It uses a 30 s steady-clock deadline, nonzero simulation clock/stamp, and timestamp age between -0.1 and 0.5 s. Failure raises `ROBOT_STATE_TIMEOUT` with a reason before IK/planning. No home state or fabricated joint positions are used. The same check guards later fresh-state reads.

Cube comparison now prints the actual PlanningScene pose and requires angular disagreement <= 1e-6 rad, alongside position error <= 0.002 m and fresh authoritative cube data. Existing controller, planning groups, launch architecture, generator, and dynamic sync were preserved.

## Startup and prechecks

Use the existing ROS environment (`.ros_venv`, `/opt/ros/jazzy/setup.bash`, `ros2_ws/install/setup.bash`) in each terminal, in this order:

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
ros2 launch home_robotics_moveit_config move_group.launch.py
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
ros2 topic echo /joint_states --once
ros2 topic echo /mujoco/objects/cube/pose --once
ros2 launch home_robotics_manipulation grasp_pose_validation.launch.py execute_pregrasp:=false keep_alive:=false
```

Do not launch another Phase 2 runtime when one is already running. Prechecks returned all 20 joint positions at stamp 17.20 s and the world-frame cube pose at 18.76 s. The final validation acquired all variables at stamp 221.90 s, age 0 s; final fresh state was 222.08 s, age 0 s.

## Measured runtime results

Full evidence: [plan-only log](validation/phase4_2/plan_only.txt). Positions are metres in `world`; quaternion order is XYZW.

| Measurement | Result |
|---|---|
| MuJoCo cube XYZ | [0.074999995838, 0.410000034854, 0.704892244580] |
| MuJoCo cube quaternion | [0.000000000094, 0.000000000011, approximately 0, 1] |
| PlanningScene cube XYZ | [0.074999995841, 0.410000034830, 0.704892244580] |
| PlanningScene cube quaternion | [0.000000000103, 0.000000000012, approximately 0, 1] |
| Initial pose sync error | 2.48606604387e-11 m; 1.81788455999e-11 rad; age 0 s |
| Pre-grasp XYZ | [0.074999995838, 0.410000034854, 0.815792244580] |
| Grasp XYZ | [0.074999995838, 0.410000034854, 0.715792244580] |
| Both target quaternions | [0.707106781187, 0.707106781187, 0, 0] |
| Planning group / TCP | panda1_manipulator / panda1_tcp |
| Pre-grasp IK / bounds | PASS / OK |
| Pre-grasp collision validity | VALID locally and through MoveIt state-validity service; 0 contact pairs |
| Pre-grasp plan | SUCCESS; OMPL RRTConnect; 0.014784936 s; 105 trajectory points |
| Planned trajectory checking | 470 samples, no touch exceptions |
| Grasp IK | PASS |
| Grasp NORMAL | VALID; bounds OK; 0 contact pairs |
| Grasp ALLOWED_TOUCH | VALID; bounds OK; 0 contact pairs |
| Approach | PASS; 21 Cartesian IK samples + 210 joint interpolation samples; not executed |
| Pre-grasp execution | Not performed (optional) |
| Final TCP position/orientation error | Not measured; no execution |
| Panda2 maximum observed joint drift | 2.1e-11 (metres for rail / radians for revolute joints) |
| Panda1 maximum finger drift | 1.2e-11 m |

Eight-DOF pre-grasp IK solution (rail in metres, joints in radians):

```text
panda1_rail_joint = -1.058957337769
panda1_joint1     =  0.041313087494
panda1_joint2     =  0.851143114416
panda1_joint3     =  0.024331860003
panda1_joint4     = -2.423047723099
panda1_joint5     =  0.137689230630
panda1_joint6     =  3.275294655037
panda1_joint7     =  0.706275078491
```

## Collision interpretation and scope

The cube width is 0.05 m and the unchanged open gripper width is 0.08 m. Consequently the generated open-gripper grasp and approach are collision-free even under NORMAL checking. There is no collision onset and the exact contact-pair set is empty, including the final approach region. This is an open-gripper pose validation, not a physical grasp-success test.

The temporary local allowed-touch matrix includes only cube ↔ panda1_left_finger and cube ↔ panda1_right_finger. Panda1 hand was not added. The global ACM remained unchanged, no SRDF edits or broad collision exemptions were made, and Panda2 remained included in full-robot collision checking. Plan joint names were checked to contain only Panda1's eight arm/rail joints. No trajectory was executed or sent to Panda2.

No GripperCommand was sent, no gripper closure occurred, no MuJoCo contact inspection for grasp success was used, and no constraint was created. Cube stayed a world object; the final scene had no attached objects. No attach, approach execution, lift, transport, or place occurred.

## Tests and remaining limitations

- [Simulation](validation/phase4_2/simulation_tests.txt): **75 passed**, 14.51 s, plugin autoload disabled as requested.
- [Build](validation/phase4_2/build.txt): **6 packages built successfully**.
- [ROS](validation/phase4_2/ros_results.txt): **31 tests, 0 errors, 0 failures, 1 skipped**. No tests were weakened.
- Existing startup warnings include missing FIFO realtime privileges and MoveIt URDF/SRDF inference fallbacks; the initial unchanged-code run briefly warned about state-monitor delivery before succeeding. Joint-state effort fields are NaN; finite measured position fields are used.
- [Missing-state negative runtime check](validation/phase4_2/missing_state.txt): remapping only the validation subscriber to an empty test topic produced `ROBOT_STATE_TIMEOUT after 30 s: no /joint_states message`, process exit 1, with no IK/planning. The real publisher remained untouched.
- Optional physical pre-grasp execution and final TCP tracking accuracy were not tested. Phase 4.2 completion here covers the required planning-only chain.
- The exact underlying transport/publisher cause of the historical no-message incident remains unproven; the new state acquisition reports a bounded explicit readiness failure instead of proceeding without data.
