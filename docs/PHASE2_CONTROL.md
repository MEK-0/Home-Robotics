# Phase 2 ROS 2 Control Implementation

## Runtime architecture

Phase 2 uses one authoritative MuJoCo model/data instance owned by
`home_robotics_bringup/mujoco_runtime`. The validated Python simulation controllers remain the
only code that writes MuJoCo `data.ctrl`. A real `ros2_control` SystemInterface exchanges state
and command samples with that runtime on private transport topics:

```text
FollowJointTrajectory / GripperCommand
  -> controller_manager and standard controllers
  -> home_robotics_mujoco_hardware/MujocoTopicSystem
  -> /mujoco/command
  -> authoritative MuJoCo runtime
  -> validated rate-limited actuator controllers
  -> data.ctrl and MuJoCo physics
  -> /mujoco/joint_states
  -> ros2_control state interfaces
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf
```

The private transport topics are implementation details. Production clients use controller
actions, `/joint_states`, TF, `/clock`, and `/reset_simulation`.

## Controller layout

- `panda1_trajectory_controller`: `panda1_rail_joint` plus seven Panda 1 arm joints.
- `panda2_trajectory_controller`: `panda2_rail_joint` plus seven Panda 2 arm joints.
- `panda1_gripper_controller`: Panda 1 coupled finger position action.
- `panda2_gripper_controller`: Panda 2 coupled finger position action.
- `joint_state_broadcaster`: authoritative publisher for all 20 joint states.

Both robots and carriages are low-level-control active in Phase 2. Phase 7 adds high-level task
allocation, collision-aware coordination, synchronized manipulation, and shared-resource policy.

## Shared-rail safety

The hardware boundary enforces the configured `0.7 m` minimum carriage separation. Unsafe
simultaneous targets cause both rail commands to hold their measured positions. The authoritative
runtime independently validates the same invariant before accepting a command sample. Joint and
rail limits and target-rate limiting remain enforced by the validated MuJoCo control layer.

## Simulation time and reset

`/clock` is derived directly from `mjData.time`. Controller manager and robot-state publisher use
`use_sim_time`. Reset returns MuJoCo time to zero, cancels active trajectory and gripper goals,
briefly quarantines stale hardware commands, restores all physical and command state, and then
resumes clock publication from zero.

## Bringup

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
```

Set `use_viewer:=true` only in a graphical session. Normal bringup starts one MuJoCo runtime and
does not start the legacy `rail_state_publisher` utility. The production `/joint_states` publisher
is always `joint_state_broadcaster`.
