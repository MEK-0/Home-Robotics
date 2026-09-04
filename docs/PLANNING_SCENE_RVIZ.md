# Static PlanningScene RViz validation

Run each command in a separate terminal.

## Terminal 1 — Phase 2

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
```

## Terminal 2 — MoveIt

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch home_robotics_moveit_config move_group.launch.py
```

## Terminal 3 — static environment

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
```

The loader should report 14 objects in `world`. Running it again must still report a
PlanningScene total of 14.

## Terminal 4 — RViz

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch home_robotics_moveit_config moveit_rviz.launch.py
```

In the Displays panel enable `RobotModel`, `TF`, and `MotionPlanning`. Expand
`MotionPlanning` and enable `Scene Geometry` → `Show Scene Geometry`. The
`Planning Scene Topic` must be `/monitored_planning_scene`. In the MotionPlanning
panel select `panda1_manipulator`, then `panda2_manipulator`; the interactive goal
marker is available when `Query Goal State` is enabled.

Safe plan-only checks:

```bash
ros2 launch home_robotics_moveit_config cartesian_pose_validation.launch.py robot:=panda1 execute:=false
ros2 launch home_robotics_moveit_config cartesian_pose_validation.launch.py robot:=panda2 execute:=false
```

Table-collision checks (expected rejection, never execution):

```bash
ros2 launch home_robotics_moveit_config cartesian_pose_validation.launch.py robot:=panda1 execute:=false expect_invalid:=true offset_x:=0.025 offset_y:=-0.06449948 offset_z:=-0.81950243
ros2 launch home_robotics_moveit_config cartesian_pose_validation.launch.py robot:=panda2 execute:=false expect_invalid:=true offset_x:=-0.025 offset_y:=0.06449948 offset_z:=-0.81950243
```

## Visual checklist

- [ ] Both Pandas visible
- [ ] Shared rail visible
- [ ] Six work surfaces/tables visible as collision geometry
- [ ] Arena boundaries visible
- [ ] Robot current state matches MuJoCo
- [ ] `panda1_manipulator` selectable
- [ ] `panda2_manipulator` selectable
- [ ] MotionPlanning scene updates correctly
- [ ] No obvious frame offset between MuJoCo scene and MoveIt scene
- [ ] No duplicate environment objects
- [ ] Safe target can be planned
- [ ] Table-collision target is rejected
