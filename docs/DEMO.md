# Home Robotics — Cube Pick, Lift & Return Demo

This document contains the terminal commands required to demonstrate the current Phase 4 manipulation baseline with **Panda1** and the **cube**.

## Demo Flow

```text
Current state
    ↓
Pre-grasp
    ↓
Approach
    ↓
Gripper close
    ↓
Grasp verification
    ↓
Temporary stabilization
    ↓
Lift
    ↓
Return to original position
    ↓
Release
    ↓
Retreat
```

## Terminal 1 — MuJoCo + ros2_control

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_bringup \
  phase2_control.launch.py \
  use_viewer:=true
```

## Terminal 2 — MoveIt 2

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_moveit_config \
  move_group.launch.py
```

## Terminal 3 — Static PlanningScene

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_moveit_config \
  planning_scene_environment.launch.py
```

## Terminal 4 — Dynamic Object Synchronization

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_manipulation \
  dynamic_object_scene_sync.launch.py
```

Tracked dynamic objects:

```text
cube
apple
purple_ball
bowl
pan
```

## Terminal 5 — RViz (Optional)

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_moveit_config \
  moveit_rviz.launch.py
```

## Readiness Check

```bash
ros2 topic echo /joint_states --once
```

Optional object-topic check:

```bash
ros2 topic list | grep -E "object|cube|dynamic"
```

## Planning-Only Demo

Always run this first:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_manipulation \
  cube_pick_lift_return_demo.launch.py \
  execute:=false
```

Do not continue if planning-only validation fails.

## Full Demo

After planning-only succeeds:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch home_robotics_manipulation \
  cube_pick_lift_return_demo.launch.py \
  execute:=true
```

Expected sequence:

```text
Panda1 moves to pre-grasp
        ↓
approaches cube
        ↓
gripper closes
        ↓
grasp is verified
        ↓
cube is stabilized
        ↓
cube is lifted
        ↓
cube is lowered to original position
        ↓
gripper opens
        ↓
Panda1 retreats
```

## Demo Success Checklist

- [ ] Panda1 moves above the cube
- [ ] Panda1 approaches the cube
- [ ] Gripper closes physically
- [ ] Cube grasp succeeds
- [ ] Cube leaves the table
- [ ] Cube remains stable during lift
- [ ] Cube returns to its original position
- [ ] Gripper opens
- [ ] Cube remains on the table
- [ ] Panda1 retreats
- [ ] Panda2 does not move unexpectedly
- [ ] No duplicate collision object appears
- [ ] No unexpected environment collision occurs

## Regression Tests

### Simulation

```bash
cd ~/Home-Robotics
source .venv/bin/activate

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q tests/simulation
```

Validated result:

```text
75 passed
```

### ROS 2

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash

cd ros2_ws
python -m colcon build
source install/setup.bash

colcon test
colcon test-result --verbose
```

Latest validated Phase 4.2 result:

```text
31 tests
0 errors
0 failures
1 skipped
```

## Current Scope

The current demo is a controlled baseline:

```text
Panda1 + cube
→ grasp
→ lift
→ return
→ release
```

It is not yet a generic household pick-and-place system.
