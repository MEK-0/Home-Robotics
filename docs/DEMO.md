# Demo

This is the primary Panda1 cube pick/place path. Use a clean ROS domain and
source the workspace in every terminal.

## Build

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
```

## Runtime

Start these processes in separate terminals:

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
ros2 launch home_robotics_moveit_config move_group.launch.py
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
```

The first command starts the Phase 2 MuJoCo/ros2_control stack. `move_group`
provides planning; the static PlanningScene adds environment geometry; dynamic
synchronization updates configured object poses from MuJoCo. RViz is optional:

```bash
ros2 launch home_robotics_moveit_config moveit_rviz.launch.py
```

## Cube pick/place

Use planning-only mode first. It validates the route without motion commands:

```bash
ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py \
  robot:=panda1 object:=cube target:=surface_left_2 execute:=false
```

For the physical simulation sequence, reset or restart the stack, then run:

```bash
ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py \
  robot:=panda1 object:=cube target:=surface_left_2 execute:=true
```

The implemented sequence verifies grasp contact before temporary stabilization,
then lifts, transports, places, releases, and verifies the cube. Panda1/cube is
the supported named-placement baseline.

## Phase 5 Action

The Action uses the same Phase 4 path. Start it after the four runtime processes:

```bash
ros2 launch home_robotics_task_executor task_executor.launch.py execute:=false
ros2 action send_goal --feedback /home_robotics/pick_and_place \
  home_robotics_interfaces/action/PickAndPlace \
  "{object_id: cube, target_id: surface_left_2}"
```

`execute:=false` is a safety mode: preflight may run, but no physical simulation
commands are issued. After a clean reset, launch with `execute:=true` to execute
the simulated cube task. The Action accepts only one active request.

## Reset and troubleshooting

Use the runtime reset service or restart all four runtime processes before a new
physical execution after a failure or interruption. Ensure controllers are active
and the static scene and object synchronizer are running before starting the demo.

Common causes of failure are stale state, missing controller/scene processes, an
object not at its configured source state, and planning/retiming rejection. The
known benchmark retiming failure is preserved in the validation evidence; do not
treat planning-only success as a physical-execution result.
