# Phase 4 — Panda1 cube pick and place

Build first:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
```

In EACH terminal, source the same environment:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Use one ROS domain consistently and only one Phase 2 runtime. Stop previous demo
stacks before starting. Wait for each readiness message before the next terminal.

## Terminal 1 — MuJoCo / Phase 2 control

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=true
```

Wait for `All Phase 2 controllers are active`. For headless operation use
`use_viewer:=false`. Do not start the old independent joint-state bridge alongside it.

## Terminal 2 — MoveIt

```bash
ros2 launch home_robotics_moveit_config move_group.launch.py
```

Wait for `You can start planning now`.

## Terminal 3 — static PlanningScene

```bash
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
```

The one-shot loader exits normally after loading the environment.

## Terminal 4 — dynamic objects

```bash
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
```

Wait for `Dynamic world diff accepted`. All five configured objects must be present.

## Terminal 5 — RViz (optional)

```bash
ros2 launch home_robotics_moveit_config moveit_rviz.launch.py
```

Use Fixed Frame `world` and display planning-scene geometry. Panda2 remains part of
collision checking and must remain stationary.

## Terminal 6 — planning-only, then physical execution

```bash
ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py \
  robot:=panda1 object:=cube target:=surface_left_2 execute:=false
```

Require `PLANNING_ONLY PASS`. This validates the full hypothetical attached route
without physical commands or remote scene mutations. It does not verify grasp contact.
Execution is always opt-in:

```bash
ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py \
  robot:=panda1 object:=cube target:=surface_left_2 execute:=true \
  result_file:=/tmp/cube-pick-place.yaml
```

Expected: approach → physical close → bilateral contact verification → temporary
stabilization → unique attachment → 10 cm lift → transport to the OTHER surface →
pre-place → descent → target support/proximity check → stabilization removal → WORLD
ownership → open → vertical retreat → actual destination contact and final pose check.

## Manual success checklist

- [ ] Panda1 approaches the cube and gripper physically closes.
- [ ] Grasp verification succeeds before stabilization/attachment.
- [ ] Cube leaves source table and remains stable during lift.
- [ ] Panda1 transports toward surface_left_2; Panda2 remains stationary.
- [ ] Cube does not intersect environment during transport.
- [ ] Panda1 reaches pre-place and lowers the cube to destination support.
- [ ] Requested support/contact or bounded support proximity is verified.
- [ ] Stabilization is removed and gripper opens.
- [ ] Cube remains on destination table with actual contact after release.
- [ ] Panda1 retreats.
- [ ] Cube exists exactly once in PlanningScene, as WORLD, with no attachment.
- [ ] Cube final footprint is inside target region and position error ≤ 0.03 m.
- [ ] No broad ACM disabling occurred.

Headless results do not substitute for checking these visual boxes yourself.

## Preserved lift-return regression

From a fresh stack, use the original commands:

```bash
ros2 launch home_robotics_manipulation cube_pick_lift_return_demo.launch.py execute:=false
ros2 launch home_robotics_manipulation cube_pick_lift_return_demo.launch.py execute:=true
```

This demo returns to the original support position; it remains a regression, not the
acceptance pick-place task.

## Second object baseline

`purple_ball` uses its authoritative 0.035 m sphere radius and the same Panda1
orchestrator. Only grasp/lift/return is enabled for this bonus baseline; named sphere
placement is deliberately unsupported. Start from a fresh stack:

```bash
ros2 launch home_robotics_manipulation cube_pick_lift_return_demo.launch.py \
  object:=purple_ball execute:=false
ros2 launch home_robotics_manipulation cube_pick_lift_return_demo.launch.py \
  object:=purple_ball execute:=true result_file:=/tmp/purple-ball.yaml
```

The gripper must close and bilateral contact must pass before stabilization/attachment.
The current physical result and any limitation are recorded in the reliability document.

## Reset and troubleshooting

After a completed task or any failure, stop Terminal 6 before reset. For a clean
repeat, stop Terminals 1, 2, 4 and 5 with Ctrl+C, then restart Terminals 1–5 in order.
This deterministically resets physics, controller targets and the PlanningScene.
The benchmark automates this exact full-stack reset. Never start a second runtime
while the first is still alive.

`/reset_simulation` remains available for existing controlled reset workflows:

```bash
ros2 service call /reset_simulation std_srvs/srv/Trigger '{}'
```

After an interrupted attached task, use the full-stack restart: resetting physics
alone is not proof that stale MoveIt attachment/controller goals were cleared.

- Missing/stale joint state or clock: check Terminal 1 and consistent ROS domain;
  do not bypass the freshness gate.
- `SCENE_SYNC_FAILED`: verify static loader completion and dynamic sync readiness;
  restarting MoveIt requires reloading both scenes.
- IK/planning/retiming failure: no preflight motion occurs; preserve the result and
  inspect the collision/path diagnostic. Do not disable collision checking.
- `NO_CONTACT` / `GRASP_UNSTABLE`: do not force weld activation. Supported open/retreat
  is attempted; reset before retry.
- `PLACE_FAILED`: healthy grasp remains held; do not manually open while unsupported.
- `OBJECT_DROPPED`: subsequent stages stop; inspect cleanup result and reset.

## Repeatable headless benchmark

Stop any stack using the selected benchmark domain first:

```bash
ros2 run home_robotics_manipulation benchmark_pick_place.py --trials 10 --domain 64 \
  --output docs/validation/phase4/benchmark.json
```

Raw logs stay in `/tmp/phase4-benchmark`; all trial outcomes are included in JSON.
See [measured reliability](PHASE4_RELIABILITY.md).

## Regression Tests

### Simulation

```bash
cd ~/Home-Robotics
source .venv/bin/activate

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q tests/simulation
```

Historical pre-closure baseline:

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

ROS_DOMAIN_ID=65 colcon test
colcon test-result --verbose
```

Historical Phase 4.2 baseline:

```text
31 tests
0 errors
0 failures
1 skipped
```


Current closure results are recorded in [PHASE4_RELIABILITY](PHASE4_RELIABILITY.md).
