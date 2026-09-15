# Phase 4.0 / 4.1 — Manipulation architecture and object scene integration

Phase 4.0 and Phase 4.1 are validated. **Phase 4 is not complete.** This implementation contains no grasp generation, gripper closure, physics contact detection, attachment, stabilization, lift, transport, place, or recovery execution.

## Package structure

```text
ros2_ws/src/home_robotics_manipulation/
├── include/home_robotics_manipulation/
│   ├── manipulation_state.hpp
│   ├── manipulation_types.hpp
│   └── object_registry.hpp
├── src/
│   ├── object_registry.cpp
│   ├── dynamic_object_scene_sync.cpp
│   └── cube_collision_validation.cpp
├── launch/
│   ├── dynamic_object_scene_sync.launch.py
│   └── cube_collision_validation.launch.py
├── scripts/validate_dynamic_scene.py
├── test/test_registry.cpp
├── CMakeLists.txt
└── package.xml
```

The existing bringup runtime gains pose publishers and an opt-in fixed debug fixture. `phase2_control.launch.py` exposes `enable_scene_validation` (default false); controller nodes, mappings and parameters remain unchanged. `test_object_pose_interface.py` tests forwarding with a stub base class and no simulator instance.

## Lifecycle and failures

`ManipulationState` has this explicit nominal sequence:

```text
IDLE → OBJECT_SELECTED → PREGRASP_PLANNED → APPROACHING
→ GRIPPER_CLOSING → GRASP_VERIFYING → GRASP_VERIFIED → OBJECT_ATTACHED
→ LIFTING → TRANSPORTING → PREPLACE_PLANNED → PLACING → RELEASING
→ PLACE_VERIFYING → PLACE_VERIFIED → DONE
```

`transition_allowed()` permits only the next nominal state, transition from a nonterminal state to FAILED, or explicit reset from DONE/FAILED to IDLE. It does not issue actions or run transitions automatically. Future orchestration must gate transitions on measured outcomes. In particular, gripper action success cannot jump directly to GRASP_VERIFIED.

`ManipulationFailureReason` includes NONE and:

```text
OBJECT_NOT_FOUND, INVALID_OBJECT_STATE, IK_FAILED, PLANNING_FAILED,
EXECUTION_FAILED, APPROACH_COLLISION, GRIPPER_FAILED, NO_CONTACT,
GRASP_UNSTABLE, OBJECT_DROPPED, PLACE_FAILED, SCENE_SYNC_FAILED
```

These are typed failure categories, not an implemented recovery policy. Scene sync currently reports invalid/stale poses or service failure in logs. A future orchestrator must use scene freshness and operation results as task preconditions.

## Responsibilities and state ownership

| Component | Responsibility | Ownership boundary |
|---|---|---|
| Manipulation orchestrator (future runtime) | Object selection, lifecycle, operation sequencing, task result/failure | Never writes MuJoCo data directly |
| MoveIt motion layer | Future pre-grasp/approach/lift/transport/place plans and collision-aware execution | Existing MoveIt controller chain; never commands MuJoCo directly |
| Gripper layer | Future open/close requests and finger/action feedback | Action success is not grasp success |
| MuJoCo runtime | Authoritative physics and object body poses; contact/stabilization information later | Existing single model/data owner |
| Dynamic scene sync | Translate current body poses into dynamic world collision objects | Owns only registered WORLD collision updates |
| Object registry | Logical IDs, body mapping, collision geometry, mobility, reserved grasp profile reference | Typed view of existing `config/objects.yaml` |

There is no second model or data instance, no physics stepping in the synchronizer, and no MoveIt-to-physics object pose feedback.

## Registry and geometry

The source of truth remains [`config/objects.yaml`](../config/objects.yaml), already consumed by `ConfigLoader` and `SceneBuilder`. The C++ registry reads its installed copy from `home_robotics_control/share/config/objects.yaml`; no independent dimension table or duplicate YAML was added. SceneBuilder names MuJoCo object bodies with the same config key. IDs must match their keys and be unique.

`ObjectMetadata` contains ID, `ObjectType`, `CollisionShapeType`, local primitive geometry/dimensions/poses, MuJoCo body name, movable flag, `ObjectOwnership`, and an optional grasp-profile reference. `ObjectPoseState` contains ID, stamped pose and ownership. No grasp pose is generated from this metadata.

| ID | ObjectType | MoveIt geometry from config | MuJoCo movable |
|---|---|---|---|
| cube | MANIPULABLE | BOX, 0.05 × 0.05 × 0.05 m | true |
| apple | MANIPULABLE | SPHERE, radius 0.04 m | true |
| purple_ball | MANIPULABLE | SPHERE, radius 0.035 m | true |
| bowl | CONTAINER | Compound: bottom CYLINDER radius 0.115 m, height 0.015 m; 8 BOX walls 0.018 × 0.080 × 0.070 m | false |
| pan | FUTURE_MANIPULABLE | Compound: CYLINDER radius 0.12 m, height 0.025 m; BOX handle 0.22 × 0.04 × 0.025 m | false |

All compound offsets and yaw angles come directly from the same YAML. Bowl bottom local position is `[0, 0, 0.0075]`; wall centres/yaws preserve the open interior. Pan body is at `[-0.08, 0, 0.0125]`, handle at `[0.09, 0, 0.025]`. MoveIt cylinder dimension order is `[height, radius]`. The apple stem is visual-only in MuJoCo and is not added as collision geometry. No mesh approximation or arbitrary sizes were introduced.

“Dynamic objects” here denotes the five objects managed by object scene sync; it does not change bowl/pan's existing fixed-body physics configuration.

## Pose interface and synchronization

The existing `Phase2MujocoRuntime` publishes `geometry_msgs/PoseStamped` at **20 Hz**, on a separate 50 ms timer from the physics update:

```text
/mujoco/objects/cube/pose
/mujoco/objects/apple/pose
/mujoco/objects/purple_ball/pose
/mujoco/objects/bowl/pose
/mujoco/objects/pan/pose
```

Topic identity is the semantic object ID; there is no positional array ordering or MarkerArray ID convention. Each message has `frame_id=world` and a timestamp derived from `data.time`, matching `/clock`. Position and quaternion are copied from the existing `data.body(id).xpos/xquat`; MuJoCo WXYZ is explicitly converted to ROS XYZW. QoS is reliable, volatile, keep-last 1; late subscribers receive the next update.

`dynamic_object_scene_sync` uses simulated ROS time and a **20 Hz** service-update timer (configurable 10–30 Hz at the node). It validates finite positions and normalized quaternions and requires all registry poses. Poses older than 0.5 s or from the future beyond 0.1 s suppress updates and report SCENE_SYNC_FAILED. Last world geometry is retained rather than silently cleared. One asynchronous request chain is in flight at a time; timed-out requests are pruned after 3 s and late callbacks ignored.

Each cycle queries attached IDs, then applies a PlanningScene **diff**, with both `scene.is_diff` and `robot_state.is_diff` true. Only registered world objects are submitted, using stable IDs and ADD for both creation and replacement. No REMOVE, world clear, ACM modification, or attachment operation is performed. Body pose lives in `CollisionObject.pose`; compound part poses stay local to the body. Repeated updates replace the same object ID.

Static ID preservation is validated against the full serialized 14-object scene captured before dynamic sync started. Dynamic expected IDs derive from the config. Counts 14 + 5 = 19 are observed acceptance evidence, not a magic constant in synchronizer logic.

## WORLD / ATTACHED lifecycle preparation

The future ownership sequence is:

```text
WORLD (scene sync owns world updates)
→ ATTACHED (manipulation owns attached collision state)
→ WORLD (scene sync resumes from MuJoCo authoritative pose)
```

`ObjectOwnership` and `world_sync_allowed()` encode the boundary. The diff builder excludes any object marked ATTACHED or present in the actual PlanningScene attached-ID set. Unit tests exercise this exclusion without attaching anything. Registry geometry remains read-only; runtime ownership belongs to manipulation.

This is preparation, not an atomic attachment implementation. Future attachment work must pause/drain outstanding world updates before its ownership transfer, then resume world publication only after detach and a fresh physics pose. That handshake, attach/detach calls, and physical grasp stabilization are not implemented here.

Future `pick("cube")` will select/validate metadata and fresh physics state, request pre-grasp/approach, command the gripper, verify actual grasp evidence, transfer scene ownership, and then lift/transport. Future `place("surface_left_2")` will plan the destination, release, verify physical support and pose, and restore WORLD ownership. These are architecture descriptions only; no pick/place API executes today.

## Validation results

- Single authoritative Phase 2 runtime, all existing controllers active, static scene loaded first.
- Five exact unique household IDs present, all `world` frames, all primitive types/dimensions/local transforms match config.
- **14 static + 5 object collision objects = 19 world objects; 0 attached objects.**
- Every static object's serialized geometry/pose remains identical to the pre-sync baseline.

Initial poses from the final validation sample are below. Quaternions use XYZW; full precision pose/quaternion pairs and every primitive comparison are in [poses-final.json](validation/phase4_1/poses-final.json). Quaternion error is `min(||q₁−q₂||, ||q₁+q₂||)` so sign-equivalent quaternions compare correctly; threshold 1e-6. Position threshold is 0.001 m.

| Object | MuJoCo XYZ (m) | PlanningScene XYZ (m) | Position error (m) | Quaternion error |
|---|---|---|---|---|
| cube | [0.0749999941342, 0.410000049124, 0.70489224458] | [0.0749999941349, 0.410000049118, 0.70489224458] | 6.22e-12 | 8.749e-12 |
| apple | [0.454999985578, 0.569999981694, 0.719632818158] | [0.45499998558, 0.569999981696, 0.719632818158] | 3.039e-12 | 9.454e-11 |
| purple_ball | [0.92999999497, 0.410000022769, 0.714632818158] | [0.92999999497, 0.410000022766, 0.714632818158] | 3.007e-12 | 1.068e-10 |
| bowl | [1.35, 0.57, 0.68] | [1.35, 0.57, 0.68] | 0 | 0 |
| pan | [1.15, -0.49, 0.68] | [1.15, -0.49, 0.68] | 0 | 0 |
Measured initial MuJoCo orientations (PlanningScene orientations differ only by the errors above):

```text
cube        [9.98618e-11, 1.19241e-11, -1.14034e-15, 1.0]
apple       [-5.69434e-7, 4.48615e-7, -1.47446e-19, 0.999999999999738]
purple_ball [8.08916e-7, 1.78707e-7, -1.30636e-18, 0.999999999999657]
bowl        [0, 0, 0, 1]
pan         [0, 0, 0, 1]
```

**Movement test:** the opt-in `/validation/shift_cube` Trigger moved the cube +0.05 m in X using the existing simulator's `set_free_joint_pose`, zeroed only cube free-joint velocity, and called `forward()`. This fixture accepts no object/pose inputs and refuses a second shift until reset. Cube X changed from approximately 0.0749999941342 to 0.124999994134 m. PlanningScene followed with measured position/quaternion error **0 / 0** and one cube ID. `/reset_simulation` restored the existing simulation and scene; restoration and static preservation passed. No physics model was created or replaced by this test.

**Collision test:** a fresh full joint-state snapshot is first checked collision-free. An offline in-bounds Panda1 RobotState is then solved specifically to intersect the cube. Exact contacts include **cube ↔ panda1_hand** and **cube ↔ panda1_link7**. Table contacts also occur, but the test requires a cube/robot contact and cannot pass on table-only collision. Local PlanningScene and `/check_state_validity` both reject the state. It is never executed, and fingers are not commanded or closed. The collision test is not grasp pose generation.

**Safe plans:** existing joint-space validation targets were planned independently with all five objects loaded:

| Group | Target validity | Plan | Time | Points | Execution |
|---|---|---|---|---|---|
| panda1_manipulator | VALID, zero target contacts | SUCCESS | 0.005294 s | 30 | disabled |
| panda2_manipulator | VALID, zero target contacts | SUCCESS | 0.016259 s | 30 | disabled |

Test-owned launch processes were stopped after validation; the simulation had already been restored through its existing reset service.

**Regression:** simulation **75 passed**; ROS build **6 packages**; ROS test result **25 tests, 0 errors, 0 failures, 1 skipped**. The skip is the existing generated-source copyright check. Focused tests cover exact/deterministic IDs, config-derived geometry, primitive transforms, invalid poses, lifecycle/failures, attachment-ready exclusion, idempotent diffs, no world clearing, launch installation, and forwarding from a stubbed single runtime. No wall-clock performance assertion is used.

Remaining limitations: manual GUI inspection is pending; these are headless API/physics validations. Existing FIFO realtime scheduling, deprecated gripper, MoveIt config inference/default workspace, mixed-joint TOTG, and tl_expected deprecation warnings remain. One initial safe-plan attempt did not receive a fresh joint state within its timeout; the rerun passed with no controller changes. The scene synchronizer is not a real-time safety barrier or a dynamic-obstacle prediction system. No grasp, attachment, or pick/place capability is claimed.

Evidence: [static baseline](validation/phase4_1/static.json), [pose/movement/reset output](validation/phase4_1/validation-final.txt), [collision rejection](validation/phase4_1/collision.txt), [Panda1 plan](validation/phase4_1/safe1.txt), [Panda2 plan](validation/phase4_1/safe2.txt), [build](validation/phase4_1/build.txt), [simulation tests](validation/phase4_1/simulation-tests.txt), [ROS results](validation/phase4_1/ros-results.txt).

## MANUAL RVIZ TEST

Close old test stacks first. In **each terminal**, source:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Terminal 1 — single MuJoCo/Phase 2 stack, optional viewer and the fixed debug fixture:

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py \
  use_viewer:=true enable_scene_validation:=true
```

Use `use_viewer:=false` for headless mode. Wait for `All Phase 2 controllers are active`. For normal operation omit `enable_scene_validation:=true`; the shift service then does not exist.

Terminal 2 — MoveIt:

```bash
ros2 launch home_robotics_moveit_config move_group.launch.py
```

Terminal 3 — existing static environment (one-shot process; exits normally):

```bash
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
```

Before starting Terminal 4, optionally capture the static-only baseline in Terminal 3 for automated pose validation:

```bash
ros2 run home_robotics_manipulation validate_dynamic_scene.py \
  --capture-static --baseline /tmp/manual_static_scene.json
```

Terminal 4 — dynamic object scene sync:

```bash
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
```

Terminal 5 — existing RViz configuration:

```bash
ros2 launch home_robotics_moveit_config moveit_rviz.launch.py
```

In RViz set Fixed Frame to `world` and keep MotionPlanning's scene geometry and scene robot displays enabled. Select `panda1_manipulator` or `panda2_manipulator` to inspect; no execution is required. Objects share the collision-scene display style and may not use MuJoCo visual colours. Bowl's open wall compound and pan's handle should be visible.

From the now-free Terminal 3, manually shift the cube once while watching MuJoCo and RViz:

```bash
ros2 service call /validation/shift_cube std_srvs/srv/Trigger '{}'
```

The cube moves +5 cm along world X. No robot should move. Restore afterwards:

```bash
ros2 service call /reset_simulation std_srvs/srv/Trigger '{}'
```

If the static baseline was captured, run the complete pose/movement/reset validator:

```bash
ros2 run home_robotics_manipulation validate_dynamic_scene.py \
  --baseline /tmp/manual_static_scene.json --movement \
  --output /tmp/manual_object_validation.json
```

Offline cube collision and optional safe plan-only checks:

```bash
ros2 launch home_robotics_manipulation cube_collision_validation.launch.py
ros2 launch home_robotics_moveit_config joint_space_validation.launch.py \
  robot:=panda1 execute:=false
ros2 launch home_robotics_moveit_config joint_space_validation.launch.py \
  robot:=panda2 execute:=false
```

Complete these visual checks yourself; they are intentionally not marked as visually verified by the headless test:

- [ ] Both Panda robots visible
- [ ] Shared rail visible
- [ ] Static environment visible
- [ ] cube visible
- [ ] apple visible
- [ ] purple_ball visible
- [ ] bowl visible
- [ ] pan visible
- [ ] Object positions visually match MuJoCo
- [ ] Moving debug cube updates in RViz
- [ ] No duplicate collision objects
- [ ] MotionPlanning scene remains responsive

Stop launched processes with Ctrl+C when finished. Dynamic sync only runs while its terminal is active; restarting move_group requires reloading the static environment. No object attachment, grasp or pick/place command is part of this procedure.
