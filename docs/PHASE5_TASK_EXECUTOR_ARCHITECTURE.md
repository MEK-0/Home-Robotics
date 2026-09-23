# Phase 5 Task Executor Architecture

`home_robotics_task_executor` provides one deterministic ROS Action boundary above the validated Phase 4 manipulation pipeline. It owns task IDs, validation, feedback, result mapping and the one-active-task rule; it does not instantiate MuJoCo, implement MoveIt planning, command joints/grippers, or mutate the PlanningScene.

```text
Action client -> /home_robotics/pick_and_place -> ManipulationAdapter -> Phase 4 cube_pick_place_demo -> MoveIt/ros2_control/MuJoCo
```

## API

`home_robotics_interfaces/action/PickAndPlace.action` accepts `object_id` and `target_id`. Results include success, task ID, final state, task failure, preserved manipulation failure, message and final pose when Phase 4 provides one. Feedback advances through validation (5), resolve object (10), resolve target (15), prepare (20), pick (35), verify (95), success (100).

The typed internal model defines PICK, PLACE, PICK_AND_PLACE and MOVE_TO_NAMED_POSE; only PICK_AND_PLACE executes in Phase 5. States include IDLE through SUCCEEDED/FAILED/CANCELING/CANCELED. Task failures include INVALID_REQUEST, OBJECT_NOT_FOUND, TARGET_NOT_FOUND, INVALID_TARGET, MANIPULATION_FAILED, TASK_BUSY and CANCELED.

## Safety and cancellation

`execute` defaults to `false`; planning-only invokes the Phase 4 preflight with no physical commands and reports success explicitly. Validation uses the authoritative object registry and scene configuration before delegating, so empty/unknown requests produce no motion. Only one accepted goal may run; a concurrent goal is rejected.

Cancellation is accepted in planning-only mode and is honored at the safe boundary before Phase 4 begins. Execution-mode cancellation is rejected once a goal is active: Phase 4 currently has no externally safe controller-interrupt boundary, and killing it could leave an object attached/unsupported. This is deliberately not an emergency stop.
