# Task Executor API

## Endpoint

`/home_robotics/pick_and_place`
Type: `home_robotics_interfaces/action/PickAndPlace`

The Action is the implemented Phase 5 task boundary. It accepts one active goal
at a time and currently routes only the validated Panda1 cube named-placement
baseline.

## Goal

| Field | Type | Meaning |
| --- | --- | --- |
| `object_id` | string | Must be `cube`. |
| `target_id` | string | Named surface in the authoritative scene; must differ from cube's source surface. |

Robot selection is internal to this baseline: Panda1 is used. The interface has
no move-home, state-query, scheduler, robot-allocation, or task-graph API.

## Feedback

| Field | Type | Meaning |
| --- | --- | --- |
| `task_id` | string | Executor-assigned identifier. |
| `current_state` | string | Current Action lifecycle state. |
| `progress` | float32 | Coarse progress indicator. |
| `message` | string | Human-readable status. |

## Result

| Field | Type | Meaning |
| --- | --- | --- |
| `success` | bool | Terminal success flag. |
| `task_id` | string | Executor-assigned identifier. |
| `final_state` | string | Terminal lifecycle state. |
| `failure_reason` | string | Executor-level reason. |
| `manipulation_failure_reason` | string | Preserved Phase 4 reason when present. |
| `message` | string | Status detail. |
| `final_object_pose` | geometry_msgs/Pose | Final pose when reported by Phase 4. |

## Policy and validation

The executor rejects concurrent goals. It validates empty IDs, unknown objects,
unsupported object/task combinations, unknown targets, same-source targets, and
the internal Panda1 baseline before delegating. The scene registry includes more
objects than the manipulation capability: the Action accepts only `cube`.
`purple_ball` is a separate grasp/lift/return baseline and is not a supported
named-placement Action goal.

`execute:=false` is the default safety mode. It may complete planning preflight
but does not issue physical simulation commands. `execute:=true` enables the
validated simulated Phase 4 sequence.

Planning-only cancellation is accepted at supported safe boundaries. Execution
mode cancellation is rejected because the current delegated pipeline does not
provide arbitrary safe interruption points.

## Current delegation limitation

The executor provides a real ROS Action boundary but starts Phase 4 using a
process-level ROS launch subprocess. This preserves the validated pipeline while
keeping Phase 5 separate. A future in-process manipulation library/API would
remove that prototype limitation; it is not implemented now.
