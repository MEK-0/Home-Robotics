# Phase 4.8: deterministic failure and recovery

The existing sequential `ManipulationState` remains authoritative. No retries,
alternate-target searches or advanced replanning loops were introduced.
Every failure records its stage, structured reason and detailed diagnostic.

| Reason | Behaviour |
|---|---|
| OBJECT_NOT_FOUND | Missing authoritative object pose: stop before further commands. |
| INVALID_OBJECT_STATE | Reject unsupported robot/object, invalid pose, stopped clock, bounds or initial state. |
| IK_FAILED | Reject infeasible candidate/path; preflight runs before physical motion. |
| PLANNING_FAILED | Stop on MoveIt planning failure. |
| EXECUTION_FAILED | Stop trajectory and subsequent stages on controller/tracking failure. |
| APPROACH_COLLISION | Reject sampled path with prohibited contacts. |
| GRIPPER_FAILED | Stop on action rejection, timeout or measured width failure. |
| NO_CONTACT | No bilateral contact: no stabilization, attachment or lift. |
| GRASP_UNSTABLE | Reject unverified/unstable grasp or excessive activation jump. |
| OBJECT_DROPPED | Stop active trajectory and subsequent stages; clear demonstrably stale weld and restore measured WORLD geometry. |
| PLACE_FAILED | Unreachable descent or missing target support: retain healthy grasp, do not release. |
| SCENE_SYNC_FAILED | Stop on missing/stale geometry, service failure, ownership duplication or failed scene update. |

Failed-grasp recovery is conservative: while still supported at the source, open the
gripper and execute a collision-checked retreat. If support/freshness cannot be
established, no speculative retreat is commanded. Verification is enforced again by
the physics runtime immediately before stabilization.

During every stabilized trajectory the orchestrator checks fresh physics data,
stabilization activity, maximum relative translation drift ≤ 0.01 m and unexpected
contacts. Unexpected contact is an APPROACH_COLLISION and retains a healthy grasp;
inactive/drifted stabilization is OBJECT_DROPPED. It calls MoveGroup stop before unwinding an execution error.
The `/mujoco/<object>/clear_dropped` service refuses to disable a healthy active weld.
Only inactive or demonstrably drifted stabilization can be cleared by this recovery.
Attachment cleanup uses fresh authoritative object pose, with no teleport. If a
service or scene handshake prevents cleanup, the report retains `cleanup_failure`;
the task is FAILED and requires an operator reset. Cleanup never reports task success.

Place failure retains the verified attached grasp when possible. No unsupported
open-gripper command is issued. A paused dynamic synchronizer is resumed in the
failure handler. State flags and the diagnostic identify incomplete cleanup. Use the
full stack restart in [DEMO](DEMO.md) before another attempt.

Focused tests exercise transition restrictions, failed-grasp stabilization rejection,
support-only release, stale-weld guard, placement geometry and scene ownership diffs.
Nominal integration trials exercise successful lifecycle and uniqueness. These tests
do not constitute exhaustive physical fault injection or real-robot safety validation.
