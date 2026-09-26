# Test strategy

Validation is layered because no one test mode establishes the complete claim.

- **Deterministic simulation tests** exercise scene/configuration loading, control,
  reset, contact, rail constraints, and simulation utilities. Recorded result:
  92 passed.
- **ROS package tests** run through colcon after building the workspace. They cover
  package contracts and component behavior. Recorded result: 56 tests, 0 errors,
  0 failures, 1 existing skipped.
- **Runtime integration checks** start the control stack, MoveIt, static scene,
  dynamic synchronization, and a demo or Action request. They establish that
  interfaces compose at runtime.
- **Manual RViz checks** inspect robot state, static geometry, and object updates.
  They are manual observations, not automated tests.
- **Phase 4 benchmark** runs ten independent full-stack simulated cube trials.
  Nine succeeded; one failed during planning/retiming before motion. The failure
  remains in the preserved evidence.

The Phase 5 focused suite has 10 Task Executor tests. It uses a fake adapter to
test Action semantics deterministically; it does not replace full-stack testing.
Future test ideas are intentionally not represented as implemented coverage.
