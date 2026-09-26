# Design decisions

1. **MuJoCo is the physics authority.** Object pose, contact, and simulation time
   come from one authoritative MuJoCo model/data runtime.

2. **ROS 2 and ros2_control separate clients from simulation internals.** Standard
   controller interfaces exchange state and command samples with the runtime.

3. **MoveIt provides planning and geometric collision checking.** It does not own
   physics or command MuJoCo directly.

4. **Shared-rail safety is separate from geometry checking.** The control boundary
   enforces 0.70 m carriage separation in addition to MoveIt collision checks.

5. **Dynamic objects synchronize from MuJoCo to MoveIt.** Static geometry is loaded
   into the PlanningScene; dynamic poses follow the physics state.

6. **Physical grasp verification precedes stabilization.** Contact and state checks
   occur before temporary stabilization and PlanningScene attachment.

7. **The Task Executor owns one task at a time.** The Action server rejects a
   concurrent goal instead of attempting scheduler or allocation behavior.

8. **Phase 4 delegation remains process-level.** The Phase 5 Action calls the
   validated Phase 4 launch path through a subprocess. It is a prototype tradeoff,
   documented rather than hidden.
