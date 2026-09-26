# Implemented manipulation

The validated named-placement baseline is Panda1 moving the cube from its
configured source surface to a named target such as `surface_left_2`.

The sequence is:

1. verify controller, scene, object-state, rail, and collision preconditions;
2. plan and approach a top-down grasp;
3. close the gripper and verify physical grasp contact;
4. apply temporary stabilization only after verification;
5. attach the object in Moveit's PlanningScene, lift, and transport;
6. place on the target, remove stabilization, release, and restore WORLD ownership;
7. verify final pose, target support/region, and ownership before success.

The benchmark preserves one retiming failure before motion. Known failure modes
include stale object state, unavailable controller or scene data, planning or
retiming rejection, missing grasp contact, object drop, and placement outside the
target region. Failure is reported rather than retried autonomously.

`purple_ball` has a limited physical grasp/lift/return baseline. It is not a
named-target placement result. Apple, bowl, pan, unknown objects, perception-led
grasping, and coordinated dual-arm manipulation are outside this implementation.
