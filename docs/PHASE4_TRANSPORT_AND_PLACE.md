# Phase 4.6–4.7: transport and placement

Scope: Panda1, known cube, simulated pick-and-place. No perception, language model,
dual-arm coordination, task API, or Phase 5 work.

`cube_pick_place_demo` and the preserved `cube_pick_lift_return_demo` compile the same
orchestrator source. Only the default named destination differs. Both default to
`execute:=false`. The existing registry, grasp generator, gripper action interface,
full-state acquisition, Cartesian IK/path sampling, MoveGroupInterface execution,
contact verifier, weld stabilization, and dynamic-scene ownership handshake are reused.

## Config-derived target

`PlacementTarget::lookup(scene.yaml, target)` reads the named surface's world pose,
`top_height`, `safe_place_region`, and workspace ownership. Only horizontal Panda1
surfaces are accepted. The first destination is `surface_left_2`, different from the
cube's `surface_left_1` source. The region centre is `[1.15, 0.49] m`, region size
`[0.675, 0.55] m`, and surface top is `0.68 m`, all from `config/scene.yaml`.
The surface pose is already the TOP of the tabletop; adding half the table thickness
again would be incorrect.

The object orientation is preserved. Rotated box half-extents come from registry
geometry. Object centre Z = surface top + vertical half-extent + 0.001 m.
For the upright 0.05 m cube this is 0.706 m. Desired TCP uses the measured rigid
object-to-TCP transform, not a guessed grasp offset. Pre-place is 0.10 m above place.
Planning-only nominal TCP XYZ: pre-place `[1.15, 0.49, 0.8169]`, place
`[1.15, 0.49, 0.7169]`, quaternion XYZW approximately `[0.707107,0.707107,0,0]`.
Physical execution recalculates from the verified measured grasp.

The collision-checked lifted grasp is the initial safe transport waypoint; the
transport path maintains clearance to the destination pre-place pose.
The complete hypothetical attached lift/transport/descent is validated before any
physical command. Execution refreshes the scene and current robot state and rechecks
paths. Sequential Cartesian IK is bounded to 2 mm translation increments, joint
interpolation checks use 0.005 rad/m increments, and TOTG uses 0.04 velocity/acceleration
scaling. The pre-grasp OMPL plan uses 0.05 scaling. All arm trajectories execute via
MoveGroupInterface and existing ros2_control controllers. Panda2 remains collision
geometry; its maximum measured drift must stay below 0.002 rad/m. Rail separation
must remain at least the existing 0.7 m baseline.

## Physics and ownership

The only simulator remains the Phase 2 runtime. The contact observer borrows its
model/data. Stabilization uses the existing inactive weld, sets its reference to the
verified current relative transform, and never changes object/finger qpos.

The cube remains attached throughout lift and transport. Environment, rail and
Panda2 collision checks remain enabled. Only the existing two finger touch links are
permitted. Shallow physical support penetration is checked explicitly (at most
0.5 mm and inside the source/destination region), rather than globally allowed.

Before release, the authoritative observer must report the requested surface's contact
or bounded support proximity: entire footprint in its safe region and bottom gap
between -0.5 and +2 mm. The commanded +1 mm gap avoids planning an interpenetrating
attached shape. This is the documented geometry-plus-pose fallback permitted by the
closure specification, not a claim of pre-release physical contact.

Release order: support verification → disable stabilization → pause/drain dynamic
sync → attached REMOVE plus fresh world ADD → verify unique ownership → resume sync
→ physically open fingers → collision-checked vertical retreat. After retreat, actual
contact with the requested surface is mandatory; proximity alone cannot pass.

Final verification also requires a fresh pose, position error ≤ 0.03 m, complete
footprint in target region, no stabilization, measured open finger width, exactly one
WORLD object and no attachment. Position and angular errors, target/final pose,
Panda2 drift and cube/gripper drift are recorded in the result file. The cube/gripper
translation drift limit is 0.01 m.

## Evidence

See [reliability](PHASE4_RELIABILITY.md) and `validation/phase4/` for measured trials.
A planning-only pass never counts as a physical pick-place success.
