# Vendored model provenance

This directory vendors the Franka Emika Panda MJCF model from the local repository:

- Repository: `https://github.com/MEK-0/dynamics-randomized-manipulation.git`
- Local source commit: `af4cb717f8df0d75f3f9ce8519cdf19f7c4fcf76`
- Upstream model lineage: MuJoCo Menagerie-style model derived from Franka's public `franka_ros/franka_description`
- Source `panda.xml` SHA-256: `96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273`
- License: Apache-2.0; the original `LICENSE`, `README.md`, and `CHANGELOG.md` are preserved beside this file.

The vendored `panda.xml` and mesh files are copied unchanged. At runtime, Home-Robotics:

1. shares the intrinsic mesh/material/default definitions between both instances,
2. prefixes body and joint names with `panda1_` or `panda2_`,
3. mounts each instance below its configured carriage/base body,
4. does not import the source actuators or keyframe because control is outside Phase 1B.2,
5. retains the source finger equality coupling and validated collision exclusion,
6. adds `pandaN_tcp` at the source MJX gripper-site offset `[0, 0, 0.1] m`, configured in `robots.yaml`.
