# geoalg roadmap

The kinematics library that tells you WHY it failed.

## v0.1 (in progress)

Done:

- [x] Free IK in geoalg-robotics (damped least squares, joint limits, retries)
- [x] Reach check: OUT_OF_REACH (measured from the last fixed point), INVALID_TARGET, NOT_REACHED
- [x] Exact tests for forward kinematics and segment distances (bugs found and fixed)
- [x] geoalg-wasm: a real bridge to geoalg-core and geoalg-robotics (48 KB, 19 KB gzipped)
- [x] Studio playground using the real library through WASM, checked by an independent FK
- [x] Generic viewer: any robot described in JSON (desktop arm, 6-joint arm, leg, 5-axis mill)
- [x] Robot selector, custom JSON with clear errors, Home button
- [x] C bridge (geoalg-ffi): motors, robots, FK and IK from C, C++ and C#
- [x] Unity: C# wrapper and a working example, tested on Unity 6.3 LTS (examples/unity)

To do:

- [ ] Rotated links (rpy) in Rust, WASM and JSON
- [ ] URDF import (tree, fixed and continuous joints, rpy), tested with the Franka Panda
- [ ] IK per chain (pick "from the hip to the front leg")
- [ ] Geometry demos in the studio: smooth motion (between) and projections
- [ ] One robot model everywhere (robotics, premium, studio)
- [ ] Finish the honest landing page (feature windows, verified platforms, SVG logo, favicon)
- [ ] Export button (robot JSON + ready-to-use code)
- [ ] README, LICENSE, CHANGELOG; publish on crates.io, npm and GitHub Releases
- [ ] Put the studio online at geoalg.dev

## v0.2 (planned)

- IK with orientation (position + rotation, using motors)
- Pro: Levenberg-Marquardt IK, detailed IK diagnosis (joint at limit, stuck solver), exact CCD
- Pro: prefer a comfortable posture among all IK solutions
- Diagnostic messages in English and Spanish
- Real 3D parts of robots (STL, DAE) in the viewer
- Calibration and smooth trajectories with speed limits
- Example with a real Arduino arm (with video)
- geoalg-expr: exact (symbolic) Jacobians for the IK, code generation (SymPy optional)
- Unreal plugin

## Later

- Python bindings
- ROS 2, and an IK plugin for MoveIt 2
- LeRobot dataset viewer: replay episodes in 3D and flag impossible poses
- Machine tools: G-code, 5-axis models (table-table, head-head, table-head), RTCP
- Dynamics and SLAM (Pro, after review)
- VR/AR, computer vision, drones
