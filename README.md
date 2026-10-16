# geoalg

**The kinematics library that tells you WHY it failed.**

![Status](https://img.shields.io/badge/status-v0.1%20in%20development-orange.svg)
[![License](https://img.shields.io/badge/license-AGPL--3.0%20%2F%20Commercial-blue.svg)](#license)

3D geometry and robot kinematics in Rust, built on Projective Geometric Algebra (PGA).
Runs in Rust, in the browser (WebAssembly), from C, and in Unity.

> **Status:** geoalg is not published on crates.io or npm yet. Everything in this
> README works today from this repository. If something is not listed under
> [What you can use today](#what-you-can-use-today), it is not shipped yet.

## Why another kinematics library?

Most IK solvers give you this:

```
Error -31: no solution found
```

geoalg tells you what happened and what to do about it:

```rust
use geoalg_robotics::demo_arm;

let arm = demo_arm(); // 3 joints, links of length 1, shoulder at height 1
let (sol, diag) = arm.ik_xyz_explained([2.9, 0.0, 0.0], &[0.1; 3], 200, 1e-6);

if !sol.converged {
    println!("{}", diag);
}
```

Output (shortened; messages are in Spanish for now, English is coming):

```
No se llego al objetivo. Lo mas cerca fue (...), a 1.068 de distancia.
  - [OUT_OF_REACH] El objetivo esta a 3.07 de la articulacion 2 (el ultimo punto fijo
    del robot), pero desde ahi el robot alcanza como maximo 2.00.
    -> Acerca el objetivo al menos 1.07 o usa eslabones mas largos.
```

Notice the detail: the target is only 2.9 away from the base, so a naive check
("is it closer than the total arm length of 3?") would say it is reachable. geoalg
measures from the last point of the robot that never moves (here, the shoulder),
so it knows the target is impossible, and by how much.

## Why Projective Geometric Algebra?

If you have worked with 4x4 matrices, Euler angles or quaternions, you know the pain:

- **Gimbal lock:** Euler angles break at certain orientations.
- **Error-prone composition:** easy to multiply matrices in the wrong order.
- **Separate code for everything:** rotations, translations and intersections all work differently.

In geoalg, a rotation plus a translation is **one object: a motor**. Motors compose
cleanly, move points, lines and planes the same way, and interpolate smoothly
without deforming anything.

## Installation

Not on crates.io yet. Use the repository directly:

```toml
[dependencies]
geoalg-core = { git = "https://github.com/071476/geoalg.git" }
geoalg-robotics = { git = "https://github.com/071476/geoalg.git" }
```

## Quick start

### Motors: turn and move in one object

```rust
use geoalg_core::{apply_point, Line, Motor, Point, Rotor, Translator};
use std::f64::consts::FRAC_PI_2;

fn main() {
    // Turn 90 degrees around the Z axis, then move 1 along X.
    let z_axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let turn = Motor::from_rotor(&Rotor::from_axis_angle(&z_axis, FRAC_PI_2));
    let shift = Motor::from_translator(&Translator::new(1.0, 0.0, 0.0));
    let m = turn.then(&shift); // first `turn`, then `shift`

    let p = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0));
    println!("{:?}", p.coords()); // about (1, 1, 0)

    // Smooth motion: the motor halfway between "no motion" and `m`.
    let half = Motor::between(&Motor::identity(), &m, 0.5);
    let q = apply_point(&half.inner(), &Point::new(1.0, 0.0, 0.0));
    println!("{:?}", q.coords());
}
```

### Your own robot arm: IK with joint limits

```rust
use geoalg_robotics::Robot;

fn main() {
    // Each joint: axis through the joint itself, link to the next joint, limits.
    let mut arm = Robot::new();
    arm.add_revolute((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.325, 0.0), (-3.14, 3.14)); // base
    arm.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-2.6, 2.6));     // shoulder
    arm.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.8, 0.0), (-2.8, 2.8));     // elbow

    let (sol, diag) = arm.ik_xyz_explained([0.3, 1.975, 0.3], &[0.0; 3], 200, 1e-6);
    println!("angles: {:?}", sol.angles);
    println!("{}", diag);
}
```

Want to see it first? The studio (geoalg-studio) shows this same arm in 3D, solved
by geoalg compiled to WebAssembly. An online version is coming soon.

## What you can use today

### Free (AGPL-3.0)

| Feature | What it does |
|---|---|
| **Geometry** | Points, lines, planes, spheres, projections, reflections |
| **Motors** | Rotation + translation in one object, composition, smooth interpolation (`between`) |
| **Forward kinematics** | Any number of joints, revolute and prismatic, tools (TCP) |
| **Inverse kinematics** | Damped least squares, joint limits, automatic retries from several starting poses |
| **Reach check** | `OUT_OF_REACH` (and by how much), `INVALID_TARGET`, `NOT_REACHED` |
| **WebAssembly** | The library in the browser: 48 KB, 19 KB gzipped |
| **C bridge** | `geoalg.h`: motors, robots, FK and IK from C, C++ or C# |
| **Unity** | C# wrapper and a working example |

### Pro (coming soon)

| Feature | Status |
|---|---|
| Levenberg-Marquardt IK with adaptive damping | Ready, waiting for release |
| Detailed IK diagnosis: which joint hit its limit, stuck solver | Ready, waiting for release |
| Exact continuous collision detection (moving spheres) | Ready, waiting for release |
| Dynamics: mass matrix, gravity, computed torque | In development |
| SLAM: pose graph optimization | In development |
| Commercial license (no AGPL obligations) | Contact us |

Want early access? Write to ali-gafer@hotmail.com with the subject "geoalg Pro waitlist".

## Platforms

| Platform | Status | Where |
|---|---|---|
| **Rust** | Works | `crates/geoalg-core`, `crates/geoalg-robotics` |
| **WebAssembly** | Works | `crates/geoalg-wasm` (`wasm-pack build crates/geoalg-wasm --target web --release`) |
| **C / C++** | Works | `crates/geoalg-ffi` (`cargo build -p geoalg-ffi --release`, header `geoalg.h`) |
| **Unity (C#)** | Works, tested on Unity 6.3 LTS (Windows) | [`examples/unity`](examples/unity) |
| Unreal, ROS 2, Python, Bevy | Planned | See the [roadmap](ROADMAP.md) |

## Tests

The robotics crate checks exact results, not "close enough": for example, with the
shoulder at 90 degrees the tip must land exactly on `(0, -2, 1)`, and the tip can
never end up farther than the arm's total length. Run them with:

```
cargo test -p geoalg-robotics
cargo test -p geoalg-ffi
cargo test -p geoalg-wasm
```

## What it does NOT do

- It is not a physics simulator (like Gazebo or MuJoCo).
- It is not a robot operating system (like ROS).
- It does not render anything: it gives you the math; you draw it.
- IK is position-only for now. Position + orientation is planned for v0.2.
- 3D only (PGA 3D).

## Roadmap

See [ROADMAP.md](ROADMAP.md). Next up for v0.1: rotated links (`rpy`), URDF import,
IK per chain, an export button in the studio, and publishing on crates.io and npm.

## License

geoalg uses a dual license:

| Who uses it | License | Cost |
|---|---|---|
| Open source projects, students, research | AGPL-3.0 | Free |
| Closed-source / commercial products | Commercial | Contact us |

**AGPL-3.0** is free, but if you use geoalg in your product (including a web app or
SaaS), you must release your product's source code under AGPL-3.0 too. If you need
to keep your code private, you need a commercial license.

See [LICENSE-AGPL](LICENSE-AGPL) and [LICENSE-COMMERCIAL](LICENSE-COMMERCIAL).

## Contact

- Commercial license and Pro waitlist: ali-gafer@hotmail.com
- Issues and ideas: [GitHub Issues](https://github.com/071476/geoalg/issues)
