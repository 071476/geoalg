# geoalg

**The kinematics library that tells you WHY it failed.**

![Status](https://img.shields.io/badge/status-v0.1%20in%20development-orange.svg)
[![License](https://img.shields.io/badge/license-AGPL--3.0%20%2F%20Commercial-blue.svg)](#license)

3D geometry and robot kinematics in Rust, built on Projective Geometric Algebra (PGA).
Runs in Rust, in the browser (WebAssembly), from C, and in Unity.

> **Status:** geoalg is not published on crates.io or npm yet. Everything in this
> README works today from this repository. If something is not listed under
> [What you can use today](#what-you-can-use-today), it is not shipped yet.

## Our promise

> Every time geoalg can't do something, it should answer with three things:
> **what happened, why, and what you can do.**

Where this works today:

| Situation | What happened | Why | What you can do |
|---|---|---|---|
| Target out of reach (`ik_xyz_explained`) | The tip did not reach the target | The target is X away from the last fixed point; the robot reaches at most Y | Move the target at least X - Y closer, or use longer links |
| Invalid target (NaN or infinity) | Nothing was computed | The target has invalid values | Check where the point comes from (often a division by zero) |
| Reachable, but not reached (free version) | The tip stopped short | Not identified in the free version | Try another starting pose or check the joint limits |

In Pro, the last case gets a real "why": which joint hit its limit, or whether the
solver got stuck. We are bringing the same three answers to the rest of the library
(the C bridge and WebAssembly still return plain error codes today).

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

Real output (messages are in Spanish for now; English is coming):

```
No se llego al objetivo. Lo mas cerca fue (1.890, 0.000, 0.348), a 1.068 de distancia.
  - [OUT_OF_REACH] El objetivo esta a 3.07 de la articulacion 2 (el ultimo punto fijo del robot), pero desde ahi el robot alcanza como maximo 2.00. -> Acerca el objetivo al menos 1.07 o usa eslabones mas largos.
```

In English: the target is 3.07 away from joint 2 (the last point of the robot that
never moves), but from there the robot reaches at most 2.00. Move the target at
least 1.07 closer, or use longer links.

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
    println!("{:?}", q.coords()); // about (1.21, 0.5, 0)
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

Real output:

```
angles: [0.7853981633201305, -0.04094591133887167, 0.661514924794567]
Listo: la punta llego al objetivo (error 3.01e-10).
```

These are the same angles the studio shows for this arm in the browser: the same
library, compiled to native code and to WebAssembly, gives the same answer.

### Run the examples

Every example in this README is a real example in the repository:

```
cargo run -p geoalg-robotics --example why_it_failed
cargo run -p geoalg-robotics --example my_arm
cargo run -p geoalg-core --example motors
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
| **C bridge** | `geoalg.h`: motors, robots, FK and IK from C, C++ or C# (`geoalg_ffi.dll` is 156 KB) |
| **Unity** | C# wrapper and a working example (tested on Unity 6.3 LTS, Windows) |

### Pro (coming soon)

| Feature | Status |
|---|---|
| Levenberg-Marquardt IK with adaptive damping | Ready, waiting for release |
| Detailed IK diagnosis: which joint hit its limit, stuck solver | Ready, waiting for release |
| Exact continuous collision detection (moving spheres, no tunnelling) | Ready, waiting for release |
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
