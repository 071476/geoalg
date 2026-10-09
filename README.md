# geoalg

**The kinematics library that tells you WHY it failed.**

[![License](https://img.shields.io/badge/license-AGPL--3.0%20%2F%20Commercial-blue.svg)](#licenses--dual-licensing)

Projective Geometric Algebra (PGA) for Rust — robotics, geometry, and math that explains itself.

## Why another kinematics library?

Most IK solvers return this:

    Error -31: no solution found

geoalg is built to return this instead:

    if !result.converged {
        println!("{}", result.diagnostic.reason);
        // "Joint 2 at limit 90 degrees (required: 112)"
        // "Target 3.4m away, arm reaches 3.0m"
        println!("{}", result.diagnostic.suggestion);
        // "Relax joint 2 to 100 degrees OR move target 0.4m closer"
    }

**No more guessing. No more trial and error. geoalg tells you exactly what went wrong and how to fix it.**

&gt; Diagnostics are on the v1 roadmap — see [Roadmap](#roadmap).
&gt; Everything under [Quick start](#quick-start) works today.

## Why Projective Geometric Algebra?

If you've worked with 4x4 matrices or quaternions, you know the problems:

- **Gimbal lock:** Euler angles break at certain orientations.
- **Complex composition:** multiplying 4x4 matrices is error-prone.
- **Hard intersections:** where a line crosses a plane requires separate code.

PGA solves all of this with one type: the **multivector**. Motors interpolate smoothly, intersections use the outer product, and there are no singularities.

## Installation

### Rust

    [dependencies]
    geoalg = { git = "https://github.com/071476/geoalg.git" }

## Quick start

Works with geoalg today:

    use geoalg::prelude::*;

    fn main() {
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(2.0, 1.0, 0.0);

        let line = p1.join(&p2);
        println!("Distance: {:.3}", p1.distance(&p2));

        let start = Motor::identity();
        let end = Rotor::from_axis_angle(&Line::z_axis(), std::f64::consts::FRAC_PI_2).to_motor();

        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let step = start.lerp(&end, t);
            println!("t={:.2} -&gt; {:?}", t, step);
        }
    }

## IK (geoalg-premium)

Position IK with Levenberg-Marquardt, joint limits and adaptive damping.
This compiles and runs against geoalg-premium today:

    use geoalg_premium::prelude::*;

    let arm = RobotArm::new(vec![
        Joint { axis: [0.0, 0.0, 1.0], offset: [0.0, 0.0, 0.0], limits: (-3.14, 3.14) },
        Joint { axis: [1.0, 0.0, 0.0], offset: [0.0, 0.8, 0.0], limits: (-1.57, 1.57) },
        Joint { axis: [1.0, 0.0, 0.0], offset: [0.0, 0.8, 0.0], limits: (-1.57, 1.57) },
    ]);

    let result = arm.ik_xyz([0.3, 1.8, 0.3], &[0.0, 0.0, 0.0], 500, 1e-3);

    println!("angles:    {:?}", result.angles);
    println!("error:     {:.6}", result.error);
    println!("converged: {}", result.converged);

## What's included

### Free (AGPL-3.0)

| Module | What it does |
|--------|-------------|
| **Geometry** | Points, lines, planes, distances, incidences |
| **Motors** | Rotation + translation in one operator |
| **Interpolation** | Motor lerp/slerp (no gimbal lock) |
| **Compat** | 4x4 matrices, quaternions, glam, nalgebra |

### Premium (Commercial license)

| Module | What it does | Status |
|--------|-------------|--------|
| **IK + Jacobians** | Levenberg-Marquardt, joint limits | ✅ Available |
| **Planning** | Trajectory planning | ✅ Available |
| **Stability** | Center of mass, ZMP, support polygon | ✅ Available |
| **SLAM** | Pose graph optimization | ✅ Available |
| **Integrators** | Symplectic / geometric integration | ✅ Available |

## Roadmap

Planned, not yet shipped. Listed here so you know where geoalg is going —
nothing in this section is available today.

- **IK diagnostics** — the "why it failed" engine: joint-limit, reach and
  collision explanations with fix suggestions. This is our core mission.
- **Deterministic mode** — bit-exact results across Intel/AMD/ARM,
  for multiplayer games and robot safety certification.
- **Dynamics** — mass matrices, Coriolis, gravity.
- **Full collision** — CCD, swept volumes, distance queries.
- **Differentiable** — auto-diff for ML training.
- **More platforms** — Python (`pip install geoalg`), C/C++ (`geoalg.h`),
  Unity (`Geoalg.cs`), ROS 2 bridge, Bevy bridge.

## What it does NOT do

- Not a rendering engine.
- PGA 3D only (for now).
- If it's not in "What's included", it's not shipped yet — check the Roadmap.

## Platforms

| Platform | How |
|----------|-----|
| **Rust** | `cargo add geoalg` |
| **WebAssembly** | `wasm-pack build --target web` |

More platforms on the roadmap (Python, C/C++, Unity, ROS 2, Bevy).

## Licenses — Dual licensing

geoalg uses a dual license model to protect the author's work and enable open use:

| Who uses it | License | Cost |
| --- | --- | --- |
| Open source projects | AGPL-3.0 | Free |
| Students and research | AGPL-3.0 | Free |
| Commercial / closed-source products | Commercial | From $19/mo |

- **AGPL-3.0:** free, but any modification must be shared under AGPL-3.0.
- **Commercial:** allows use in proprietary products without sharing code.

&gt; **Important:** AGPL-3.0 requires you to release your full source code
&gt; if you use geoalg in a web application or SaaS product. If you need
&gt; to keep your code proprietary, purchase a Commercial License.

Commercial plans:

| Plan | Price | Developers | Products | Support |
| --- | --- | --- | --- | --- |
| Starter | $19/mo | 1 | 1 | Email (48h) |
| Team | $49/mo | Up to 3 | Unlimited | Priority (24h) |
| Business | $149/mo | Up to 10 | Unlimited | Priority (24h) |
| Enterprise | Contact | Unlimited | Unlimited | SLA guaranteed |

Annual billing: 2 months free.

See [LICENSE-AGPL](LICENSE-AGPL) and [LICENSE-COMMERCIAL](LICENSE-COMMERCIAL).

## Links

- **[Playground](https://geoalg.dev)** — try geoalg in your browser
- **[Pricing](https://geoalg.dev/pricing)** — commercial license
- **[GitHub](https://github.com/071476/geoalg)** — source code

## Support the project

If geoalg is useful to you, consider supporting it:
- **GitHub Sponsors:** https://github.com/sponsors/071476
- **Commercial inquiries:** ali-gafer@hotmail.com

## License

Dual license: AGPL-3.0 + Commercial. See [LICENSE-AGPL](LICENSE-AGPL) and [LICENSE-COMMERCIAL](LICENSE-COMMERCIAL).