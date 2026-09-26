use geoalg::core::algebra::metric::Metric;
use geoalg::core::ops::screw::between;
use geoalg::prelude::*;
use std::fmt::Write as _;

fn chain(m: &Motor, g: &Motor) -> Motor {
    Motor::from_dense(m.inner().geo(&g.inner(), &Metric::pga()))
}

fn rot_z(a: f64) -> Motor {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    Motor::from_rotor(&Rotor::from_axis_angle(&axis, a))
}

fn rot_x(a: f64) -> Motor {
    let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
    Motor::from_rotor(&Rotor::from_axis_angle(&axis, a))
}

fn off_z(len: f64) -> Motor {
    Motor::from_translator(&Translator::new(0.0, 0.0, len))
}

fn arm_points(j1: &Motor, j2: &Motor, j3: &Motor) -> [(f64, f64, f64); 4] {
    let mut pose = Motor::identity();
    let mut pts = [(0.0, 0.0, 0.0); 4];
    pose = chain(&pose, j1);
    pose = chain(&pose, &off_z(1.0));
    pts[1] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();
    pose = chain(&pose, j2);
    pose = chain(&pose, &off_z(1.0));
    pts[2] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();
    pose = chain(&pose, j3);
    pose = chain(&pose, &off_z(1.0));
    pts[3] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();
    pts
}

fn write_arm(out: &mut String, tag: &str, pts: &[(f64, f64, f64); 4]) {
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        writeln!(
            out,
            "{} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
            tag, a.0, a.1, a.2, b.0, b.1, b.2
        )
        .unwrap();
    }
}

fn main() {
    // Pose A y pose B: tres articulaciones cada una.
    let a1 = rot_z(0.0);
    let a2 = rot_x(0.6);
    let a3 = rot_x(-0.4);

    let b1 = rot_z(2.2);
    let b2 = rot_x(-0.7);
    let b3 = rot_x(1.1);

    let ghost_a = arm_points(&a1, &a2, &a3);
    let ghost_b = arm_points(&b1, &b2, &b3);

    let frames = 48;
    let mut out = String::new();
    writeln!(out, "# geoalg interpolate pose").unwrap();
    write_arm(&mut out, "ghostA", &ghost_a);
    write_arm(&mut out, "ghostB", &ghost_b);

    for f in 0..frames {
        let t = f as f64 / (frames - 1) as f64;
        // La joya de la corona: interpolacion de motores.
        let j1 = between(&a1, &b1, t);
        let j2 = between(&a2, &b2, t);
        let j3 = between(&a3, &b3, t);
        let pts = arm_points(&j1, &j2, &j3);
        writeln!(out, "frame {}", f).unwrap();
        write_arm(&mut out, "segment", &pts);
        writeln!(
            out,
            "path {:.6} {:.6} {:.6}",
            pts[3].0, pts[3].1, pts[3].2
        )
        .unwrap();
    }

    std::fs::write("pose.txt", out).expect("no se pudo escribir pose.txt");
    println!("Interpolacion exportada a pose.txt ({} cuadros)", frames);
    let (x, y, z) = ghost_a[3];
    println!("Efector en pose A: ({x:.2}, {y:.2}, {z:.2})");
    let (x, y, z) = ghost_b[3];
    println!("Efector en pose B: ({x:.2}, {y:.2}, {z:.2})");
}