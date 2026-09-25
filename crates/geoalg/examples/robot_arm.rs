use geoalg::core::algebra::metric::Metric;
use geoalg::prelude::*;
use std::fmt::Write as _;

// Compone motores: aplica `g` en el marco local de `m`.
fn chain(m: &Motor, g: &Motor) -> Motor {
    Motor::from_dense(m.inner().geo(&g.inner(), &Metric::pga()))
}

fn rot_z(angle: f64) -> Motor {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle))
}

fn rot_x(angle: f64) -> Motor {
    let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
    Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle))
}

fn offset_z(len: f64) -> Motor {
    Motor::from_translator(&Translator::new(0.0, 0.0, len))
}

// Cinematica directa: 3 articulaciones (base, hombro, codo) y 3 eslabones de 1.
fn frame_points(a1: f64, a2: f64, a3: f64) -> [(f64, f64, f64); 4] {
    let mut pose = Motor::identity();
    let mut pts = [(0.0, 0.0, 0.0); 4];

    pose = chain(&pose, &rot_z(a1)); // giro de base sobre Z
    pose = chain(&pose, &offset_z(1.0));
    pts[1] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();

    pose = chain(&pose, &rot_x(a2)); // hombro sobre X
    pose = chain(&pose, &offset_z(1.0));
    pts[2] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();

    pose = chain(&pose, &rot_x(a3)); // codo sobre X
    pose = chain(&pose, &offset_z(1.0));
    pts[3] = apply_point(&pose.inner(), &Point::origin()).coords().unwrap();

    pts
}

fn main() {
    let frames = 48;
    let tau = std::f64::consts::PI * 2.0;
    let mut out = String::new();
    writeln!(out, "# geoalg robot arm").unwrap();
    for f in 0..frames {
        let t = f as f64 / frames as f64 * tau;
        let pts = frame_points(t, 0.8 * t.sin(), 0.6 * (2.0 * t).sin());
        writeln!(out, "frame {}", f).unwrap();
        for p in &pts {
            writeln!(out, "point {:.6} {:.6} {:.6}", p.0, p.1, p.2).unwrap();
        }
        for w in pts.windows(2) {
            writeln!(
                out,
                "segment {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
                w[0].0, w[0].1, w[0].2, w[1].0, w[1].1, w[1].2
            )
            .unwrap();
        }
    }
    std::fs::write("arm.txt", out).expect("no se pudo escribir arm.txt");
    println!("Brazo exportado a arm.txt ({} cuadros)", frames);
}