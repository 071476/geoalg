use geoalg::core::ops::screw::between;
use geoalg::prelude::*;
use std::fmt::Write as _;
use std::f64::consts::PI;

fn shoulder_motor(angle: f64) -> Motor {
    let axis = Line::through_points(
        &Point::new(0.0, 0.0, 1.5),
        &Point::new(0.0, 1.0, 1.5),
    );
    Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle))
}

fn main() {
    let frames = 48;
    let mut out = String::new();
    writeln!(out, "# geoalg animate figure").unwrap();

    let head = (0.0, 0.0, 2.0);
    let neck = (0.0, 0.0, 1.8);
    let chest = (0.0, 0.0, 1.5);
    let hip = (0.0, 0.0, 1.0);
    let foot_l = (-0.35, 0.0, 0.0);
    let foot_r = (0.35, 0.0, 0.0);

    let hand_l_rest = Point::new(-0.5, 0.0, 0.3);
    let hand_r_rest = Point::new(0.5, 0.0, 0.3);

    let arm_l_a = shoulder_motor(0.0);
    let arm_l_b = shoulder_motor(PI * 0.25);
    let arm_r_a = shoulder_motor(0.0);
    let arm_r_b = shoulder_motor(-PI * 0.55);

    for f in 0..frames {
        let phase = f as f64 / frames as f64 * PI * 2.0;
        let t = (1.0 - phase.cos()) * 0.5;

        let m_l = between(&arm_l_a, &arm_l_b, t);
        let m_r = between(&arm_r_a, &arm_r_b, t);

        let hand_l = apply_point(&m_l.inner(), &hand_l_rest).coords().unwrap();
        let hand_r = apply_point(&m_r.inner(), &hand_r_rest).coords().unwrap();

        writeln!(out, "frame {}", f).unwrap();

        let body_segs = [
            (head, neck),
            (neck, chest),
            (chest, hip),
            (hip, foot_l),
            (hip, foot_r),
        ];
        for (a, b) in &body_segs {
            writeln!(out, "segment {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
                a.0, a.1, a.2, b.0, b.1, b.2).unwrap();
        }
        writeln!(out, "segment {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
            chest.0, chest.1, chest.2, hand_l.0, hand_l.1, hand_l.2).unwrap();
        writeln!(out, "segment {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
            chest.0, chest.1, chest.2, hand_r.0, hand_r.1, hand_r.2).unwrap();

        for p in [head, neck, chest, hip, foot_l, foot_r, hand_l, hand_r] {
            writeln!(out, "point {:.4} {:.4} {:.4}", p.0, p.1, p.2).unwrap();
        }
    }

    std::fs::write("figure.txt", out).expect("no se pudo escribir figure.txt");
    println!("Personaje exportado a figure.txt ({} cuadros)", frames);
}