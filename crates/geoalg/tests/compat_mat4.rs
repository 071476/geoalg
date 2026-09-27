// Test de integracion: compatibilidad Motor <-> Mat4 como la usaria un programador.
use geoalg::prelude::*;
use geoalg::compat;

fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-4 && (a.1 - b.1).abs() < 1e-4 && (a.2 - b.2).abs() < 1e-4
}

#[test]
fn motor_mat4_roundtrip() {
    let r = pga3::rotor((0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
    let t = pga3::translator(1.0, 2.0, 3.0);
    let m = r.to_motor().then(&t.to_motor());

    let mat = compat::motor_to_matrix4(&m);
    let back = compat::matrix4_to_motor(&mat);

    let p = apply_point(&back.inner(), &Point::origin()).coords().unwrap();
    assert!(close(p, (1.0, 2.0, 3.0)), "origen movido a {p:?}");
}

#[test]
fn motor_mat4_apunta_lo_mismo() {
    let r = pga3::rotor((0.0, 1.0, 0.0), 0.7);
    let m = r.to_motor();

    let mat = compat::motor_to_matrix4(&m);
    let back = compat::matrix4_to_motor(&mat);

    // Ambos motores deben mover el punto (1, 0, 0) al mismo lugar.
    let p1 = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap();
    let p2 = apply_point(&back.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap();
    assert!(close(p1, p2), "{p1:?} vs {p2:?}");
}

#[test]
fn pose_roundtrip() {
    let m = pga3::motor_pos_quat((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0));
    let (pos, quat) = compat::motor_to_pose(&m);
    assert!(close(pos, (1.0, 2.0, 3.0)));
    assert!((quat.0 - 1.0).abs() < 1e-4 && quat.1.abs() < 1e-4);

    let back = compat::pose_to_motor(pos, quat);
    let p = apply_point(&back.inner(), &Point::origin()).coords().unwrap();
    assert!(close(p, (1.0, 2.0, 3.0)));
}

#[test]
fn euler_roundtrip() {
    let r = pga3::rotor((0.0, 1.0, 0.0), 0.8);
    let (roll, pitch, yaw) = compat::rotor_to_euler(&r);
    let back = compat::euler_to_rotor(roll, pitch, yaw);
    let q1 = compat::rotor_to_quaternion(&r);
    let q2 = compat::rotor_to_quaternion(&back);
    assert!(
        (q1.0 - q2.0).abs() < 1e-3 && (q1.1 - q2.1).abs() < 1e-3
            && (q1.2 - q2.2).abs() < 1e-3 && (q1.3 - q2.3).abs() < 1e-3,
        "{q1:?} vs {q2:?}"
    );
}