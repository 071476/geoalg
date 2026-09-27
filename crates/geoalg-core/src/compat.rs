// compat: conversiones entre geoalg y los formatos estandar de la industria.
//
// - Matriz 4x4: juegos (OpenGL, Unity, Unreal).
// - Cuaternion (w, x, y, z): motores 3D (Unity, Unreal, glTF).
// - Euler (roll, pitch, yaw): MATLAB, robótica clásica.
// - Pose (posición + cuaternion): ROS, USD, glTF.

use crate::algebra::metric::Metric;
use crate::ops::sandwich::apply_point;
use crate::types::motor::Motor;
use crate::types::point::Point;
use crate::types::rotor::Rotor;
use crate::types::translator::Translator;

// === Matriz 4x4 (row-major: 4 filas de 4 numeros) ===

// Motor -> matriz de transformacion homogenea 4x4.
pub fn motor_to_matrix4(m: &Motor) -> [[f64; 4]; 4] {
    let o = apply_point(&m.inner(), &Point::origin()).coords().unwrap();
    let px = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap();
    let py = apply_point(&m.inner(), &Point::new(0.0, 1.0, 0.0)).coords().unwrap();
    let pz = apply_point(&m.inner(), &Point::new(0.0, 0.0, 1.0)).coords().unwrap();
    [
        [px.0 - o.0, py.0 - o.0, pz.0 - o.0, o.0],
        [px.1 - o.1, py.1 - o.1, pz.1 - o.1, o.1],
        [px.2 - o.2, py.2 - o.2, pz.2 - o.2, o.2],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

// Matriz 4x4 -> motor (usa cuaternion + traslacion por dentro).
pub fn matrix4_to_motor(m: &[[f64; 4]; 4]) -> Motor {
    let t = Motor::from_translator(&Translator::new(m[0][3], m[1][3], m[2][3]));
    // Extraer la rotacion de la matriz superior izquierda 3x3 via cuaternion.
    let r = matrix3_to_rotor(&[[m[0][0], m[0][1], m[0][2]],
                               [m[1][0], m[1][1], m[1][2]],
                               [m[2][0], m[2][1], m[2][2]]]);
    let metric = Metric::pga();
    Motor::from_dense(t.inner().geo(&r.to_motor().inner(), &metric))
}

// === Cuaternion (w, x, y, z) ===

// Rotor -> cuaternion.
pub fn rotor_to_quaternion(r: &Rotor) -> (f64, f64, f64, f64) {
    let w = r.inner().get(0);
    let x = -r.inner().get(crate::algebra::basis::E2 | crate::algebra::basis::E3);
    let y = r.inner().get(crate::algebra::basis::E1 | crate::algebra::basis::E3);
    let z = -r.inner().get(crate::algebra::basis::E1 | crate::algebra::basis::E2);
    (w, x, y, z)
}

// Cuaternion -> rotor.
pub fn quaternion_to_rotor(q: (f64, f64, f64, f64)) -> Rotor {
    use crate::algebra::basis::{E1, E2, E3};
    let mut d = crate::multivector::dense::Dense::zero();
    d.set(0, q.0);
    d.set(E2 | E3, -q.1);
    d.set(E1 | E3, q.2);
    d.set(E1 | E2, -q.3);
    Rotor::from_dense(d)
}

// === Euler XYZ (roll=X, pitch=Y, yaw=Z) ===

// Rotor -> angulos de Euler en radianes.
pub fn rotor_to_euler(r: &Rotor) -> (f64, f64, f64) {
    let (w, x, y, z) = rotor_to_quaternion(&r.normalize());
    // Formula estandar de cuaternion a Euler XYZ.
    let sinr = 2.0 * (w * x + y * z);
    let cosr = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr.atan2(cosr);
    let sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = sinp.asin();
    let siny = 2.0 * (w * z + x * y);
    let cosy = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny.atan2(cosy);
    (roll, pitch, yaw)
}

// Angulos de Euler -> rotor.
pub fn euler_to_rotor(roll: f64, pitch: f64, yaw: f64) -> Rotor {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;
    quaternion_to_rotor((w, x, y, z))
}

// === Pose: posicion + cuaternion (formato ROS, glTF) ===

// Motor -> pose.
pub fn motor_to_pose(m: &Motor) -> ((f64, f64, f64), (f64, f64, f64, f64)) {
    let pos = apply_point(&m.inner(), &Point::origin()).coords().unwrap();
    // Extraer la rotacion del motor (parte par sin e0).
    let r = Rotor::from_dense(m.inner().grade(0) + m.inner().grade(2));
    (pos, rotor_to_quaternion(&r.normalize()))
}

// Pose -> motor.
pub fn pose_to_motor(pos: (f64, f64, f64), quat: (f64, f64, f64, f64)) -> Motor {
    let t = Motor::from_translator(&Translator::new(pos.0, pos.1, pos.2));
    let r = quaternion_to_rotor(quat).to_motor();
    let metric = Metric::pga();
    Motor::from_dense(t.inner().geo(&r.inner(), &metric))
}

// Matriz 3x3 de rotacion -> rotor (via cuaternion interno).
fn matrix3_to_rotor(m: &[[f64; 3]; 3]) -> Rotor {
    let trace = m[0][0] + m[1][1] + m[2][2];
    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    };
    quaternion_to_rotor((w, x, y, z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::line::Line;
    use std::f64::consts::PI;

    fn close4(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
            && (a.2 - b.2).abs() < 1e-6 && (a.3 - b.3).abs() < 1e-6
    }

    #[test]
    fn cuaternion_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, 1.2);
        let q = rotor_to_quaternion(&r);
        let back = quaternion_to_rotor(q);
        assert!(close4(rotor_to_quaternion(&r), rotor_to_quaternion(&back)));
    }

    #[test]
    fn euler_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 1.0, 0.0));
        let r = Rotor::from_axis_angle(&axis, 0.8);
        let (roll, pitch, yaw) = rotor_to_euler(&r);
        let back = euler_to_rotor(roll, pitch, yaw);
        assert!(close4(rotor_to_quaternion(&r), rotor_to_quaternion(&back)));
    }

    #[test]
    fn matriz_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let rot = Rotor::from_axis_angle(&axis, PI / 2.0).to_motor();
        let tra = Motor::from_translator(&Translator::new(1.0, 2.0, 3.0));
        let m = rot.then(&tra);
        let mat = motor_to_matrix4(&m);
        let back = matrix4_to_motor(&mat);
        // El origen debe ir a (1, 2, 3).
        let p = apply_point(&back.inner(), &Point::origin()).coords().unwrap();
        assert!((p.0 - 1.0).abs() < 1e-6 && (p.1 - 2.0).abs() < 1e-6 && (p.2 - 3.0).abs() < 1e-6);
    }

    #[test]
    fn pose_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let rot = Rotor::from_axis_angle(&axis, 0.5).to_motor();
        let tra = Motor::from_translator(&Translator::new(-1.0, 0.0, 2.0));
        let m = rot.then(&tra);
        let (pos, quat) = motor_to_pose(&m);
        let back = pose_to_motor(pos, quat);
        let p = apply_point(&back.inner(), &Point::origin()).coords().unwrap();
        assert!((p.0 - pos.0).abs() < 1e-6, "x: {} vs {}", p.0, pos.0);
        assert!((p.1 - pos.1).abs() < 1e-6);
        assert!((p.2 - pos.2).abs() < 1e-6);
    }
}