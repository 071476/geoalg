// glam: puente entre geoalg y la libreria glam (Bevy, juegos, motores).
// Conversiones: Motor <-> Mat4, Rotor <-> Quat, Point <-> Vec3.

use glam::{Mat4, Quat, Vec3};
use geoalg_core::compat;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::rotor::Rotor;

// === Point <-> Vec3 ===

pub fn point_to_vec3(p: &Point) -> Vec3 {
    let (x, y, z) = p.coords().expect("point_to_vec3: punto ideal");
    Vec3::new(x as f32, y as f32, z as f32)
}

pub fn vec3_to_point(v: Vec3) -> Point {
    Point::new(v.x as f64, v.y as f64, v.z as f64)
}

// === Rotor <-> Quat ===

pub fn rotor_to_quat(r: &Rotor) -> Quat {
    let (w, x, y, z) = compat::rotor_to_quaternion(r);
    Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32)
}

pub fn quat_to_rotor(q: Quat) -> Rotor {
    compat::quaternion_to_rotor((q.w as f64, q.x as f64, q.y as f64, q.z as f64))
}

// === Motor <-> Mat4 ===

pub fn motor_to_mat4(m: &Motor) -> Mat4 {
    let t = compat::motor_to_matrix4(m);
    Mat4::from_cols_array(&[
        t[0][0] as f32, t[1][0] as f32, t[2][0] as f32, t[3][0] as f32,
        t[0][1] as f32, t[1][1] as f32, t[2][1] as f32, t[3][1] as f32,
        t[0][2] as f32, t[1][2] as f32, t[2][2] as f32, t[3][2] as f32,
        t[0][3] as f32, t[1][3] as f32, t[2][3] as f32, t[3][3] as f32,
    ])
}

pub fn mat4_to_motor(m: Mat4) -> Motor {
    let a = m.to_cols_array();
    let t = [
        [a[0] as f64, a[4] as f64, a[8] as f64, a[12] as f64],
        [a[1] as f64, a[5] as f64, a[9] as f64, a[13] as f64],
        [a[2] as f64, a[6] as f64, a[10] as f64, a[14] as f64],
        [0.0, 0.0, 0.0, 1.0],
    ];
    compat::matrix4_to_motor(&t)
}

// === Motor <-> Transform (posicion + rotacion) ===

pub fn motor_to_transform(m: &Motor) -> (Vec3, Quat) {
    let (pos, quat) = compat::motor_to_pose(m);
    (
        Vec3::new(pos.0 as f32, pos.1 as f32, pos.2 as f32),
        Quat::from_xyzw(quat.1 as f32, quat.2 as f32, quat.3 as f32, quat.0 as f32),
    )
}

pub fn transform_to_motor(pos: Vec3, rot: Quat) -> Motor {
    compat::pose_to_motor(
        (pos.x as f64, pos.y as f64, pos.z as f64),
        (rot.w as f64, rot.x as f64, rot.y as f64, rot.z as f64),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_core::types::line::Line;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn point_vec3_roundtrip() {
        let p = Point::new(1.0, -2.0, 3.0);
        let v = point_to_vec3(&p);
        let back = vec3_to_point(v);
        let (x, y, z) = back.coords().unwrap();
        assert!((x - 1.0).abs() < 1e-5);
        assert!((y + 2.0).abs() < 1e-5);
        assert!((z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn rotor_quat_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, 1.0);
        let q = rotor_to_quat(&r);
        let back = quat_to_rotor(q);
        let a = geoalg_core::compat::rotor_to_quaternion(&r);
        let b = geoalg_core::compat::rotor_to_quaternion(&back);
        assert!((a.0 - b.0).abs() < 1e-4);
    }

    #[test]
    fn motor_mat4_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let rot = Rotor::from_axis_angle(&axis, FRAC_PI_2).to_motor();
        let tra = Motor::from_translator(&geoalg_core::types::translator::Translator::new(1.0, 2.0, 3.0));
        let m = rot.then(&tra);
        let mat = motor_to_mat4(&m);
        let back = mat4_to_motor(mat);
        let (pos, _) = motor_to_transform(&back);
        assert!((pos.x - 1.0).abs() < 1e-3);
        assert!((pos.y - 2.0).abs() < 1e-3);
        assert!((pos.z - 3.0).abs() < 1e-3);
    }

    #[test]
    fn transform_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let rot = Rotor::from_axis_angle(&axis, 0.5).to_motor();
        let tra = Motor::from_translator(&geoalg_core::types::translator::Translator::new(-1.0, 0.0, 2.0));
        let m = rot.then(&tra);
        let (pos, quat) = motor_to_transform(&m);
        let back = transform_to_motor(pos, quat);
        let (p2, _) = motor_to_transform(&back);
        assert!((pos.x - p2.x).abs() < 1e-3);
        assert!((pos.y - p2.y).abs() < 1e-3);
        assert!((pos.z - p2.z).abs() < 1e-3);
    }
}