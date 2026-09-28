// geoalg-ffi: puente C para Unity, Unreal y otros motores de juegos.
// Cada función recibe y devuelve datos planos (floats, arrays) que
// C, C++ y C# entienden sin traducción.

use geoalg_core::compat;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::line::Line;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::rotor::Rotor;
use geoalg_core::types::translator::Translator;

// === Tipos C ===

/// Posición 3D compatible con C.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GeoVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Cuaternión (w, x, y, z) compatible con C.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GeoQuat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Matriz 4x4 (row-major) compatible con C.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GeoMat4 {
    pub m: [f64; 16], // 4 filas de 4 columnas
}

/// Motor (pose rígida) como 8 floats: escalar + 6 bivectores + pseudoscalar.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GeoMotor {
    pub data: [f64; 8],
}

// === Crear motores ===

/// Motor identidad (sin movimiento).
#[no_mangle]
pub extern "C" fn geoalg_motor_identity() -> GeoMotor {
    let m = Motor::identity();
    let d = m.inner();
    GeoMotor {
        data: [
            d.get(0), d.get(5), d.get(6), d.get(9),
            d.get(10), d.get(3), d.get(12), d.get(15),
        ],
    }
}

/// Motor de traslación.
#[no_mangle]
pub extern "C" fn geoalg_motor_from_translation(dx: f64, dy: f64, dz: f64) -> GeoMotor {
    let m = Motor::from_translator(&Translator::new(dx, dy, dz));
    let d = m.inner();
    GeoMotor {
        data: [
            d.get(0), d.get(5), d.get(6), d.get(9),
            d.get(10), d.get(3), d.get(12), d.get(15),
        ],
    }
}

/// Motor de rotación alrededor de un eje que pasa por el origen.
#[no_mangle]
pub extern "C" fn geoalg_motor_from_rotation(
    ax: f64, ay: f64, az: f64,
    angle: f64,
) -> GeoMotor {
    let axis = Line::through_points(
        &Point::origin(),
        &Point::new(ax, ay, az),
    );
    let m = Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle));
    let d = m.inner();
    GeoMotor {
        data: [
            d.get(0), d.get(5), d.get(6), d.get(9),
            d.get(10), d.get(3), d.get(12), d.get(15),
        ],
    }
}

// === Componer y transformar ===

/// Composición: `out = a * b` (aplicar `a` primero, luego `b`).
#[no_mangle]
pub extern "C" fn geoalg_motor_compose(a: &GeoMotor, b: &GeoMotor) -> GeoMotor {
    let ma = motor_from_c(a);
    let mb = motor_from_c(b);
    let m = ma.then(&mb);
    motor_to_c(&m)
}

/// Aplicar motor a un punto → devuelve el punto transformado.
#[no_mangle]
pub extern "C" fn geoalg_motor_apply_point(
    m: &GeoMotor,
    x: f64, y: f64, z: f64,
) -> GeoVec3 {
    let motor = motor_from_c(m);
    let p = apply_point(&motor.inner(), &Point::new(x, y, z));
    let (px, py, pz) = p.coords().unwrap_or((0.0, 0.0, 0.0));
    GeoVec3 { x: px, y: py, z: pz }
}

// === Convertir al mundo exterior ===

/// Motor → Matriz 4x4 (row-major) para OpenGL/Unity/Unreal.
#[no_mangle]
pub extern "C" fn geoalg_motor_to_mat4(m: &GeoMotor) -> GeoMat4 {
    let motor = motor_from_c(m);
    let t = compat::motor_to_matrix4(&motor);
    GeoMat4 {
        m: [
            t[0][0], t[0][1], t[0][2], t[0][3],
            t[1][0], t[1][1], t[1][2], t[1][3],
            t[2][0], t[2][1], t[2][2], t[2][3],
            t[3][0], t[3][1], t[3][2], t[3][3],
        ],
    }
}

/// Matriz 4x4 → Motor.
#[no_mangle]
pub extern "C" fn geoalg_mat4_to_motor(m: &GeoMat4) -> GeoMotor {
    let t = [
        [m.m[0], m.m[1], m.m[2], m.m[3]],
        [m.m[4], m.m[5], m.m[6], m.m[7]],
        [m.m[8], m.m[9], m.m[10], m.m[11]],
        [m.m[12], m.m[13], m.m[14], m.m[15]],
    ];
    let motor = compat::matrix4_to_motor(&t);
    motor_to_c(&motor)
}

/// Motor → posición + cuaternión (formato ROS/Unity/gltf).
#[no_mangle]
pub extern "C" fn geoalg_motor_to_pose(m: &GeoMotor) -> GeoVec3 {
    let motor = motor_from_c(m);
    let (pos, _quat) = compat::motor_to_pose(&motor);
    GeoVec3 { x: pos.0, y: pos.1, z: pos.2 }
}

/// Motor → cuaternión.
#[no_mangle]
pub extern "C" fn geoalg_motor_to_quat(m: &GeoMotor) -> GeoQuat {
    let motor = motor_from_c(m);
    let (_pos, quat) = compat::motor_to_pose(&motor);
    GeoQuat { w: quat.0, x: quat.1, y: quat.2, z: quat.3 }
}

/// Posición + cuaternión → Motor.
#[no_mangle]
pub extern "C" fn geoalg_pose_to_motor(
    px: f64, py: f64, pz: f64,
    qw: f64, qx: f64, qy: f64, qz: f64,
) -> GeoMotor {
    let m = compat::pose_to_motor(
        (px, py, pz),
        (qw, qx, qy, qz),
    );
    motor_to_c(&m)
}

// === Helpers internos ===

fn motor_to_c(m: &Motor) -> GeoMotor {
    let d = m.inner();
    GeoMotor {
        data: [
            d.get(0), d.get(5), d.get(6), d.get(9),
            d.get(10), d.get(3), d.get(12), d.get(15),
        ],
    }
}

fn motor_from_c(m: &GeoMotor) -> Motor {
    let mut d = geoalg_core::multivector::dense::Dense::zero();
    d.set(0, m.data[0]);
    d.set(5, m.data[1]);  // e12
    d.set(6, m.data[2]);  // e13
    d.set(9, m.data[3]);  // e23
    d.set(10, m.data[4]); // e01
    d.set(3, m.data[5]);  // e02
    d.set(12, m.data[6]); // e03
    d.set(15, m.data[7]); // e0123
    Motor::from_dense(d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_identity_ffi() {
        let m = geoalg_motor_identity();
        let p = geoalg_motor_apply_point(&m, 1.0, 2.0, 3.0);
        assert!((p.x - 1.0).abs() < 1e-9);
        assert!((p.y - 2.0).abs() < 1e-9);
        assert!((p.z - 3.0).abs() < 1e-9);
    }

    #[test]
    fn motor_translation_ffi() {
        let m = geoalg_motor_from_translation(1.0, 2.0, 3.0);
        let p = geoalg_motor_apply_point(&m, 0.0, 0.0, 0.0);
        assert!((p.x - 1.0).abs() < 1e-9);
        assert!((p.y - 2.0).abs() < 1e-9);
        assert!((p.z - 3.0).abs() < 1e-9);
    }

    #[test]
    fn motor_mat4_roundtrip_ffi() {
        let rot = geoalg_motor_from_rotation(0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2);
        let tra = geoalg_motor_from_translation(1.0, 0.0, 0.0);
        let m = geoalg_motor_compose(&rot, &tra);
        let mat = geoalg_motor_to_mat4(&m);
        let back = geoalg_mat4_to_motor(&mat);
        let p = geoalg_motor_apply_point(&back, 1.0, 0.0, 0.0);
        // Gira (1,0,0) → (0,1,0), traslada +X → (1,1,0).
        assert!((p.x - 1.0).abs() < 1e-4);
        assert!((p.y - 1.0).abs() < 1e-4);
        assert!(p.z.abs() < 1e-4);
    }

    #[test]
    fn pose_roundtrip_ffi() {
        let m = geoalg_pose_to_motor(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0);
        let pos = geoalg_motor_to_pose(&m);
        assert!((pos.x - 1.0).abs() < 1e-4);
        assert!((pos.y - 2.0).abs() < 1e-4);
        assert!((pos.z - 3.0).abs() < 1e-4);
    }
}