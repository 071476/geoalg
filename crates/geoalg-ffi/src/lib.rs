// geoalg-ffi: puente C para Unity, Unreal y otros motores.
// Cada funcion recibe y devuelve datos planos (numeros y arreglos) que
// C, C++ y C# entienden sin traduccion.
//
// Dos partes:
//   1. Motores: crear, componer y convertir poses (matriz, cuaternion).
//   2. Robots: crear un robot, agregarle articulaciones, FK e IK.
//      Es el MISMO motor que usa el studio en el navegador.
//
// Convenciones (IMPORTANTE para Unity y Unreal):
// - geoalg usa la regla de la MANO DERECHA (como ROS, three.js y glTF).
//   Unity y Unreal usan la MANO IZQUIERDA: hay que convertir los ejes
//   al pasar datos a esos motores (ver el wrapper de C# de Unity).
// - Las matrices 4x4 salen por FILAS (row-major), con la traslacion en
//   m[3], m[7] y m[11] (convencion de vectores columna, como Unity).
//   Para OpenGL (por columnas) hay que transponerla.
// - Los cuaterniones van en orden (w, x, y, z).
// - GeoMotor es OPACO: sus 8 numeros no deben interpretarse a mano.
// - Si se pasa un puntero NULL, la funcion lo trata como el motor
//   identidad (no truena).

use std::panic::{catch_unwind, AssertUnwindSafe};

use geoalg_core::algebra::basis::Blade;
use geoalg_core::compat;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::line::Line;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::rotor::Rotor;
use geoalg_core::types::translator::Translator;
use geoalg_robotics::fk::forward;
use geoalg_robotics::model::{demo_arm, Robot};

// === Tipos C ===

/// Posicion 3D compatible con C.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GeoVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Cuaternion (w, x, y, z) compatible con C.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GeoQuat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Posicion + cuaternion juntos (formato ROS / glTF).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GeoPose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

/// Matriz 4x4 por filas (row-major), traslacion en m[3], m[7], m[11].
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GeoMat4 {
    pub m: [f64; 16],
}

/// Motor (movimiento rigido: giro + traslacion). Es OPACO: usa las
/// funciones de esta libreria para crearlo y leerlo.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GeoMotor {
    pub data: [f64; 8],
}

// === Crear motores ===

/// Motor identidad (sin movimiento).
#[no_mangle]
pub extern "C" fn geoalg_motor_identity() -> GeoMotor {
    motor_to_c(&Motor::identity())
}

/// Motor de traslacion.
#[no_mangle]
pub extern "C" fn geoalg_motor_from_translation(dx: f64, dy: f64, dz: f64) -> GeoMotor {
    motor_to_c(&Motor::from_translator(&Translator::new(dx, dy, dz)))
}

/// Motor de rotacion de `angle` radianes alrededor del eje (ax, ay, az)
/// que pasa por el origen. Si el eje es (0, 0, 0), devuelve la identidad.
#[no_mangle]
pub extern "C" fn geoalg_motor_from_rotation(ax: f64, ay: f64, az: f64, angle: f64) -> GeoMotor {
    let len = (ax * ax + ay * ay + az * az).sqrt();
    if !len.is_finite() || len < 1e-15 || !angle.is_finite() {
        return geoalg_motor_identity();
    }
    let axis = Line::through_points(
        &Point::origin(),
        &Point::new(ax / len, ay / len, az / len),
    );
    motor_to_c(&Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle)))
}

// === Componer y transformar ===

/// Composicion: aplica `a` primero y luego `b`.
///
/// # Safety
/// `a` y `b` deben ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_compose(a: *const GeoMotor, b: *const GeoMotor) -> GeoMotor {
    let ma = read_motor(a);
    let mb = read_motor(b);
    motor_to_c(&ma.then(&mb))
}

/// Aplica el motor a un punto y devuelve el punto movido.
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_apply_point(m: *const GeoMotor, x: f64, y: f64, z: f64) -> GeoVec3 {
    let motor = read_motor(m);
    let p = apply_point(&motor.inner(), &Point::new(x, y, z));
    let (px, py, pz) = p.coords().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    GeoVec3 { x: px, y: py, z: pz }
}

// === Convertir al mundo exterior ===

/// Motor a matriz 4x4 por filas (traslacion en m[3], m[7], m[11]).
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_to_mat4(m: *const GeoMotor) -> GeoMat4 {
    let motor = read_motor(m);
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

/// Matriz 4x4 por filas a motor.
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMat4 valido. NULL devuelve la identidad.
#[no_mangle]
pub unsafe extern "C" fn geoalg_mat4_to_motor(m: *const GeoMat4) -> GeoMotor {
    if m.is_null() {
        return geoalg_motor_identity();
    }
    // SAFETY: revisado arriba que no es NULL; el llamador garantiza que es valido.
    let m = &*m;
    let t = [
        [m.m[0], m.m[1], m.m[2], m.m[3]],
        [m.m[4], m.m[5], m.m[6], m.m[7]],
        [m.m[8], m.m[9], m.m[10], m.m[11]],
        [m.m[12], m.m[13], m.m[14], m.m[15]],
    ];
    motor_to_c(&compat::matrix4_to_motor(&t))
}

/// Solo la posicion del motor (a donde manda el origen).
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_to_position(m: *const GeoMotor) -> GeoVec3 {
    let motor = read_motor(m);
    let (pos, _quat) = compat::motor_to_pose(&motor);
    GeoVec3 { x: pos.0, y: pos.1, z: pos.2 }
}

/// Solo el giro del motor, como cuaternion (w, x, y, z).
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_to_quat(m: *const GeoMotor) -> GeoQuat {
    let motor = read_motor(m);
    let (_pos, quat) = compat::motor_to_pose(&motor);
    GeoQuat { w: quat.0, x: quat.1, y: quat.2, z: quat.3 }
}

/// Posicion + cuaternion juntos (formato ROS / glTF).
///
/// # Safety
/// `m` debe ser NULL o apuntar a un GeoMotor valido.
#[no_mangle]
pub unsafe extern "C" fn geoalg_motor_to_pose(m: *const GeoMotor) -> GeoPose {
    let motor = read_motor(m);
    let (pos, quat) = compat::motor_to_pose(&motor);
    GeoPose {
        x: pos.0,
        y: pos.1,
        z: pos.2,
        qw: quat.0,
        qx: quat.1,
        qy: quat.2,
        qz: quat.3,
    }
}

/// Posicion + cuaternion a motor.
#[no_mangle]
pub extern "C" fn geoalg_pose_to_motor(
    px: f64, py: f64, pz: f64,
    qw: f64, qx: f64, qy: f64, qz: f64,
) -> GeoMotor {
    motor_to_c(&compat::pose_to_motor((px, py, pz), (qw, qx, qy, qz)))
}

// =====================================================================
// === Robots ===
// =====================================================================

/// Todo salio bien.
pub const GEOALG_OK: i32 = 0;
/// Llego un puntero vacio (NULL).
pub const GEOALG_ERR_NULL: i32 = -1;
/// Un argumento no tiene sentido (NaN, tamano equivocado, min > max...).
pub const GEOALG_ERR_ARGUMENT: i32 = -2;
/// Algo fallo dentro de geoalg. No deberia pasar; si pasa, avisa.
pub const GEOALG_ERR_INTERNAL: i32 = -99;

/// Robot opaco: C y C# solo ven un puntero, nunca su contenido.
pub struct GeoRobot {
    inner: Robot,
}

// Atrapa cualquier falla de Rust para que no tumbe Unity.
fn guard<F: FnOnce() -> i32>(f: F) -> i32 {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(GEOALG_ERR_INTERNAL)
}

fn finite3(a: f64, b: f64, c: f64) -> bool {
    a.is_finite() && b.is_finite() && c.is_finite()
}

/// Crea un robot vacio. Liberalo con geoalg_robot_free.
#[no_mangle]
pub extern "C" fn geoalg_robot_new() -> *mut GeoRobot {
    Box::into_raw(Box::new(GeoRobot { inner: Robot::new() }))
}

/// Crea el brazo demo de 3 articulaciones (Z hacia arriba, tramos de 1).
#[no_mangle]
pub extern "C" fn geoalg_robot_demo_arm() -> *mut GeoRobot {
    Box::into_raw(Box::new(GeoRobot { inner: demo_arm() }))
}

/// Libera un robot. Pasar NULL no hace nada.
///
/// # Safety
/// `robot` debe venir de geoalg_robot_new o geoalg_robot_demo_arm,
/// y no se debe usar despues de liberarlo.
#[no_mangle]
pub unsafe extern "C" fn geoalg_robot_free(robot: *mut GeoRobot) {
    if !robot.is_null() {
        drop(Box::from_raw(robot));
    }
}

/// Numero de articulaciones, o GEOALG_ERR_NULL.
///
/// # Safety
/// `robot` debe ser un robot valido o NULL.
#[no_mangle]
pub unsafe extern "C" fn geoalg_robot_joint_count(robot: *const GeoRobot) -> i32 {
    match robot.as_ref() {
        Some(r) => r.inner.len() as i32,
        None => GEOALG_ERR_NULL,
    }
}

// Comun a revolute y prismatic.
unsafe fn add_joint(
    robot: *mut GeoRobot,
    revolute: bool,
    dir: (f64, f64, f64),
    offset: (f64, f64, f64),
    min: f64,
    max: f64,
) -> i32 {
    let r = match robot.as_mut() {
        Some(r) => r,
        None => return GEOALG_ERR_NULL,
    };
    let largo = (dir.0 * dir.0 + dir.1 * dir.1 + dir.2 * dir.2).sqrt();
    if !finite3(dir.0, dir.1, dir.2)
        || !finite3(offset.0, offset.1, offset.2)
        || !(largo > 1e-12)
        || !min.is_finite()
        || !max.is_finite()
        || min > max
    {
        return GEOALG_ERR_ARGUMENT;
    }
    guard(|| {
        // El eje pasa por la propia articulacion: origin = (0, 0, 0).
        if revolute {
            r.inner.add_revolute((0.0, 0.0, 0.0), dir, offset, (min, max));
        } else {
            r.inner.add_prismatic((0.0, 0.0, 0.0), dir, offset, (min, max));
        }
        GEOALG_OK
    })
}

/// Agrega una articulacion que GIRA sobre el eje (dx, dy, dz).
/// (ox, oy, oz) es el tramo hasta la siguiente articulacion.
/// min y max en radianes.
///
/// # Safety
/// `robot` debe ser un robot valido o NULL.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn geoalg_robot_add_revolute(
    robot: *mut GeoRobot,
    dx: f64,
    dy: f64,
    dz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    min: f64,
    max: f64,
) -> i32 {
    add_joint(robot, true, (dx, dy, dz), (ox, oy, oz), min, max)
}

/// Agrega una articulacion que DESLIZA sobre el eje (dx, dy, dz).
/// min y max en las mismas unidades que las distancias.
///
/// # Safety
/// `robot` debe ser un robot valido o NULL.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn geoalg_robot_add_prismatic(
    robot: *mut GeoRobot,
    dx: f64,
    dy: f64,
    dz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    min: f64,
    max: f64,
) -> i32 {
    add_joint(robot, false, (dx, dy, dz), (ox, oy, oz), min, max)
}

/// Cinematica directa.
/// `q`: n valores (uno por articulacion; n debe ser igual a joint_count).
/// `out_points`: aqui se escriben (n + 1) puntos x, y, z seguidos:
/// la base y despues cada articulacion; el ultimo es la punta.
/// `out_len` debe ser al menos (n + 1) * 3.
///
/// # Safety
/// `q` debe apuntar a n doubles y `out_points` a out_len doubles.
#[no_mangle]
pub unsafe extern "C" fn geoalg_robot_fk(
    robot: *const GeoRobot,
    q: *const f64,
    n: usize,
    out_points: *mut f64,
    out_len: usize,
) -> i32 {
    let r = match robot.as_ref() {
        Some(r) => r,
        None => return GEOALG_ERR_NULL,
    };
    if n != r.inner.len() || out_len < (n + 1) * 3 {
        return GEOALG_ERR_ARGUMENT;
    }
    if (n > 0 && q.is_null()) || out_points.is_null() {
        return GEOALG_ERR_NULL;
    }
    let q: &[f64] = if n == 0 { &[] } else { std::slice::from_raw_parts(q, n) };
    if q.iter().any(|v| !v.is_finite()) {
        return GEOALG_ERR_ARGUMENT;
    }
    let out = std::slice::from_raw_parts_mut(out_points, out_len);
    guard(|| {
        let fk = forward(&r.inner, q);
        for (i, p) in fk.joints.iter().enumerate() {
            out[i * 3] = p.0;
            out[i * 3 + 1] = p.1;
            out[i * 3 + 2] = p.2;
        }
        GEOALG_OK
    })
}

/// Cinematica inversa de posicion: llevar la punta a (tx, ty, tz).
/// `q0`: postura inicial (n valores). `out_q`: aqui se escriben n angulos.
/// `out_error`: distancia final al objetivo (puede ser NULL si no la quieres).
///
/// Devuelve 1 si llego, 0 si no llego (out_q trae la mejor postura
/// encontrada), o un codigo de error negativo.
///
/// # Safety
/// `q0` y `out_q` deben apuntar a n doubles; `out_error` a un double o NULL.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn geoalg_robot_ik(
    robot: *const GeoRobot,
    tx: f64,
    ty: f64,
    tz: f64,
    q0: *const f64,
    n: usize,
    max_iter: u32,
    tol: f64,
    out_q: *mut f64,
    out_error: *mut f64,
) -> i32 {
    let r = match robot.as_ref() {
        Some(r) => r,
        None => return GEOALG_ERR_NULL,
    };
    if n != r.inner.len() || !finite3(tx, ty, tz) || !(tol > 0.0) || max_iter == 0 {
        return GEOALG_ERR_ARGUMENT;
    }
    if n > 0 && (q0.is_null() || out_q.is_null()) {
        return GEOALG_ERR_NULL;
    }
    let q0: &[f64] = if n == 0 { &[] } else { std::slice::from_raw_parts(q0, n) };
    if q0.iter().any(|v| !v.is_finite()) {
        return GEOALG_ERR_ARGUMENT;
    }
    let out: &mut [f64] = if n == 0 {
        &mut []
    } else {
        std::slice::from_raw_parts_mut(out_q, n)
    };
    guard(|| {
        let sol = r.inner.ik_xyz([tx, ty, tz], q0, max_iter as usize, tol);
        for (o, a) in out.iter_mut().zip(sol.angles.iter()) {
            *o = *a;
        }
        if !out_error.is_null() {
            *out_error = sol.error;
        }
        if sol.converged {
            1
        } else {
            0
        }
    })
}

// === Ayudas internas ===

// Lee un motor desde un puntero de C. NULL cuenta como identidad.
unsafe fn read_motor(p: *const GeoMotor) -> Motor {
    if p.is_null() {
        return Motor::identity();
    }
    // SAFETY: revisado arriba que no es NULL; el llamador garantiza que es valido.
    motor_from_c(&*p)
}

// Orden interno de los 8 componentes del motor dentro de Dense.
// Es el mismo para leer y para escribir, asi que nunca se pierde nada.
const SLOTS: [Blade; 8] = [0, 5, 6, 9, 10, 3, 12, 15];

fn motor_to_c(m: &Motor) -> GeoMotor {
    let d = m.inner();
    let mut data = [0.0; 8];
    for (k, &slot) in SLOTS.iter().enumerate() {
        data[k] = d.get(slot);
    }
    GeoMotor { data }
}

fn motor_from_c(m: &GeoMotor) -> Motor {
    let mut d = geoalg_core::multivector::dense::Dense::zero();
    for (k, &slot) in SLOTS.iter().enumerate() {
        d.set(slot, m.data[k]);
    }
    Motor::from_dense(d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
    use std::ptr;

    const EPS: f64 = 1e-9;

    fn cerca(p: GeoVec3, x: f64, y: f64, z: f64) -> bool {
        (p.x - x).abs() < EPS && (p.y - y).abs() < EPS && (p.z - z).abs() < EPS
    }

    fn giro_z_90() -> GeoMotor {
        geoalg_motor_from_rotation(0.0, 0.0, 1.0, FRAC_PI_2)
    }

    #[test]
    fn motor_identity_ffi() {
        let m = geoalg_motor_identity();
        let p = unsafe { geoalg_motor_apply_point(&m, 1.0, 2.0, 3.0) };
        assert!(cerca(p, 1.0, 2.0, 3.0), "{:?}", p);
    }

    #[test]
    fn motor_translation_ffi() {
        let m = geoalg_motor_from_translation(1.0, 2.0, 3.0);
        let p = unsafe { geoalg_motor_apply_point(&m, 0.0, 0.0, 0.0) };
        assert!(cerca(p, 1.0, 2.0, 3.0), "{:?}", p);
    }

    #[test]
    fn motor_rotation_ffi() {
        // Mano derecha: girar (1,0,0) 90 grados en Z da (0,1,0).
        let m = giro_z_90();
        let p = unsafe { geoalg_motor_apply_point(&m, 1.0, 0.0, 0.0) };
        assert!(cerca(p, 0.0, 1.0, 0.0), "{:?}", p);
    }

    #[test]
    fn eje_sin_normalizar_y_eje_cero() {
        // Un eje largo gira igual que uno de largo 1.
        let m = geoalg_motor_from_rotation(0.0, 0.0, 5.0, FRAC_PI_2);
        let p = unsafe { geoalg_motor_apply_point(&m, 1.0, 0.0, 0.0) };
        assert!(cerca(p, 0.0, 1.0, 0.0), "{:?}", p);
        // Un eje (0,0,0) no tiene sentido: devuelve la identidad.
        let m = geoalg_motor_from_rotation(0.0, 0.0, 0.0, 1.0);
        let p = unsafe { geoalg_motor_apply_point(&m, 1.0, 2.0, 3.0) };
        assert!(cerca(p, 1.0, 2.0, 3.0), "{:?}", p);
    }

    #[test]
    fn motor_compose_order() {
        let rot = giro_z_90();
        let tra = geoalg_motor_from_translation(1.0, 0.0, 0.0);
        // Primero girar, luego trasladar: (1,0,0) -> (0,1,0) -> (1,1,0).
        let a = unsafe { geoalg_motor_compose(&rot, &tra) };
        let p = unsafe { geoalg_motor_apply_point(&a, 1.0, 0.0, 0.0) };
        assert!(cerca(p, 1.0, 1.0, 0.0), "girar y trasladar: {:?}", p);
        // Primero trasladar, luego girar: (1,0,0) -> (2,0,0) -> (0,2,0).
        let b = unsafe { geoalg_motor_compose(&tra, &rot) };
        let p = unsafe { geoalg_motor_apply_point(&b, 1.0, 0.0, 0.0) };
        assert!(cerca(p, 0.0, 2.0, 0.0), "trasladar y girar: {:?}", p);
    }

    #[test]
    fn motor_mat4_roundtrip_ffi() {
        let rot = giro_z_90();
        let tra = geoalg_motor_from_translation(1.0, 0.0, 0.0);
        let m = unsafe { geoalg_motor_compose(&rot, &tra) };
        let mat = unsafe { geoalg_motor_to_mat4(&m) };
        let back = unsafe { geoalg_mat4_to_motor(&mat) };
        let p = unsafe { geoalg_motor_apply_point(&back, 1.0, 0.0, 0.0) };
        assert!(cerca(p, 1.0, 1.0, 0.0), "{:?}", p);
    }

    #[test]
    fn mat4_acomodo_por_filas() {
        // Giro 90 en Z y luego traslado (1,2,3).
        let rot = giro_z_90();
        let tra = geoalg_motor_from_translation(1.0, 2.0, 3.0);
        let m = unsafe { geoalg_motor_compose(&rot, &tra) };
        let a = unsafe { geoalg_motor_to_mat4(&m) }.m;
        // Traslacion en la ultima columna.
        assert!((a[3] - 1.0).abs() < EPS && (a[7] - 2.0).abs() < EPS && (a[11] - 3.0).abs() < EPS, "{:?}", a);
        // Giro de 90 en Z por filas: [0 -1 0] [1 0 0] [0 0 1].
        assert!(a[0].abs() < EPS && (a[1] + 1.0).abs() < EPS, "fila 0: {:?}", &a[0..4]);
        assert!((a[4] - 1.0).abs() < EPS && a[5].abs() < EPS, "fila 1: {:?}", &a[4..8]);
        assert!((a[10] - 1.0).abs() < EPS, "fila 2: {:?}", &a[8..12]);
        // Ultima fila: 0 0 0 1.
        assert!(a[12].abs() < EPS && a[13].abs() < EPS && a[14].abs() < EPS && (a[15] - 1.0).abs() < EPS, "{:?}", &a[12..16]);
    }

    #[test]
    fn cuaternion_de_giro_en_z() {
        // q y -q son el mismo giro: se compara sin importar el signo.
        let q = unsafe { geoalg_motor_to_quat(&giro_z_90()) };
        let dot = q.w * FRAC_1_SQRT_2 + q.z * FRAC_1_SQRT_2;
        assert!((dot.abs() - 1.0).abs() < EPS && q.x.abs() < EPS && q.y.abs() < EPS, "{:?}", q);
    }

    #[test]
    fn quat_roundtrip_ffi() {
        let m = geoalg_motor_from_rotation(0.3, -0.5, 0.8, 1.1);
        let q = unsafe { geoalg_motor_to_quat(&m) };
        let back = geoalg_pose_to_motor(0.0, 0.0, 0.0, q.w, q.x, q.y, q.z);
        let a = unsafe { geoalg_motor_apply_point(&m, 1.0, 2.0, 3.0) };
        let b = unsafe { geoalg_motor_apply_point(&back, 1.0, 2.0, 3.0) };
        assert!(cerca(b, a.x, a.y, a.z), "{:?} vs {:?}", a, b);
    }

    #[test]
    fn pose_roundtrip_ffi() {
        // Pose con giro y traslacion: debe volver al mismo movimiento.
        let rot = geoalg_motor_from_rotation(0.3, -0.5, 0.8, 1.1);
        let tra = geoalg_motor_from_translation(1.0, 2.0, 3.0);
        let m = unsafe { geoalg_motor_compose(&rot, &tra) };
        let pose = unsafe { geoalg_motor_to_pose(&m) };
        assert!((pose.x - 1.0).abs() < EPS && (pose.y - 2.0).abs() < EPS && (pose.z - 3.0).abs() < EPS, "{:?}", pose);
        let back = geoalg_pose_to_motor(pose.x, pose.y, pose.z, pose.qw, pose.qx, pose.qy, pose.qz);
        for &(x, y, z) in &[(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.5, -2.0, 4.0)] {
            let a = unsafe { geoalg_motor_apply_point(&m, x, y, z) };
            let b = unsafe { geoalg_motor_apply_point(&back, x, y, z) };
            assert!(cerca(b, a.x, a.y, a.z), "{:?} vs {:?}", a, b);
        }
    }

    #[test]
    fn position_coincide_con_pose() {
        let m = geoalg_motor_from_translation(4.0, 5.0, 6.0);
        let p = unsafe { geoalg_motor_to_position(&m) };
        assert!(cerca(p, 4.0, 5.0, 6.0), "{:?}", p);
    }

    #[test]
    fn null_no_truena() {
        // NULL se trata como la identidad.
        let p = unsafe { geoalg_motor_apply_point(ptr::null(), 1.0, 2.0, 3.0) };
        assert!(cerca(p, 1.0, 2.0, 3.0), "{:?}", p);
        let m = unsafe { geoalg_mat4_to_motor(ptr::null()) };
        let p = unsafe { geoalg_motor_apply_point(&m, 1.0, 2.0, 3.0) };
        assert!(cerca(p, 1.0, 2.0, 3.0), "{:?}", p);
    }

    // --- Robots ---

    #[test]
    fn robot_demo_fk_brazo_recto() {
        unsafe {
            let r = geoalg_robot_demo_arm();
            assert_eq!(geoalg_robot_joint_count(r), 3);
            let q = [0.0; 3];
            let mut out = [0.0; 12];
            assert_eq!(geoalg_robot_fk(r, q.as_ptr(), 3, out.as_mut_ptr(), 12), GEOALG_OK);
            // La punta (ultimo punto) queda en (0, 0, 3).
            assert!(out[9].abs() < EPS && out[10].abs() < EPS && (out[11] - 3.0).abs() < EPS);
            geoalg_robot_free(r);
        }
    }

    #[test]
    fn robot_demo_fk_hombro_90() {
        // Mismo resultado exacto que en geoalg-robotics: (0, -2, 1).
        unsafe {
            let r = geoalg_robot_demo_arm();
            let q = [0.0, FRAC_PI_2, 0.0];
            let mut out = [0.0; 12];
            assert_eq!(geoalg_robot_fk(r, q.as_ptr(), 3, out.as_mut_ptr(), 12), GEOALG_OK);
            assert!(out[9].abs() < EPS && (out[10] + 2.0).abs() < EPS && (out[11] - 1.0).abs() < EPS, "{:?}", &out[9..12]);
            geoalg_robot_free(r);
        }
    }

    #[test]
    fn robot_armado_a_mano_igual_que_demo() {
        unsafe {
            let r = geoalg_robot_new();
            assert_eq!(geoalg_robot_add_revolute(r, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, -3.14, 3.14), GEOALG_OK);
            assert_eq!(geoalg_robot_add_revolute(r, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -2.0, 2.0), GEOALG_OK);
            assert_eq!(geoalg_robot_add_revolute(r, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -2.0, 2.0), GEOALG_OK);
            let d = geoalg_robot_demo_arm();
            let q = [0.5, -0.3, 0.8];
            let mut a = [0.0; 12];
            let mut b = [0.0; 12];
            geoalg_robot_fk(r, q.as_ptr(), 3, a.as_mut_ptr(), 12);
            geoalg_robot_fk(d, q.as_ptr(), 3, b.as_mut_ptr(), 12);
            for k in 0..12 {
                assert!((a[k] - b[k]).abs() < 1e-12, "k={k}: {} vs {}", a[k], b[k]);
            }
            geoalg_robot_free(r);
            geoalg_robot_free(d);
        }
    }

    #[test]
    fn robot_ik_llega_y_la_fk_lo_confirma() {
        unsafe {
            let r = geoalg_robot_demo_arm();
            // Objetivo que sabemos alcanzable: la punta con q = (0.3, 0.4, -0.5).
            let q = [0.3, 0.4, -0.5];
            let mut pts = [0.0; 12];
            geoalg_robot_fk(r, q.as_ptr(), 3, pts.as_mut_ptr(), 12);
            let (tx, ty, tz) = (pts[9], pts[10], pts[11]);

            let q0 = [0.1; 3];
            let mut sol = [0.0; 3];
            let mut err = -1.0;
            let res = geoalg_robot_ik(r, tx, ty, tz, q0.as_ptr(), 3, 200, 1e-6, sol.as_mut_ptr(), &mut err);
            assert_eq!(res, 1, "no llego, error = {err}");
            assert!(err >= 0.0 && err < 1e-6);

            let mut check = [0.0; 12];
            geoalg_robot_fk(r, sol.as_ptr(), 3, check.as_mut_ptr(), 12);
            let d = ((check[9] - tx).powi(2) + (check[10] - ty).powi(2) + (check[11] - tz).powi(2)).sqrt();
            assert!(d < 1e-6, "la punta quedo a {d} del objetivo");
            geoalg_robot_free(r);
        }
    }

    #[test]
    fn robot_ik_fuera_de_alcance_devuelve_cero() {
        unsafe {
            let r = geoalg_robot_demo_arm();
            let q0 = [0.1; 3];
            let mut sol = [0.0; 3];
            let res = geoalg_robot_ik(r, 5.0, 0.0, 0.0, q0.as_ptr(), 3, 200, 1e-6, sol.as_mut_ptr(), ptr::null_mut());
            assert_eq!(res, 0);
            assert!(sol.iter().all(|v| v.is_finite()));
            geoalg_robot_free(r);
        }
    }

    #[test]
    fn robot_errores_claros() {
        unsafe {
            let null: *mut GeoRobot = ptr::null_mut();
            assert_eq!(geoalg_robot_joint_count(null), GEOALG_ERR_NULL);
            assert_eq!(geoalg_robot_add_revolute(null, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, -1.0, 1.0), GEOALG_ERR_NULL);
            geoalg_robot_free(null); // no debe tronar

            let r = geoalg_robot_new();
            // Eje (0,0,0): no tiene direccion.
            assert_eq!(geoalg_robot_add_revolute(r, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0), GEOALG_ERR_ARGUMENT);
            // min > max.
            assert_eq!(geoalg_robot_add_prismatic(r, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 1.0), GEOALG_ERR_ARGUMENT);
            assert_eq!(geoalg_robot_joint_count(r), 0);

            // Tamano equivocado en FK.
            let d = geoalg_robot_demo_arm();
            let q = [0.0; 2];
            let mut out = [0.0; 12];
            assert_eq!(geoalg_robot_fk(d, q.as_ptr(), 2, out.as_mut_ptr(), 12), GEOALG_ERR_ARGUMENT);
            // Objetivo NaN en IK.
            let q3 = [0.0; 3];
            let mut sol = [0.0; 3];
            assert_eq!(
                geoalg_robot_ik(d, f64::NAN, 0.0, 0.0, q3.as_ptr(), 3, 50, 1e-6, sol.as_mut_ptr(), ptr::null_mut()),
                GEOALG_ERR_ARGUMENT
            );
            geoalg_robot_free(r);
            geoalg_robot_free(d);
        }
    }
}
