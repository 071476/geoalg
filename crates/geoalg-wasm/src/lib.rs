//! geoalg-wasm: puente entre geoalg y JavaScript.
//!
//! Este crate NO hace matematicas propias: solo traduce. Todo el calculo
//! lo hacen geoalg-core (motores) y geoalg-robotics (FK e IK), que tienen
//! sus propias pruebas. Si algo se arregla alla, aqui se arregla solo.
//!
//! Convencion: las rotaciones siguen la regla de la mano derecha, igual
//! que three.js. Las matrices salen en orden de columnas (column-major),
//! que es el que usa `THREE.Matrix4.fromArray`.

use geoalg_core::{apply_point, Line, Motor, Point, Rotor, Translator};
use geoalg_robotics::{forward, IKSolution, Robot};
use wasm_bindgen::prelude::*;

// ===================== Motor =====================

/// Movimiento rigido (giro + traslacion), hecho con el Motor de geoalg-core.
#[wasm_bindgen]
pub struct WasmMotor {
    inner: Motor,
}

// Aplica un motor a un punto y devuelve sus coordenadas.
fn ap(m: &Motor, x: f64, y: f64, z: f64) -> [f64; 3] {
    match apply_point(&m.inner(), &Point::new(x, y, z)).coords() {
        Some((a, b, c)) => [a, b, c],
        None => [f64::NAN, f64::NAN, f64::NAN],
    }
}

#[wasm_bindgen]
impl WasmMotor {
    /// Motor identidad (no mueve nada).
    #[wasm_bindgen(constructor)]
    pub fn new() -> WasmMotor {
        WasmMotor { inner: Motor::identity() }
    }

    /// Traslacion pura.
    #[wasm_bindgen(js_name = fromTranslation)]
    pub fn from_translation(dx: f64, dy: f64, dz: f64) -> WasmMotor {
        WasmMotor { inner: Motor::from_translator(&Translator::new(dx, dy, dz)) }
    }

    /// Giro de `angle` radianes alrededor del eje (ax, ay, az) que pasa por el origen.
    #[wasm_bindgen(js_name = fromRotation)]
    pub fn from_rotation(ax: f64, ay: f64, az: f64, angle: f64) -> WasmMotor {
        let len = (ax * ax + ay * ay + az * az).sqrt();
        if len < 1e-15 {
            return WasmMotor::new();
        }
        let axis = Line::through_points(&Point::origin(), &Point::new(ax / len, ay / len, az / len));
        WasmMotor { inner: Motor::from_rotor(&Rotor::from_axis_angle(&axis, angle)) }
    }

    /// Composicion: `a.andThen(b)` aplica primero `a` y luego `b`.
    /// (No se llama `then` porque en JavaScript eso confunde a las Promesas.)
    #[wasm_bindgen(js_name = andThen)]
    pub fn and_then(&self, next: &WasmMotor) -> WasmMotor {
        WasmMotor { inner: self.inner.then(&next.inner) }
    }

    /// Movimiento inverso: deshace este motor.
    pub fn inverse(&self) -> WasmMotor {
        WasmMotor { inner: self.inner.inverse() }
    }

    /// Mueve un punto. Devuelve [x, y, z].
    #[wasm_bindgen(js_name = applyPoint)]
    pub fn apply_point(&self, x: f64, y: f64, z: f64) -> Vec<f64> {
        ap(&self.inner, x, y, z).to_vec()
    }

    /// A donde manda el origen: la parte de traslacion. Devuelve [x, y, z].
    #[wasm_bindgen(js_name = toPosition)]
    pub fn to_position(&self) -> Vec<f64> {
        ap(&self.inner, 0.0, 0.0, 0.0).to_vec()
    }

    /// Matriz 4x4 en orden de columnas, lista para `THREE.Matrix4.fromArray`.
    #[wasm_bindgen(js_name = toMat4)]
    pub fn to_mat4(&self) -> Vec<f64> {
        let (o, ex, ey, ez) = self.frame();
        vec![
            ex[0], ex[1], ex[2], 0.0,
            ey[0], ey[1], ey[2], 0.0,
            ez[0], ez[1], ez[2], 0.0,
            o[0], o[1], o[2], 1.0,
        ]
    }

    /// Cuaternion de la rotacion, en el orden de three.js: [x, y, z, w].
    #[wasm_bindgen(js_name = toQuat)]
    pub fn to_quat(&self) -> Vec<f64> {
        let (_, ex, ey, ez) = self.frame();
        // Matriz de rotacion: columnas ex, ey, ez.
        let (m00, m01, m02) = (ex[0], ey[0], ez[0]);
        let (m10, m11, m12) = (ex[1], ey[1], ez[1]);
        let (m20, m21, m22) = (ex[2], ey[2], ez[2]);
        let tr = m00 + m11 + m22;
        let (x, y, z, w) = if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
        } else if m00 > m11 && m00 > m22 {
            let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
            (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
        } else if m11 > m22 {
            let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
            ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
        } else {
            let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
            ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
        };
        vec![x, y, z, w]
    }
}

impl Default for WasmMotor {
    fn default() -> Self {
        Self::new()
    }
}

impl WasmMotor {
    // Origen movido y los tres ejes girados (sin traslacion).
    fn frame(&self) -> ([f64; 3], [f64; 3], [f64; 3], [f64; 3]) {
        let o = ap(&self.inner, 0.0, 0.0, 0.0);
        let sub = |p: [f64; 3]| [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
        let ex = sub(ap(&self.inner, 1.0, 0.0, 0.0));
        let ey = sub(ap(&self.inner, 0.0, 1.0, 0.0));
        let ez = sub(ap(&self.inner, 0.0, 0.0, 1.0));
        (o, ex, ey, ez)
    }
}

// ===================== Robot =====================

/// Un brazo robotico de geoalg-robotics: FK e IK.
#[wasm_bindgen]
pub struct WasmRobot {
    inner: Robot,
}

/// Resultado de la IK, para JavaScript.
#[wasm_bindgen]
pub struct WasmIkResult {
    angles: Vec<f64>,
    error: f64,
    converged: bool,
    iterations: usize,
}

#[wasm_bindgen]
impl WasmIkResult {
    #[wasm_bindgen(getter)]
    pub fn angles(&self) -> Vec<f64> {
        self.angles.clone()
    }
    #[wasm_bindgen(getter)]
    pub fn error(&self) -> f64 {
        self.error
    }
    #[wasm_bindgen(getter)]
    pub fn converged(&self) -> bool {
        self.converged
    }
    #[wasm_bindgen(getter)]
    pub fn iterations(&self) -> usize {
        self.iterations
    }
}

impl From<IKSolution> for WasmIkResult {
    fn from(s: IKSolution) -> Self {
        WasmIkResult { angles: s.angles, error: s.error, converged: s.converged, iterations: s.iterations }
    }
}

#[wasm_bindgen]
impl WasmRobot {
    /// Robot vacio. Agrega articulaciones con addRevolute / addPrismatic.
    #[wasm_bindgen(constructor)]
    pub fn new() -> WasmRobot {
        WasmRobot { inner: Robot::new() }
    }

    /// El brazo de ejemplo: base en Z y dos articulaciones en X, tramos de 1.
    #[wasm_bindgen(js_name = demoArm)]
    pub fn demo_arm() -> WasmRobot {
        WasmRobot { inner: geoalg_robotics::demo_arm() }
    }

    /// Agrega una articulacion que gira sobre el eje (dx, dy, dz).
    /// El eje pasa por la propia articulacion; (ox, oy, oz) es el tramo
    /// hasta la siguiente. Limites en radianes.
    #[wasm_bindgen(js_name = addRevolute)]
    #[allow(clippy::too_many_arguments)]
    pub fn add_revolute(&mut self, dx: f64, dy: f64, dz: f64, ox: f64, oy: f64, oz: f64, lo: f64, hi: f64) {
        self.inner.add_revolute((0.0, 0.0, 0.0), (dx, dy, dz), (ox, oy, oz), (lo, hi));
    }

    /// Agrega una articulacion que desliza sobre el eje (dx, dy, dz).
    #[wasm_bindgen(js_name = addPrismatic)]
    #[allow(clippy::too_many_arguments)]
    pub fn add_prismatic(&mut self, dx: f64, dy: f64, dz: f64, ox: f64, oy: f64, oz: f64, lo: f64, hi: f64) {
        self.inner.add_prismatic((0.0, 0.0, 0.0), (dx, dy, dz), (ox, oy, oz), (lo, hi));
    }

    /// Numero de articulaciones.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Posicion de la punta para los angulos dados. Devuelve [x, y, z].
    /// Si faltan angulos, se toman como 0 (nunca truena).
    #[wasm_bindgen(js_name = tipPosition)]
    pub fn tip_position(&self, angles: &[f64]) -> Vec<f64> {
        let q = self.padded(angles);
        let p = *forward(&self.inner, &q).joints.last().unwrap();
        vec![p.0, p.1, p.2]
    }

    /// Posicion de cada articulacion (base incluida), en una lista plana:
    /// [x0, y0, z0, x1, y1, z1, ...].
    #[wasm_bindgen(js_name = jointPositions)]
    pub fn joint_positions(&self, angles: &[f64]) -> Vec<f64> {
        let q = self.padded(angles);
        forward(&self.inner, &q)
            .joints
            .iter()
            .flat_map(|p| [p.0, p.1, p.2])
            .collect()
    }

    /// IK gratis: angulos para llevar la punta a (x, y, z).
    #[wasm_bindgen(js_name = ikXyz)]
    pub fn ik_xyz(&self, x: f64, y: f64, z: f64, initial: &[f64], max_iter: usize, tol: f64) -> WasmIkResult {
        let q0 = self.padded(initial);
        self.inner.ik_xyz([x, y, z], &q0, max_iter, tol).into()
    }
}

impl Default for WasmRobot {
    fn default() -> Self {
        Self::new()
    }
}

impl WasmRobot {
    // Completa con ceros si JavaScript manda menos angulos de los necesarios.
    fn padded(&self, angles: &[f64]) -> Vec<f64> {
        let mut q = angles.to_vec();
        q.resize(self.inner.len(), 0.0);
        q
    }
}

/// Version de geoalg-wasm.
#[wasm_bindgen]
pub fn version() -> String {
    format!("geoalg-wasm {}", env!("CARGO_PKG_VERSION"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_1_SQRT_2};

    const EPS: f64 = 1e-9;

    fn cerca(a: &[f64], b: &[f64]) -> bool {
        a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() < EPS)
    }

    #[test]
    fn girar_90_en_z() {
        // Antes este caso devolvia (1,0,0): no giraba nada.
        let m = WasmMotor::from_rotation(0.0, 0.0, 1.0, FRAC_PI_2);
        let p = m.apply_point(1.0, 0.0, 0.0);
        assert!(cerca(&p, &[0.0, 1.0, 0.0]), "obtuve {:?}", p);
    }

    #[test]
    fn girar_y_luego_trasladar() {
        // Antes la traslacion ignoraba el giro.
        let rot = WasmMotor::from_rotation(0.0, 0.0, 1.0, FRAC_PI_2);
        let tra = WasmMotor::from_translation(1.0, 0.0, 0.0);
        let p = rot.and_then(&tra).apply_point(1.0, 0.0, 0.0);
        assert!(cerca(&p, &[1.0, 1.0, 0.0]), "obtuve {:?}", p);
    }

    #[test]
    fn dos_giros_seguidos() {
        let r = WasmMotor::from_rotation(0.0, 0.0, 1.0, FRAC_PI_2);
        let p = r.and_then(&r).apply_point(1.0, 0.0, 0.0);
        assert!(cerca(&p, &[-1.0, 0.0, 0.0]), "obtuve {:?}", p);
    }

    #[test]
    fn inverso_deshace() {
        let m = WasmMotor::from_rotation(0.3, -0.5, 0.8, 1.1).and_then(&WasmMotor::from_translation(2.0, -1.0, 0.5));
        let q = m.apply_point(1.0, 2.0, 3.0);
        let back = m.inverse().apply_point(q[0], q[1], q[2]);
        assert!(cerca(&back, &[1.0, 2.0, 3.0]), "obtuve {:?}", back);
    }

    #[test]
    fn matriz_en_orden_de_columnas() {
        // Giro 90 en Z y luego traslado (5,6,7).
        let m = WasmMotor::from_rotation(0.0, 0.0, 1.0, FRAC_PI_2)
            .and_then(&WasmMotor::from_translation(5.0, 6.0, 7.0));
        let a = m.to_mat4();
        assert!(cerca(&a[0..3], &[0.0, 1.0, 0.0]), "columna X: {:?}", &a[0..3]);
        assert!(cerca(&a[4..7], &[-1.0, 0.0, 0.0]), "columna Y: {:?}", &a[4..7]);
        assert!(cerca(&a[8..11], &[0.0, 0.0, 1.0]), "columna Z: {:?}", &a[8..11]);
        assert!(cerca(&a[12..15], &[5.0, 6.0, 7.0]), "traslacion: {:?}", &a[12..15]);
    }

    #[test]
    fn cuaternion_de_giro_en_z() {
        let q = WasmMotor::from_rotation(0.0, 0.0, 1.0, FRAC_PI_2).to_quat();
        assert!(cerca(&q, &[0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2]), "obtuve {:?}", q);
    }

    #[test]
    fn robot_demo_fk_exacta() {
        let r = WasmRobot::demo_arm();
        let p = r.tip_position(&[0.0, FRAC_PI_2, 0.0]);
        assert!(cerca(&p, &[0.0, -2.0, 1.0]), "obtuve {:?}", p);
    }

    #[test]
    fn faltan_angulos_no_truena() {
        let r = WasmRobot::demo_arm();
        let p = r.tip_position(&[]);
        assert!(cerca(&p, &[0.0, 0.0, 3.0]), "obtuve {:?}", p);
        assert_eq!(r.joint_positions(&[]).len(), 12);
    }

    #[test]
    fn robot_ik_llega() {
        let r = WasmRobot::demo_arm();
        let t = r.tip_position(&[0.3, 0.4, -0.5]);
        let s = r.ik_xyz(t[0], t[1], t[2], &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(s.converged(), "error: {}", s.error());
        let p = r.tip_position(&s.angles());
        assert!(p.iter().zip(&t).all(|(a, b)| (a - b).abs() < 1e-6), "punta {:?}, objetivo {:?}", p, t);
    }

    #[test]
    fn robot_armado_a_mano() {
        // Brazo con eje Y hacia arriba, como en three.js:
        // base gira en Y; hombro y codo giran en X; tramos hacia +Y.
        let mut r = WasmRobot::new();
        r.add_revolute(0.0, 1.0, 0.0, 0.0, 0.325, 0.0, -3.14, 3.14);
        r.add_revolute(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -2.6, 2.6);
        r.add_revolute(1.0, 0.0, 0.0, 0.0, 0.8, 0.0, -2.8, 2.8);
        assert_eq!(r.len(), 3);
        let p = r.tip_position(&[0.0, 0.0, 0.0]);
        assert!(cerca(&p, &[0.0, 2.125, 0.0]), "obtuve {:?}", p);
        // Hombro a 90: los dos tramos (1.8) giran de +Y hacia +Z.
        let p = r.tip_position(&[0.0, FRAC_PI_2, 0.0]);
        assert!(cerca(&p, &[0.0, 0.325, 1.8]), "obtuve {:?}", p);
    }
}
