// Jacobiano: que articulaciones mover para llegar a un punto.

use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;

// Cinematica directa: 3 articulaciones -> posicion del efector.
pub type FK = fn(&[f64; 3]) -> (f64, f64, f64);

// Jacobiano numerico 3x3: columna i = efecto de mover la articulacion i.
pub fn jacobian3(fk: FK, q: &[f64; 3], h: f64) -> [[f64; 3]; 3] {
    let mut j = [[0.0; 3]; 3];
    for col in 0..3 {
        let mut qp = *q;
        let mut qm = *q;
        qp[col] += h;
        qm[col] -= h;
        let p = fk(&qp);
        let m = fk(&qm);
        j[0][col] = (p.0 - m.0) / (2.0 * h);
        j[1][col] = (p.1 - m.1) / (2.0 * h);
        j[2][col] = (p.2 - m.2) / (2.0 * h);
    }
    j
}

// Resuelve J * dq = err (3x3, eliminacion de Gauss).
pub fn solve3(j: [[f64; 3]; 3], err: [f64; 3]) -> [f64; 3] {
    let mut a = j;
    let mut b = err;
    for col in 0..3 {
        let mut piv = col;
        for r in col + 1..3 {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        b.swap(col, piv);
        let d = a[col][col];
        for r in col + 1..3 {
            let f = a[r][col] / d;
            for c in col..3 {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for r in (0..3).rev() {
        let mut s = b[r];
        for c in r + 1..3 {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    x
}

// Cinematica inversa simple: mueve las articulaciones hasta que el efector
// llega al objetivo (itera un numero fijo de veces).
pub fn reach(fk: FK, q0: &[f64; 3], target: (f64, f64, f64), iters: usize) -> [f64; 3] {
    let mut q = *q0;
    for _ in 0..iters {
        let p = fk(&q);
        let err = [target.0 - p.0, target.1 - p.1, target.2 - p.2];
        let j = jacobian3(fk, &q, 1e-6);
        let dq = solve3(j, err);
        for i in 0..3 {
            q[i] += dq[i];
        }
    }
    q
}

// Ejemplo de cinematica directa: brazo de 3 articulaciones.
// Devuelve el motor del efector para los angulos dados.
pub fn arm_motor(q: &[f64; 3]) -> Motor {
    use geoalg_core::algebra::metric::Metric;
    use geoalg_core::types::line::Line;
    use geoalg_core::types::rotor::Rotor;
    use geoalg_core::types::translator::Translator;

    let az = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let ax = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));

    let m1 = Motor::from_rotor(&Rotor::from_axis_angle(&az, q[0]));
    let t1 = Motor::from_translator(&Translator::new(0.0, 0.0, 1.0));
    let m2 = Motor::from_rotor(&Rotor::from_axis_angle(&ax, q[1]));
    let t2 = Motor::from_translator(&Translator::new(0.0, 0.0, 1.0));
    let m3 = Motor::from_rotor(&Rotor::from_axis_angle(&ax, q[2]));
    let t3 = Motor::from_translator(&Translator::new(0.0, 0.0, 1.0));

    let m = Metric::pga();
    m1.inner()
        .geo(&t1.inner(), &m)
        .geo(&m2.inner(), &m)
        .geo(&t2.inner(), &m)
        .geo(&m3.inner(), &m)
        .geo(&t3.inner(), &m);
    let chain = m1.inner()
        .geo(&t1.inner(), &m)
        .geo(&m2.inner(), &m)
        .geo(&t2.inner(), &m)
        .geo(&m3.inner(), &m)
        .geo(&t3.inner(), &m);
    Motor::from_dense(chain)
}

// Cinematica directa: posicion del efector para los angulos dados.
pub fn arm_position(q: &[f64; 3]) -> (f64, f64, f64) {
    apply_point(&arm_motor(q).inner(), &Point::origin()).coords().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reach_llega_al_objetivo() {
        let target = (0.5, 0.3, 1.5);
        let q = reach(arm_position, &[0.0, 0.2, -0.1], target, 50);
        let p = arm_position(&q);
        let err = (p.0 - target.0).powi(2) + (p.1 - target.1).powi(2) + (p.2 - target.2).powi(2);
        assert!(err.sqrt() < 1e-6, "error: {err}");
    }

    #[test]
    fn reach_estructura_correcta() {
        let q = reach(arm_position, &[0.1, 0.1, 0.1], (0.2, 0.0, 2.0), 80);
        let p = arm_position(&q);
        assert!((p.0 - 0.2).abs() < 1e-4);
        assert!(p.1.abs() < 1e-4);
        assert!((p.2 - 2.0).abs() < 1e-4);
    }
}