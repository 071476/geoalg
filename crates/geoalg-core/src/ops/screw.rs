// Exponencial y logaritmo de motores (interpolacion de poses).
// Un motor unitario M se escribe M = exp(L), con L un bivector de 6
// numeros: rotacion (e12, e13, e23) y traslacion (e01, e02, e03).

use crate::algebra::basis::{Blade, E0, E1, E2, E3};
use crate::algebra::metric::Metric;
use crate::multivector::dense::Dense;
use crate::types::motor::Motor;

const B12: Blade = E1 | E2;
const B13: Blade = E1 | E3;
const B23: Blade = E2 | E3;
const B01: Blade = E0 | E1;
const B02: Blade = E0 | E2;
const B03: Blade = E0 | E3;
const BLADES: [Blade; 6] = [B12, B13, B23, B01, B02, B03];

// Exponencial de un bivector: devuelve el motor exp(L).
// Serie de Taylor con escalado: exp(L) = (exp(L/16))^16.
pub fn exp(l: &Dense) -> Motor {
    let m = Metric::pga();
    let x = *l * (1.0 / 16.0);
    let mut term = Dense::one();
    let mut sum = Dense::one();
    for n in 1..=10usize {
        term = term.geo(&x, &m) * (1.0 / n as f64);
        sum = sum + term;
    }
    for _ in 0..4 {
        sum = sum.geo(&sum, &m);
    }
    Motor::from_dense(sum)
}

// Resuelve un sistema 6x6 (eliminacion de Gauss con pivote).
fn solve6(a: [[f64; 6]; 6], b: [f64; 6]) -> [f64; 6] {
    let mut a = a;
    let mut b = b;
    for col in 0..6 {
        let mut piv = col;
        for r in col + 1..6 {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        b.swap(col, piv);
        let d = a[col][col];
        for r in col + 1..6 {
            let f = a[r][col] / d;
            for c in col..6 {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 6];
    for r in (0..6).rev() {
        let mut s = b[r];
        for c in r + 1..6 {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    x
}

// Logaritmo: devuelve el bivector L tal que exp(L) = m.
// Metodo de Newton con jacobiano numerico.
pub fn log(m: &Motor) -> Dense {
    let target = m.inner();
    let mut l = Dense::zero();
    for &b in &BLADES {
        l.set(b, target.get(b));
    }
    let h = 1e-6;
    for _ in 0..60 {
        let err = exp(&l).inner() - target;
        let mut j = [[0.0; 6]; 6];
        for (col, &b) in BLADES.iter().enumerate() {
            let mut lp = l;
            lp.set(b, l.get(b) + h);
            let mut lm = l;
            lm.set(b, l.get(b) - h);
            let dp = exp(&lp).inner();
            let dm = exp(&lm).inner();
            for (row, &br) in BLADES.iter().enumerate() {
                j[row][col] = (dp.get(br) - dm.get(br)) / (2.0 * h);
            }
        }
        let mut rhs = [0.0; 6];
        for (row, &br) in BLADES.iter().enumerate() {
            rhs[row] = -err.get(br);
        }
        let d = solve6(j, rhs);
        let mut step2 = 0.0;
        for (i, &b) in BLADES.iter().enumerate() {
            l.set(b, l.get(b) + d[i]);
            step2 += d[i] * d[i];
        }
        if step2 < 1e-18 {
            break;
        }
    }
    l
}

// Movimiento suave entre dos motores (screw interpolation).
// t = 0 devuelve a, t = 1 devuelve b.
pub fn between(a: &Motor, b: &Motor, t: f64) -> Motor {
    let m = Metric::pga();
    let rel = a.inverse().inner().geo(&b.inner(), &m);
    let l = log(&Motor::from_dense(rel)) * t;
    Motor::from_dense(a.inner().geo(&exp(&l).inner(), &m))
}

// Extrae el eje y paso del tornillo de un motor: (eje, paso, angulo).
pub fn screw_axis(m: &Motor) -> ((f64, f64, f64), f64, f64) {
    let l = log(m);
    let rx = l.get(B23);
    let ry = -l.get(B13);
    let rz = l.get(B12);
    let angle = (rx * rx + ry * ry + rz * rz).sqrt();
    if angle < 1e-12 {
        return ((0.0, 0.0, 1.0), 0.0, 0.0);
    }
    let axis = (rx / angle, ry / angle, rz / angle);
    // Paso: componente de traslacion a lo largo del eje / angulo.
    let tx = l.get(B01);
    let ty = l.get(B02);
    let tz = l.get(B03);
    let pitch = (tx * axis.0 + ty * axis.1 + tz * axis.2) / angle;
    (axis, pitch, angle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::sandwich::apply_point;
    use crate::types::line::Line;
    use crate::types::point::Point;
    use crate::types::rotor::Rotor;
    use crate::types::translator::Translator;

    fn close3(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6 && (a.2 - b.2).abs() < 1e-6
    }

    #[test]
    fn exp_y_log_roundtrip() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, 1.0).to_motor();
        let t = Translator::new(1.0, -2.0, 0.5).to_motor();
        let m = r.then(&t);
        let back = exp(&log(&m));
        assert!(back.inner().approx_eq(&m.inner(), 1e-6));
    }

    #[test]
    fn between_extremos() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 1.0, 0.0));
        let a = Rotor::from_axis_angle(&axis, 0.3).to_motor();
        let b = Rotor::from_axis_angle(&axis, 1.5).to_motor();
        assert!(between(&a, &b, 0.0).inner().approx_eq(&a.inner(), 1e-6));
        assert!(between(&a, &b, 1.0).inner().approx_eq(&b.inner(), 1e-6));
    }

    #[test]
    fn interpola_rotacion() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let a = Motor::identity();
        let b = Rotor::from_axis_angle(&axis, 1.2).to_motor();
        let mid = between(&a, &b, 0.5);
        let expect = Rotor::from_axis_angle(&axis, 0.6).to_motor();
        assert!(close3(
            apply_point(&mid.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap(),
            apply_point(&expect.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap()
        ));
    }
}