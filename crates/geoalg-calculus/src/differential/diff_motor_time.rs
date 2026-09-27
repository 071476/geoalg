// diff_motor_time: velocidad de un motor en un instante (twist velocity).
// Devuelve un bivector de 6 numeros: rotacion (e12, e13, e23) y
// traslacion (e01, e02, e03). Es la receta de "cuanto giro y cuanto me
// desplazo por segundo" — la velocidad tuerca del movimiento tornillo.

use geoalg_core::algebra::basis::{Blade, E0, E1, E2, E3};
use geoalg_core::algebra::metric::Metric;
use geoalg_core::multivector::dense::Dense;
use geoalg_core::ops::screw::{exp, log};
use geoalg_core::types::motor::Motor;

const B12: Blade = E1 | E2;
const B13: Blade = E1 | E3;
const B23: Blade = E2 | E3;
const B01: Blade = E0 | E1;
const B02: Blade = E0 | E2;
const B03: Blade = E0 | E3;

// Velocidad de un motor: bivector L tal que exp(L * dt) * M(t) = M(t + dt).
pub fn diff_motor_time(m: impl Fn(f64) -> Motor, t: f64, h: f64) -> Dense {
    let m0 = m(t);
    let m1 = m(t + h);
    let rel = m0.inverse().inner().geo(&m1.inner(), &Metric::pga());
    log(&Motor::from_dense(rel)) * (1.0 / h)
}

// Los 6 numeros del bivector: [e12, e13, e23, e01, e02, e03].
pub fn twist_coefficients(l: &Dense) -> [f64; 6] {
    [l.get(B12), l.get(B13), l.get(B23), l.get(B01), l.get(B02), l.get(B03)]
}

// Reconstruye un motor a partir de un bivector de velocidad y dt.
pub fn step(motor: &Motor, vel: &Dense, dt: f64) -> Motor {
    let metric = Metric::pga();
    Motor::from_dense(motor.inner().geo(&exp(&(*vel * dt)).inner(), &metric))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_core::types::line::Line;
    use geoalg_core::types::point::Point;
    use geoalg_core::types::rotor::Rotor;
    use geoalg_core::types::translator::Translator;

    #[test]
    fn traslacion_velocidad_constante() {
        // M(t) = traslacion (t, 0, 0): velocidad constante en e01.
        let f = |t: f64| Motor::from_translator(&Translator::new(t, 0.0, 0.0));
        let v0 = diff_motor_time(&f, 0.5, 1e-6);
        let v1 = diff_motor_time(&f, 1.5, 1e-6);
        assert!(v0.approx_eq(&v1, 1e-4), "velocidad deberia ser constante");
    }

    #[test]
    fn rotacion_velocidad_constante() {
        // M(t) = rotor de angulo t sobre z: velocidad constante en e12.
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let f = |t: f64| Rotor::from_axis_angle(&axis, t).to_motor();
        let v0 = diff_motor_time(&f, 0.3, 1e-6);
        let v1 = diff_motor_time(&f, 1.7, 1e-6);
        assert!(v0.approx_eq(&v1, 1e-4), "velocidad deberia ser constante");
    }

    #[test]
    fn step_reconstruye_movimiento() {
        // Movimiento tornillo con velocidad constante: exp(t * L).
        let mut l = Dense::zero();
        l.set(E1 | E2, -0.5);  // rotacion constante
        l.set(E0 | E3, -0.3);  // traslacion constante
        let f = move |t: f64| exp(&(l * t));
        let dt = 0.01;
        let m0 = f(0.5);
        let v = diff_motor_time(&f, 0.5, 1e-6);
        let m1 = step(&m0, &v, dt);
        let expect = f(0.5 + dt);
        assert!(m1.inner().approx_eq(&expect.inner(), 1e-3), "step no coincide");
    }
    
}