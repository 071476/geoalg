// Velocidad de una pose: cuanta rotacion y traslacion por segundo.

use crate::numeric::finite_diff::derivative;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;

// Velocidad angular (rad/s) de un rotor que pasa de a a b en `dt` segundos.
pub fn angular_speed(a: &Motor, b: &Motor, dt: f64) -> f64 {
    let diff = a.inverse().inner().geo(&b.inner(), &geoalg_core::algebra::metric::Metric::pga());
    let m = Motor::from_dense(diff);
    let p = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0));
    let (x, y, _) = p.coords().unwrap();
    let angle = y.atan2(x);
    angle.abs() / dt
}

// Velocidad lineal (unidades/s) del efector de un brazo.
pub fn linear_speed(joints_fn: impl Fn(f64) -> Motor, t: f64, h: f64) -> (f64, f64, f64) {
    let pos = |s: f64| -> (f64, f64, f64) {
        apply_point(&joints_fn(s).inner(), &Point::origin()).coords().unwrap()
    };
    let vx = derivative(|s| pos(s).0, t, h);
    let vy = derivative(|s| pos(s).1, t, h);
    let vz = derivative(|s| pos(s).2, t, h);
    (vx, vy, vz)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_core::types::translator::Translator;
    use geoalg_core::types::line::Line;
    use geoalg_core::types::rotor::Rotor;

    #[test]
    fn velocidad_angular_constante() {
        // Rotor que gira a velocidad constante: angulo = t.
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r0 = Rotor::from_axis_angle(&axis, 0.5).to_motor();
        let r1 = Rotor::from_axis_angle(&axis, 0.5 + 0.01).to_motor();
        let w = angular_speed(&r0, &r1, 0.01);
        assert!((w - 1.0).abs() < 1e-4);
    }

    #[test]
    fn velocidad_lineal_traslacion() {
        // Motor que traslada (t, 0, 0): velocidad = (1, 0, 0).
        let f = |t: f64| Motor::from_translator(&Translator::new(t, 0.0, 0.0));
        let v = linear_speed(f, 0.5, 1e-6);
        assert!((v.0 - 1.0).abs() < 1e-4);
        assert!(v.1.abs() < 1e-4 && v.2.abs() < 1e-4);
    }
}