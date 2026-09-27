//! Motor: movimiento rígido propio (rotación + traslación).
//!
//! M = T R, vive en los grados {0, 2, 4} del PGA 3D. Actúa con el sandwich
//! X ↦ M X M̃ y es unitario (M M̃ = 1): su inverso es su reverso.

use crate::algebra::metric::Metric;
use crate::multivector::dense::Dense;
use crate::types::rotor::Rotor;
use crate::types::translator::Translator;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor(Dense);

impl Motor {
    pub fn identity() -> Self {
        Motor(Dense::one())
    }

    pub fn from_rotor(r: &Rotor) -> Self {
        Motor(r.inner())
    }

    pub fn from_translator(t: &Translator) -> Self {
        Motor(t.inner())
    }

    pub fn from_dense(mv: Dense) -> Self {
        Motor(mv)
    }

    pub fn inner(&self) -> Dense {
        self.0
    }

    /// Inverso (= reverso para motores unitarios).
    pub fn inverse(&self) -> Self {
        Motor(self.0.reverse())
    }

    /// Normaliza a M M̃ = 1.
    pub fn normalize(&self) -> Self {
        let m = Metric::pga();
        let n2 = self.0.geo(&self.0.reverse(), &m).scalar_part();
        assert!(n2 > 1e-12, "normalize: motor nulo");
        Motor(self.0 * (1.0 / n2.sqrt()))
    }

    /// Composición: `a.then(b)` aplica primero `a` y luego `b`.
    pub fn then(&self, next: &Motor) -> Motor {
        Motor(next.0.geo(&self.0, &Metric::pga()))
    }

    /// Exponencial de un bivector: crea un motor.
    pub fn exp(l: &Dense) -> Motor {
        crate::ops::screw::exp(l)
    }

    /// Logaritmo: el bivector L tal que exp(L) = self.
    pub fn log(&self) -> Dense {
        crate::ops::screw::log(self)
    }

    /// Motor intermedio entre dos motores (t en [0, 1]).
    pub fn between(a: &Motor, b: &Motor, t: f64) -> Motor {
        crate::ops::screw::between(a, b, t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::sandwich::apply_point;
    use crate::types::line::Line;
    use crate::types::point::Point;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-9;

    fn approx(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS && (a.2 - b.2).abs() < EPS
    }

    #[test]
    fn girar_y_trasladar() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let rot = Motor::from_rotor(&Rotor::from_axis_angle(&axis, PI / 2.0));
        let tra = Motor::from_translator(&Translator::new(1.0, 0.0, 0.0));
        let m = rot.then(&tra);
        // (1,0,0) --gira 90°--> (0,1,0) --traslada--> (1,1,0)
        let p = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0));
        assert!(approx(p.coords().unwrap(), (1.0, 1.0, 0.0)));
    }

    #[test]
    fn inverso_deshace() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 1.0, 0.0));
        let rot = Rotor::from_axis_angle(&axis, 1.1).to_motor();
        let tra = Translator::new(-2.0, 5.0, 0.5).to_motor();
        let m = rot.then(&tra);
        let p = Point::new(1.0, 2.0, 3.0);
        let q = apply_point(&m.inner(), &p);
        let restored = apply_point(&m.inverse().inner(), &q);
        assert!(approx(restored.coords().unwrap(), p.coords().unwrap()));
    }

    #[test]
    fn motor_unitario() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let rot = Rotor::from_axis_angle(&axis, 0.9).to_motor();
        let tra = Translator::new(3.0, -1.0, 2.0).to_motor();
        let m = rot.then(&tra);
        let prod = m.inner().geo(&m.inverse().inner(), &Metric::pga());
        assert!(prod.approx_eq(&Dense::one(), EPS));
    }
}
 