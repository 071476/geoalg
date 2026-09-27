//! Rotor: rotación propia del espacio (versor par, grados 0 y 2).
//!
//! Un rotor gira puntos, rectas y planos con el sandwich `X ↦ R X R̃`:
//!
//! ```text
//! R = cos(θ/2) - sin(θ/2) · L̂          (L̂ = eje normalizado, L̂² = -1)
//! ```
//!
//! Convención verificada: con `L̂ = e12` (eje z) y θ = π/2, el punto
//! (1, 0, 0) pasa a (0, 1, 0) — giro positivo según la regla de la mano
//! derecha alrededor del eje orientado.

use crate::algebra::metric::Metric;
use crate::multivector::dense::Dense;
use crate::types::line::Line;
use crate::types::motor::Motor;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotor(Dense);

impl Rotor {
    pub fn identity() -> Self {
        Rotor::from_dense(Dense::one())
    }

    /// Rotación de `angle` radianes alrededor de `axis`.
    ///
    /// La recta se normaliza sola; su dirección debe ser no nula (las rectas
    /// ideales generan traslaciones, no rotaciones).
    pub fn from_axis_angle(axis: &Line, angle: f64) -> Self {
        let (dx, dy, dz) = axis.direction();
        let n = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(n > 1e-12, "from_axis_angle: el eje no tiene dirección");
        let l = axis.inner() * (1.0 / n);
        let h = angle * 0.5;
        Rotor(Dense::scalar(h.cos()) + l * (-h.sin()))
    }

    /// Inverso (= reverso, porque R R̃ = 1).
    pub fn inverse(&self) -> Self {
        Rotor(self.0.reverse())
    }

    /// Normaliza a norma 1.
    pub fn normalize(&self) -> Self {
        let n = self.0.norm(&Metric::pga());
        assert!(n > 1e-12, "normalize: rotor nulo");
        Rotor(self.0 * (1.0 / n))
    }

    /// Magnitud del giro en [0, 2π].
    pub fn angle(&self) -> f64 {
        let r = self.normalize();
        2.0 * r.0.scalar_part().clamp(-1.0, 1.0).acos()
    }

    /// Composición: `a.then(b)` aplica primero `a` y luego `b`.
    pub fn then(&self, next: &Rotor) -> Rotor {
        Rotor(next.0.geo(&self.0, &Metric::pga()))
    }

    pub fn to_motor(&self) -> Motor {
        Motor::from_dense(self.0)
    }

    /// Interpolacion esferica entre dos rotors (t en [0, 1]).
    pub fn slerp(a: &Rotor, b: &Rotor, t: f64) -> Rotor {
        let m = crate::ops::screw::between(&a.to_motor(), &b.to_motor(), t);
        Rotor(m.inner())
    }
    
    pub fn inner(&self) -> Dense {
        self.0
    }

    pub fn from_dense(mv: Dense) -> Self {
        Rotor(mv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::sandwich::apply_point;
    use crate::types::point::Point;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-9;

    fn approx(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS && (a.2 - b.2).abs() < EPS
    }

    #[test]
    fn giro_de_90_grados_sobre_z() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, PI / 2.0);
        let p = apply_point(&r.inner(), &Point::new(1.0, 0.0, 0.0));
        assert!(approx(p.coords().unwrap(), (0.0, 1.0, 0.0)));
    }

    #[test]
    fn giro_de_90_grados_sobre_x() {
        let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let r = Rotor::from_axis_angle(&axis, PI / 2.0);
        let p = apply_point(&r.inner(), &Point::new(0.0, 1.0, 0.0));
        assert!(approx(p.coords().unwrap(), (0.0, 0.0, 1.0)));
    }

    #[test]
    fn composicion_y_su_inverso() {
        let eje_z = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let eje_x = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let rz = Rotor::from_axis_angle(&eje_z, 0.7);
        let rx = Rotor::from_axis_angle(&eje_x, -1.3);
        let m = rz.then(&rx); // primero rz, luego rx
        let p = Point::new(1.0, 2.0, 3.0);
        let q = apply_point(&m.inner(), &p);
        let restored = apply_point(&m.inverse().inner(), &q);
        assert!(approx(restored.coords().unwrap(), p.coords().unwrap()));
    }

    #[test]
    fn angulo_del_rotor() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, 1.0);
        assert!((r.angle() - 1.0).abs() < EPS);
    }
}
