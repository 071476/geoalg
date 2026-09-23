//! Recta como bivector del PGA 3D.
//!
//! Es a la vez el encuentro de dos planos (wedge de dos vectores) y la unión
//! de dos puntos (dual del wedge de los duales). Sus componentes `e23, e13, e12`
//! codifican la dirección y las `e01, e02, e03` el momento respecto del origen
//! (`p × d` para cualquier punto `p` de la recta).

use crate::algebra::basis::{Blade, E0, E1, E2, E3};
use crate::algebra::product::wedge;
use crate::multivector::dense::Dense;
use crate::ops::dual::dual;
use crate::types::plane::Plane;
use crate::types::point::Point;

const E01: Blade = E0 | E1;
const E02: Blade = E0 | E2;
const E03: Blade = E0 | E3;
const E12: Blade = E1 | E2;
const E13: Blade = E1 | E3;
const E23: Blade = E2 | E3;

const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line(Dense);

impl Line {
    /// Recta que une dos puntos (orientada de `p` hacia `q`).
    pub fn through_points(p: &Point, q: &Point) -> Self {
        Line(dual(&wedge(&dual(&p.inner()), &dual(&q.inner()))))
    }

    /// Recta de intersección de dos planos.
    pub fn from_planes(p: &Plane, q: &Plane) -> Self {
        Line(wedge(&p.inner(), &q.inner()))
    }

    /// Vector de dirección de la recta.
    pub fn direction(&self) -> (f64, f64, f64) {
        (self.0.get(E23), -self.0.get(E13), self.0.get(E12))
    }

    /// Momento respecto del origen: `p × d`.
    pub fn moment(&self) -> (f64, f64, f64) {
        (self.0.get(E01), self.0.get(E02), self.0.get(E03))
    }

    /// Incidencia: `P` pertenece a la recta si y solo si
    /// `dual(P) ∧ dual(L) = 0`.
    pub fn contains_point(&self, p: &Point) -> bool {
        wedge(&dual(&p.inner()), &dual(&self.0)).approx_eq(&Dense::zero(), EPS)
    }

    pub fn from_dense(mv: Dense) -> Self {
        Line(mv)
    }

    pub fn inner(&self) -> Dense {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recta_por_dos_puntos() {
        let l = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let (dx, dy, dz) = l.direction();
        assert!((dx - 1.0).abs() < EPS && dy.abs() < EPS && dz.abs() < EPS);
        assert!(l.contains_point(&Point::new(5.0, 0.0, 0.0)));
        assert!(!l.contains_point(&Point::new(5.0, 1.0, 0.0)));
    }

    #[test]
    fn encuentro_de_planos() {
        let py = Plane::new(0.0, 1.0, 0.0, 0.0); // y = 0
        let pz = Plane::new(0.0, 0.0, 1.0, 0.0); // z = 0
        let l = Line::from_planes(&py, &pz); // eje X
        assert!(l.contains_point(&Point::new(2.0, 0.0, 0.0)));
        assert!(!l.contains_point(&Point::new(0.0, 1.0, 0.0)));
    }

    #[test]
    fn momento() {
        // Recta paralela al eje X por (0, 1, 0): momento = p × d = (0, 0, -1).
        let l = Line::through_points(&Point::new(0.0, 1.0, 0.0), &Point::new(1.0, 1.0, 0.0));
        let (mx, my, mz) = l.moment();
        assert!(mx.abs() < EPS && my.abs() < EPS && (mz + 1.0).abs() < EPS);
    }
}
