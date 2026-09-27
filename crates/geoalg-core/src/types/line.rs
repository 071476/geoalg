//! Recta como bivector del PGA 3D.
//!
//! Es a la vez el encuentro de dos planos (wedge de dos vectores) y la uniÃ³n
//! de dos puntos (dual del wedge de los duales). Sus componentes `e23, e13, e12`
//! codifican la direcciÃ³n y las `e01, e02, e03` el momento respecto del origen
//! (`p Ã— d` para cualquier punto `p` de la recta).

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

    /// Recta de intersecciÃ³n de dos planos.
    pub fn from_planes(p: &Plane, q: &Plane) -> Self {
        Line(wedge(&p.inner(), &q.inner()))
    }

    /// Vector de direcciÃ³n de la recta.
    pub fn direction(&self) -> (f64, f64, f64) {
        (self.0.get(E23), -self.0.get(E13), self.0.get(E12))
    }

    /// Momento respecto del origen: `p Ã— d`.
    pub fn moment(&self) -> (f64, f64, f64) {
        (self.0.get(E01), self.0.get(E02), self.0.get(E03))
    }

    /// Incidencia: `P` pertenece a la recta si y solo si
    /// `dual(P) âˆ§ dual(L) = 0`.
    pub fn contains_point(&self, p: &Point) -> bool {
        wedge(&dual(&p.inner()), &dual(&self.0)).approx_eq(&Dense::zero(), EPS)
    }

    /// Distancia de un punto finito a la recta (cualquier recta).
    pub fn distance_to(&self, p: &Point) -> f64 {
        let (dx, dy, dz) = self.direction();
        let n = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(n > 1e-12, "distance_to: la recta no tiene direccion");
        let dir = (dx / n, dy / n, dz / n);
        let (px, py, pz) = p.coords().expect("distance_to: punto ideal");

        // Punto de la recta mas cercano al origen: c = d x m
        let (mx, my, mz) = self.moment();
        let cx = dir.1 * mz - dir.2 * my;
        let cy = dir.2 * mx - dir.0 * mz;
        let cz = dir.0 * my - dir.1 * mx;

        // Distancia = |(p - c) x dir|
        let vx = px - cx;
        let vy = py - cy;
        let vz = pz - cz;
        let cross_x = vy * dir.2 - vz * dir.1;
        let cross_y = vz * dir.0 - vx * dir.2;
        let cross_z = vx * dir.1 - vy * dir.0;
        (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z).sqrt()
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
        // Recta paralela al eje X por (0, 1, 0): momento = p Ã— d = (0, 0, -1).
        let l = Line::through_points(&Point::new(0.0, 1.0, 0.0), &Point::new(1.0, 1.0, 0.0));
        let (mx, my, mz) = l.moment();
        assert!(mx.abs() < EPS && my.abs() < EPS && (mz + 1.0).abs() < EPS);
    }

    #[test]
    fn distancia_punto_recta() {
        // Recta: eje X. Punto (0, 3, 0): distancia = 3.
        let l = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let d = l.distance_to(&Point::new(5.0, 3.0, 0.0));
        assert!((d - 3.0).abs() < EPS, "esperaba 3, obtuve {d}");

        // Recta paralela al eje X por (0, 1, 0). Punto (0, 4, 0): distancia = 3.
        let l2 = Line::through_points(&Point::new(0.0, 1.0, 0.0), &Point::new(1.0, 1.0, 0.0));
        let d2 = l2.distance_to(&Point::new(7.0, 4.0, 0.0));
        assert!((d2 - 3.0).abs() < EPS, "esperaba 3, obtuve {d2}");
    }
}
