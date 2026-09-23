//! Punto de R³ como trivector del PGA 3D.
//!
//! Convención (PGA basado en planos, firma R(3,0,1)):
//!
//! ```text
//! P(x, y, z) = e123 - x·e023 + y·e013 - z·e012
//! ```
//!
//! que coincide con el encuentro de los tres planos `X = x`, `Y = y`, `Z = z`.
//! Los puntos son homogéneos: `λP` representa el mismo punto. El peso
//! `w = coef(e123)` vale 1 en puntos finitos y 0 en los ideales (direcciones).

use crate::algebra::basis::{Blade, E0, E1, E2, E3};
use crate::multivector::dense::Dense;

const E012: Blade = E0 | E1 | E2;
const E013: Blade = E0 | E1 | E3;
const E023: Blade = E0 | E2 | E3;
const E123: Blade = E1 | E2 | E3;

const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point(Dense);

impl Point {
    /// Punto finito (x, y, z).
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        let mut mv = Dense::from_blade(E123, 1.0);
        mv.set(E023, -x);
        mv.set(E013, y);
        mv.set(E012, -z);
        Point(mv)
    }

    /// Punto ideal (dirección) dado por un vector de desplazamiento.
    pub fn ideal(dx: f64, dy: f64, dz: f64) -> Self {
        let mut mv = Dense::zero();
        mv.set(E023, -dx);
        mv.set(E013, dy);
        mv.set(E012, -dz);
        Point(mv)
    }

    /// El origen (0, 0, 0).
    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Peso homogéneo (coeficiente de e123). Vale 0 en puntos ideales.
    pub fn w(&self) -> f64 {
        self.0.get(E123)
    }

    pub fn is_ideal(&self) -> bool {
        self.w().abs() < EPS
    }

    /// Coordenadas euclídeas (None si el punto es ideal).
    pub fn coords(&self) -> Option<(f64, f64, f64)> {
        let w = self.w();
        if w.abs() < EPS {
            None
        } else {
            Some((
                -self.0.get(E023) / w,
                self.0.get(E013) / w,
                -self.0.get(E012) / w,
            ))
        }
    }

    /// Vector de desplazamiento de un punto ideal.
    pub fn direction(&self) -> (f64, f64, f64) {
        (-self.0.get(E023), self.0.get(E013), -self.0.get(E012))
    }

    /// Distancia euclídea entre dos puntos finitos.
    pub fn distance_to(&self, other: &Point) -> f64 {
        let (ax, ay, az) = self.coords().expect("distance_to: punto ideal sin posición");
        let (bx, by, bz) = other.coords().expect("distance_to: punto ideal sin posición");
        ((ax - bx).powi(2) + (ay - by).powi(2) + (az - bz).powi(2)).sqrt()
    }

    pub fn from_dense(mv: Dense) -> Self {
        Point(mv)
    }

    pub fn inner(&self) -> Dense {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordenadas_y_homogeneidad() {
        let p = Point::new(1.0, -2.0, 3.0);
        assert_eq!(p.coords(), Some((1.0, -2.0, 3.0)));
        assert_eq!(p.w(), 1.0);
        let q = Point::from_dense(p.inner() * 7.0);
        assert_eq!(q.coords(), p.coords());
    }

    #[test]
    fn punto_ideal() {
        let d = Point::ideal(0.0, 0.0, 5.0);
        assert!(d.is_ideal());
        assert_eq!(d.coords(), None);
        assert_eq!(d.direction(), (0.0, 0.0, 5.0));
    }

    #[test]
    fn distancia() {
        let d = Point::origin().distance_to(&Point::new(3.0, 4.0, 0.0));
        assert!((d - 5.0).abs() < EPS);
    }
}