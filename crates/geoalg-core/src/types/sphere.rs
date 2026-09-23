//! Esfera como tipo derivado (centro + radio).
//!
//! En PGA 3D los primitivos de primera clase son punto, recta y plano; la
//! esfera se guarda como par (centro, radio) y las transformaciones se
//! aplican sobre el centro mediante el sandwich de un motor.

use crate::types::point::Point;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    center: Point,
    radius: f64,
}

impl Sphere {
    pub fn new(center: Point, radius: f64) -> Self {
        Sphere { center, radius }
    }

    pub fn center(&self) -> Point {
        self.center
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Distancia con signo a la superficie: < 0 dentro, 0 sobre, > 0 fuera.
    pub fn surface_distance(&self, p: &Point) -> f64 {
        self.center.distance_to(p) - self.radius
    }

    /// ¿Está el punto sobre la superficie (con la tolerancia dada)?
    pub fn contains(&self, p: &Point, eps: f64) -> bool {
        self.surface_distance(p).abs() <= eps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn superficie() {
        let s = Sphere::new(Point::origin(), 2.0);
        assert!(s.contains(&Point::new(2.0, 0.0, 0.0), 1e-9));
        assert!(s.contains(&Point::new(0.0, -2.0, 0.0), 1e-9));
        assert!(!s.contains(&Point::new(2.0, 2.0, 0.0), 1e-9));
        assert!(s.surface_distance(&Point::origin()) < 0.0);
    }
}