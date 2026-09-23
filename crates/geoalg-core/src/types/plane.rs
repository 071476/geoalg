
//! Plano como vector del PGA 3D.
//!
//! El plano `a·x + b·y + c·z + d = 0` se representa como
//! `π = d·e0 + a·e1 + b·e2 + c·e3`. Es homogéneo: `λπ` es el mismo plano
//! (con λ < 0 se invierte la orientación).
//!
//! Incidencia: un punto `P` pertenece al plano si y solo si `π ∧ P = 0`,
//! que en coeficientes equivale a `a·x + b·y + c·z + d = 0`.

use crate::algebra::basis::{E0, E1, E2, E3};
use crate::algebra::product::wedge;
use crate::multivector::dense::Dense;
use crate::ops::dual::dual;
use crate::types::point::Point;

const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane(Dense);

impl Plane {
    /// Plano `a·x + b·y + c·z + d = 0`.
    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        let mut mv = Dense::zero();
        mv.set(E1, a);
        mv.set(E2, b);
        mv.set(E3, c);
        mv.set(E0, d);
        Plane(mv)
    }

    /// Plano con normal (nx, ny, nz) que pasa por el punto (px, py, pz).
    pub fn from_normal_and_point(nx: f64, ny: f64, nz: f64, px: f64, py: f64, pz: f64) -> Self {
        let d = -(nx * px + ny * py + nz * pz);
        Self::new(nx, ny, nz, d)
    }

    /// Plano que pasa por tres puntos (unión de los tres).
    ///
    /// La unión se calcula con duales: `join(P, Q, R) = dual(dual(P) ∧ dual(Q) ∧ dual(R))`.
    /// Se normaliza a normal unitaria y se orienta según la regla de la mano
    /// derecha respecto del orden `p, q, r`.
    pub fn from_three_points(p: &Point, q: &Point, r: &Point) -> Self {
        let mv = dual(&wedge(
            &wedge(&dual(&p.inner()), &dual(&q.inner())),
            &dual(&r.inner()),
        ));
        let mut pl = Plane(mv).normalize();
        if let (Some((px, py, pz)), Some((qx, qy, qz)), Some((rx, ry, rz))) =
            (p.coords(), q.coords(), r.coords())
        {
            let (ux, uy, uz) = (qx - px, qy - py, qz - pz);
            let (vx, vy, vz) = (rx - px, ry - py, rz - pz);
            let (cx, cy, cz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
            if pl.a() * cx + pl.b() * cy + pl.c() * cz < 0.0 {
                pl.0 = pl.0 * -1.0;
            }
        }
        pl
    }

    pub fn a(&self) -> f64 { self.0.get(E1) }
    pub fn b(&self) -> f64 { self.0.get(E2) }
    pub fn c(&self) -> f64 { self.0.get(E3) }
    pub fn d(&self) -> f64 { self.0.get(E0) }

    /// Normal del plano.
    pub fn normal(&self) -> (f64, f64, f64) {
        (self.a(), self.b(), self.c())
    }

    /// Normaliza la normal a norma 1 (conserva la orientación).
    pub fn normalize(&self) -> Self {
        let n = (self.a().powi(2) + self.b().powi(2) + self.c().powi(2)).sqrt();
        if n < EPS {
            *self
        } else {
            Plane(self.0 * (1.0 / n))
        }
    }

    /// Incidencia: `P` pertenece al plano si y solo si `π ∧ P = 0`.
    pub fn contains(&self, p: &Point) -> bool {
        wedge(&self.0, &p.inner()).approx_eq(&Dense::zero(), EPS)
    }

    /// Distancia con signo desde un punto finito.
    pub fn distance_to(&self, p: &Point) -> f64 {
        let (x, y, z) = p.coords().expect("distance_to: el punto es ideal");
        let n = (self.a().powi(2) + self.b().powi(2) + self.c().powi(2)).sqrt();
        (self.a() * x + self.b() * y + self.c() * z + self.d()) / n
    }

    pub fn from_dense(mv: Dense) -> Self {
        Plane(mv)
    }

    pub fn inner(&self) -> Dense {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incidencia() {
        let p = Plane::new(0.0, 0.0, 1.0, 0.0); // plano z = 0
        assert!(p.contains(&Point::new(1.0, 2.0, 0.0)));
        assert!(p.contains(&Point::origin()));
        assert!(!p.contains(&Point::new(0.0, 0.0, 1.0)));
    }

    #[test]
    fn distancia() {
        let p = Plane::new(1.0, 0.0, 0.0, 0.0); // plano x = 0
        assert!((p.distance_to(&Point::new(3.0, 0.0, 0.0)) - 3.0).abs() < EPS);
        assert!((p.distance_to(&Point::new(-2.0, 5.0, 5.0)) + 2.0).abs() < EPS);
    }

    #[test]
    fn plano_por_tres_puntos() {
        let p = Point::new(1.0, 0.0, 0.0);
        let q = Point::new(0.0, 1.0, 0.0);
        let r = Point::new(0.0, 0.0, 1.0);
        let pl = Plane::from_three_points(&p, &q, &r); // x + y + z = 1
        let s = 1.0 / 3.0f64.sqrt();
        assert!((pl.a() - s).abs() < EPS);
        assert!((pl.b() - s).abs() < EPS);
        assert!((pl.c() - s).abs() < EPS);
        assert!((pl.d() + s).abs() < EPS);
        assert!(pl.contains(&p) && pl.contains(&q) && pl.contains(&r));
    }
}