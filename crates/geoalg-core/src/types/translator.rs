//! Traslador: traslación pura (versor par: escalar + bivectores con e0).
//!
//! ```text
//! T = 1 - ½(dx·e01 + dy·e02 + dz·e03)      X ↦ T X T̃
//! ```
//!
//! Convención verificada: con `T = 1 - ½e01`, el origen (0, 0, 0) va a
//! (1, 0, 0). Cumple T T̃ = 1, así que su inverso es su reverso.

use crate::algebra::basis::{Blade, E0, E1, E2, E3};
use crate::multivector::dense::Dense;
use crate::types::motor::Motor;

const E01: Blade = E0 | E1;
const E02: Blade = E0 | E2;
const E03: Blade = E0 | E3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Translator(Dense);

impl Translator {
    pub fn identity() -> Self {
        Translator(Dense::one())
    }

    /// Traslación por el vector (dx, dy, dz).
    pub fn new(dx: f64, dy: f64, dz: f64) -> Self {
        let mut t = Dense::one();
        t.set(E01, -0.5 * dx);
        t.set(E02, -0.5 * dy);
        t.set(E03, -0.5 * dz);
        Translator(t)
    }

    /// Vector de desplazamiento.
    pub fn displacement(&self) -> (f64, f64, f64) {
        (
            -2.0 * self.0.get(E01),
            -2.0 * self.0.get(E02),
            -2.0 * self.0.get(E03),
        )
    }

    /// Inverso (= reverso, porque T T̃ = 1).
    pub fn inverse(&self) -> Self {
        Translator(self.0.reverse())
    }

    /// Composición: `a.then(b)` aplica primero `a` y luego `b`
    /// (las traslaciones conmutan: basta sumar los desplazamientos).
    pub fn then(&self, next: &Translator) -> Translator {
        let (ax, ay, az) = self.displacement();
        let (bx, by, bz) = next.displacement();
        Translator::new(ax + bx, ay + by, az + bz)
    }

    pub fn to_motor(&self) -> Motor {
        Motor::from_dense(self.0)
    }

        /// Interpolacion lineal entre dos traslaciones (t en [0, 1]).
    pub fn lerp(a: &Translator, b: &Translator, t: f64) -> Translator {
        let (ax, ay, az) = a.displacement();
        let (bx, by, bz) = b.displacement();
        Translator::new(
            ax + (bx - ax) * t,
            ay + (by - ay) * t,
            az + (bz - az) * t,
        )
    }
    
    pub fn inner(&self) -> Dense {
        self.0
    }

    pub fn from_dense(mv: Dense) -> Self {
        Translator(mv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::sandwich::apply_point;
    use crate::types::point::Point;

    const EPS: f64 = 1e-9;

    fn approx(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS && (a.2 - b.2).abs() < EPS
    }

    #[test]
    fn traslada_el_origen() {
        let t = Translator::new(1.0, 2.0, 3.0);
        let p = apply_point(&t.inner(), &Point::origin());
        assert!(approx(p.coords().unwrap(), (1.0, 2.0, 3.0)));
    }

    #[test]
    fn desplazamiento_roundtrip() {
        let t = Translator::new(-4.0, 0.5, 9.0);
        assert!(approx(t.displacement(), (-4.0, 0.5, 9.0)));
    }

    #[test]
    fn composicion_conmutativa() {
        let a = Translator::new(1.0, 0.0, 0.0);
        let b = Translator::new(0.0, 2.0, 0.0);
        let p = apply_point(&a.then(&b).inner(), &Point::origin());
        assert!(approx(p.coords().unwrap(), (1.0, 2.0, 0.0)));
    }

    #[test]
    fn inverso_deshace() {
        let t = Translator::new(3.0, -1.0, 7.0);
        let p = Point::new(1.0, 2.0, 3.0);
        let q = apply_point(&t.inner(), &p);
        let restored = apply_point(&t.inverse().inner(), &q);
        assert!(approx(restored.coords().unwrap(), p.coords().unwrap()));
    }
}