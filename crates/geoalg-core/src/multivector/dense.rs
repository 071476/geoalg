//! Representación densa de un multivector: 16 coeficientes fijos.

use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::algebra::basis::{grade as blade_grade, Blade, BLADE_NAMES, NBLADES};
use crate::algebra::metric::Metric;
use crate::algebra::product;

/// Multivector con todos los coeficientes almacenados explícitamente,
/// indexados por la máscara de la lámina básica.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dense(pub [f64; NBLADES]);

impl Dense {
    pub const fn zero() -> Self {
        Dense([0.0; NBLADES])
    }

    pub const fn one() -> Self {
        let mut c = [0.0; NBLADES];
        c[0] = 1.0;
        Dense(c)
    }

    pub const fn scalar(s: f64) -> Self {
        let mut c = [0.0; NBLADES];
        c[0] = s;
        Dense(c)
    }

    /// Una lámina básica con el coeficiente dado (p. ej. `from_blade(E1, 3.0)`).
    pub const fn from_blade(b: Blade, value: f64) -> Self {
        let mut c = [0.0; NBLADES];
        c[b as usize] = value;
        Dense(c)
    }

    #[inline]
    pub const fn get(&self, b: Blade) -> f64 {
        self.0[b as usize]
    }

    #[inline]
    pub fn set(&mut self, b: Blade, value: f64) {
        self.0[b as usize] = value;
    }

    #[inline]
    pub const fn scalar_part(&self) -> f64 {
        self.0[0]
    }

    pub fn is_zero(&self) -> bool {
        self.0.iter().all(|&c| c == 0.0)
    }

    /// Proyección sobre el grado `k` (el resto se pone a cero).
    pub fn grade(&self, k: usize) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.0.iter().enumerate() {
            if blade_grade(i as Blade) == k {
                out.0[i] = c;
            }
        }
        out
    }

    /// Grados presentes: el bit `k` está activo si el grado `k` no es nulo.
    pub fn active_grades(&self) -> u8 {
        let mut mask = 0u8;
        for (i, &c) in self.0.iter().enumerate() {
            if c != 0.0 {
                mask |= 1 << blade_grade(i as Blade);
            }
        }
        mask
    }

    /// Reverso (invierte el orden de los factores): (-1)^(k(k-1)/2).
    pub fn reverse(&self) -> Self {
        self.map_grades(|k, c| {
            if (k * k.saturating_sub(1) / 2) % 2 == 0 { c } else { -c }
        })
    }

    /// Involución de grado: (-1)^k.
    pub fn grade_involution(&self) -> Self {
        self.map_grades(|k, c| if k % 2 == 0 { c } else { -c })
    }

    /// Conjugado de Clifford: (-1)^(k(k+1)/2).
    pub fn conjugate(&self) -> Self {
        self.map_grades(|k, c| {
            if (k * (k + 1) / 2) % 2 == 0 { c } else { -c }
        })
    }

    fn map_grades(&self, f: impl Fn(usize, f64) -> f64) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.0.iter().enumerate() {
            out.0[i] = f(blade_grade(i as Blade), c);
        }
        out
    }

    // ---- productos (la métrica se pasa explícita) ----

    pub fn geo(&self, other: &Dense, m: &Metric) -> Dense {
        product::geo(self, other, m)
    }

    pub fn wedge(&self, other: &Dense) -> Dense {
        product::wedge(self, other)
    }

    /// Contracción izquierda `self ⌋ other`.
    pub fn lc(&self, other: &Dense, m: &Metric) -> Dense {
        product::left_contract(self, other, m)
    }

    /// Contracción derecha `self ⌊ other`.
    pub fn rc(&self, other: &Dense, m: &Metric) -> Dense {
        product::right_contract(self, other, m)
    }

    /// Producto escalar ⟨ab⟩₀.
    pub fn dot(&self, other: &Dense, m: &Metric) -> f64 {
        product::scalar_product(self, other, m)
    }

    /// Cuadrado de la norma: ⟨M M̃⟩₀ (0 para elementos nulos degenerados).
    pub fn norm2(&self, m: &Metric) -> f64 {
        self.dot(&self.reverse(), m)
    }

    pub fn norm(&self, m: &Metric) -> f64 {
        self.norm2(m).abs().sqrt()
    }

    /// Igualdad con tolerancia (para comparar flotantes).
    pub fn approx_eq(&self, other: &Dense, eps: f64) -> bool {
        self.0
            .iter()
            .zip(other.0.iter())
            .all(|(a, b)| (a - b).abs() <= eps)
    }
}

impl Add for Dense {
    type Output = Dense;
    fn add(self, o: Dense) -> Dense {
        let mut out = self;
        for (x, &y) in out.0.iter_mut().zip(o.0.iter()) {
            *x += y;
        }
        out
    }
}

impl Sub for Dense {
    type Output = Dense;
    fn sub(self, o: Dense) -> Dense {
        let mut out = self;
        for (x, &y) in out.0.iter_mut().zip(o.0.iter()) {
            *x -= y;
        }
        out
    }
}

impl Neg for Dense {
    type Output = Dense;
    fn neg(self) -> Dense {
        let mut out = self;
        for x in out.0.iter_mut() {
            *x = -*x;
        }
        out
    }
}

impl Mul<f64> for Dense {
    type Output = Dense;
    fn mul(self, s: f64) -> Dense {
        let mut out = self;
        for x in out.0.iter_mut() {
            *x *= s;
        }
        out
    }
}

impl Mul<Dense> for f64 {
    type Output = Dense;
    fn mul(self, mv: Dense) -> Dense {
        mv * self
    }
}

impl Div<f64> for Dense {
    type Output = Dense;
    fn div(self, s: f64) -> Dense {
        self * (1.0 / s)
    }
}

impl AddAssign for Dense {
    fn add_assign(&mut self, o: Dense) {
        *self = *self + o;
    }
}

impl SubAssign for Dense {
    fn sub_assign(&mut self, o: Dense) {
        *self = *self - o;
    }
}

impl MulAssign<f64> for Dense {
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

impl fmt::Display for Dense {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        for (i, &c) in self.0.iter().enumerate() {
            if c == 0.0 {
                continue;
            }
            if !first {
                write!(f, " + ")?;
            }
            first = false;
            if i == 0 {
                write!(f, "{c}")?;
            } else {
                write!(f, "{c}*{}", BLADE_NAMES[i])?;
            }
        }
        if first {
            write!(f, "0")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebra::basis::{E1, E2};

    #[test]
    fn reverse_cambia_signo_en_bivectores() {
        let b = Dense::from_blade(E1 | E2, 2.0);
        assert_eq!(b.reverse(), -b);
        assert_eq!(Dense::scalar(3.0).reverse(), Dense::scalar(3.0));
    }

    #[test]
    fn proyeccion_de_grado() {
        let mv = Dense::from_blade(E1, 1.0) + Dense::from_blade(E1 | E2, 1.0);
        assert!(mv.grade(1).approx_eq(&Dense::from_blade(E1, 1.0), 1e-12));
        assert_eq!(mv.active_grades(), 0b0110);
    }
}