//! Representación por grados: un bloque por grado, omitiendo los nulos.
//!
//! Útil para saltarse cruces de grados vacíos en algebras grandes.

use crate::algebra::basis::{grade as blade_grade, Blade, DIM};
use crate::algebra::metric::Metric;
use crate::algebra::product;
use crate::multivector::dense::Dense;

const N_GRADES: usize = DIM + 1;

/// Multivector separado en bloques por grado.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Graded {
    grades: [Option<Dense>; N_GRADES],
}

impl Graded {
    /// Separa un multivector denso en bloques por grado (descarta los nulos).
    pub fn new(mv: Dense) -> Self {
        let mut grades = [None; N_GRADES];
        for (i, &c) in mv.0.iter().enumerate() {
            if c != 0.0 {
                let k = blade_grade(i as Blade);
                let block = grades[k].get_or_insert(Dense::zero());
                block.0[i] = c;
            }
        }
        Graded { grades }
    }

    pub fn zero() -> Self {
        Graded { grades: [None; N_GRADES] }
    }

    pub fn scalar(s: f64) -> Self {
        Self::new(Dense::scalar(s))
    }

    /// Bloque del grado `k` (solo ese grado), o `None` si es nulo.
    pub fn grade(&self, k: usize) -> Option<Dense> {
        self.grades[k]
    }

    /// Grados activos: bit `k` a 1 si el grado `k` está presente.
    pub fn active_grades(&self) -> u8 {
        let mut mask = 0u8;
        for (k, g) in self.grades.iter().enumerate() {
            if g.is_some() {
                mask |= 1 << k;
            }
        }
        mask
    }

    pub fn to_dense(&self) -> Dense {
        let mut out = Dense::zero();
        for g in self.grades.iter().flatten() {
            out = out + *g;
        }
        out
    }

    fn map_blocks(&self, f: impl Fn(Dense) -> Dense) -> Self {
        let mut grades = [None; N_GRADES];
        for (k, g) in self.grades.iter().enumerate() {
            if let Some(block) = g {
                let mapped = f(*block);
                if !mapped.is_zero() {
                    grades[k] = Some(mapped);
                }
            }
        }
        Graded { grades }
    }

    pub fn reverse(&self) -> Self {
        self.map_blocks(|b| b.reverse())
    }

    pub fn grade_involution(&self) -> Self {
        self.map_blocks(|b| b.grade_involution())
    }

    pub fn conjugate(&self) -> Self {
        self.map_blocks(|b| b.conjugate())
    }

    pub fn add(&self, other: &Graded) -> Self {
        Self::new(self.to_dense() + other.to_dense())
    }

    pub fn sub(&self, other: &Graded) -> Self {
        Self::new(self.to_dense() - other.to_dense())
    }

    pub fn scale(&self, s: f64) -> Self {
        self.map_blocks(|b| b * s)
    }

    /// Producto geométrico cruzando solo los grados activos.
    pub fn geo(&self, other: &Graded, m: &Metric) -> Graded {
        let mut out = Dense::zero();
        for a in self.grades.iter().flatten() {
            for b in other.grades.iter().flatten() {
                out = out + product::geo(a, b, m);
            }
        }
        Graded::new(out)
    }

    pub fn wedge(&self, other: &Graded) -> Graded {
        let mut out = Dense::zero();
        for a in self.grades.iter().flatten() {
            for b in other.grades.iter().flatten() {
                out = out + product::wedge(a, b);
            }
        }
        Graded::new(out)
    }

    pub fn lc(&self, other: &Graded, m: &Metric) -> Graded {
        let mut out = Dense::zero();
        for a in self.grades.iter().flatten() {
            for b in other.grades.iter().flatten() {
                out = out + product::left_contract(a, b, m);
            }
        }
        Graded::new(out)
    }

    pub fn rc(&self, other: &Graded, m: &Metric) -> Graded {
        let mut out = Dense::zero();
        for a in self.grades.iter().flatten() {
            for b in other.grades.iter().flatten() {
                out = out + product::right_contract(a, b, m);
            }
        }
        Graded::new(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebra::basis::{E0, E1, E2};

    #[test]
    fn separa_y_reconstruye() {
        let mv = Dense::from_blade(E1, 1.0) + Dense::from_blade(E0 | E1 | E2, 2.0);
        let g = Graded::new(mv);
        assert_eq!(g.active_grades(), 0b01010); // grados 1 y 3
        assert!(g.to_dense().approx_eq(&mv, 1e-12));
    }

    #[test]
    fn geo_omite_grados_vacios() {
        let a = Graded::new(Dense::from_blade(E1, 1.0));
        let b = Graded::new(Dense::from_blade(E2, 1.0));
        let expected = Graded::new(Dense::from_blade(E1 | E2, 1.0));
        assert_eq!(a.geo(&b, &Metric::pga()), expected);
    }
}