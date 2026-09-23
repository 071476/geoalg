//! Representación dispersa: solo las láminas con coeficiente no nulo.

use std::cmp::Ordering;

use crate::algebra::basis::{grade as blade_grade, Blade};
use crate::algebra::metric::Metric;
use crate::algebra::product;
use crate::multivector::dense::Dense;
use crate::multivector::graded::Graded;

/// Multivector como lista ordenada de `(lámina, coeficiente)` no nulos.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Sparse {
    terms: Vec<(Blade, f64)>,
}

impl Sparse {
    pub fn new() -> Self {
        Sparse { terms: Vec::new() }
    }

    pub fn terms(&self) -> &[(Blade, f64)] {
        &self.terms
    }

    pub fn nnz(&self) -> usize {
        self.terms.len()
    }

    pub fn get(&self, b: Blade) -> f64 {
        match self.terms.binary_search_by_key(&b, |t| t.0) {
            Ok(i) => self.terms[i].1,
            Err(_) => 0.0,
        }
    }

    /// Asigna un coeficiente (elimina la entrada si el valor es 0).
    pub fn set(&mut self, b: Blade, value: f64) {
        match self.terms.binary_search_by_key(&b, |t| t.0) {
            Ok(i) => {
                if value == 0.0 {
                    self.terms.remove(i);
                } else {
                    self.terms[i].1 = value;
                }
            }
            Err(i) => {
                if value != 0.0 {
                    self.terms.insert(i, (b, value));
                }
            }
        }
    }

    pub fn to_dense(&self) -> Dense {
        let mut out = Dense::zero();
        for &(b, c) in &self.terms {
            out.set(b, c);
        }
        out
    }
        /// Construye un multivector disperso desde uno denso (omite ceros).
    pub fn from_dense(d: &Dense) -> Self {
        let mut s = Sparse::new();
        for (i, &c) in d.0.iter().enumerate() {
            if c != 0.0 {
                s.terms.push((i as Blade, c));
            }
        }
        s
    }

    pub fn add(&self, other: &Sparse) -> Self {
        let mut out = Vec::with_capacity(self.terms.len() + other.terms.len());
        let (mut i, mut j) = (0, 0);
        while i < self.terms.len() && j < other.terms.len() {
            match self.terms[i].0.cmp(&other.terms[j].0) {
                Ordering::Less => {
                    out.push(self.terms[i]);
                    i += 1;
                }
                Ordering::Greater => {
                    out.push(other.terms[j]);
                    j += 1;
                }
                Ordering::Equal => {
                    let s = self.terms[i].1 + other.terms[j].1;
                    if s != 0.0 {
                        out.push((self.terms[i].0, s));
                    }
                    i += 1;
                    j += 1;
                }
            }
        }
        out.extend_from_slice(&self.terms[i..]);
        out.extend_from_slice(&other.terms[j..]);
        Sparse { terms: out }
    }

    pub fn sub(&self, other: &Sparse) -> Self {
        self.add(&other.scale(-1.0))
    }

    pub fn scale(&self, s: f64) -> Self {
        Sparse {
            terms: self.terms.iter().map(|&(b, c)| (b, c * s)).collect(),
        }
    }

    pub fn reverse(&self) -> Self {
        Sparse {
            terms: self
                .terms
                .iter()
                .map(|&(b, c)| {
                    let k = blade_grade(b);
                    let sign = if (k * k.saturating_sub(1) / 2) % 2 == 0 { 1.0 } else { -1.0 };
                    (b, sign * c)
                })
                .collect(),
        }
    }

    /// Producto geométrico (acumula en buffer denso y vuelve a compactar).
    pub fn geo(&self, other: &Sparse, m: &Metric) -> Sparse {
        let mut acc = Dense::zero();
        for &(ba, ca) in &self.terms {
            for &(bb, cb) in &other.terms {
                let (r, s) = product::blade_geo(ba, bb, m);
                if s != 0.0 {
                    acc.0[r as usize] += s * ca * cb;
                }
            }
        }
        Sparse::from_dense(&acc)
    }

    pub fn wedge(&self, other: &Sparse) -> Sparse {
        let mut acc = Dense::zero();
        for &(ba, ca) in &self.terms {
            for &(bb, cb) in &other.terms {
                if ba & bb != 0 {
                    continue;
                }
                acc.0[(ba ^ bb) as usize] += product::blade_reorder_sign(ba, bb) * ca * cb;
            }
        }
        Sparse::from_dense(&acc)
    }
}

impl From<Dense> for Sparse {
    fn from(d: Dense) -> Self {
        Sparse::from_dense(&d)
    }
}

impl From<Sparse> for Dense {
    fn from(s: Sparse) -> Self {
        s.to_dense()
    }
}

impl From<Graded> for Sparse {
    fn from(g: Graded) -> Self {
        Sparse::from_dense(&g.to_dense())
    }
}

impl From<Sparse> for Graded {
    fn from(s: Sparse) -> Self {
        Graded::new(s.to_dense())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebra::basis::{E1, E2};

    #[test]
    fn set_y_get() {
        let mut s = Sparse::new();
        s.set(E2, 3.0);
        s.set(E1, 1.0);
        assert_eq!(s.get(E1), 1.0);
        assert_eq!(s.get(E2), 3.0);
        assert_eq!(s.nnz(), 2);
        s.set(E1, 0.0);
        assert_eq!(s.nnz(), 1);
    }

    #[test]
    fn roundtrip_con_denso() {
        let d = Dense::from_blade(E1, 2.0) - Dense::from_blade(E1 | E2, 5.0);
        let s = Sparse::from_dense(&d);
        assert_eq!(s.nnz(), 2);
        assert!(s.to_dense().approx_eq(&d, 1e-12));
    }

    #[test]
    fn geo_disperso() {
        let a = Sparse::from(Dense::from_blade(E1, 1.0));
        let b = Sparse::from(Dense::from_blade(E2, 1.0));
        let ab = a.geo(&b, &Metric::pga());
        assert_eq!(ab.get(E1 | E2), 1.0);
    }
}