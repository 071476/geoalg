//! Firma métrica del álgebra.

use super::basis::DIM;

/// Métrica diagonal: `squares[i] = e_i * e_i`.
///
/// Se pasa explícitamente a los productos para poder cambiar de firma
/// (PGA, euclídea, Minkowski...) sin tocar el resto del código.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Metric {
    pub squares: [f64; DIM],
}

impl Metric {
    /// PGA 3D — R(3,0,1): e1² = e2² = e3² = +1, e0² = 0.
    pub const fn pga() -> Self {
        Metric { squares: [0.0, 1.0, 1.0, 1.0] }
    }

    /// R(4,0): todo positivo.
    pub const fn euclidean() -> Self {
        Metric { squares: [1.0, 1.0, 1.0, 1.0] }
    }

    /// R(3,1): tres positivas y una negativa.
    pub const fn minkowski() -> Self {
        Metric { squares: [1.0, 1.0, 1.0, -1.0] }
    }

    /// e_i * e_i.
    #[inline]
    pub const fn square(&self, i: usize) -> f64 {
        self.squares[i]
    }
}