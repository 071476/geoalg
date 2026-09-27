//! Productos entre láminas básicas y entre multivectores densos.

use super::basis::{Blade, NBLADES};
use super::metric::Metric;
use crate::multivector::dense::Dense;

/// Signo que aparece al reordenar la concatenación `a · b` a orden canónico.
/// (Es la única fuente de signos del producto exterior.)
pub fn blade_reorder_sign(a: Blade, b: Blade) -> f64 {
    let mut sign = 1.0;
    let mut rest = b;
    while rest != 0 {
        let i = rest.trailing_zeros();
        // Vectores de `a` que van después de `e_i` en el orden canónico.
        if (((a as u32) >> (i + 1)).count_ones() & 1) == 1 {
            sign = -sign;
        }
        rest &= rest - 1;
    }
    sign
}

/// Producto geométrico de dos láminas básicas.
///
/// Devuelve `(lámina_resultado, factor)`. Si el factor es `0.0` el término se
/// anula (p. ej. `e0 * e0 = 0` en métrica degenerada).
pub fn blade_geo(a: Blade, b: Blade, m: &Metric) -> (Blade, f64) {
    debug_assert!((a as usize) < NBLADES && (b as usize) < NBLADES);
    let mut sign = blade_reorder_sign(a, b);
    // Vectores comunes a `a` y `b`: se aniquilan y aportan e_i * e_i.
    let mut common = a & b;
    while common != 0 {
        let i = common.trailing_zeros();
        sign *= m.square(i as usize);
        common &= common - 1;
    }
    (a ^ b, sign)
}

fn combine(a: &Dense, b: &Dense, m: &Metric, keep: fn(Blade, Blade) -> bool) -> Dense {
    let mut out = Dense::zero();
    for (ba, &ca) in a.0.iter().enumerate() {
        if ca == 0.0 {
            continue;
        }
        for (bb, &cb) in b.0.iter().enumerate() {
            if cb == 0.0 {
                continue;
            }
            let (ba, bb) = (ba as Blade, bb as Blade);
            if !keep(ba, bb) {
                continue;
            }
            let (r, s) = blade_geo(ba, bb, m);
            if s != 0.0 {
                out.0[r as usize] += s * ca * cb;
            }
        }
    }
    out
}

/// Producto geométrico.
pub fn geo(a: &Dense, b: &Dense, m: &Metric) -> Dense {
    combine(a, b, m, |_, _| true)
}

/// Contracción izquierda `a ⌋ b` (grado resultado: grade(b) - grade(a)).
pub fn left_contract(a: &Dense, b: &Dense, m: &Metric) -> Dense {
    combine(a, b, m, |a, b| a & b == a)
}

/// Contracción derecha `a ⌊ b` (grado resultado: grade(a) - grade(b)).
pub fn right_contract(a: &Dense, b: &Dense, m: &Metric) -> Dense {
    combine(a, b, m, |a, b| a & b == b)
}

/// Producto exterior (independiente de la métrica).
pub fn wedge(a: &Dense, b: &Dense) -> Dense {
    let mut out = Dense::zero();
    for (ba, &ca) in a.0.iter().enumerate() {
        if ca == 0.0 {
            continue;
        }
        for (bb, &cb) in b.0.iter().enumerate() {
            if cb == 0.0 {
                continue;
            }
            let (ba, bb) = (ba as Blade, bb as Blade);
            if ba & bb != 0 {
                continue; // solo se combinan láminas disjuntas
            }
            out.0[(ba ^ bb) as usize] += blade_reorder_sign(ba, bb) * ca * cb;
        }
    }
    out
}

/// Producto escalar: parte escalar del producto geométrico, ⟨ab⟩₀.
pub fn scalar_product(a: &Dense, b: &Dense, m: &Metric) -> f64 {
    let mut s = 0.0;
    for i in 0..NBLADES {
        if a.0[i] == 0.0 || b.0[i] == 0.0 {
            continue;
        }
        let (_, f) = blade_geo(i as Blade, i as Blade, m);
        s += f * a.0[i] * b.0[i];
    }
    s
}

/// Conmutador: (ab - ba) / 2. Mide cuánto "no conmutan" dos elementos.
pub fn commutator(a: &Dense, b: &Dense, m: &Metric) -> Dense {
    (geo(a, b, m) - geo(b, a, m)) * 0.5
}

/// Anticonmutador: (ab + ba) / 2.
pub fn anticommutator(a: &Dense, b: &Dense, m: &Metric) -> Dense {
    (geo(a, b, m) + geo(b, a, m)) * 0.5
}
