//! Dual (complementariedad) de láminas y multivectores.
//!
//! En dimensión 4 el dual de una lámina de grado `k` tiene grado `4 - k`:
//! vectores (planos) ↔ trivectores (puntos), y los bivectores (rectas) entre
//! sí. Se define por complemento de máscaras, con el signo tal que
//! `b ∧ (s · b*) = I`.
//!
//! El dual intercambia «encuentro» (wedge) con «unión» de elementos:
//! por eso `types::line` lo usa para comprobar incidencias
//! (punto ∈ recta  ⟺  dual(punto) ∧ dual(recta) = 0).

use crate::algebra::basis::{grade, Blade, I};
use crate::algebra::product::blade_reorder_sign;
use crate::multivector::dense::Dense;

/// Dual de una lámina: devuelve `(b*, s)` tales que `b ∧ (s·b*) = I`.
pub fn blade_dual(b: Blade) -> (Blade, f64) {
    let comp = (!b) & I;
    (comp, blade_reorder_sign(b, comp))
}

/// Dual de un multivector completo.
pub fn dual(mv: &Dense) -> Dense {
    let mut out = Dense::zero();
    for (i, &c) in mv.0.iter().enumerate() {
        if c != 0.0 {
            let (j, s) = blade_dual(i as Blade);
            out.0[j as usize] += s * c;
        }
    }
    out
}

/// Anti-dual: inversa del dual (`undual(dual(M)) == M`).
pub fn undual(mv: &Dense) -> Dense {
    let mut out = dual(mv);
    for (i, c) in out.0.iter_mut().enumerate() {
        // dual ∘ dual = (-1)^(k(4-k)) = (-1)^k sobre cada lámina.
        if grade(i as Blade) % 2 == 1 {
            *c = -*c;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebra::basis::{E0, E1, E2, E3};
    use crate::algebra::product::wedge;

    #[test]
    fn dual_de_laminas() {
        // e23 ∧ e23* debe dar el pseudoscalar I.
        let (j, s) = blade_dual(E2 | E3);
        let d = Dense::from_blade(j, s);
        let prod = wedge(&Dense::from_blade(E2 | E3, 1.0), &d);
        assert!(prod.approx_eq(&Dense::from_blade(I, 1.0), 1e-12));
    }

    #[test]
    fn undual_es_inversa() {
        let mv = Dense::from_blade(E1 | E2 | E3, 2.0) + Dense::from_blade(E0, -3.0) + Dense::one();
        assert!(undual(&dual(&mv)).approx_eq(&mv, 1e-12));
    }
}