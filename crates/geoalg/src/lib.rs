pub mod algebras;
pub mod prelude;

pub use geoalg_core as core;
pub use geoalg_core::compat;

#[cfg(test)]
mod tests {
    use crate::prelude::*;
    use std::f64::consts::PI;

    fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 && (a.2 - b.2).abs() < 1e-9
    }

    #[test]
    fn prelude_gira_puntos() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, PI / 2.0);
        let p = apply_point(&r.inner(), &Point::new(1.0, 0.0, 0.0));
        assert!(close(p.coords().unwrap(), (0.0, 1.0, 0.0)));
    }

    #[test]
    fn prelude_mueve_puntos() {
        let t = Translator::new(1.0, 2.0, 3.0);
        let p = apply_point(&t.inner(), &Point::origin());
        assert!(close(p.coords().unwrap(), (1.0, 2.0, 3.0)));
    }

    #[test]
    fn prelude_compat_cuaternion() {
        let q = compat::rotor_to_quaternion(&Rotor::identity());
        assert!((q.0 - 1.0).abs() < 1e-9 && q.1.abs() < 1e-9);
    }
}