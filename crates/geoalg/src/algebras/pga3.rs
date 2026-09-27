// PGA 3D — R(3,0,1): el álgebra principal de geoalg 1.0.
// Puntos, rectas, planos, rotors, translators y motors.

pub use geoalg_core::algebra::metric::Metric;
pub use geoalg_core::compat;
pub use geoalg_core::ops::screw::{between, exp, log};
pub use geoalg_core::types::line::Line;
pub use geoalg_core::types::motor::Motor;
pub use geoalg_core::types::plane::Plane;
pub use geoalg_core::types::point::Point;
pub use geoalg_core::types::rotor::Rotor;
pub use geoalg_core::types::sphere::Sphere;
pub use geoalg_core::types::translator::Translator;

pub const METRIC: Metric = Metric::pga();

// Atajos para construir rápidamente.
pub fn point(x: f64, y: f64, z: f64) -> Point {
    Point::new(x, y, z)
}

pub fn plane(a: f64, b: f64, c: f64, d: f64) -> Plane {
    Plane::new(a, b, c, d)
}

pub fn line(x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64) -> Line {
    Line::through_points(&Point::new(x1, y1, z1), &Point::new(x2, y2, z2))
}

pub fn rotor(axis: (f64, f64, f64), angle: f64) -> Rotor {
    let l = Line::through_points(&Point::origin(), &Point::new(axis.0, axis.1, axis.2));
    Rotor::from_axis_angle(&l, angle)
}

pub fn translator(dx: f64, dy: f64, dz: f64) -> Translator {
    Translator::new(dx, dy, dz)
}

pub fn motor_pos_quat(pos: (f64, f64, f64), quat: (f64, f64, f64, f64)) -> Motor {
    compat::pose_to_motor(pos, quat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_core::ops::sandwich::apply_point;

    #[test]
    fn atajos_funcionan() {
        let r = rotor((0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        let t = translator(1.0, 0.0, 0.0);
        let m = r.to_motor().then(&t.to_motor());
        let p = apply_point(&m.inner(), &point(1.0, 0.0, 0.0));
        let (x, y, z) = p.coords().unwrap();
        assert!((x - 1.0).abs() < 1e-9 && (y - 1.0).abs() < 1e-9 && z.abs() < 1e-9);
    }

    #[test]
    fn motor_desde_pose() {
        let m = motor_pos_quat((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0));
        let p = apply_point(&m.inner(), &Point::origin()).coords().unwrap();
        assert!((p.0 - 1.0).abs() < 1e-9 && (p.1 - 2.0).abs() < 1e-9 && (p.2 - 3.0).abs() < 1e-9);
    }
}