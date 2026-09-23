use std::f64::consts::PI;

use crate::algebra::metric::Metric;
use crate::multivector::dense::Dense;
use crate::types::line::Line;
use crate::types::plane::Plane;
use crate::types::point::Point;
use crate::types::rotor::Rotor;
use crate::types::sphere::Sphere;

pub fn sandwich(v: &Dense, x: &Dense, m: &Metric) -> Dense {
    v.geo(x, m).geo(&v.reverse(), m)
}

pub fn apply_point(v: &Dense, p: &Point) -> Point {
    Point::from_dense(sandwich(v, &p.inner(), &Metric::pga()))
}

pub fn apply_plane(v: &Dense, pl: &Plane) -> Plane {
    Plane::from_dense(sandwich(v, &pl.inner(), &Metric::pga()))
}

pub fn apply_line(v: &Dense, l: &Line) -> Line {
    Line::from_dense(sandwich(v, &l.inner(), &Metric::pga()))
}

pub fn apply_sphere(v: &Dense, s: &Sphere) -> Sphere {
    Sphere::new(apply_point(v, &s.center()), s.radius())
}

pub fn reflect_point_in_plane(plane: &Plane, p: &Point) -> Point {
    let pi = plane.normalize();
    apply_point(&pi.inner(), p)
}

pub fn reflect_point_in_line(line: &Line, p: &Point) -> Point {
    let r = Rotor::from_axis_angle(line, PI);
    apply_point(&r.inner(), p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::translator::Translator;

    fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 && (a.2 - b.2).abs() < 1e-9
    }

    #[test]
    fn traslador_mueve_puntos() {
        let t = Translator::new(1.0, 2.0, 3.0);
        let p = apply_point(&t.inner(), &Point::origin());
        assert!(close(p.coords().unwrap(), (1.0, 2.0, 3.0)));
    }

    #[test]
    fn rotor_gira_puntos() {
        let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let r = Rotor::from_axis_angle(&axis, PI / 2.0);
        let p = apply_point(&r.inner(), &Point::new(1.0, 0.0, 0.0));
        assert!(close(p.coords().unwrap(), (0.0, 1.0, 0.0)));
    }

    #[test]
    fn reflexion_en_plano() {
        let pl = Plane::new(1.0, 0.0, 0.0, -1.0);
        let p = reflect_point_in_plane(&pl, &Point::new(2.0, 5.0, 7.0));
        assert!(close(p.coords().unwrap(), (0.0, 5.0, 7.0)));
    }

    #[test]
    fn reflexion_en_recta() {
        let l = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let p = reflect_point_in_line(&l, &Point::new(1.0, 2.0, 3.0));
        assert!(close(p.coords().unwrap(), (-1.0, -2.0, 3.0)));
    }
}