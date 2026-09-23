use crate::ops::sandwich::{reflect_point_in_line, reflect_point_in_plane};
use crate::types::line::Line;
use crate::types::plane::Plane;
use crate::types::point::Point;
use crate::types::sphere::Sphere;

pub fn project_point_on_plane(plane: &Plane, p: &Point) -> Point {
    let r = reflect_point_in_plane(plane, p);
    Point::from_dense((p.inner() + r.inner()) * 0.5)
}

pub fn project_point_on_line(line: &Line, p: &Point) -> Point {
    let r = reflect_point_in_line(line, p);
    Point::from_dense((p.inner() + r.inner()) * 0.5)
}

pub fn project_point_on_sphere(s: &Sphere, p: &Point) -> Point {
    let c = s.center().coords().expect("centro ideal");
    let q = p.coords().expect("punto ideal");
    let dx = q.0 - c.0;
    let dy = q.1 - c.1;
    let dz = q.2 - c.2;
    let n = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(n > 1e-12, "el punto es el centro");
    let r = s.radius();
    Point::new(c.0 + r * dx / n, c.1 + r * dy / n, c.2 + r * dz / n)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 && (a.2 - b.2).abs() < 1e-9
    }

    #[test]
    fn pie_sobre_plano() {
        let pl = Plane::new(1.0, 0.0, 0.0, -1.0);
        let f = project_point_on_plane(&pl, &Point::new(2.0, 5.0, 7.0));
        assert!(close(f.coords().unwrap(), (1.0, 5.0, 7.0)));
    }

    #[test]
    fn pie_sobre_recta() {
        let l = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let f = project_point_on_line(&l, &Point::new(1.0, 2.0, 3.0));
        assert!(close(f.coords().unwrap(), (0.0, 0.0, 3.0)));
    }

    #[test]
    fn pie_sobre_esfera() {
        let s = Sphere::new(Point::origin(), 2.0);
        let f = project_point_on_sphere(&s, &Point::new(2.0, 2.0, 0.0));
        let (x, y, z) = f.coords().unwrap();
        let r = (x * x + y * y + z * z).sqrt();
        assert!((r - 2.0).abs() < 1e-9);
    }
}
