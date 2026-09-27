// Camara: un motor que mira hacia un objetivo.
// En el espacio de camara, Z+ es hacia adelante. La camara proyecta
// puntos del mundo a coordenadas de pantalla (x, y, profundidad).

use geoalg_core::algebra::metric::Metric;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::line::Line;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::rotor::Rotor;
use geoalg_core::types::translator::Translator;

pub struct Camera {
    pose: Motor,
    focal: f64,
}

impl Camera {
    // Crea una camara en `eye` que mira hacia `target`.
    pub fn look_at(eye: (f64, f64, f64), target: (f64, f64, f64), focal: f64) -> Self {
        let dir = (target.0 - eye.0, target.1 - eye.1, target.2 - eye.2);
        let n = (dir.0 * dir.0 + dir.1 * dir.1 + dir.2 * dir.2).sqrt();
        let d = (dir.0 / n, dir.1 / n, dir.2 / n);

        // Rotor que gira +Z hacia la direccion de la mirada.
        let ax = -d.1;
        let ay = d.0;
        let axis_n = (ax * ax + ay * ay).sqrt();
        let rot = if axis_n < 1e-12 {
            if d.2 > 0.0 {
                Motor::identity()
            } else {
                let l = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
                Motor::from_rotor(&Rotor::from_axis_angle(&l, std::f64::consts::PI))
            }
        } else {
            let l = Line::through_points(
                &Point::origin(),
                &Point::new(ax / axis_n, ay / axis_n, 0.0),
            );
            let angle = d.2.clamp(-1.0, 1.0).acos();
            Motor::from_rotor(&Rotor::from_axis_angle(&l, angle))
        };

        let trans = Motor::from_translator(&Translator::new(eye.0, eye.1, eye.2));
        let m = Metric::pga();
        let pose = Motor::from_dense(trans.inner().geo(&rot.inner(), &m));

        Camera { pose, focal }
    }

    pub fn pose(&self) -> &Motor {
        &self.pose
    }

    // Punto del mundo -> espacio de camara.
    pub fn to_camera(&self, p: &Point) -> Point {
        apply_point(&self.pose.inverse().inner(), p)
    }

    // Punto del mundo -> pantalla (x, y, profundidad).
    // z > 0: delante de la camara. z <= 0: detras.
    pub fn project(&self, p: &Point) -> (f64, f64, f64) {
        let cp = self.to_camera(p);
        let (x, y, z) = cp.coords().expect("punto ideal");
        (self.focal * x / z, self.focal * y / z, z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mira_hacia_delante() {
        let cam = Camera::look_at((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 1.0);
        let (x, y, z) = cam.project(&Point::new(0.0, 0.0, 5.0));
        assert!(x.abs() < 1e-9 && y.abs() < 1e-9);
        assert!((z - 5.0).abs() < 1e-9);
    }

    #[test]
    fn punto_lateral() {
        let cam = Camera::look_at((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 1.0);
        let (x, y, _z) = cam.project(&Point::new(1.0, 0.0, 1.0));
        assert!((x - 1.0).abs() < 1e-9);
        assert!(y.abs() < 1e-9);
    }

    #[test]
    fn profundidad_correcta() {
        let cam = Camera::look_at((0.0, 0.0, 5.0), (0.0, 0.0, 0.0), 1.0);
        let (x, y, z) = cam.project(&Point::origin());
        assert!(x.abs() < 1e-9 && y.abs() < 1e-9);
        assert!((z - 5.0).abs() < 1e-6, "profundidad: {z}");
    }
}