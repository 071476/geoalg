// Model: describe un robot con sus articulaciones y eslabones.
// En vez de matrices DH, cada articulacion es un motor: el eje donde
// gira (o desliza) y el desplazamiento hasta la siguiente articulacion.

use geoalg_core::algebra::metric::Metric;
use geoalg_core::types::line::Line;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::rotor::Rotor;
use geoalg_core::types::translator::Translator;

pub enum JointKind {
    // Articulacion que gira alrededor de su eje.
    Revolute,
    // Articulacion que se desliza a lo largo de su eje.
    Prismatic,
}

pub struct Joint {
    pub kind: JointKind,
    // Eje de rotacion (Revolute) o de desplazamiento (Prismatic).
    pub axis: Line,
    // Offset fijo desde esta articulacion hasta la siguiente.
    pub offset: (f64, f64, f64),
    // Limites del movimiento (min, max) en radianes o unidades.
    pub limits: (f64, f64),
}

pub struct Robot {
    pub joints: Vec<Joint>,
}

impl Robot {
    pub fn new() -> Self {
        Robot { joints: Vec::new() }
    }

    // Agrega una articulacion que gira sobre `axis` con un offset.
    pub fn add_revolute(
        &mut self,
        origin: (f64, f64, f64),
        direction: (f64, f64, f64),
        offset: (f64, f64, f64),
        limits: (f64, f64),
    ) {
        let axis = Line::through_points(
            &Point::new(origin.0, origin.1, origin.2),
            &Point::new(
                origin.0 + direction.0,
                origin.1 + direction.1,
                origin.2 + direction.2,
            ),
        );
        self.joints.push(Joint {
            kind: JointKind::Revolute,
            axis,
            offset,
            limits,
        });
    }

    // Agrega una articulacion que desliza sobre `axis` con un offset.
    pub fn add_prismatic(
        &mut self,
        origin: (f64, f64, f64),
        direction: (f64, f64, f64),
        offset: (f64, f64, f64),
        limits: (f64, f64),
    ) {
        let axis = Line::through_points(
            &Point::new(origin.0, origin.1, origin.2),
            &Point::new(
                origin.0 + direction.0,
                origin.1 + direction.1,
                origin.2 + direction.2,
            ),
        );
        self.joints.push(Joint {
            kind: JointKind::Prismatic,
            axis,
            offset,
            limits,
        });
    }

    pub fn len(&self) -> usize {
        self.joints.len()
    }

    // Motor local de una articulacion para el valor q.
    pub fn joint_motor(&self, i: usize, q: f64) -> Motor {
        let j = &self.joints[i];
        let m = Metric::pga();
        let motion = match j.kind {
            JointKind::Revolute => {
                Motor::from_rotor(&Rotor::from_axis_angle(&j.axis, q)).inner()
            }
            JointKind::Prismatic => {
                let d = j.axis.direction();
                let n = (d.0 * d.0 + d.1 * d.1 + d.2 * d.2).sqrt();
                Motor::from_translator(&Translator::new(
                    d.0 / n * q,
                    d.1 / n * q,
                    d.2 / n * q,
                ))
                .inner()
            }
        };
        let off = Motor::from_translator(&Translator::new(
            j.offset.0,
            j.offset.1,
            j.offset.2,
        ))
        .inner();
        Motor::from_dense(motion.geo(&off, &m))
    }
}

// Brazo de 3 articulaciones revolutas: el mismo de los ejemplos.
pub fn demo_arm() -> Robot {
    let mut r = Robot::new();
    r.add_revolute((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (-3.14, 3.14));
    r.add_revolute((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-2.0, 2.0));
    r.add_revolute((0.0, 0.0, 2.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-2.0, 2.0));
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brazo_demo_tiene_tres_articulaciones() {
        let r = demo_arm();
        assert_eq!(r.len(), 3);
    }

    #[test]
    fn articulacion_gira() {
        let r = demo_arm();
        let m = r.joint_motor(0, 0.0);
        let off = Motor::from_translator(&Translator::new(0.0, 0.0, 1.0));
        assert!(m.inner().approx_eq(&off.inner(), 1e-9));
    }

    #[test]
    fn articulacion_desliza() {
        let mut r = Robot::new();
        r.add_prismatic((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 5.0));
        let m = r.joint_motor(0, 2.0);
        let expect = Motor::from_translator(&Translator::new(0.0, 0.0, 2.0));
        assert!(m.inner().approx_eq(&expect.inner(), 1e-9));
    }
}