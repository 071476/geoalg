// FK: Forward Kinematics — dado el modelo del robot y los angulos de las
// articulaciones, calcula la posicion de cada eslabon y del extremo.

use crate::model::Robot;
use geoalg_core::algebra::metric::Metric;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;

// Resultado de la cinematica directa.
pub struct FKResult {
    // Motor de cada eslabon (desde la base hasta el extremo).
    pub links: Vec<Motor>,
    // Posicion de cada articulacion (incluida la base).
    pub joints: Vec<(f64, f64, f64)>,
    // Motor del extremo (end effector).
    pub tip: Motor,
}

// Calcula la cinematica directa para los angulos dados.
// Los angulos q[i] corresponden a la articulacion i.
pub fn forward(robot: &Robot, q: &[f64]) -> FKResult {
    let m = Metric::pga();
    let mut links = Vec::with_capacity(robot.len());
    let mut joints = Vec::with_capacity(robot.len() + 1);
    let mut pose = Motor::identity().inner();

    joints.push((0.0, 0.0, 0.0));

    for i in 0..robot.len() {
        let local = robot.joint_motor(i, q[i]);
        pose = pose.geo(&local.inner(), &m);
        let link = Motor::from_dense(pose);
        let p = apply_point(&link.inner(), &Point::origin()).coords().unwrap();
        joints.push(p);
        links.push(link);
    }

    let tip = Motor::from_dense(pose);
    FKResult { links, joints, tip }
}

// Solo la posicion del extremo (mas rapido si no necesitas los eslabones).
pub fn tip_position(robot: &Robot, q: &[f64]) -> (f64, f64, f64) {
    let r = forward(robot, q);
    *r.joints.last().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::demo_arm;

    #[test]
    fn brazo_recto_llega_a_3() {
        // Brazo estirado hacia arriba: extremo en (0, 0, 3).
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, 0.0, 0.0]);
        assert!(pos.0.abs() < 1e-9 && pos.1.abs() < 1e-9);
        assert!((pos.2 - 3.0).abs() < 1e-9, "esperaba z=3, obtuve z={}", pos.2);
    }

    #[test]
    fn cuatro_puntos_en_la_cadena() {
        // 3 articulaciones = 4 puntos (base + 3 articulaciones).
        let r = demo_arm();
        let fk = forward(&r, &[0.5, 0.3, -0.2]);
        assert_eq!(fk.joints.len(), 4);
        assert_eq!(fk.links.len(), 3);
    }

    #[test]
    fn giro_de_base() {
        // Doblar el hombro 90 grados y girar la base: el extremo sale en +X.
        let r = demo_arm();
        let pos = tip_position(&r, &[std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(pos.0 > 0.5, "esperaba x>0.5, obtuve x={}", pos.0);
    }

    #[test]
    fn hombro_dobra_el_brazo() {
        // Con el hombro a 90 grados, el brazo se dobla y baja la altura.
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(pos.2 < 3.0, "esperaba z<3, obtuve z={}", pos.2);
    }
}