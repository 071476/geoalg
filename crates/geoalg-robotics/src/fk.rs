// FK: Forward Kinematics — dado el modelo del robot y los angulos de las
// articulaciones, calcula la posicion de cada eslabon y del extremo.

use crate::model::Robot;
use crate::tool::Tool;
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
// Funciona para cualquier numero de articulaciones N.
pub fn forward(robot: &Robot, q: &[f64]) -> FKResult {
    let m = Metric::pga();
    let mut links = Vec::with_capacity(robot.len());
    let mut joints = Vec::with_capacity(robot.len() + 1);
    let mut pose = Motor::identity().inner();

    joints.push((0.0, 0.0, 0.0));

    for i in 0..robot.len() {
        let local = robot.joint_motor(i, q[i]);
        // Transformacion de paso: pose acumulada * motor local.
        pose = pose.geo(&local.inner(), &m);
        let link = Motor::from_dense(pose);
        let p = apply_point(&link.inner(), &Point::origin()).coords().unwrap();
        joints.push(p);
        links.push(link);
    }

    let tip = Motor::from_dense(pose);
    FKResult { links, joints, tip }
}

// FK con herramienta: calcula la cadena completa incluyendo el TCP.
pub fn forward_with_tool(robot: &Robot, q: &[f64], tool: &Tool) -> FKResult {
    let m = Metric::pga();
    let base = forward(robot, q);
    let tcp = Motor::from_dense(base.tip.inner().geo(&tool.offset().inner(), &m));
    let tip_pos = apply_point(&tcp.inner(), &Point::origin()).coords().unwrap();

    let mut links = base.links;
    links.push(tcp);
    let mut joints = base.joints;
    joints.push(tip_pos);

    FKResult { links, joints, tip: tcp }
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
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, 0.0, 0.0]);
        assert!(pos.0.abs() < 1e-9 && pos.1.abs() < 1e-9);
        assert!((pos.2 - 3.0).abs() < 1e-9, "esperaba z=3, obtuve z={}", pos.2);
    }

    #[test]
    fn cuatro_puntos_en_la_cadena() {
        let r = demo_arm();
        let fk = forward(&r, &[0.5, 0.3, -0.2]);
        assert_eq!(fk.joints.len(), 4);
        assert_eq!(fk.links.len(), 3);
    }

    #[test]
    fn giro_de_base() {
        let r = demo_arm();
        let pos = tip_position(&r, &[std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(pos.0 > 0.5, "esperaba x>0.5, obtuve x={}", pos.0);
    }

    #[test]
    fn hombro_dobra_el_brazo() {
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(pos.2 < 3.0, "esperaba z<3, obtuve z={}", pos.2);
    }

    #[test]
    fn forward_con_tool() {
        let r = demo_arm();
        let tool = Tool::from_offset(0.0, 0.0, 0.5);
        let fk = forward_with_tool(&r, &[0.0, 0.0, 0.0], &tool);
        // Brazo recto de 3 + tool de 0.5 = extremo en z = 3.5.
        assert_eq!(fk.joints.len(), 5);
        assert_eq!(fk.links.len(), 4);
        let (_, _, z) = *fk.joints.last().unwrap();
        assert!((z - 3.5).abs() < 1e-9, "esperaba z=3.5, obtuve z={z}");
    }

    #[test]
    fn cadena_de_cinco_articulaciones() {
        use crate::model::Robot;
        let mut r = Robot::new();
        for i in 0..5 {
            let z = i as f64;
            r.add_revolute((0.0, 0.0, z), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (-3.14, 3.14));
        }
        let fk = forward(&r, &[0.0; 5]);
        assert_eq!(fk.joints.len(), 6);
        let (_, _, z) = *fk.joints.last().unwrap();
        assert!((z - 5.0).abs() < 1e-9, "esperaba z=5, obtuve z={z}");
    }
}