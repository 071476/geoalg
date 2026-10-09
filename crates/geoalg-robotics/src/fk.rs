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
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-9;

    // Compara dos posiciones con tolerancia.
    fn cerca(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS && (a.2 - b.2).abs() < EPS
    }

    #[test]
    fn brazo_recto_llega_a_3() {
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, 0.0, 0.0]);
        assert!(cerca(pos, (0.0, 0.0, 3.0)), "esperaba (0,0,3), obtuve {:?}", pos);
    }

    #[test]
    fn cuatro_puntos_en_la_cadena() {
        let r = demo_arm();
        let fk = forward(&r, &[0.5, 0.3, -0.2]);
        assert_eq!(fk.joints.len(), 4);
        assert_eq!(fk.links.len(), 3);
    }

    // Prueba EXACTA: hombro a 90 grados.
    // Hombro en z=1, quedan 2 de brazo por encima, giran hacia -Y.
    #[test]
    fn hombro_90_exacto() {
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, FRAC_PI_2, 0.0]);
        assert!(cerca(pos, (0.0, -2.0, 1.0)), "esperaba (0,-2,1), obtuve {:?}", pos);
    }

    // Prueba EXACTA: donde queda cada articulacion con el hombro a 90.
    #[test]
    fn articulaciones_en_su_lugar() {
        let r = demo_arm();
        let fk = forward(&r, &[0.0, FRAC_PI_2, 0.0]);
        assert!(cerca(fk.joints[0], (0.0, 0.0, 0.0)), "base: {:?}", fk.joints[0]);
        assert!(cerca(fk.joints[1], (0.0, 0.0, 1.0)), "hombro: {:?}", fk.joints[1]);
        assert!(cerca(fk.joints[2], (0.0, -1.0, 1.0)), "codo: {:?}", fk.joints[2]);
        assert!(cerca(fk.joints[3], (0.0, -2.0, 1.0)), "punta: {:?}", fk.joints[3]);
    }

    // Prueba EXACTA: solo el codo a 90 grados.
    // Codo en z=2, el ultimo tramo de 1 gira hacia -Y.
    #[test]
    fn codo_90_exacto() {
        let r = demo_arm();
        let pos = tip_position(&r, &[0.0, 0.0, FRAC_PI_2]);
        assert!(cerca(pos, (0.0, -1.0, 2.0)), "esperaba (0,-1,2), obtuve {:?}", pos);
    }

    // Prueba EXACTA: base y hombro a 90 grados.
    #[test]
    fn base_y_hombro_exacto() {
        let r = demo_arm();
        let pos = tip_position(&r, &[FRAC_PI_2, FRAC_PI_2, 0.0]);
        assert!(cerca(pos, (2.0, 0.0, 1.0)), "esperaba (2,0,1), obtuve {:?}", pos);
    }

    // La punta nunca puede quedar mas lejos que el largo total del brazo.
    #[test]
    fn nunca_mas_lejos_que_su_largo() {
        let r = demo_arm();
        let mut q = [0.0; 3];
        for a in 0..8 {
            for b in 0..8 {
                for c in 0..8 {
                    q[0] = a as f64 * 0.7 - 2.5;
                    q[1] = b as f64 * 0.5 - 2.0;
                    q[2] = c as f64 * 0.5 - 2.0;
                    let p = tip_position(&r, &q);
                    let d = (p.0 * p.0 + p.1 * p.1 + p.2 * p.2).sqrt();
                    assert!(d <= 3.0 + EPS, "q={:?} punta a {} (largo total 3)", q, d);
                }
            }
        }
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
        assert!((z - 3.5).abs() < EPS, "esperaba z=3.5, obtuve z={z}");
    }

    #[test]
    fn cadena_de_cinco_articulaciones() {
        use crate::model::Robot;
        let mut r = Robot::new();
        for _ in 0..5 {
            r.add_revolute((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (-3.14, 3.14));
        }
        let fk = forward(&r, &[0.0; 5]);
        assert_eq!(fk.joints.len(), 6);
        let (_, _, z) = *fk.joints.last().unwrap();
        assert!((z - 5.0).abs() < EPS, "esperaba z=5, obtuve z={z}");
    }
}
