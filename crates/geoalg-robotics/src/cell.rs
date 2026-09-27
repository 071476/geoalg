// Simulacion de celda: varios robots trabajando juntos sin chocar.
// Una celda es un espacio de trabajo con varios brazos y obstaculos.
// La regla clave: ningun brazo puede tocar a otro ni a un obstaculo.

use crate::tool::{ArmFK, Tool};

pub struct RobotArm {
    pub fk: ArmFK,
    pub tool: Tool,
    pub q: [f64; 3],
    pub base: (f64, f64, f64),
}

impl RobotArm {
    // Puntos de las articulaciones del brazo en el mundo (4 puntos).
       pub fn joints(&self) -> [(f64, f64, f64); 4] {
        use geoalg_core::algebra::metric::Metric;
        use geoalg_core::ops::sandwich::apply_point;
        use geoalg_core::types::line::Line;
        use geoalg_core::types::motor::Motor;
        use geoalg_core::types::point::Point;
        use geoalg_core::types::rotor::Rotor;
        use geoalg_core::types::translator::Translator;

        let m = Metric::pga();
        let az = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
        let ax = Line::through_points(&Point::origin(), &Point::new(1.0, 0.0, 0.0));
        let off = Motor::from_translator(&Translator::new(0.0, 0.0, 1.0)).inner();

        let mut ch = Motor::identity().inner();
        let mut pts = [self.base; 4];

        for i in 0..3 {
            let axis = if i == 0 { &az } else { &ax };
            let r = Motor::from_rotor(&Rotor::from_axis_angle(axis, self.q[i])).inner();
            ch = ch.geo(&r, &m).geo(&off, &m);
            let p = apply_point(&ch, &Point::origin()).coords().unwrap();
            pts[i + 1] = (p.0 + self.base.0, p.1 + self.base.1, p.2 + self.base.2);
        }
        pts
    }
    

    // Segmentos del brazo (3 eslabones) para comprobar choques.
    pub fn segments(&self) -> [((f64, f64, f64), (f64, f64, f64)); 3] {
        let j = self.joints();
        [(j[0], j[1]), (j[1], j[2]), (j[2], j[3])]
    }
}

// Distancia entre dos segmentos 3D (mas corta distancia entre las rectas).
pub fn segment_distance(
    a0: (f64, f64, f64), a1: (f64, f64, f64),
    b0: (f64, f64, f64), b1: (f64, f64, f64),
) -> f64 {
    let u = (a1.0 - a0.0, a1.1 - a0.1, a1.2 - a0.2);
    let v = (b1.0 - b0.0, b1.1 - b0.1, b1.2 - b0.2);
    let w = (a0.0 - b0.0, a0.1 - b0.1, a0.2 - b0.2);
    let dot = |p: (f64, f64, f64), q: (f64, f64, f64)| p.0 * q.0 + p.1 * q.1 + p.2 * q.2;

    let a = dot(u, u);
    let b = dot(u, v);
    let c = dot(v, v);
    let d = dot(u, w);
    let e = dot(v, w);
    let den = a * c - b * b;

    let (s, t) = if den.abs() < 1e-12 {
        (0.0, (e / c).clamp(0.0, 1.0))
    } else {
        let s = ((b * e - c * d) / den).clamp(0.0, 1.0);
        let t = ((a * e - b * d) / den).clamp(0.0, 1.0);
        (s, t)
    };

    let p = (a0.0 + s * u.0, a0.1 + s * u.1, a0.2 + s * u.2);
    let q = (b0.0 + t * v.0, b0.1 + t * v.1, b0.2 + t * v.2);
    let dx = p.0 - q.0;
    let dy = p.1 - q.1;
    let dz = p.2 - q.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub struct Cell {
    pub arms: Vec<RobotArm>,
    pub obstacles: Vec<((f64, f64, f64), (f64, f64, f64))>,
}

impl Cell {
    pub fn new() -> Self {
        Cell { arms: Vec::new(), obstacles: Vec::new() }
    }

    pub fn add_arm(&mut self, arm: RobotArm) {
        self.arms.push(arm);
    }

    pub fn add_obstacle(&mut self, segment: ((f64, f64, f64), (f64, f64, f64))) {
        self.obstacles.push(segment);
    }

    // Comprueba choques entre todos los brazos y los obstaculos.
    // Devuelve el par en choque y la distancia minima, si la hay.
    pub fn check_collisions(&self, clearance: f64) -> Option<(String, f64)> {
        let mut min = f64::INFINITY;
        let mut who = String::new();

        // Brazo contra brazo.
        for i in 0..self.arms.len() {
            for j in i + 1..self.arms.len() {
                for sa in &self.arms[i].segments() {
                    for sb in &self.arms[j].segments() {
                        let d = segment_distance(sa.0, sa.1, sb.0, sb.1);
                        if d < min {
                            min = d;
                            who = format!("brazo{i}-brazo{j}");
                        }
                    }
                }
            }
        }

        // Brazo contra obstaculo.
        for (i, arm) in self.arms.iter().enumerate() {
            for (k, ob) in self.obstacles.iter().enumerate() {
                for s in &arm.segments() {
                    let d = segment_distance(s.0, s.1, ob.0, ob.1);
                    if d < min {
                        min = d;
                        who = format!("brazo{i}-obstaculo{k}");
                    }
                }
            }
        }

        if min < clearance {
            Some((who, min))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_calculus::differential::jacobian::arm_motor;

    fn arm_at(base: (f64, f64, f64), q: [f64; 3]) -> RobotArm {
        RobotArm {
            fk: arm_motor,
            tool: Tool::from_offset(0.0, 0.0, 0.0),
            q,
            base,
        }
    }

    #[test]
    fn distancia_segmentos_paralelos() {
        // Dos segmentos paralelos separados 2 unidades en Y.
        let d = segment_distance(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (0.0, 2.0, 0.0), (1.0, 2.0, 0.0),
        );
        assert!((d - 2.0).abs() < 1e-9, "esperaba 2.0, obtuve {d}");
    }

    #[test]
    fn distancia_segmentos_que_se_cruzan() {
        // Dos segmentos que se cruzan: distancia 0.
        let d = segment_distance(
            (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0), (0.0, 1.0, 0.0),
        );
        assert!(d < 1e-9, "esperaba 0, obtuve {d}");
    }

    #[test]
    fn brazos_lejanos_no_chocan() {
        let mut cell = Cell::new();
        cell.add_arm(arm_at((0.0, 0.0, 0.0), [0.0, 0.3, -0.2]));
        cell.add_arm(arm_at((10.0, 10.0, 0.0), [1.0, 0.5, 0.1]));
        assert!(cell.check_collisions(0.5).is_none());
    }

    #[test]
    fn brazo_choca_con_obstaculo() {
        let mut cell = Cell::new();
        cell.add_arm(arm_at((0.0, 0.0, 0.0), [0.0, 0.0, 0.0]));
        // Obstaculo justo al lado del brazo estirado (va hacia +Z hasta 3).
        cell.add_obstacle(((0.1, 0.0, 1.5), (0.1, 0.0, 2.5)));
        let hit = cell.check_collisions(0.5);
        assert!(hit.is_some(), "deberia haber choque");
    }
}