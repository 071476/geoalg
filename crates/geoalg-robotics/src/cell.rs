// Simulacion de celda: varios robots trabajando juntos sin chocar.
// Una celda es un espacio de trabajo con varios brazos y obstaculos.
// La regla clave: ningun brazo puede tocar a otro ni a un obstaculo.

use crate::tool::{ArmFK, Tool};

pub struct RobotArm {
    // PENDIENTE: `fk` y `tool` todavia no se usan en `joints()`.
    // El brazo de la celda es el brazo demo (3 tramos de 1).
    // Mas adelante `joints()` debe usar el modelo `Robot` y `fk::forward`.
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
        // Ejes locales: pasan por la propia articulacion (origen).
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

// Distancia mas corta entre dos segmentos 3D.
//
// Primero busca el par de puntos mas cercanos entre las rectas completas.
// Si alguno cae fuera de su segmento, lo recorta al extremo y RECALCULA
// el punto del otro segmento. Recortar los dos por separado da distancias
// mas grandes que las reales, y eso hace que no se detecten choques.
pub fn segment_distance(
    a0: (f64, f64, f64), a1: (f64, f64, f64),
    b0: (f64, f64, f64), b1: (f64, f64, f64),
) -> f64 {
    const EPS: f64 = 1e-12;
    let u = (a1.0 - a0.0, a1.1 - a0.1, a1.2 - a0.2);
    let v = (b1.0 - b0.0, b1.1 - b0.1, b1.2 - b0.2);
    let w = (a0.0 - b0.0, a0.1 - b0.1, a0.2 - b0.2);
    let dot = |p: (f64, f64, f64), q: (f64, f64, f64)| p.0 * q.0 + p.1 * q.1 + p.2 * q.2;

    let a = dot(u, u);
    let b = dot(u, v);
    let c = dot(v, v);
    let d = dot(u, w);
    let e = dot(v, w);

    let (s, t) = if a < EPS && c < EPS {
        // Los dos segmentos son puntos.
        (0.0, 0.0)
    } else if a < EPS {
        // El primer segmento es un punto.
        (0.0, (e / c).clamp(0.0, 1.0))
    } else if c < EPS {
        // El segundo segmento es un punto.
        ((-d / a).clamp(0.0, 1.0), 0.0)
    } else {
        let den = a * c - b * b;
        // Si son paralelos, cualquier s sirve como punto de partida.
        let s0 = if den > EPS {
            ((b * e - c * d) / den).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // Punto del segundo segmento mas cercano a s0.
        let t0 = (b * s0 + e) / c;
        if t0 < 0.0 {
            ((-d / a).clamp(0.0, 1.0), 0.0)
        } else if t0 > 1.0 {
            (((b - d) / a).clamp(0.0, 1.0), 1.0)
        } else {
            (s0, t0)
        }
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

impl Default for Cell {
    fn default() -> Self {
        Self::new()
    }
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
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-9;

    fn arm_at(base: (f64, f64, f64), q: [f64; 3]) -> RobotArm {
        RobotArm {
            fk: arm_motor,
            tool: Tool::from_offset(0.0, 0.0, 0.0),
            q,
            base,
        }
    }

    fn cerca(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS && (a.2 - b.2).abs() < EPS
    }

    #[test]
    fn distancia_segmentos_paralelos() {
        // Dos segmentos paralelos separados 2 unidades en Y.
        let d = segment_distance(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (0.0, 2.0, 0.0), (1.0, 2.0, 0.0),
        );
        assert!((d - 2.0).abs() < EPS, "esperaba 2.0, obtuve {d}");
    }

    #[test]
    fn distancia_paralelos_desplazados() {
        // Paralelos pero sin solaparse: lo mas cercano es de (1,0,0) a (5,1,0).
        let d = segment_distance(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (5.0, 1.0, 0.0), (6.0, 1.0, 0.0),
        );
        let esperado = 17.0_f64.sqrt();
        assert!((d - esperado).abs() < EPS, "esperaba {esperado}, obtuve {d}");
    }

    #[test]
    fn distancia_con_recorte() {
        // El punto mas cercano de las rectas cae fuera de los segmentos.
        // Recortar los dos por separado daba 1.0; lo correcto es 1/sqrt(5).
        let d = segment_distance(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0), (1.0, 0.0, 1.0),
        );
        let esperado = 1.0 / 5.0_f64.sqrt();
        assert!((d - esperado).abs() < EPS, "esperaba {esperado}, obtuve {d}");
    }

    #[test]
    fn distancia_de_punto_a_segmento() {
        // El primer "segmento" es un punto.
        let d = segment_distance(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (1.0, -1.0, 0.0), (1.0, 1.0, 0.0),
        );
        assert!((d - 1.0).abs() < EPS, "esperaba 1.0, obtuve {d}");
    }

    #[test]
    fn distancia_segmentos_que_se_cruzan() {
        // Dos segmentos que se cruzan: distancia 0.
        let d = segment_distance(
            (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0), (0.0, 1.0, 0.0),
        );
        assert!(d < EPS, "esperaba 0, obtuve {d}");
    }

    #[test]
    fn articulaciones_del_brazo_exactas() {
        // Hombro a 90 grados, base movida a (5,0,0).
        let arm = arm_at((5.0, 0.0, 0.0), [0.0, FRAC_PI_2, 0.0]);
        let j = arm.joints();
        assert!(cerca(j[0], (5.0, 0.0, 0.0)), "base: {:?}", j[0]);
        assert!(cerca(j[1], (5.0, 0.0, 1.0)), "hombro: {:?}", j[1]);
        assert!(cerca(j[2], (5.0, -1.0, 1.0)), "codo: {:?}", j[2]);
        assert!(cerca(j[3], (5.0, -2.0, 1.0)), "punta: {:?}", j[3]);
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
