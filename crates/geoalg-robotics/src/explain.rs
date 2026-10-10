//! Explain: diagnostico basico de la IK (version abierta).
//!
//! Detecta dos cosas:
//!   - INVALID_TARGET: el objetivo tiene NaN o infinito.
//!   - OUT_OF_REACH:   el objetivo esta mas lejos de lo que el robot alcanza.
//! Cualquier otro fallo se reporta como NOT_REACHED.
//!
//! El diagnostico detallado (que articulacion quedo topada en su limite,
//! si el solver se atoro, y mas) es parte de geoalg-premium.
//!
//! En vez de solo "converged: false", devuelve algo como:
//!   "El objetivo esta a 3.07 de la articulacion 2 (el ultimo punto fijo
//!    del robot), pero desde ahi alcanza como maximo 2.00.
//!    -> Acerca el objetivo al menos 1.07."
//!
//! Uso:
//!   let (sol, diag) = robot.ik_xyz_explained(target, &q0, 200, 1e-6);
//!   if !diag.ok() { println!("{}", diag); }
//!
//! Supone lo mismo que model.rs recomienda: cada eje pasa por su
//! propia articulacion (origin = (0, 0, 0)).

use std::fmt;

use crate::fk::{forward, forward_with_tool};
use crate::ik::IKSolution;
use crate::model::{JointKind, Robot};
use crate::tool::Tool;

// Que tanto puede moverse un punto para seguir contando como "fijo".
const FIXED_EPS: f64 = 1e-9;

/// Un motivo concreto por el que la IK no llego.
#[derive(Clone, Debug, PartialEq)]
pub enum Problem {
    /// El objetivo tiene NaN o infinito.
    InvalidTarget,
    /// El objetivo esta mas lejos de lo que el robot puede alcanzar.
    /// `anchor` es el ultimo punto del robot que nunca se mueve
    /// (0 = la base); las distancias se miden desde ahi.
    OutOfReach { anchor: usize, distance: f64, max_reach: f64 },
    /// El objetivo esta dentro del alcance, pero no se llego.
    NotReached { error: f64, iterations: usize },
}

// "la base" o "la articulacion N (el ultimo punto fijo del robot)".
fn anchor_name(anchor: usize) -> String {
    if anchor == 0 {
        "la base".to_string()
    } else {
        format!("la articulacion {} (el ultimo punto fijo del robot)", anchor + 1)
    }
}

impl Problem {
    /// Codigo corto para la API y el studio (colores, iconos...).
    pub fn code(&self) -> &'static str {
        match self {
            Problem::InvalidTarget => "INVALID_TARGET",
            Problem::OutOfReach { .. } => "OUT_OF_REACH",
            Problem::NotReached { .. } => "NOT_REACHED",
        }
    }

    /// Que paso, en palabras sencillas.
    pub fn message(&self) -> String {
        match self {
            Problem::InvalidTarget => {
                "El objetivo tiene valores invalidos (NaN o infinito).".to_string()
            }
            Problem::OutOfReach { anchor, distance, max_reach } => format!(
                "El objetivo esta a {:.2} de {}, pero desde ahi el robot alcanza como maximo {:.2}.",
                distance,
                anchor_name(*anchor),
                max_reach
            ),
            Problem::NotReached { error, iterations } => format!(
                "El objetivo esta dentro del alcance, pero tras {} vueltas la punta quedo a {:.4}.",
                iterations, error
            ),
        }
    }

    /// Que puede hacer el desarrollador.
    pub fn suggestion(&self) -> String {
        match self {
            Problem::InvalidTarget => {
                "Revisa de donde sale el punto: suele ser una division entre cero.".to_string()
            }
            Problem::OutOfReach {
                distance,
                max_reach,
                ..
            } => format!(
                "Acerca el objetivo al menos {:.2} o usa eslabones mas largos.",
                distance - max_reach
            ),
            Problem::NotReached { .. } => {
                "Prueba otra postura inicial o revisa los limites de las articulaciones.".to_string()
            }
        }
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{}] {} -> {}", self.code(), self.message(), self.suggestion())
    }
}

/// El diagnostico completo de una IK.
#[derive(Clone, Debug)]
pub struct Diagnosis {
    pub converged: bool,
    pub error: f64,
    /// Lo mas cerca que llego la punta.
    pub closest: [f64; 3],
    pub problems: Vec<Problem>,
}

impl Diagnosis {
    /// true si llego y no hay nada que reportar.
    pub fn ok(&self) -> bool {
        self.converged && self.problems.is_empty()
    }

    /// Todo el diagnostico como texto.
    pub fn text(&self) -> String {
        if self.ok() {
            return format!("Listo: la punta llego al objetivo (error {:.2e}).", self.error);
        }
        let mut s = format!(
            "No se llego al objetivo. Lo mas cerca fue ({:.3}, {:.3}, {:.3}), a {:.3} de distancia.",
            self.closest[0], self.closest[1], self.closest[2], self.error
        );
        for p in &self.problems {
            s.push_str("\n  - ");
            s.push_str(&p.to_string());
        }
        s
    }

    /// JSON sencillo para mandarlo al studio (sin depender de serde).
    pub fn to_json(&self) -> String {
        let problems: Vec<String> = self
            .problems
            .iter()
            .map(|p| {
                format!(
                    "{{\"code\":{},\"message\":{},\"suggestion\":{}}}",
                    json_str(p.code()),
                    json_str(&p.message()),
                    json_str(&p.suggestion())
                )
            })
            .collect();
        format!(
            "{{\"converged\":{},\"error\":{},\"closest\":[{},{},{}],\"problems\":[{}]}}",
            self.converged,
            json_num(self.error),
            json_num(self.closest[0]),
            json_num(self.closest[1]),
            json_num(self.closest[2]),
            problems.join(",")
        )
    }
}

impl fmt::Display for Diagnosis {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.text())
    }
}

fn json_str(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn json_num(v: f64) -> String {
    if v.is_finite() {
        format!("{}", v)
    } else {
        "null".to_string()
    }
}

// Posicion de la punta (con o sin herramienta).
fn tip(robot: &Robot, tool: Option<&Tool>, q: &[f64]) -> [f64; 3] {
    let r = match tool {
        Some(t) => forward_with_tool(robot, q, t),
        None => forward(robot, q),
    };
    let p = *r.joints.last().unwrap();
    [p.0, p.1, p.2]
}

fn dist(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

// Largo de la herramienta: distancia entre la brida y el TCP (es constante).
fn tool_length(robot: &Robot, tool: Option<&Tool>) -> f64 {
    match tool {
        Some(t) => {
            let zeros = vec![0.0; robot.len()];
            dist(&tip(robot, None, &zeros), &tip(robot, Some(t), &zeros))
        }
        None => 0.0,
    }
}

// Lo mas que pueden separar las articulaciones desde `from` hasta la punta.
fn reach_from(robot: &Robot, from: usize) -> f64 {
    let mut total = 0.0;
    for j in robot.joints.iter().skip(from) {
        let (x, y, z) = j.offset;
        total += (x * x + y * y + z * z).sqrt();
        if let JointKind::Prismatic = j.kind {
            let (lo, hi) = j.limits;
            total += lo.abs().max(hi.abs());
        }
    }
    total
}

/// Alcance maximo desde la base: suma de todos los eslabones,
/// mas lo que pueden estirarse las prismaticas, mas la herramienta.
/// Es un tope seguro: el robot NUNCA puede llegar mas lejos que esto.
pub fn max_reach(robot: &Robot, tool: Option<&Tool>) -> f64 {
    reach_from(robot, 0) + tool_length(robot, tool)
}

// Posturas de prueba repartidas dentro de los limites (sin numeros al azar,
// para que el resultado sea siempre el mismo).
fn sample_postures(robot: &Robot) -> Vec<Vec<f64>> {
    let fracs: [&[f64]; 6] = [
        &[0.5],
        &[0.1, 0.9],
        &[0.9, 0.1],
        &[0.3, 0.7, 0.2],
        &[0.75, 0.25, 0.6, 0.4],
        &[0.15, 0.55, 0.85],
    ];
    fracs
        .iter()
        .map(|f| {
            robot
                .joints
                .iter()
                .enumerate()
                .map(|(i, j)| {
                    let (lo, hi) = j.limits;
                    lo + (hi - lo) * f[i % f.len()]
                })
                .collect()
        })
        .collect()
}

/// El ultimo punto de la cadena que NUNCA se mueve, sin importar los angulos
/// (por ejemplo el hombro, si la base solo gira sobre su propio eje).
/// Devuelve (indice del punto, posicion). El indice 0 es la base.
pub fn fixed_anchor(robot: &Robot) -> (usize, [f64; 3]) {
    let postures = sample_postures(robot);
    let chains: Vec<Vec<[f64; 3]>> = postures
        .iter()
        .map(|q| {
            forward(robot, q)
                .joints
                .iter()
                .map(|p| [p.0, p.1, p.2])
                .collect()
        })
        .collect();
    let mut anchor = 0;
    for k in 1..=robot.len() {
        let first = chains[0][k];
        if chains.iter().all(|c| dist(&c[k], &first) <= FIXED_EPS) {
            anchor = k;
        } else {
            break;
        }
    }
    (anchor, chains[0][anchor])
}

/// Revisa una solucion de IK y explica que paso.
pub fn diagnose(
    robot: &Robot,
    tool: Option<&Tool>,
    target: [f64; 3],
    solution: &IKSolution,
) -> Diagnosis {
    let mut problems = Vec::new();

    let mut q = solution.angles.clone();
    q.resize(robot.len(), 0.0);
    let closest = tip(robot, tool, &q);

    if target.iter().any(|v| !v.is_finite()) {
        problems.push(Problem::InvalidTarget);
    } else if !solution.converged {
        // 1. Esta demasiado lejos? Se mide desde el ultimo punto fijo del
        //    robot: medir desde la base dejaria pasar objetivos que tampoco
        //    se alcanzan (por ejemplo, al lado de la base pero muy abajo).
        let (anchor, anchor_pos) = fixed_anchor(robot);
        let distance = dist(&anchor_pos, &target);
        let reach = reach_from(robot, anchor) + tool_length(robot, tool);
        if distance > reach + 1e-9 {
            problems.push(Problem::OutOfReach {
                anchor,
                distance,
                max_reach: reach,
            });
        } else {
            // 2. Esta al alcance, pero no se llego.
            problems.push(Problem::NotReached {
                error: solution.error,
                iterations: solution.iterations,
            });
        }
    }

    Diagnosis {
        converged: solution.converged,
        error: solution.error,
        closest,
        problems,
    }
}

impl Robot {
    /// Igual que `ik_xyz`, pero ademas devuelve el diagnostico.
    pub fn ik_xyz_explained(
        &self,
        target: [f64; 3],
        initial: &[f64],
        max_iter: usize,
        tol: f64,
    ) -> (IKSolution, Diagnosis) {
        explained(self, None, target, initial, max_iter, tol)
    }

    /// Igual que `ik_xyz_with_tool`, pero ademas devuelve el diagnostico.
    pub fn ik_xyz_with_tool_explained(
        &self,
        tool: &Tool,
        target: [f64; 3],
        initial: &[f64],
        max_iter: usize,
        tol: f64,
    ) -> (IKSolution, Diagnosis) {
        explained(self, Some(tool), target, initial, max_iter, tol)
    }
}

fn explained(
    robot: &Robot,
    tool: Option<&Tool>,
    target: [f64; 3],
    initial: &[f64],
    max_iter: usize,
    tol: f64,
) -> (IKSolution, Diagnosis) {
    let solution = if target.iter().any(|v| !v.is_finite()) {
        // Con NaN no vale la pena ni intentar.
        let mut q = initial.to_vec();
        q.resize(robot.len(), 0.0);
        IKSolution {
            angles: q,
            error: f64::INFINITY,
            converged: false,
            iterations: 0,
        }
    } else {
        match tool {
            Some(t) => robot.ik_xyz_with_tool(t, target, initial, max_iter, tol),
            None => robot.ik_xyz(target, initial, max_iter, tol),
        }
    };
    let diag = diagnose(robot, tool, target, &solution);
    (solution, diag)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::tip_position;
    use crate::model::demo_arm;

    // Igual que demo_arm, pero con limites del hombro a elegir.
    fn arm_with_shoulder(lo: f64, hi: f64) -> Robot {
        let mut r = Robot::new();
        r.add_revolute((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (-3.14, 3.14));
        r.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (lo, hi));
        r.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-2.0, 2.0));
        r
    }

    #[test]
    fn alcance_maximo_del_demo() {
        let reach = max_reach(&demo_arm(), None);
        assert!((reach - 3.0).abs() < 1e-9, "alcance: {reach}");
    }

    #[test]
    fn alcance_con_herramienta() {
        let tool = Tool::from_offset(0.0, 0.0, 0.5);
        let reach = max_reach(&demo_arm(), Some(&tool));
        assert!((reach - 3.5).abs() < 1e-9, "alcance: {reach}");
    }

    #[test]
    fn el_hombro_es_el_ultimo_punto_fijo() {
        // La base solo gira sobre su eje: el hombro (0,0,1) nunca se mueve.
        let (k, p) = fixed_anchor(&demo_arm());
        assert_eq!(k, 1);
        assert!(dist(&p, &[0.0, 0.0, 1.0]) < 1e-9, "punto fijo: {:?}", p);
    }

    #[test]
    fn objetivo_alcanzable_sin_problemas() {
        let r = demo_arm();
        let p = tip_position(&r, &[0.3, 0.4, -0.5]);
        let (s, d) = r.ik_xyz_explained([p.0, p.1, p.2], &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(s.converged);
        assert!(d.ok(), "{}", d);
    }

    #[test]
    fn fuera_de_alcance() {
        let r = demo_arm();
        let (_, d) = r.ik_xyz_explained([5.0, 0.0, 0.0], &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(!d.converged);
        assert!(
            matches!(d.problems.as_slice(), [Problem::OutOfReach { .. }]),
            "{:?}",
            d.problems
        );
        assert!(d.closest.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn fuera_de_alcance_aunque_este_cerca_de_la_base() {
        // (2.9, 0, 0) esta a 2.9 de la base (menos que 3), pero a 3.07 del
        // hombro, que solo alcanza 2: es imposible llegar.
        let r = demo_arm();
        let (_, d) = r.ik_xyz_explained([2.9, 0.0, 0.0], &[0.1, 0.1, 0.1], 200, 1e-6);
        match d.problems.as_slice() {
            [Problem::OutOfReach { anchor, distance, max_reach }] => {
                assert_eq!(*anchor, 1);
                assert!((distance - (2.9_f64 * 2.9 + 1.0).sqrt()).abs() < 1e-9);
                assert!((max_reach - 2.0).abs() < 1e-9);
            }
            otro => panic!("esperaba OutOfReach, obtuve {:?}", otro),
        }
    }

    #[test]
    fn abajo_de_la_base_tambien_es_fuera_de_alcance() {
        // (0, 0, -1.5) esta a 2.5 del hombro, que solo alcanza 2.
        let r = demo_arm();
        let (_, d) = r.ik_xyz_explained([0.0, 0.0, -1.5], &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(
            matches!(d.problems.as_slice(), [Problem::OutOfReach { anchor: 1, .. }]),
            "{:?}",
            d.problems
        );
    }

    #[test]
    fn al_alcance_pero_no_se_llego() {
        // El objetivo necesita el hombro a 1 rad, pero este robot solo llega a 0.1.
        // Esta dentro del alcance, asi que la version abierta dice NOT_REACHED.
        let libre = arm_with_shoulder(-2.0, 2.0);
        let p = tip_position(&libre, &[0.0, 1.0, 0.0]);
        let r = arm_with_shoulder(-0.1, 0.1);
        let (_, d) = r.ik_xyz_explained([p.0, p.1, p.2], &[0.0, 0.0, 0.0], 200, 1e-6);
        assert!(!d.converged);
        assert!(
            matches!(d.problems.as_slice(), [Problem::NotReached { .. }]),
            "{:?}",
            d.problems
        );
    }

    #[test]
    fn objetivo_invalido() {
        let r = demo_arm();
        let (s, d) = r.ik_xyz_explained([f64::NAN, 0.0, 0.0], &[0.0, 0.0, 0.0], 50, 1e-6);
        assert!(!s.converged);
        assert_eq!(d.problems, vec![Problem::InvalidTarget]);
    }

    #[test]
    fn json_para_el_studio() {
        let r = demo_arm();
        let (_, d) = r.ik_xyz_explained([5.0, 0.0, 0.0], &[0.1, 0.1, 0.1], 100, 1e-6);
        let j = d.to_json();
        assert!(j.contains("\"converged\":false"), "{j}");
        assert!(j.contains("OUT_OF_REACH"), "{j}");
    }
}
