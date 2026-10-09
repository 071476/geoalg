//! IK gratis: cinematica inversa por minimos cuadrados amortiguados (DLS).
//!
//! Dado un punto objetivo, busca los angulos para que la punta del brazo
//! (o de la herramienta) llegue ahi. Funciona con cualquier `Robot`,
//! con cualquier numero de articulaciones, y respeta sus limites.
//!
//! Version gratis: amortiguamiento simple + reintentos desde varios puntos.
//! La version Pro (geoalg-premium) agrega Levenberg-Marquardt adaptativo
//! y diagnosticos que explican por que no se llego al objetivo.

use crate::fk::{forward, forward_with_tool};
use crate::model::Robot;
use crate::tool::Tool;

/// Resultado de la IK. Misma forma que en geoalg-premium, para que el
/// usuario pueda pasar de gratis a Pro sin cambiar su codigo.
#[derive(Clone, Debug)]
pub struct IKSolution {
    /// Angulo (o desplazamiento) de cada articulacion.
    pub angles: Vec<f64>,
    /// Distancia final entre la punta y el objetivo.
    pub error: f64,
    /// true si el error quedo por debajo de la tolerancia.
    pub converged: bool,
    /// Vueltas usadas en total (sumando los reintentos).
    pub iterations: usize,
}

/// Paso maximo por vuelta (radianes o unidades). Evita saltos bruscos.
const MAX_STEP: f64 = 0.5;
/// Amortiguamiento: alto lejos del objetivo (estable), bajo cerca (preciso).
const DAMPING_MAX: f64 = 0.05;
const DAMPING_MIN: f64 = 1e-3;

impl Robot {
    /// IK de posicion: lleva la punta del brazo al punto `target`.
    pub fn ik_xyz(&self, target: [f64; 3], initial: &[f64], max_iter: usize, tol: f64) -> IKSolution {
        solve(self, None, target, initial, max_iter, tol)
    }

    /// IK de posicion con herramienta: lleva la punta de la HERRAMIENTA
    /// (el TCP: punta del soldador, de la pinza...) al punto `target`.
    pub fn ik_xyz_with_tool(
        &self,
        tool: &Tool,
        target: [f64; 3],
        initial: &[f64],
        max_iter: usize,
        tol: f64,
    ) -> IKSolution {
        solve(self, Some(tool), target, initial, max_iter, tol)
    }
}

// Posicion de la punta (con o sin herramienta).
fn tip(robot: &Robot, tool: Option<&Tool>, q: &[f64]) -> [f64; 3] {
    let p = match tool {
        Some(t) => *forward_with_tool(robot, q, t).joints.last().unwrap(),
        None => *forward(robot, q).joints.last().unwrap(),
    };
    [p.0, p.1, p.2]
}

fn dist(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

// Recorta cada articulacion a sus limites.
fn clamp_to_limits(robot: &Robot, q: &mut [f64]) {
    for (i, j) in robot.joints.iter().enumerate() {
        let (lo, hi) = j.limits;
        if lo <= hi && q[i].is_finite() {
            q[i] = q[i].clamp(lo, hi);
        }
    }
}

// Puntos de partida: el que dio el usuario y cuatro mas repartidos
// dentro de los limites. Si uno se atora, se prueba el siguiente.
fn seeds(robot: &Robot, initial: &[f64]) -> Vec<Vec<f64>> {
    let n = robot.len();
    let mut first = initial.to_vec();
    first.resize(n, 0.0);
    let mut out = vec![first];
    for f in [0.25, -0.25, 0.4, -0.4] {
        let s: Vec<f64> = robot
            .joints
            .iter()
            .map(|j| {
                let (lo, hi) = j.limits;
                (lo + hi) / 2.0 + (hi - lo) * f
            })
            .collect();
        out.push(s);
    }
    for s in out.iter_mut() {
        clamp_to_limits(robot, s);
    }
    out
}

// Un intento de IK desde un punto de partida.
fn attempt(
    robot: &Robot,
    tool: Option<&Tool>,
    target: &[f64; 3],
    start: Vec<f64>,
    max_iter: usize,
    tol: f64,
) -> IKSolution {
    let n = robot.len();
    let mut q = start;
    let mut iters = 0;

    while iters < max_iter {
        let p = tip(robot, tool, &q);
        let e = [target[0] - p[0], target[1] - p[1], target[2] - p[2]];
        let en = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt();
        if en < tol {
            return IKSolution { angles: q, error: en, converged: true, iterations: iters };
        }

        // Jacobiano numerico (3 x n): como cambia la punta al mover cada articulacion.
        let h = 1e-6;
        let mut jac = vec![[0.0_f64; 3]; n];
        for k in 0..n {
            let mut qp = q.clone();
            let mut qm = q.clone();
            qp[k] += h;
            qm[k] -= h;
            let a = tip(robot, tool, &qp);
            let b = tip(robot, tool, &qm);
            for i in 0..3 {
                jac[k][i] = (a[i] - b[i]) / (2.0 * h);
            }
        }

        // dq = Jt * (J*Jt + lambda^2 * I)^-1 * e
        let lam = en.clamp(DAMPING_MIN, DAMPING_MAX);
        let mut a = [[0.0_f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for col in jac.iter() {
                    s += col[i] * col[j];
                }
                if i == j {
                    s += lam * lam;
                }
                a[i][j] = s;
            }
        }
        let y = solve3(a, e);
        let mut dq: Vec<f64> = jac
            .iter()
            .map(|col| col[0] * y[0] + col[1] * y[1] + col[2] * y[2])
            .collect();

        // Nunca dar un paso demasiado grande.
        let biggest = dq.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if !biggest.is_finite() {
            break;
        }
        if biggest > MAX_STEP {
            for v in dq.iter_mut() {
                *v *= MAX_STEP / biggest;
            }
        }
        for k in 0..n {
            q[k] += dq[k];
        }
        clamp_to_limits(robot, &mut q);
        iters += 1;
    }

    let p = tip(robot, tool, &q);
    let en = dist(target, &p);
    IKSolution { angles: q, error: en, converged: en < tol, iterations: iters }
}

fn solve(
    robot: &Robot,
    tool: Option<&Tool>,
    target: [f64; 3],
    initial: &[f64],
    max_iter: usize,
    tol: f64,
) -> IKSolution {
    // Robot sin articulaciones: no hay nada que mover.
    if robot.is_empty() {
        let p = tip(robot, tool, &[]);
        let en = dist(&target, &p);
        return IKSolution { angles: Vec::new(), error: en, converged: en < tol, iterations: 0 };
    }

    let mut best: Option<IKSolution> = None;
    let mut total = 0;
    for s in seeds(robot, initial) {
        let r = attempt(robot, tool, &target, s, max_iter, tol);
        total += r.iterations;
        let done = r.converged;
        let better = match &best {
            None => true,
            Some(b) => r.error < b.error,
        };
        if better {
            best = Some(r);
        }
        if done {
            break;
        }
    }
    // `seeds` siempre devuelve al menos un punto de partida.
    let mut best = best.unwrap();
    best.iterations = total;
    best
}

// Evita dividir entre cero SIN perder el signo del pivote.
fn safe_pivot(p: f64) -> f64 {
    if p.abs() < 1e-12 {
        if p < 0.0 { -1e-12 } else { 1e-12 }
    } else {
        p
    }
}

// Resuelve un sistema 3x3 con eliminacion gaussiana y pivoteo.
fn solve3(mut m: [[f64; 3]; 3], mut r: [f64; 3]) -> [f64; 3] {
    for c in 0..3 {
        let mut mx = c;
        for k in c + 1..3 {
            if m[k][c].abs() > m[mx][c].abs() {
                mx = k;
            }
        }
        m.swap(c, mx);
        r.swap(c, mx);
        let pv = safe_pivot(m[c][c]);
        for k in c + 1..3 {
            let f = m[k][c] / pv;
            for j in c..3 {
                m[k][j] -= f * m[c][j];
            }
            r[k] -= f * r[c];
        }
    }
    let mut x = [0.0_f64; 3];
    for i in (0..3).rev() {
        let mut s = r[i];
        for j in i + 1..3 {
            s -= m[i][j] * x[j];
        }
        x[i] = s / safe_pivot(m[i][i]);
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::tip_position;
    use crate::model::demo_arm;

    fn tip3(r: &Robot, q: &[f64]) -> [f64; 3] {
        let p = tip_position(r, q);
        [p.0, p.1, p.2]
    }

    #[test]
    fn llega_a_objetivo_alcanzable() {
        let r = demo_arm();
        let t = tip3(&r, &[0.3, 0.4, -0.5]);
        let s = r.ik_xyz(t, &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(s.converged, "no convergio: error={}, iters={}", s.error, s.iterations);
        // Comprobacion independiente: la FK de la respuesta llega al objetivo.
        let p = tip3(&r, &s.angles);
        assert!(dist(&p, &t) < 1e-6, "la punta quedo en {:?}, objetivo {:?}", p, t);
    }

    #[test]
    fn arranca_con_el_brazo_estirado() {
        // Brazo totalmente recto: el caso mas dificil para el Jacobiano.
        let r = demo_arm();
        let t = tip3(&r, &[0.2, 0.7, 0.6]);
        let s = r.ik_xyz(t, &[0.0, 0.0, 0.0], 200, 1e-6);
        assert!(s.converged, "no convergio: error={}", s.error);
    }

    #[test]
    fn con_herramienta() {
        let r = demo_arm();
        let tool = Tool::from_offset(0.0, 0.0, 0.5);
        let p = *forward_with_tool(&r, &[0.5, -0.3, 0.8], &tool).joints.last().unwrap();
        let t = [p.0, p.1, p.2];
        let s = r.ik_xyz_with_tool(&tool, t, &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(s.converged, "no convergio con herramienta: error={}", s.error);
    }

    #[test]
    fn fuera_de_alcance_no_converge() {
        // El hombro esta en (0,0,1) y alcanza 2: el punto (5,0,0) queda lejos.
        let r = demo_arm();
        let s = r.ik_xyz([5.0, 0.0, 0.0], &[0.1, 0.1, 0.1], 200, 1e-6);
        assert!(!s.converged, "no deberia decir que llego");
        assert!(s.error.is_finite() && s.error >= 3.0, "error: {}", s.error);
        assert!(s.angles.iter().all(|a| a.is_finite()), "angulos: {:?}", s.angles);
    }

    #[test]
    fn muchos_objetivos_y_respeta_limites() {
        // 216 objetivos repartidos por todo el espacio de trabajo.
        let r = demo_arm();
        let mut ok = 0;
        let mut total = 0;
        for a in 0..6 {
            for b in 0..6 {
                for c in 0..6 {
                    let q = [-3.0 + a as f64 * 1.2, -1.8 + b as f64 * 0.7, -1.8 + c as f64 * 0.7];
                    let t = tip3(&r, &q);
                    let s = r.ik_xyz(t, &[0.1, 0.1, 0.1], 200, 1e-6);
                    total += 1;
                    if s.converged {
                        ok += 1;
                    }
                    for (v, j) in s.angles.iter().zip(r.joints.iter()) {
                        let (lo, hi) = j.limits;
                        assert!(*v >= lo - 1e-12 && *v <= hi + 1e-12, "angulo {v} fuera de ({lo}, {hi})");
                    }
                }
            }
        }
        assert!(ok >= 212, "solo convergieron {ok} de {total}");
    }

    #[test]
    fn robot_vacio() {
        let r = Robot::new();
        assert!(r.ik_xyz([0.0, 0.0, 0.0], &[], 10, 1e-6).converged);
        assert!(!r.ik_xyz([1.0, 0.0, 0.0], &[], 10, 1e-6).converged);
    }
}
