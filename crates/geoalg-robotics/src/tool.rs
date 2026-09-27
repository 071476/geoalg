// Tool/TCP: la herramienta en la punta del brazo.
// El TCP (Tool Center Point) es el punto real de trabajo: la punta del
// soldador, del taladro o de la pinza. El brazo termina en la "brida"
// y la herramienta anade un desplazamiento y una orientacion.

use geoalg_core::algebra::metric::Metric;
use geoalg_core::ops::sandwich::apply_point;
use geoalg_core::types::motor::Motor;
use geoalg_core::types::point::Point;
use geoalg_core::types::translator::Translator;

// Cinematica directa de un brazo: angulos -> motor de la brida.
pub type ArmFK = fn(&[f64; 3]) -> Motor;

pub struct Tool {
    offset: Motor,
}

impl Tool {
    pub fn new(offset: Motor) -> Self {
        Tool { offset }
    }

    // Herramienta con solo desplazamiento (sin girar).
    pub fn from_offset(x: f64, y: f64, z: f64) -> Self {
        let t = Translator::new(x, y, z);
        Tool { offset: Motor::from_translator(&t) }
    }

    pub fn offset(&self) -> &Motor {
        &self.offset
    }

    // Motor del TCP en el mundo: brida * herramienta.
    pub fn tcp_motor(&self, flange: &Motor) -> Motor {
        let m = Metric::pga();
        Motor::from_dense(flange.inner().geo(&self.offset.inner(), &m))
    }

    // Posicion del TCP en el mundo.
    pub fn tip(&self, flange: &Motor) -> (f64, f64, f64) {
        let tcp = self.tcp_motor(flange);
        apply_point(&tcp.inner(), &Point::origin()).coords().unwrap()
    }
}

// Cinematica inversa: mueve las articulaciones para que el TCP llegue al
// objetivo (el punto real de trabajo, no la brida).
pub fn reach_tcp(
    fk: ArmFK,
    tool: &Tool,
    q0: &[f64; 3],
    target: (f64, f64, f64),
    iters: usize,
) -> [f64; 3] {
    let h = 1e-6;
    let mut q = *q0;
    for _ in 0..iters {
        let p = tool.tip(&fk(&q));
        let err = [target.0 - p.0, target.1 - p.1, target.2 - p.2];
        let mut j = [[0.0; 3]; 3];
        for col in 0..3 {
            let mut qp = q;
            let mut qm = q;
            qp[col] += h;
            qm[col] -= h;
            let a = tool.tip(&fk(&qp));
            let b = tool.tip(&fk(&qm));
            j[0][col] = (a.0 - b.0) / (2.0 * h);
            j[1][col] = (a.1 - b.1) / (2.0 * h);
            j[2][col] = (a.2 - b.2) / (2.0 * h);
        }
        let dq = geoalg_calculus::differential::jacobian::solve3(j, err);
        for i in 0..3 {
            q[i] += dq[i];
        }
    }
    q
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_calculus::differential::jacobian::arm_motor;

    #[test]
    fn tcp_desplaza_la_punta() {
        // Brazo recto: brida en (0, 0, 3). Herramienta de 0.5: TCP en 3.5.
        let tool = Tool::from_offset(0.0, 0.0, 0.5);
        let (x, y, z) = tool.tip(&arm_motor(&[0.0, 0.0, 0.0]));
        assert!(x.abs() < 1e-9 && y.abs() < 1e-9);
        assert!((z - 3.5).abs() < 1e-9, "esperaba 3.5, obtuve {z}");
    }

    #[test]
    fn reach_tcp_llega_con_la_herramienta() {
        let tool = Tool::from_offset(0.0, 0.0, 0.5);
        let target = (0.4, 0.2, 2.0);
        let q = reach_tcp(arm_motor, &tool, &[0.1, 0.1, 0.1], target, 60);
        let (x, y, z) = tool.tip(&arm_motor(&q));
        let e = (x - target.0).powi(2) + (y - target.1).powi(2) + (z - target.2).powi(2);
        assert!(e.sqrt() < 1e-6, "error: {e}");
    }

    #[test]
    fn sin_herramienta_igual_que_brida() {
        let tool = Tool::from_offset(0.0, 0.0, 0.0);
        let flange = arm_motor(&[0.5, -0.3, 0.2]);
        let tip = tool.tip(&flange);
        let fl = apply_point(&flange.inner(), &Point::origin()).coords().unwrap();
        assert!((tip.0 - fl.0).abs() < 1e-12);
    }
}