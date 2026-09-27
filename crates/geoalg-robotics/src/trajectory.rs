// Trayectoria de herramienta: la punta del TCP sigue una curva.
// Ejemplo: una linea de soldadura donde el soldador debe tocar varios
// puntos de una pieza con precision.

use crate::tool::{reach_tcp, ArmFK, Tool};

pub struct ToolPath {
    points: Vec<(f64, f64, f64)>,
}

impl ToolPath {
    pub fn new(points: Vec<(f64, f64, f64)>) -> Self {
        ToolPath { points }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    // Punto interpolado en t (0..=1) del camino completo.
    pub fn point_at(&self, t: f64) -> (f64, f64, f64) {
        let n = self.points.len();
        assert!(n >= 2, "se necesitan al menos 2 puntos");
        let pos = t.clamp(0.0, 1.0) * (n - 1) as f64;
        let i = (pos as usize).min(n - 2);
        let local = pos - i as f64;
        let a = self.points[i];
        let b = self.points[i + 1];
        (
            a.0 + (b.0 - a.0) * local,
            a.1 + (b.1 - a.1) * local,
            a.2 + (b.2 - a.2) * local,
        )
    }

    // Longitud total de la polilinea.
    pub fn length(&self) -> f64 {
        let mut total = 0.0;
        for w in self.points.windows(2) {
            let dx = w[1].0 - w[0].0;
            let dy = w[1].1 - w[0].1;
            let dz = w[1].2 - w[0].2;
            total += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        total
    }

    // Sigue el camino: devuelve los angulos de las articulaciones en cada
    // paso para que el TCP recorra la trayectoria.
    pub fn follow(&self, fk: ArmFK, tool: &Tool, q0: &[f64; 3], steps: usize) -> Vec<[f64; 3]> {
        let mut q = *q0;
        let mut out = Vec::with_capacity(steps + 1);
        out.push(q);
        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let target = self.point_at(t);
            q = reach_tcp(fk, tool, &q, target, 30);
            out.push(q);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_calculus::differential::jacobian::arm_motor;

    #[test]
    fn interpolacion_recta() {
        let path = ToolPath::new(vec![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)]);
        let mid = path.point_at(0.5);
        assert!((mid.0 - 0.5).abs() < 1e-9);
        assert!(mid.1.abs() < 1e-9);
        assert!((mid.2 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn longitud_recta() {
        let path = ToolPath::new(vec![(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]);
        assert!((path.length() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn tcp_sigue_el_camino() {
        let tool = Tool::from_offset(0.0, 0.0, 0.3);
        let path = ToolPath::new(vec![
            (0.0, 0.0, 1.5),
            (0.3, 0.0, 1.5),
            (0.6, 0.0, 1.5),
        ]);
        let joints = path.follow(arm_motor, &tool, &[0.0, 0.3, -0.3], 5);
        let tip = tool.tip(&arm_motor(joints.last().unwrap()));
        let end = path.point_at(1.0);
        let err = (tip.0 - end.0).powi(2) + (tip.1 - end.1).powi(2) + (tip.2 - end.2).powi(2);
        assert!(err.sqrt() < 1e-4, "error: {err}");
    }
}