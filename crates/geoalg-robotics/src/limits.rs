// Limits: limites articulares de un robot.
// Un robot real no puede girar indefinidamente: cada articulacion tiene
// un rango minimo y maximo. Estas funciones verifican y recortan.

use crate::model::Robot;

// Comprueba si una configuracion q esta dentro de los limites del robot.
pub fn within_limits(robot: &Robot, q: &[f64]) -> bool {
    for (i, &qi) in q.iter().enumerate() {
        if i >= robot.len() {
            break;
        }
        let (min, max) = robot.joints[i].limits;
        if qi < min || qi > max {
            return false;
        }
    }
    true
}

// Recorta una configuracion a los limites del robot.
pub fn clamp(robot: &Robot, q: &[f64]) -> Vec<f64> {
    q.iter()
        .enumerate()
        .map(|(i, &qi)| {
            if i >= robot.len() {
                return qi;
            }
            let (min, max) = robot.joints[i].limits;
            qi.max(min).min(max)
        })
        .collect()
}

// Distancia al limite mas cercano: cuanto falta para llegar al borde.
// Sirve para planificar trayectorias que no se acerquen al limite.
pub fn limit_distance(robot: &Robot, q: &[f64]) -> f64 {
    let mut min_dist = f64::INFINITY;
    for (i, &qi) in q.iter().enumerate() {
        if i >= robot.len() {
            break;
        }
        let (min, max) = robot.joints[i].limits;
        let d = (qi - min).min(max - qi);
        if d < min_dist {
            min_dist = d;
        }
    }
    min_dist
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::demo_arm;

    #[test]
    fn dentro_de_limites() {
        let r = demo_arm();
        assert!(within_limits(&r, &[0.0, 0.5, -0.5]));
    }

    #[test]
    fn fuera_de_limites() {
        let r = demo_arm();
        // El hombro (indice 1) tiene limites (-2, 2).
        assert!(!within_limits(&r, &[0.0, 3.0, 0.0]));
    }

    #[test]
    fn clamp_recorta() {
        let r = demo_arm();
        let q = clamp(&r, &[0.0, 5.0, -5.0]);
        assert!((q[1] - 2.0).abs() < 1e-9);
        assert!((q[2] + 2.0).abs() < 1e-9);
    }

    #[test]
    fn distancia_al_limite() {
        let r = demo_arm();
        // Articulacion 1 con limites (-2, 2): q=1.5 => distancia = 0.5.
        let d = limit_distance(&r, &[0.0, 1.5, 0.0]);
        assert!((d - 0.5).abs() < 1e-9, "esperaba 0.5, obtuve {d}");
    }
}