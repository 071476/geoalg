// Integral temporal: suma cambios pequenos para saber donde estas.

use geoalg_core::ops::screw::exp;
use geoalg_core::types::motor::Motor;
use geoalg_core::multivector::dense::Dense;

// Acumula velocidades (bivectores) a lo largo del tiempo: devuelve la
// trayectoria de motores desde la pose inicial `m0`.
// `velocity` devuelve el bivector de velocidad en el instante `t`.
pub fn integrate_pose(
    m0: Motor,
    velocity: impl Fn(f64) -> Dense,
    t0: f64,
    t1: f64,
    dt: f64,
) -> Vec<Motor> {
    let steps = ((t1 - t0) / dt).ceil() as usize;
    let mut out = Vec::with_capacity(steps + 1);
    let mut m = m0;
    out.push(m);
    for i in 0..steps {
        let t = t0 + dt * i as f64;
        // Un paso pequeño: exp(v * dt) aplicado al motor actual.
        let step = exp(&(velocity(t) * dt));
        m = Motor::from_dense(step.inner().geo(&m.inner(), &geoalg_core::algebra::metric::Metric::pga()));
        out.push(m);
    }
    out
}

// Integral de una funcion escalar f: R -> R (regla del trapecio).
pub fn integrate_scalar(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let mut s = 0.5 * (f(a) + f(b));
    for i in 1..n {
        s += f(a + h * i as f64);
    }
    s * h
}

// Distancia recorrida por un punto a lo largo de una trayectoria de motores.
pub fn path_length(traj: &[Motor], p: (f64, f64, f64)) -> f64 {
    use geoalg_core::ops::sandwich::apply_point;
    use geoalg_core::types::point::Point;

    let pt = Point::new(p.0, p.1, p.2);
    let mut total = 0.0;
    for w in traj.windows(2) {
        let a = apply_point(&w[0].inner(), &pt).coords().unwrap();
        let b = apply_point(&w[1].inner(), &pt).coords().unwrap();
        let d = (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2);
        total += d.sqrt();
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integral_escalar_constante() {
        // f(x) = 2, integral de 0 a 3 = 6
        let s = integrate_scalar(|_| 2.0, 0.0, 3.0, 100);
        assert!((s - 6.0).abs() < 1e-9);
    }

    #[test]
    fn integral_escalar_lineal() {
        // f(x) = x, integral de 0 a 1 = 0.5
        let s = integrate_scalar(|x| x, 0.0, 1.0, 1000);
        assert!((s - 0.5).abs() < 1e-4);
    }

    #[test]
    fn trayectoria_traslacion_recta() {
        // Velocidad constante (1, 0, 0) durante 1 segundo: recorre 1 unidad.
        let v = |_t: f64| {
            let mut d = Dense::zero();
            d.set(geoalg_core::algebra::basis::E0 | geoalg_core::algebra::basis::E1, -0.5);
            d
        };
        let traj = integrate_pose(Motor::identity(), v, 0.0, 1.0, 0.01);
        let len = path_length(&traj, (0.0, 0.0, 0.0));
        assert!((len - 1.0).abs() < 1e-3, "esperaba 1.0, obtuve {len}");
    }
}