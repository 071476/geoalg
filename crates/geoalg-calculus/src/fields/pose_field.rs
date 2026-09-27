// Campo de poses: asigna un motor a cada punto del espacio/tiempo.
// Ejemplos: campo de viento, campo de fuerzas, trayectoria de camara.

use geoalg_core::types::motor::Motor;

// Un campo es una funcion que devuelve un motor para cada (x, y, z, t).
pub type PoseField = Box<dyn Fn(f64, f64, f64, f64) -> Motor>;


// Muestrea el campo sobre una malla 3D y devuelve los motores.
pub fn sample(
    field: &dyn Fn(f64, f64, f64, f64) -> Motor,

    x0: f64, x1: f64,
    y0: f64, y1: f64,
    z0: f64, z1: f64,
    t: f64,
    n: usize,
) -> Vec<((f64, f64, f64), Motor)> {
    let mut out = Vec::with_capacity(n * n * n);
    let dx = (x1 - x0) / (n - 1).max(1) as f64;
    let dy = (y1 - y0) / (n - 1).max(1) as f64;
    let dz = (z1 - z0) / (n - 1).max(1) as f64;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let x = x0 + dx * i as f64;
                let y = y0 + dy * j as f64;
                let z = z0 + dz * k as f64;
                out.push(((x, y, z), field(x, y, z, t)));
            }
        }
    }
    out
}

// Campo constante: el mismo motor en todos los puntos.
pub fn constant(m: Motor) -> impl Fn(f64, f64, f64, f64) -> Motor {
    move |_, _, _, _| m
}

// Campo que mezcla dos campos segun el tiempo (t en [0, 1]).
 pub fn blend(
    a: impl Fn(f64, f64, f64, f64) -> Motor,
    b: impl Fn(f64, f64, f64, f64) -> Motor,
) -> impl Fn(f64, f64, f64, f64) -> Motor {
    use geoalg_core::ops::screw::between;
    move |x, y, z, t| {
        let ma = a(x, y, z, t);
        let mb = b(x, y, z, t);
        between(&ma, &mb, t.clamp(0.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geoalg_core::types::point::Point;
    use geoalg_core::types::translator::Translator;
    use geoalg_core::ops::sandwich::apply_point;

    #[test]
    fn campo_constante() {
        let t = Translator::new(1.0, 0.0, 0.0).to_motor();
        let f = constant(t);
        let m = f(5.0, 5.0, 5.0, 0.0);
        let p = apply_point(&m.inner(), &Point::origin()).coords().unwrap();
        assert!((p.0 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn malla_3d() {
        let t = Motor::identity();
        let f = constant(t);
        let pts = sample(&f, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 0.0, 3);
        assert_eq!(pts.len(), 27);
    }

    #[test]
    fn blend_intermedio() {
        let ta = Motor::identity();
        let tb = Translator::new(2.0, 0.0, 0.0).to_motor();
        let f = blend(constant(ta), constant(tb));
        let m = f(0.0, 0.0, 0.0, 0.5);
        let p = apply_point(&m.inner(), &Point::origin()).coords().unwrap();
        assert!((p.0 - 1.0).abs() < 1e-6, "esperaba x=1, obtuve x={}", p.0);
    }
}