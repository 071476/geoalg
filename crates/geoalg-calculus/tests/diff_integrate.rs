// Test de integracion: derivar e integrar como lo usaria un programador.
use geoalg_calculus::numeric::finite_diff;
use geoalg_calculus::integral::time_integral;

#[test]
fn derivada_de_una_funcion() {
    // f(x) = x^2, f'(3) = 6.
    let f = |x: f64| x * x;
    let d = finite_diff::derivative(f, 3.0, 1e-6);
    assert!((d - 6.0).abs() < 1e-3, "esperaba 6, obtuve {d}");
}

#[test]
fn gradiente_de_una_funcion() {
    // f(x) = suma de cuadrados, gradiente en (2, 3, 0, 0, 0, 0) = (4, 6, 0, 0, 0, 0).
    let f = |v: &[f64; 6]| v.iter().map(|x| x * x).sum::<f64>();
    let x = [2.0, 3.0, 0.0, 0.0, 0.0, 0.0];
    let g = finite_diff::gradient(f, &x, 1e-6);
    assert!((g[0] - 4.0).abs() < 1e-3, "gx: {}", g[0]);
    assert!((g[1] - 6.0).abs() < 1e-3, "gy: {}", g[1]);
}

#[test]
fn integral_de_constante() {
    // Integral de 2.0 de 0 a 3 = 6.0.
    let s = time_integral::integrate_scalar(|_| 2.0, 0.0, 3.0, 1);
    assert!((s - 6.0).abs() < 1e-9, "esperaba 6, obtuve {s}");
}

#[test]
fn integral_de_funcion_lineal() {
    // Integral de v(t) = t de 0 a 2 = 2.0.
    let s = time_integral::integrate_scalar(|t| t, 0.0, 2.0, 1000);
    assert!((s - 2.0).abs() < 0.01, "esperaba 2, obtuve {s}");
}