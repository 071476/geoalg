// Derivadas numericas por diferencias finitas.

// Derivada central de una funcion f: R -> R.
pub fn derivative<F: Fn(f64) -> f64>(f: F, x: f64, h: f64) -> f64 {
    (f(x + h) - f(x - h)) / (2.0 * h)
}

// Gradiente de una funcion f: R^n -> R (n = 6 en nuestro caso).
pub fn gradient<F: Fn(&[f64; 6]) -> f64>(f: F, x: &[f64; 6], h: f64) -> [f64; 6] {
    let mut g = [0.0; 6];
    for i in 0..6 {
        let mut xp = *x;
        let mut xm = *x;
        xp[i] += h;
        xm[i] -= h;
        g[i] = (f(&xp) - f(&xm)) / (2.0 * h);
    }
    g
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derivada_de_x2() {
        // f(x) = x^2, f'(3) = 6
        let d = derivative(|x| x * x, 3.0, 1e-6);
        assert!((d - 6.0).abs() < 1e-6);
    }

    #[test]
    fn gradiente_de_suma_cuadrados() {
        // f(x) = x0^2 + ... + x5^2, gradiente = 2x
        let x = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let g = gradient(|v| v.iter().map(|c| c * c).sum(), &x, 1e-6);
        for i in 0..6 {
            assert!((g[i] - 2.0 * x[i]).abs() < 1e-4);
        }
    }
}