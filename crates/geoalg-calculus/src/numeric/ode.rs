// Integradores de ecuaciones diferenciales (Euler y RK4).
// El estado es un vector de 6 numeros: sirve para angulos de
// articulaciones o para los 6 parametros de un motor.

pub type State = [f64; 6];

// Un paso de Euler simple: y' = f(t, y).
pub fn euler<F: Fn(f64, &State) -> State>(f: F, t: f64, y: &State, dt: f64) -> State {
    let k = f(t, y);
    let mut out = *y;
    for i in 0..6 {
        out[i] += dt * k[i];
    }
    out
}

// Un paso de Runge-Kutta de orden 4 (mucho mas preciso).
pub fn rk4<F: Fn(f64, &State) -> State>(f: F, t: f64, y: &State, dt: f64) -> State {
    let half = dt * 0.5;

    let k1 = f(t, y);

    let mut y2 = *y;
    for i in 0..6 {
        y2[i] += half * k1[i];
    }
    let k2 = f(t + half, &y2);

    let mut y3 = *y;
    for i in 0..6 {
        y3[i] += half * k2[i];
    }
    let k3 = f(t + half, &y3);

    let mut y4 = *y;
    for i in 0..6 {
        y4[i] += dt * k3[i];
    }
    let k4 = f(t + dt, &y4);

    let mut out = *y;
    for i in 0..6 {
        out[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    out
}

// Simula desde t0 hasta t1 con pasos de dt, devolviendo todos los estados.
pub fn simulate<F: Fn(f64, &State) -> State>(
    f: F,
    y0: State,
    t0: f64,
    t1: f64,
    dt: f64,
    use_rk4: bool,
) -> Vec<State> {
    let steps = ((t1 - t0) / dt).ceil() as usize;
    let mut out = Vec::with_capacity(steps + 1);
    let mut y = y0;
    let mut t = t0;
    out.push(y);
    for _ in 0..steps {
        y = if use_rk4 {
            rk4(&f, t, &y, dt)
        } else {
            euler(&f, t, &y, dt)
        };
        t += dt;
        out.push(y);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_con_constante() {
        // y' = 1 => y crece lineal
        let y = euler(|_, _| [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, &[0.0; 6], 0.1);
        assert!((y[0] - 0.1).abs() < 1e-12);
    }

        #[test]
    fn rk4_exponencial() {
        // y' = y, y(0) = 1 => y(t) = e^t. En t = 1 vale e.
        let traj = simulate(|_, s| [s[0], 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 1.0, 0.01, true);
        let y = traj.last().unwrap();
        assert!((y[0] - std::f64::consts::E).abs() < 1e-4);
    }
    
    

    #[test]
    fn simular_linea() {
        let traj = simulate(|_, _| [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6], 0.0, 1.0, 0.1, true);
        assert_eq!(traj.len(), 11);
        assert!((traj[10][0] - 1.0).abs() < 1e-9);
    }
}