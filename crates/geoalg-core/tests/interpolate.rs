// Test de integracion: interpolacion de poses como la usaria un programador.
use geoalg_core::{between, exp, log, Line, Motor, Point, Rotor, Translator};
use geoalg_core::ops::sandwich::apply_point;

fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6 && (a.2 - b.2).abs() < 1e-6
}

#[test]
fn interpolacion_extremos() {
    let a = Motor::from_translator(&Translator::new(0.0, 0.0, 0.0));
    let b = Motor::from_translator(&Translator::new(2.0, 4.0, 6.0));
    let p0 = between(&a, &b, 0.0);
    let p1 = between(&a, &b, 1.0);
    let o = Point::origin();
    assert!(close(apply_point(&p0.inner(), &o).coords().unwrap(), (0.0, 0.0, 0.0)));
    assert!(close(apply_point(&p1.inner(), &o).coords().unwrap(), (2.0, 4.0, 6.0)));
}

#[test]
fn interpolacion_medio() {
    let a = Motor::from_translator(&Translator::new(0.0, 0.0, 0.0));
    let b = Motor::from_translator(&Translator::new(2.0, 4.0, 6.0));
    let mid = between(&a, &b, 0.5);
    let p = apply_point(&mid.inner(), &Point::origin()).coords().unwrap();
    assert!(close(p, (1.0, 2.0, 3.0)));
}

#[test]
fn exp_log_roundtrip() {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let m = Rotor::from_axis_angle(&axis, 1.2).to_motor();
    let l = log(&m);
    let back = exp(&l);
    let o = Point::origin();
    assert!(close(
        apply_point(&back.inner(), &o).coords().unwrap(),
        apply_point(&m.inner(), &o).coords().unwrap()
    ));
}

#[test]
fn interpolacion_rotacion() {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let a = Rotor::from_axis_angle(&axis, 0.0).to_motor();
    let b = Rotor::from_axis_angle(&axis, std::f64::consts::FRAC_PI_2).to_motor();
    let mid = between(&a, &b, 0.5);
    // 45 grados sobre Z: (1,0,0) va a (cos45, sin45, 0).
    let p = apply_point(&mid.inner(), &Point::new(1.0, 0.0, 0.0)).coords().unwrap();
    let c = std::f64::consts::FRAC_PI_4.cos();
    let s = std::f64::consts::FRAC_PI_4.sin();
    assert!(close(p, (c, s, 0.0)));
}