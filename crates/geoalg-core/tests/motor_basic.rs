// Test de integracion: Motors y sandwich como los usaria un programador.
use geoalg_core::{
    apply_point, Line, Motor, Point, Rotor, Translator,
};
use std::f64::consts::FRAC_PI_2;

fn close(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 && (a.2 - b.2).abs() < 1e-9
}

#[test]
fn traslacion_mueve_puntos() {
    let t = Motor::from_translator(&Translator::new(1.0, 2.0, 3.0));
    let p = apply_point(&t.inner(), &Point::origin());
    assert!(close(p.coords().unwrap(), (1.0, 2.0, 3.0)));
}

#[test]
fn rotacion_gira_puntos() {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let r = Motor::from_rotor(&Rotor::from_axis_angle(&axis, FRAC_PI_2));
    let p = apply_point(&r.inner(), &Point::new(1.0, 0.0, 0.0));
    assert!(close(p.coords().unwrap(), (0.0, 1.0, 0.0)));
}

#[test]
fn composicion_de_motor() {
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let rot = Motor::from_rotor(&Rotor::from_axis_angle(&axis, FRAC_PI_2));
    let tra = Motor::from_translator(&Translator::new(1.0, 0.0, 0.0));
    let m = rot.then(&tra);
    let p = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0));
    // Gira (1,0,0) a (0,1,0), luego traslada +X: (1,1,0).
    assert!(close(p.coords().unwrap(), (1.0, 1.0, 0.0)));
}

#[test]
fn inverso_deshace() {
    let axis = Line::through_points(&Point::origin(), &Point::new(1.0, 1.0, 0.0));
    let rot = Motor::from_rotor(&Rotor::from_axis_angle(&axis, 0.7));
    let tra = Motor::from_translator(&Translator::new(-2.0, 0.5, 3.0));
    let m = rot.then(&tra);
    let p = Point::new(1.0, 2.0, 3.0);
    let q = apply_point(&m.inner(), &p);
    let back = apply_point(&m.inverse().inner(), &q);
    assert!(close(back.coords().unwrap(), p.coords().unwrap()));
}