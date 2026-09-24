use geoalg::prelude::*;
use std::f64::consts::PI;

fn main() {
    // 1. Crear un movimiento: girar 90 grados sobre el eje Z y luego trasladar (1, 0, 0)
    let axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let rotation = Motor::from_rotor(&Rotor::from_axis_angle(&axis, PI / 2.0));
    let translation = Motor::from_translator(&Translator::new(1.0, 0.0, 0.0));
    let movement = rotation.then(&translation);

    // 2. El mismo movimiento transforma puntos
    let p = Point::new(1.0, 0.0, 0.0);
    let p2 = apply_point(&movement.inner(), &p);
    let (x, y, z) = p2.coords().unwrap();
    println!("Punto (1, 0, 0) se mueve a ({x:.2}, {y:.2}, {z:.2})");

    // 3. ... y tambien esferas (el radio se conserva)
    let ball = Sphere::new(Point::origin(), 0.5);
    let ball2 = apply_sphere(&movement.inner(), &ball);
    let (cx, cy, cz) = ball2.center().coords().unwrap();
    println!("Esfera r={:.2} con centro en ({cx:.2}, {cy:.2}, {cz:.2})", ball2.radius());

    // 4. El inverso deshace el movimiento
    let back = apply_point(&movement.inverse().inner(), &p2);
    let (bx, by, bz) = back.coords().unwrap();
    println!("Y de vuelta: ({bx:.2}, {by:.2}, {bz:.2})");
}