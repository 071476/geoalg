// Ejemplo del README: girar y mover en un solo objeto (motor).
// Correr con:  cargo run -p geoalg-core --example motors

use geoalg_core::{apply_point, Line, Motor, Point, Rotor, Translator};
use std::f64::consts::FRAC_PI_2;

fn main() {
    // Turn 90 degrees around the Z axis, then move 1 along X.
    let z_axis = Line::through_points(&Point::origin(), &Point::new(0.0, 0.0, 1.0));
    let turn = Motor::from_rotor(&Rotor::from_axis_angle(&z_axis, FRAC_PI_2));
    let shift = Motor::from_translator(&Translator::new(1.0, 0.0, 0.0));
    let m = turn.then(&shift); // first `turn`, then `shift`

    let p = apply_point(&m.inner(), &Point::new(1.0, 0.0, 0.0));
    println!("{:?}", p.coords()); // about (1, 1, 0)

    // Smooth motion: the motor halfway between "no motion" and `m`.
    let half = Motor::between(&Motor::identity(), &m, 0.5);
    let q = apply_point(&half.inner(), &Point::new(1.0, 0.0, 0.0));
    println!("{:?}", q.coords());
}
