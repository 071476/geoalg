use geoalg::prelude::*;

fn main() {
    // El suelo: el plano z = 0
    let ground = Plane::new(0.0, 0.0, 1.0, 0.0);

    // Rayo 1: cae en picado desde (0, 0, 2)
    let ray1 = Line::through_points(&Point::new(0.0, 0.0, 2.0), &Point::new(0.0, 0.0, 0.0));

    // Rayo 2: vuela horizontal desde (0, 0, 2) - paralelo al suelo
    let ray2 = Line::through_points(&Point::new(0.0, 0.0, 2.0), &Point::new(1.0, 0.0, 2.0));

    // Encuentro recta-plano = wedge de sus multivectores
    let hit1 = Point::from_dense(ray1.inner().wedge(&ground.inner()));
    let hit2 = Point::from_dense(ray2.inner().wedge(&ground.inner()));

    match hit1.coords() {
        Some((x, y, z)) => println!("Rayo 1 choca con el suelo en ({x:.2}, {y:.2}, {z:.2})"),
        None => println!("Rayo 1 es paralelo al suelo"),
    }
    match hit2.coords() {
        Some((x, y, z)) => println!("Rayo 2 choca con el suelo en ({x:.2}, {y:.2}, {z:.2})"),
        None => println!("Rayo 2 es paralelo: se cruzan en el infinito"),
    }
}