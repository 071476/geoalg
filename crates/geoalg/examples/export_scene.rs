use geoalg::prelude::*;
use std::fmt::Write as _;

fn main() {
    // La escena: el suelo z = 0 y dos rayos que caen desde (0, 0, 2)
    let ground = Plane::new(0.0, 0.0, 1.0, 0.0);
    let ray1 = Line::through_points(&Point::new(0.0, 0.0, 2.0), &Point::new(0.0, 0.0, 0.0));
    let ray2 = Line::through_points(&Point::new(0.0, 0.0, 2.0), &Point::new(1.0, 0.0, 2.0));

    // El encuentro recta-plano lo calcula geoalg (wedge)
    let hit1 = Point::from_dense(ray1.inner().wedge(&ground.inner()));
    let hit2 = Point::from_dense(ray2.inner().wedge(&ground.inner()));

    let mut out = String::new();
    writeln!(out, "# geoalg scene").unwrap();
    writeln!(out, "plane {} {} {} {}", ground.a(), ground.b(), ground.c(), ground.d()).unwrap();

    let (x, y, z) = hit1.coords().expect("ray1 debe chocar con el suelo");
    writeln!(out, "segment ray1 0 0 2 {x} {y} {z}").unwrap();
    writeln!(out, "point hit1 {x} {y} {z}").unwrap();

    writeln!(out, "segment ray2 0 0 2 1 0 2").unwrap();
    if hit2.coords().is_none() {
        writeln!(out, "label ray2 ideal").unwrap();
    }

    std::fs::write("scene.txt", out).expect("no se pudo escribir scene.txt");
    println!("Escena exportada a scene.txt");
    println!("Rayo 1 choca en ({x:.2}, {y:.2}, {z:.2})");
    if hit2.coords().is_none() {
        println!("Rayo 2 es paralelo al suelo (punto ideal)");
    }
}