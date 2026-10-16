// Ejemplo del README: tu propio brazo, con IK y limites.
// Correr con:  cargo run -p geoalg-robotics --example my_arm

use geoalg_robotics::Robot;

fn main() {
    // Each joint: axis through the joint itself, link to the next joint, limits.
    let mut arm = Robot::new();
    arm.add_revolute((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.325, 0.0), (-3.14, 3.14)); // base
    arm.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-2.6, 2.6)); // shoulder
    arm.add_revolute((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.8, 0.0), (-2.8, 2.8)); // elbow

    let (sol, diag) = arm.ik_xyz_explained([0.3, 1.975, 0.3], &[0.0; 3], 200, 1e-6);
    println!("angles: {:?}", sol.angles);
    println!("{}", diag);
}
