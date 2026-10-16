// Ejemplo del README: la IK que explica POR QUE no llego.
// Correr con:  cargo run -p geoalg-robotics --example why_it_failed

use geoalg_robotics::demo_arm;

fn main() {
    let arm = demo_arm(); // 3 joints, links of length 1, shoulder at height 1
    let (sol, diag) = arm.ik_xyz_explained([2.9, 0.0, 0.0], &[0.1; 3], 200, 1e-6);

    if !sol.converged {
        println!("{}", diag);
    }
}
