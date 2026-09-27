// robot_arm: ejemplo industrial — model, FK, tool, trajectory y limits
// trabajando juntos, como lo usaria un cliente de robótica.

use geoalg::prelude::*;

fn main() {
    // 1. Modelo: brazo de 3 articulaciones (hombro, codo, muñeca).
    let robot = model::demo_arm();
    println!("Robot: {} articulaciones", robot.len());

    // 2. FK: calcular el extremo con angulos dados.
    let q = [0.0, 0.5, -0.3];
    let fk_result = fk::forward(&robot, &q);
    let pos = *fk_result.joints.last().unwrap();
    println!("FK: extremo en ({:.3}, {:.3}, {:.3})", pos.0, pos.1, pos.2);

    // 3. Tool: agregar una herramienta de 0.5 unidades en Z.
    let tool = tool::Tool::from_offset(0.0, 0.0, 0.5);
    let fk_tool = fk::forward_with_tool(&robot, &q, &tool);
    let tcp = *fk_tool.joints.last().unwrap();
    println!("Tool: TCP en ({:.3}, {:.3}, {:.3})", tcp.0, tcp.1, tcp.2);

    // 4. Limits: verificar que la configuracion es valida.
    let valida = limits::within_limits(&robot, &q);
    println!("Limits: configuracion valida = {valida}");

    // 5. Trajectory: mover el robot en linea recta.
    let a = fk::forward(&robot, &[0.0, 0.0, 0.0]);
    let b = fk::forward(&robot, &[1.0, 0.5, -0.5]);
    let mid = Motor::between(&a.tip, &b.tip, 0.5);
    let mid_pos = apply_point(&mid.inner(), &Point::origin()).coords().unwrap();
    println!(
        "Trajectory: pose intermedia en ({:.3}, {:.3}, {:.3})",
        mid_pos.0, mid_pos.1, mid_pos.2
    );

    // 6. Compat: convertir el motor a Mat4 (para Unity/OpenGL).
    let mat = compat::motor_to_matrix4(&mid);
    println!("Compat: Mat4[0][3] = {:.3} (traslacion X)", mat[0][3]);

    println!("\nrobot_arm: todos los modulos funcionan juntos.");
}