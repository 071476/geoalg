// Test de integracion: cinematica directa como la usaria un programador.
use geoalg_robotics::fk;
use geoalg_robotics::model::demo_arm;
use geoalg_robotics::tool::Tool;

#[test]
fn brazo_estirado_llega_arriba() {
    let r = demo_arm();
    let pos = fk::tip_position(&r, &[0.0, 0.0, 0.0]);
    assert!(pos.0.abs() < 1e-9 && pos.1.abs() < 1e-9);
    assert!((pos.2 - 3.0).abs() < 1e-9, "esperaba z=3, obtuve z={}", pos.2);
}

#[test]
fn giro_base_rota_el_extremo() {
    let r = demo_arm();
    let pos = fk::tip_position(&r, &[std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0]);
    assert!(pos.0 > 0.5, "esperaba x>0.5, obtuve x={}", pos.0);
}

#[test]
fn tool_desplaza_el_extremo() {
    let r = demo_arm();
    let tool = Tool::from_offset(0.0, 0.0, 0.5);
    let fk = fk::forward_with_tool(&r, &[0.0, 0.0, 0.0], &tool);
    let (_, _, z) = *fk.joints.last().unwrap();
    assert!((z - 3.5).abs() < 1e-9, "esperaba z=3.5, obtuve z={z}");
}

#[test]
fn cadena_larga_funciona() {
    use geoalg_robotics::model::Robot;
    let mut r = Robot::new();
    for i in 0..6 {
        let z = i as f64;
        r.add_revolute((0.0, 0.0, z), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (-3.14, 3.14));
    }
    let pos = fk::tip_position(&r, &[0.0; 6]);
    assert!((pos.2 - 6.0).abs() < 1e-9, "esperaba z=6, obtuve z={}", pos.2);
}