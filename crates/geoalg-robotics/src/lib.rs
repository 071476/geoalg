pub mod cell;
pub mod fk;
pub mod ik;
pub mod limits;
pub mod model;
pub mod tool;
pub mod trajectory;

// "Menu" de la libreria: lo esencial a mano.
// Asi el usuario puede escribir `use geoalg_robotics::{Robot, IKSolution};`
pub use fk::{forward, forward_with_tool, tip_position, FKResult};
pub use ik::IKSolution;
pub use model::{demo_arm, Joint, JointKind, Robot};
pub use tool::Tool;
