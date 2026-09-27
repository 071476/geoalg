pub mod algebra;
pub mod multivector;
pub mod ops;
pub mod types;
pub mod compat;

// Exports públicos limpios: lo esencial a mano.
pub use algebra::basis::{Blade, E0, E1, E2, E3, I};
pub use algebra::metric::Metric;
pub use algebra::product::{
    anticommutator, commutator, geo, left_contract, right_contract, scalar_product, wedge,
};
pub use multivector::dense::Dense;
pub use ops::dual::{dual, undual};
pub use ops::projection::{project_point_on_line, project_point_on_plane, project_point_on_sphere};
pub use ops::sandwich::{
    apply_line, apply_plane, apply_point, apply_sphere, reflect_point_in_line,
    reflect_point_in_plane, sandwich,
};
pub use ops::screw::{between, exp, log, screw_axis};
pub use types::line::Line;
pub use types::motor::Motor;
pub use types::plane::Plane;
pub use types::point::Point;
pub use types::rotor::Rotor;
pub use types::sphere::Sphere;
pub use types::translator::Translator;