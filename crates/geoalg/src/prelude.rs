pub use crate::algebras::{cga3, euclidean3, minkowski, pga3};

// Nucleo: algebra y multivectores.
pub use geoalg_core::algebra::metric::Metric;
pub use geoalg_core::multivector::dense::Dense;

// Geometria.
pub use geoalg_core::types::line::Line;
pub use geoalg_core::types::motor::Motor;
pub use geoalg_core::types::plane::Plane;
pub use geoalg_core::types::point::Point;
pub use geoalg_core::types::rotor::Rotor;
pub use geoalg_core::types::sphere::Sphere;
pub use geoalg_core::types::translator::Translator;

// Operaciones.
pub use geoalg_core::ops::projection::*;
pub use geoalg_core::ops::sandwich::*;
pub use geoalg_core::ops::screw::{between, exp, log, screw_axis};

// Compatibilidad con el mundo exterior.
pub use geoalg_core::compat;