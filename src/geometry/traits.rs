use nalgebra::{Point3, ClosedDiv};
use num_traits::{NumCast, Float};
use super::primitives::Box3;

pub trait Number: nalgebra_glm::Number + NumCast + ClosedDiv {}
impl<T> Number for T where T: nalgebra_glm::Number + NumCast + ClosedDiv {}

pub trait RealNumber: nalgebra_glm::RealNumber + Float {}
impl<T> RealNumber for T where T: nalgebra_glm::RealNumber + Float {}

pub trait HasScalarType {
    type ScalarType: Number;
}

/// 3D bounding box
pub trait HasBBox3: HasScalarType {
    /// Returns axis aligned bounding box of object
    fn bbox(&self) -> Box3<Self::ScalarType>;
}

/// Closest point to primitive query
pub trait ClosestPoint3: HasScalarType {
    /// Returns closest point on primitive to given point 
    fn closest_point(&self, point: &Point3<Self::ScalarType>) -> Point3<Self::ScalarType>;
}
