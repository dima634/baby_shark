use nalgebra::Point3;
use crate::mesh::traits::Floating;
use super::primitives::Box3;

pub trait HasScalarType {
    type ScalarType: Floating;
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
