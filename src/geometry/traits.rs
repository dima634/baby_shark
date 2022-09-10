use nalgebra::Point3;
use crate::mesh::traits::Floating;
use super::primitives::Box3;

/// 3D bounding box
pub trait HasBBox3 {
    type ScalarType: Floating;

    /// Returns axis aligned bounding box of object
    fn bbox(&self) -> Box3<Self::ScalarType>;
}

/// Closest point to primitive query
pub trait ClosestPoint3<TScalar: Floating> {
    /// Returns closest point on primitive to given point 
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar>;
}
