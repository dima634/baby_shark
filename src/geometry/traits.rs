use simba::scalar::ClosedDiv;
use num_traits::{Float, FloatConst, NumCast};

use crate::helpers::aliases::Vec3;

use super::primitives::{box3::Box3, triangle3::Triangle3, plane3::Plane3};

pub trait Number: nalgebra_glm::Number + NumCast + ClosedDiv {}
impl<T> Number for T where T: nalgebra_glm::Number + NumCast + ClosedDiv {}

pub trait RealNumber: nalgebra_glm::RealNumber + Float + FloatConst {}
impl<T> RealNumber for T where T: nalgebra_glm::RealNumber + Float + FloatConst {}

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
    fn closest_point(&self, point: &Vec3<Self::ScalarType>) -> Vec3<Self::ScalarType>;
}

pub trait IntersectsTriangle3: HasScalarType {
    type Output;

    fn intersects_triangle3_at(&self, triangle: &Triangle3<Self::ScalarType>) -> Option<Self::Output>;
}

pub trait IntersectsPlane3: HasScalarType {
    type Output;

    fn intersects_plane3_at(&self, plane: &Plane3<Self::ScalarType>) -> Option<Self::Output>;
}

pub trait Intersects<TPrimitive: HasScalarType>: HasScalarType {
    type Output;

    fn intersects_at(&self, primitive: &TPrimitive) -> Option<Self::Output>;
}
