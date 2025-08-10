use super::primitives::{box3::Box3, plane3::Plane3, triangle3::Triangle3};
use crate::helpers::aliases::Vec3;
use nalgebra::ClosedDivAssign;
use num_traits as nt;

pub use nalgebra::{ComplexField, RealField};
pub use num_traits::{AsPrimitive, Bounded, One, Zero};

pub trait Cast:
    nt::AsPrimitive<i32>
    + nt::AsPrimitive<isize>
    + nt::AsPrimitive<usize>
    + nt::AsPrimitive<f32>
    + nt::AsPrimitive<f64>
    + nt::FromPrimitive
{
    fn u8(value: u8) -> Self {
        Self::from_u8(value).unwrap()
    }

    fn u32(value: u32) -> Self {
        Self::from_u32(value).unwrap()
    }

    fn i32(value: i32) -> Self {
        Self::from_i32(value).unwrap()
    }

    fn isize(value: isize) -> Self {
        Self::from_isize(value).unwrap()
    }

    fn usize(value: usize) -> Self {
        Self::from_usize(value).unwrap()
    }

    fn f32(value: f32) -> Self {
        Self::from_f32(value).unwrap()
    }

    fn f64(value: f64) -> Self {
        Self::from_f64(value).unwrap()
    }
}

impl<T> Cast for T where
    T: nt::AsPrimitive<i32>
        + nt::AsPrimitive<isize>
        + nt::AsPrimitive<usize>
        + nt::AsPrimitive<f32>
        + nt::AsPrimitive<f64>
        + nt::FromPrimitive
{
}

pub trait Number: nalgebra_glm::Number + Cast + ClosedDivAssign {
    fn two() -> Self {
        Self::one() + Self::one()
    }
}
impl<T> Number for T where T: nalgebra_glm::Number + Cast + ClosedDivAssign {}

pub trait RealNumber: nalgebra_glm::RealNumber + Number {
    fn inf() -> Self;
    fn neg_inf() -> Self;
    fn half() -> Self;
}

impl RealNumber for f32 {
    fn inf() -> Self {
        f32::INFINITY
    }

    fn neg_inf() -> Self {
        f32::NEG_INFINITY
    }

    fn half() -> Self {
        0.5
    }
}

impl RealNumber for f64 {
    fn inf() -> Self {
        f64::INFINITY
    }

    fn neg_inf() -> Self {
        f64::NEG_INFINITY
    }

    fn half() -> Self {
        0.5
    }
}

pub trait HasScalarType {
    type Scalar: Number;
}

/// 3D bounding box
pub trait HasBBox3: HasScalarType {
    /// Returns axis aligned bounding box of object
    fn bbox(&self) -> Box3<Self::Scalar>;
}

/// Closest point to primitive query
pub trait ClosestPoint3: HasScalarType {
    /// Returns closest point on primitive to given point
    fn closest_point(&self, point: &Vec3<Self::Scalar>) -> Vec3<Self::Scalar>;
}

pub trait IntersectsTriangle3: HasScalarType {
    type Output;

    fn intersects_triangle3_at(&self, triangle: &Triangle3<Self::Scalar>) -> Option<Self::Output>;
}

pub trait IntersectsPlane3: HasScalarType {
    type Output;

    fn intersects_plane3_at(&self, plane: &Plane3<Self::Scalar>) -> Option<Self::Output>;
}

pub trait Intersects<TPrimitive: HasScalarType>: HasScalarType {
    type Output;

    fn intersects_at(&self, primitive: &TPrimitive) -> Option<Self::Output>;
}
