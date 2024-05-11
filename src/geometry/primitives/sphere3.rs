use super::box3::Box3;
use crate::{
    geometry::traits::{HasBBox3, HasScalarType, RealNumber},
    helpers::aliases::Vec3,
};

/// 3D sphere
pub struct Sphere3<TScalar: RealNumber> {
    center: Vec3<TScalar>,
    radius: TScalar,
}

impl<TScalar: RealNumber> Sphere3<TScalar> {
    pub fn new(center: Vec3<TScalar>, radius: TScalar) -> Self {
        Self { center, radius }
    }

    #[inline]
    pub fn intersects_box3(&self, bbox: &Box3<TScalar>) -> bool {
        bbox.squared_distance(&self.center) <= self.radius * self.radius
    }
}

impl<TScalar: RealNumber> HasScalarType for Sphere3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> HasBBox3 for Sphere3<TScalar> {
    #[inline]
    fn bbox(&self) -> Box3<Self::ScalarType> {
        Box3::new(
            self.center.add_scalar(-self.radius),
            self.center.add_scalar(self.radius),
        )
    }
}
