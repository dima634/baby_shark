use nalgebra::Vector3;

use crate::{geometry::traits::{RealNumber, HasScalarType, ClosestPoint3}, helpers::aliases::Vec3};

use super::{line3::Line3, plane3::Plane3, box3::Box3};

/// 3D ray
pub struct Ray3<TScalar: RealNumber> { 
    line: Line3<TScalar> 
}

impl<TScalar: RealNumber> Ray3<TScalar> {
    pub fn new(point: Vec3<TScalar>, direction: Vector3<TScalar>) -> Self {
        Self { line: Line3::new(point, direction) }
    }

    #[inline]
    pub fn get_origin(&self) -> &Vec3<TScalar> {
        return self.line.get_point();
    }

    #[inline]
    pub fn get_direction(&self) -> &Vec3<TScalar> {
        return self.line.get_direction();
    }

    #[inline]
    pub fn get_line(&self) -> &Line3<TScalar> {
        &self.line
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_plane3_at(plane) {
            return self.is_on_ray(t);
        }

        None
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        self.intersects_plane3_at(plane).is_some()
    }

    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_box3_at(aabb) {
            return self.is_on_ray(t);
        }

        None
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        self.intersects_box3_at(aabb).is_some()
    }  

    fn is_on_ray(&self, t: TScalar) -> Option<TScalar> {
        if t < TScalar::zero() {
            return None;
        }

        Some(t)
    }
}

impl<TScalar: RealNumber> HasScalarType for Ray3<TScalar> {
    type Scalar = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Ray3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Vec3<TScalar>) -> Vec3<TScalar> {
        let mut t = self.line.parameter_at(point);

        if t < TScalar::zero() {
            t = TScalar::zero();
        }

        return self.line.get_point() + self.line.get_direction().scale(t);
    }
}
