use nalgebra::{Point3, Vector3};
use num_traits::Float;

use crate::geometry::traits::{RealNumber, HasScalarType, ClosestPoint3};

use super::box3::Box3;

/// n * x - d = 0
pub struct Plane3<TScalar: RealNumber> {
    normal: Vector3<TScalar>,
    distance: TScalar
}

impl<TScalar: RealNumber> Plane3<TScalar> {
    pub fn new(normal: Vector3<TScalar>, d: TScalar) -> Self { 
        return Self { normal, distance: d };
    }

    /// Given three noncollinear points (ordered ccw), compute plane equation
    pub fn from_points(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> Self {
        let normal = (b - a).cross(&(c - a)).normalize();
        let d = normal.dot(&a.coords);

        return Self { normal, distance: d };
    }

    #[inline]
    pub fn get_normal(&self) -> &Vector3<TScalar> {
        return &self.normal;
    }

    #[inline]
    pub fn get_distance(&self) -> TScalar {
        return self.distance;
    }

    /// Returns signed distance from point to plane
    #[inline]
    pub fn distance(&self, point: &Point3<TScalar>) -> TScalar {
        return (self.normal.dot(&point.coords) - self.distance) / self.normal.dot(&self.normal); 
    }

    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        // These two lines not necessary with a (center, extents) AABB representation
        let c = aabb.get_center();
        let e = aabb.get_max() - c;
        // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
        let r = e[0]*Float::abs(self.normal[0]) + e[1]*Float::abs(self.normal[1]) + e[2]*Float::abs(self.normal[2]);
        // Compute distance of box center from plane
        let s = self.normal.dot(&c.coords) - self.distance;
        // Intersection occurs when distance s falls within [-r,+r] interval
        return Float::abs(s) <= r
    }
}

impl<TScalar: RealNumber> HasScalarType for Plane3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Plane3<TScalar> {
    /// Returns closest point on plane to given point
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let t = self.distance(point);
        return point - self.normal.scale(t); 
    }
}
