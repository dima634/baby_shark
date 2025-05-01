use nalgebra::{Point3, Vector3};
use num_traits::Float;

use crate::{geometry::traits::{
    RealNumber, 
    HasScalarType, 
    ClosestPoint3, 
    Number, 
    IntersectsPlane3
}, helpers::aliases::Vec3};

use super::{box3::Box3, line3::Line3};

/// n * x - d = 0
pub struct Plane3<TScalar: Number> {
    normal: Vector3<TScalar>,
    distance: TScalar
}

impl<TScalar: RealNumber> Plane3<TScalar> {
    pub fn new(normal: Vector3<TScalar>, d: TScalar) -> Self { 
        Self { normal, distance: d }
    }

    /// Given three noncollinear points (ordered ccw), compute plane equation
    pub fn from_points(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> Self {
        let normal = (b - a).cross(&(c - a)).normalize();
        let d = normal.dot(&a.coords);

        Self { normal, distance: d }
    }

    #[inline]
    pub fn get_normal(&self) -> &Vector3<TScalar> {
        &self.normal
    }

    #[inline]
    pub fn get_distance(&self) -> TScalar {
        self.distance
    }

    /// Returns signed distance from point to plane
    #[inline]
    pub fn distance_to_point(&self, point: &Vec3<TScalar>) -> TScalar {
        (self.normal.dot(point) - self.distance) / self.normal.dot(&self.normal)
    }

    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        // These two lines not necessary with a (center, extents) AABB representation
        let c = aabb.get_center();
        let e = aabb.get_max() - c;
        // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
        let r = e[0]*Float::abs(self.normal[0]) + e[1]*Float::abs(self.normal[1]) + e[2]*Float::abs(self.normal[2]);
        // Compute distance of box center from plane
        let s = self.normal.dot(&c) - self.distance;
        // Intersection occurs when distance s falls within [-r,+r] interval
        Float::abs(s) <= r
    }
}

impl<TScalar: RealNumber> HasScalarType for Plane3<TScalar> {
    type Scalar = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Plane3<TScalar> {
    /// Returns closest point on plane to given point
    #[inline]
    fn closest_point(&self, point: &Vec3<TScalar>) -> Vec3<TScalar> {
        let t = self.distance_to_point(point);
        point - self.normal.scale(t)
    }
}

pub enum Plane3Plane3Intersection<TScalar: RealNumber> {
    Line(Line3<TScalar>),
    Plane
}

impl<TScalar: RealNumber> IntersectsPlane3 for Plane3<TScalar> {
    type Output = Plane3Plane3Intersection<TScalar>;

    fn intersects_plane3_at(&self, other: &Plane3<Self::Scalar>) -> Option<Self::Output> {
        let n00 = self.normal.dot(&self.normal);
        let n01 = self.normal.dot(&other.normal);
        let n11 = other.normal.dot(&other.normal);
        let det = n00 * n11 - n01 * n01;

        if det == TScalar::zero() {
            if self.distance == other.distance || self.distance == -other.distance {
                return Some(Plane3Plane3Intersection::Plane);
            } else {
                return None;
            }
        }

        let inv_det = TScalar::one() / det;
        let c0 = n11 * self.distance - n01 * other.distance * inv_det;
        let c1 = n00 * other.distance - n01 * self.distance * inv_det;

        let line = Line3::new(
            self.normal * c0 + other.normal * c1,
            self.normal.cross(&other.normal)
        );

        Some(Plane3Plane3Intersection::Line(line))
    }
}
