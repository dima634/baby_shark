use std::mem::swap;

use num_traits::Float;

use crate::{geometry::traits::{RealNumber, HasScalarType, ClosestPoint3}, helpers::aliases::Vec3};

use super::{plane3::Plane3, box3::Box3};

/// Infinite line. l(t) = p + v*t
#[derive(PartialEq, Debug)]
pub struct Line3<TScalar: RealNumber> {
    point: Vec3<TScalar>,
    direction: Vec3<TScalar>
}

impl<TScalar: RealNumber> Line3<TScalar> {
    pub fn new(point: Vec3<TScalar>, direction: Vec3<TScalar>) -> Self {
        Self { point, direction }
    }

    pub fn from_points(p1: &Vec3<TScalar>, p2: &Vec3<TScalar>) -> Self {
        Self {
            direction: (p2 - p1).normalize(),
            point: *p1
        }
    }

    #[inline]
    pub fn get_point(&self) -> &Vec3<TScalar> {
        &self.point
    }

    #[inline]
    pub fn get_direction(&self) -> &Vec3<TScalar> {
        &self.direction
    }

    #[inline]
    pub fn parameter_at(&self, point: &Vec3<TScalar>) -> TScalar {
        (point - self.point).dot(&self.direction)
    }

    #[inline]
    pub fn point_at(&self, t: TScalar) -> Vec3<TScalar> {
        self.point + self.direction.scale(t)
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        let dot = plane.get_normal().dot(&self.direction);

        if dot.is_zero() {
            return None;
        }

        return Some((plane.get_distance() - plane.get_normal().dot(&self.point)) / dot);
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        self.intersects_plane3_at(plane).is_some()
    }    
    
    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        let mut t_min = TScalar::neg_infinity();
        let mut t_max = TScalar::infinity();

        // For all three slabs
        for i in 0..3 {
            if Float::abs(self.direction[i]) < TScalar::epsilon() {
                // Ray is parallel to slab. No hit if origin not within slab
                if self.point[i] < aabb.get_min()[i] || self.point[i] > aabb.get_max()[i] {
                    return None;
                }
            } else {
                // Compute intersection t value of ray with near and far plane of slab
                let ood = TScalar::one() / self.direction[i];
                let mut t1 = (aabb.get_min()[i] - self.point[i]) * ood;
                let mut t2 = (aabb.get_max()[i] - self.point[i]) * ood;

                // Make t1 be intersection with near plane, t2 with far plane
                if t1 > t2 {
                    swap(&mut t1, &mut t2);
                }

                // Compute the intersection of slab intersection intervals
                if t1 > t_min {
                    t_min = t1;
                }

                if t2 < t_max { 
                    t_max = t2;
                }

                // Exit with no collision as soon as slab intersection becomes empty
                if t_min > t_max { 
                    return None;
                }
            }
        }
        
        Some(t_min)
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        self.intersects_box3_at(aabb).is_some()
    }
}

impl<TScalar: RealNumber> HasScalarType for Line3<TScalar> {
    type Scalar = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Line3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Vec3<TScalar>) -> Vec3<TScalar> {
        let t = self.parameter_at(point);
        self.point + self.direction.scale(t)
    }
}

#[cfg(test)]
mod tests {
    use crate::{geometry::{primitives::line3::Line3, traits::ClosestPoint3}, helpers::aliases::Vec3f};

    #[test]
    fn line_closest_point() {
        let line = Line3::<f32>::new(Vec3f::zeros(), Vec3f::x_axis().xyz());

        let point1 = Vec3f::new(1.0, 1.0, 0.0);
        assert_eq!(Vec3f::new(1.0, 0.0, 0.0), line.closest_point(&point1));

        let point2 = Vec3f::new(0.0, 1.0, 0.0);
        assert_eq!(Vec3f::new(0.0, 0.0, 0.0), line.closest_point(&point2));

        let point2 = Vec3f::new(0.25, 5.0, 0.0);
        assert_eq!(Vec3f::new(0.25, 0.0, 0.0), line.closest_point(&point2));
    }

    #[test]
    fn line_parameter_at() {
        let line = Line3::<f32>::new(Vec3f::zeros(), Vec3f::new(1.0, 1.0, 0.0).normalize());

        assert_eq!(1.4142135, line.parameter_at(&Vec3f::new(1.0, 1.0, 0.0)));
        assert_eq!(2.828427, line.parameter_at(&Vec3f::new(2.0, 2.0, 0.0)));
        assert_eq!(-1.4142135, line.parameter_at(&Vec3f::new(-1.0, -1.0, 0.0)));
    }
}
