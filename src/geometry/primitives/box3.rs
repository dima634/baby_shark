use std::ops::Add;

use nalgebra_glm::{max2, min2};
use num_traits::{cast, Bounded, Float};

use crate::{
    geometry::traits::{ClosestPoint3, HasScalarType, Number, RealNumber},
    helpers::aliases::Vec3,
};

use super::{line_segment3::LineSegment3, plane3::Plane3, sphere3::Sphere3, triangle3::Triangle3};

/// 3D bounding box
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Box3<TScalar: Number> {
    min: Vec3<TScalar>,
    max: Vec3<TScalar>,
}

impl<TScalar: Number> Box3<TScalar> {
    pub fn new(min: Vec3<TScalar>, max: Vec3<TScalar>) -> Self {
        return Self { min, max };
    }

    pub fn empty() -> Self {
        return Self {
            min: Vec3::max_value(),
            max: Vec3::min_value(),
        };
    }

    #[inline]
    pub fn get_min(&self) -> &Vec3<TScalar> {
        return &self.min;
    }

    #[inline]
    pub fn get_max(&self) -> &Vec3<TScalar> {
        return &self.max;
    }

    #[inline]
    pub fn get_center(&self) -> Vec3<TScalar> {
        return (self.min + self.max) * cast::<f32, TScalar>(0.5).unwrap();
    }

    #[inline]
    pub fn size_x(&self) -> TScalar {
        return self.max.x - self.min.x;
    }

    #[inline]
    pub fn size_y(&self) -> TScalar {
        return self.max.y - self.min.y;
    }

    #[inline]
    pub fn size_z(&self) -> TScalar {
        return self.max.z - self.min.z;
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        return self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z;
    }

    #[inline]
    pub fn union_box(&mut self, other: &Box3<TScalar>) -> &mut Self {
        self.max = max2(&self.get_max(), &other.get_max());
        self.min = min2(&self.get_min(), &other.get_min());

        self
    }

    #[inline]
    pub fn union_point(&mut self, p: &Vec3<TScalar>) -> &mut Self {
        self.max = max2(&self.get_max(), &p);
        self.min = min2(&self.get_min(), &p);

        self
    }

    /// Returns the ith box vertex in order: (x,y,z),(X,y,z),(x,Y,z),(X,Y,z),(x,y,Z),(X,y,Z),(x,Y,Z),(X,Y,Z)
    #[inline]
    pub fn vertex(&self, i: u8) -> Vec3<TScalar> {
        return Vec3::new(
            self.min.x + TScalar::from(i % 2).unwrap() * self.size_x(),
            self.min.y + TScalar::from((i / 2) % 2).unwrap() * self.size_y(),
            self.min.z + TScalar::from(if i > 3 { 1 } else { 0 }).unwrap() * self.size_z(),
        );
    }

    #[inline]
    pub fn volume(&self) -> TScalar {
        return self.size_x() * self.size_y() * self.size_z();
    }

    pub fn squared_distance(&self, point: &Vec3<TScalar>) -> TScalar {
        let mut sq_distance = TScalar::zero();

        for i in 0..3 {
            let v = point[i];

            if v < self.min[i] {
                sq_distance += (self.min[i] - v) * (self.min[i] - v);
            }

            if v > self.max[i] {
                sq_distance += (v - self.max[i]) * (v - self.max[i]);
            }
        }

        return sq_distance;
    }

    #[inline]
    pub fn contains_point(&self, point: &Vec3<TScalar>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    #[inline]
    pub fn area(&self) -> TScalar {
        let diag = self.max - self.min;
        TScalar::from(2).unwrap() * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z)
    }

    /// Test bbox - bbox intersection
    pub fn intersects_box3(&self, other: &Box3<TScalar>) -> bool {
        if self.max[0] < other.min[0] || self.min[0] > other.max[0] {
            return false;
        }

        if self.max[1] < other.min[1] || self.min[1] > other.max[1] {
            return false;
        }

        if self.max[2] < other.min[2] || self.min[2] > other.max[2] {
            return false;
        }

        return true;
    }
}

impl<TScalar: RealNumber> Add<&Box3<TScalar>> for Box3<TScalar> {
    type Output = Box3<TScalar>;

    #[inline]
    fn add(mut self, rhs: &Box3<TScalar>) -> Self::Output {
        self.union_box(rhs);
        self
    }
}

impl<TScalar: RealNumber> Add<&Vec3<TScalar>> for Box3<TScalar> {
    type Output = Box3<TScalar>;

    #[inline]
    fn add(mut self, rhs: &Vec3<TScalar>) -> Self::Output {
        self.union_point(rhs);
        self
    }
}

impl<TScalar: RealNumber> Box3<TScalar> {
    /// Returns the ith diagonal of box
    #[inline]
    pub fn diagonal(&self, i: u8) -> LineSegment3<TScalar> {
        return LineSegment3::new(&self.vertex(i), &self.vertex(7 - i));
    }

    #[inline]
    pub fn size_max(&self) -> TScalar {
        let xy = Float::max(self.size_x(), self.size_y());
        return Float::max(xy, self.size_z());
    }

    ///
    /// Returns the continuous position of a point relative to the corners of the box,
    /// where a point at the minimum corner has offset,
    /// a point at the maximum corner has offset, and so forth
    ///
    pub fn offset(&self, p: &Vec3<TScalar>) -> Vec3<TScalar> {
        let mut o = p - self.min;

        if self.max.x > self.min.x {
            o.x /= self.max.x - self.min.x;
        }

        if self.max.y > self.min.y {
            o.y /= self.max.y - self.min.y;
        }

        if self.max.z > self.min.z {
            o.z /= self.max.z - self.min.z;
        }

        o
    }

    /// Test bbox - plane intersection
    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return plane.intersects_box3(self);
    }

    /// Test bbox - triangle intersection
    #[inline]
    pub fn intersects_triangle3(&self, triangle: &Triangle3<TScalar>) -> bool {
        return triangle.intersects_box3(self);
    }

    #[inline]
    pub fn intersects_sphere3(&self, sphere: &Sphere3<TScalar>) -> bool {
        return sphere.intersects_box3(self);
    }
}

impl<TScalar: RealNumber> HasScalarType for Box3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Box3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Vec3<TScalar>) -> Vec3<TScalar> {
        return Vec3::from(min2(&max2(&self.min, &point), &self.max));
    }
}

#[cfg(test)]
mod tests {
    use crate::{geometry::primitives::box3::Box3, helpers::aliases::Vec3f};

    #[test]
    fn test_union() {
        let mut box1 = Box3::new(Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 1.0, 1.0));
        let box2 = Box3::new(Vec3f::new(0.5, 0.5, 0.5), Vec3f::new(1.5, 1.5, 1.5));

        box1.union_box(&box2);

        assert_eq!(
            box1,
            Box3::new(Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.5, 1.5, 1.5))
        );

        let mut empty = Box3::empty();
        empty.union_box(&box2);

        assert_eq!(empty, box2);
    }
}
