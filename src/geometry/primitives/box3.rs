use nalgebra::Point3;
use nalgebra_glm::{min2, max2};
use num_traits::{cast, Float};

use crate::{geometry::traits::{ClosestPoint3, HasScalarType, RealNumber, Number}};

use super::{line_segment3::LineSegment3, plane3::Plane3, triangle3::Triangle3, sphere3::Sphere3};

/// 3D bounding box
#[derive(Clone, Copy)]
pub struct Box3<TScalar: Number> {
    min: Point3<TScalar>,
    max: Point3<TScalar>
}

impl<TScalar: Number> Box3<TScalar> {
    pub fn new(min: Point3<TScalar>, max: Point3<TScalar>) -> Self {
        return Self { min, max } ;
    }

    pub fn empty() -> Self {
        return Self {
            min: Point3::origin(), 
            max: Point3::origin() 
        } ;
    }

    #[inline]
    pub fn get_min(&self) -> &Point3<TScalar> {
        return &self.min;
    }

    #[inline]
    pub fn get_max(&self) -> &Point3<TScalar> {
        return &self.max;
    }

    #[inline]
    pub fn get_center(&self) -> Point3<TScalar> {
        return (self.min + self.max.coords) * cast(0.5).unwrap();
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
    pub fn add_box3(&mut self, other: &Box3<TScalar>) -> &mut Self {
        self.max = max2(&self.get_max().coords, &other.get_max().coords).into();
        self.min = min2(&self.get_min().coords, &other.get_min().coords).into();

        return self;
    }

    /// Returns the ith box vertex in order: (x,y,z),(X,y,z),(x,Y,z),(X,Y,z),(x,y,Z),(X,y,Z),(x,Y,Z),(X,Y,Z)
    #[inline]
    pub fn vertex(&self, i: u8) -> Point3<TScalar> {
        return Point3::new(
            self.min.x + TScalar::from(i % 2).unwrap() * self.size_x(), 
            self.min.y + TScalar::from((i / 2) % 2).unwrap() * self.size_y(), 
            self.min.z + TScalar::from(if i > 3 {1} else {0}).unwrap() * self.size_z()
        );
    }

    #[inline]
    pub fn volume(&self) -> TScalar {
        return self.size_x() * self.size_y() * self.size_z();
    }

    pub fn squared_distance(&self, point: &Point3<TScalar>) -> TScalar {
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
    pub fn contains_point(&self, point: &Point3<TScalar>) -> bool {
        return 
            point.x >= self.min.x && point.x <= self.max.x &&
            point.y >= self.min.y && point.y <= self.max.y &&
            point.z >= self.min.z && point.z <= self.max.z;
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
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        return Point3::from(min2(&max2(&self.min.coords, &point.coords), &self.max.coords));
    }
}
