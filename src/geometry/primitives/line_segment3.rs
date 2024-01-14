use crate::{geometry::traits::{RealNumber, HasScalarType, ClosestPoint3}, helpers::aliases::Vec3};

use super::{line3::Line3, plane3::Plane3, box3::Box3};

/// 3D line segment
#[derive(PartialEq, Debug)]
pub struct LineSegment3<TScalar: RealNumber> {
    line: Line3<TScalar>,
    length: TScalar
}

impl<TScalar: RealNumber> LineSegment3<TScalar> {
    pub fn new(start: &Vec3<TScalar>, end: &Vec3<TScalar>) -> Self { 
        return Self { 
            line: Line3::from_points(start, end), 
            length: (end - start).norm()
        }; 
    }

    #[inline]
    pub fn from_line_and_t(line: &Line3<TScalar>, t_start: TScalar, t_end: TScalar) -> Self {
        return Self::new(&line.point_at(t_start), &line.point_at(t_end));
    }
    
    #[inline]
    pub fn get_start(&self) -> &Vec3<TScalar> {
        return self.line.get_point();
    }

    #[inline]
    pub fn get_end(&self) -> Vec3<TScalar> {
        return self.line.point_at(self.length);
    }

    #[inline]
    pub fn get_line(&self) -> &Line3<TScalar> {
        return &self.line;
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_plane3_at(plane) {
            if self.is_on_segment(t) {
                return Some(t);
            }
        }

        return None;
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return self.intersects_plane3_at(plane).is_some();
    }    
    
    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_box3_at(aabb) {
            if self.is_on_segment(t) {
                return Some(t);
            }
        }

        return None;
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        return self.intersects_box3_at(aabb).is_some();
    }  

    #[inline]
    pub fn is_on_segment(&self, t: TScalar) -> bool {
        return t >= TScalar::zero() && t <= self.length;
    }
}

impl<TScalar: RealNumber> HasScalarType for LineSegment3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for LineSegment3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Vec3<TScalar>) -> Vec3<TScalar> {
        let mut t = self.line.parameter_at(point);

        if t < TScalar::zero() {
            t = TScalar::zero();
        } else if t > self.length {
            t = self.length;
        }

        return self.line.point_at(t);
    }
}
