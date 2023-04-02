use nalgebra::{Point2, Vector2};
use num_traits::Float;

use crate::geometry::traits::{RealNumber, Intersects, HasScalarType};

use super::line_segment2::LineSegment2;

pub struct Line2<TScalar: RealNumber> {
    p1: Point2<TScalar>,
    p2: Point2<TScalar>
}

impl<TScalar: RealNumber> Line2<TScalar> {
    pub fn new(p1: Point2<TScalar>, p2: Point2<TScalar>) -> Self { 
        return Self { p1, p2 };
    }

    #[inline]
    pub fn origin(&self) -> Point2<TScalar> {
        return self.p1;
    }

    #[inline]
    pub fn direction(&self) -> Vector2<TScalar> {
        return self.p2 - self.p1;
    }
    
    #[inline]
    pub fn point_at(&self, t: TScalar) -> Point2<TScalar> {
        return self.p1 + (self.p2 - self.p1) * t;
    }

    #[inline]
    pub fn parameter_at(&self, p: Point2<TScalar>) -> TScalar {
        return (p - self.p1).dot(&(self.p2 - self.p1));
    }

    pub fn intersects_line2_at_t(&self, other: &Line2<TScalar>) -> Option<TScalar> {
        // Graphic Gems III p. 199-202
        let by = other.p1.y - other.p2.y;
        let bx = other.p1.x - other.p2.x;
        let cx = self.p1.x - other.p1.x;
        let cy = self.p1.y - other.p1.y;
        let ax = self.p2.x - self.p1.x;
        let ay = self.p2.y - self.p1.y;

        let nom = by * cx - bx * cy;
        let denom = ay * bx - ax * by;

        let do_not_intersects =
            Float::abs(denom) < TScalar::epsilon() ||
            (
                denom > TScalar::zero() &&
                (
                    (nom < TScalar::zero() || nom > denom) ||
                    (nom > TScalar::zero() || nom < denom)
                )
            );
        
        if do_not_intersects {
            return None;
        }

        return Some(nom / denom)
    }
}

impl<TScalar: RealNumber> HasScalarType for Line2<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> Intersects<Line2<TScalar>> for Line2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, other: &Line2<TScalar>) -> Option<Self::Output> {
        return self.intersects_line2_at_t(other).and_then(|t| Some(self.point_at(t)));
    }
}

impl<TScalar: RealNumber> Intersects<LineSegment2<TScalar>> for Line2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, segment: &LineSegment2<TScalar>) -> Option<Self::Output> {
        return segment.intersects_at(self);
    }
}
