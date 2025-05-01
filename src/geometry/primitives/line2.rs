use nalgebra::Point2;
use num_traits::Float;

use crate::geometry::traits::{HasScalarType, Intersects, RealNumber};

use super::line_segment2::LineSegment2;

/// 2d line
#[derive(Debug)]
pub struct Line2<TScalar: RealNumber> {
    p1: Point2<TScalar>,
    p2: Point2<TScalar>,
}

impl<TScalar: RealNumber> Line2<TScalar> {
    pub fn new(p1: Point2<TScalar>, p2: Point2<TScalar>) -> Self {
        Self { p1, p2 }
    }

    // Returns start of the line
    #[inline]
    pub fn origin(&self) -> &Point2<TScalar> {
        &self.p1
    }

    #[inline]
    pub fn end(&self) -> &Point2<TScalar> {
        &self.p2
    }

    /// Returns point at parameter `t`
    #[inline]
    pub fn point_at(&self, t: TScalar) -> Point2<TScalar> {
        self.p1 + (self.p2 - self.p1) * t
    }

    ///
    /// Returns intersection of line with another line.
    /// `(intersection parameter at self, intersection parameter at other)`
    ///
    pub fn intersects_line2_at_t(&self, other: &Line2<TScalar>) -> Option<(TScalar, TScalar)> {
        // Graphic Gems III p. 199-202
        let by = other.p1.y - other.p2.y;
        let bx = other.p1.x - other.p2.x;
        let cx = self.p1.x - other.p1.x;
        let cy = self.p1.y - other.p1.y;
        let ax = self.p2.x - self.p1.x;
        let ay = self.p2.y - self.p1.y;

        let num1 = by * cx - bx * cy;
        let denom1 = ay * bx - ax * by;

        if Float::abs(denom1) < TScalar::epsilon() {
            return None;
        }

        let num2 = ax * cy - ay * cx;
        let denom2 = ay * bx - ax * by;

        Some((num1 / denom1, num2 / denom2))
    }
}

impl<TScalar: RealNumber> HasScalarType for Line2<TScalar> {
    type Scalar = TScalar;
}

impl<TScalar: RealNumber> Intersects<Line2<TScalar>> for Line2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, other: &Line2<TScalar>) -> Option<Self::Output> {
        self.intersects_line2_at_t(other).map(|(t1, _)| self.point_at(t1))
    }
}

impl<TScalar: RealNumber> Intersects<LineSegment2<TScalar>> for Line2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, segment: &LineSegment2<TScalar>) -> Option<Self::Output> {
        segment.intersects_at(self)
    }
}
