use nalgebra::Point2;

use crate::geometry::traits::{RealNumber, HasScalarType, Intersects};

use super::line2::Line2;

pub struct LineSegment2<TScalar: RealNumber>(Line2<TScalar>);

impl<TScalar: RealNumber> LineSegment2<TScalar> {
    pub fn new(p1: Point2<TScalar>, p2: Point2<TScalar>) -> Self {
        return Self(Line2::new(p1, p2));
    }

    #[inline]
    pub fn line(&self) -> &Line2<TScalar> {
        return &self.0;
    }

    #[inline]
    pub fn at(&self, t: TScalar) -> Point2<TScalar> {
        return self.0.point_at(t);
    }
}

impl<TScalar: RealNumber> HasScalarType for LineSegment2<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> Intersects<Line2<TScalar>> for LineSegment2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, line: &Line2<TScalar>) -> Option<Self::Output> {
        let t = self.line().intersects_line2_at_t(line);
        return t.and_then(|t| {
            if t >= TScalar::zero() && t <= TScalar::one() 
                { Some(self.line().point_at(t)) }
            else
                { None }
        });
    }
}

impl<TScalar: RealNumber> Intersects<LineSegment2<TScalar>> for LineSegment2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, segment: &LineSegment2<TScalar>) -> Option<Self::Output> {
        let t1 = self.line().intersects_line2_at_t(segment.line());
        if let Some(t1) = t1 {
            if t1 < TScalar::zero() || t1 > TScalar::one() {
                return None;
            }

            let t2 = segment.line().parameter_at(self.at(t1));

            if t2 < TScalar::zero() || t2 > TScalar::one() {
                return None;
            }

            return Some(self.at(t1));
        } else {
            return None;
        }
    }
}
