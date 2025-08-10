use nalgebra::Point2;
use crate::geometry::traits::{RealNumber, HasScalarType, Intersects};
use super::line2::Line2;

// 2d line segment
#[derive(Debug)]
pub struct LineSegment2<TScalar: RealNumber>(Line2<TScalar>);

impl<R: RealNumber> LineSegment2<R> {
    pub fn new(p1: Point2<R>, p2: Point2<R>) -> Self {
        Self(Line2::new(p1, p2))
    }

    #[inline]
    pub fn line(&self) -> &Line2<R> {
        &self.0
    }

    #[inline]
    pub fn contains_point(&self, p: &Point2<R>) -> bool {
        let len = 
            (self.0.origin() - p).norm() + 
            (self.0.end() - p).norm() - 
            (self.0.origin() - self.0.end()).norm();
        len < R::default_epsilon()
    }

    #[inline]
    pub fn at(&self, t: R) -> Point2<R> {
        self.0.point_at(t)
    }

    #[inline]
    pub fn intersects_line_segment2_at(&self, other: &Self) -> Option<(R, R)> {
        self.line().intersects_line2_at_t(other.line())
            .filter(|(t1, t2)| *t1 >= R::zero() && *t1 <= R::one() && *t2 >= R::zero() && *t2 <= R::one())
    }
}

impl<TScalar: RealNumber> HasScalarType for LineSegment2<TScalar> {
    type Scalar = TScalar;
}

impl<TScalar: RealNumber> Intersects<Line2<TScalar>> for LineSegment2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, line: &Line2<TScalar>) -> Option<Self::Output> {
        let t = self.line().intersects_line2_at_t(line);
        return t.and_then(|(t1, _)| {
            if t1 >= TScalar::zero() && t1 <= TScalar::one() 
                { Some(self.line().point_at(t1)) }
            else
                { None }
        });
    }
}

impl<TScalar: RealNumber> Intersects<LineSegment2<TScalar>> for LineSegment2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, segment: &LineSegment2<TScalar>) -> Option<Self::Output> {
        self.intersects_line_segment2_at(segment).map(|(t, _)| self.at(t))
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point2;
    use test_case::test_case;

    use crate::geometry::traits::Intersects;

    use super::LineSegment2;

    #[test_case(
        LineSegment2::new(Point2::new(1.0, 1.0), Point2::new(1.66, 1.66)),
        LineSegment2::new(Point2::new(2.0, 1.0), Point2::new(1.0, 2.0))
        => Some(Point2::new(1.5, 1.5)); 
        "When intersecting"
    )]
    #[test_case(
        LineSegment2::new(Point2::new(2.0, 1.0), Point2::new(1.0, 2.0)),
        LineSegment2::new(Point2::new(1.0, 1.0), Point2::new(1.66, 1.66))
        => Some(Point2::new(1.5, 1.5)); 
        "When intersecting (reordered)"
    )]
    fn segment2_segment2_intersection(s1: LineSegment2<f32>, s2: LineSegment2<f32>) -> Option<Point2<f32>> {
        s1.intersects_at(&s2)
    }
}
