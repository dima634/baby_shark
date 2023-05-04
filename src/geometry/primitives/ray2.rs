use nalgebra::Point2;

use crate::geometry::traits::{RealNumber, HasScalarType, Intersects};

use super::{line2::Line2, line_segment2::LineSegment2};

pub struct Ray2<TScalar: RealNumber>(Line2<TScalar>);

impl<TScalar: RealNumber> Ray2<TScalar> {
    /// origin = p1, direction = p2 - p1
    pub fn from_points(p1: Point2<TScalar>, p2: Point2<TScalar>) -> Self {
        return Self(Line2::new(p1, p2));
    }

    #[inline]
    pub fn line(&self) -> &Line2<TScalar> {
        return &self.0;
    }

    #[inline]
    pub fn point_at(&self, t: TScalar) -> Point2<TScalar> {
        return self.0.point_at(t);
    }
}

impl<TScalar: RealNumber> HasScalarType for Ray2<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> Intersects<LineSegment2<TScalar>> for Ray2<TScalar> {
    type Output = Point2<TScalar>;

    #[inline]
    fn intersects_at(&self, segment: &LineSegment2<TScalar>) -> Option<Self::Output> {
        let t = self.line().intersects_line2_at_t(segment.line());
        return t.and_then(|(t1, t2)| {
            let not_intersecting =
                t1 < TScalar::zero() ||                         // outside ray
                t2 < TScalar::zero() || t2 > TScalar::one();    // outside segment
            
            if not_intersecting {
                return None;
            }

            return Some(self.point_at(t1));
        });
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point2;
    use test_case::test_case;

    use crate::geometry::{primitives::line_segment2::LineSegment2, traits::Intersects};

    use super::Ray2;

    #[test_case( 
        Ray2::from_points(
            Point2::new(0.0, 0.0),
            Point2::new(5.0, 5.0)
        ),
        LineSegment2::new(
            Point2::new(1.0, 0.0),
            Point2::new(0.0, 1.0)
        ) => Some(Point2::new(0.5, 0.5));
        "When intersection"
    )]
    #[test_case( 
        Ray2::from_points(
            Point2::new(2.0, 2.0),
            Point2::new(5.0, 5.0)
        ),
        LineSegment2::new(
            Point2::new(1.0, 0.0),
            Point2::new(0.0, 1.0)
        ) => None;
        "When not intersecting"
    )]
    fn test_ray2_segment2_intersection(ray: Ray2<f32>, segment: LineSegment2<f32>) -> Option<Point2<f32>> {
        return ray.intersects_at(&segment);
    }
}
