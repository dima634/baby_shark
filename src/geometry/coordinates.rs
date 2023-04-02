use std::cmp::Ordering;

use nalgebra::Point2;
use num_traits::Float;

use super::traits::RealNumber;

#[derive(PartialEq, Eq)]
pub struct PolarPoint2<TScalar: RealNumber>(Point2<TScalar>);

impl<TScalar: RealNumber> PolarPoint2<TScalar> {
    pub fn new(radius: TScalar, angle: TScalar) -> Self {
        let mut angle = angle % TScalar::pi();

        if angle < TScalar::zero() {
            angle += TScalar::two_pi();
        }

        return Self(Point2::new(radius, angle));
    }

    #[inline]
    pub fn radius(&self) -> TScalar {
        return self.0.x;
    }

    #[inline]
    pub fn angle(&self) -> TScalar {
        return self.0.y;
    }

    #[inline]
    pub fn partial_cmp_by_radius(&self, other: &Self) -> Option<Ordering> {
        let radius_ord = self.radius().partial_cmp(&other.radius());

        return radius_ord.map(|ord| match ord {
            Ordering::Less => Ordering::Less,
            Ordering::Greater => Ordering::Greater,
            Ordering::Equal => self.angle().partial_cmp(&other.angle()).unwrap()
        });
    }
}

///
/// `x` is radius and `y` is CW angle
/// 
pub fn polar2<TScalar: RealNumber>(pole: &Point2<TScalar>, point: &Point2<TScalar>) -> PolarPoint2<TScalar> {
    let radius = (point - pole).norm();

    if pole.y == point.y {
        return PolarPoint2::new(radius, TScalar::zero());
    } 

    let mut acos = Float::acos((point.x - pole.x) / radius);

    if point.y < pole.y {
        acos += TScalar::pi();
    }

    return PolarPoint2::new(radius, acos);
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::Point2;

    use crate::geometry::coordinates::{polar2, PolarPoint2};

    #[test]
    fn test_polar() {
        let pole = Point2::new(2.0, 2.0);

        let cartesian = Point2::new(1.0, 1.0);
        let expected = PolarPoint2::new(1.4142135623731, 0.78539816339749313);
        let polar = polar2(&pole, &cartesian);

        assert!(expected.radius() - polar.radius() < 1e-8);
        assert!(expected.angle() - polar.angle() < 1e-8);
    }

    #[test]
    fn test_polar_zero_angle() {
        let pole = Point2::new(2.0, 2.0);

        let cartesian = Point2::new(2.0, 1.0);
        let expected = PolarPoint2::new(1.0, 0.0);
        let polar = polar2(&pole, &cartesian);

        assert!(expected.radius() - polar.radius() < 1e-8);
        assert!(expected.angle() - polar.angle() < 1e-8);
    }

    #[test]
    fn test_polar_pi() {
        let pole = Point2::new(2.0, 2.0);

        let cartesian = Point2::new(2.0, 3.0);
        let expected = PolarPoint2::new(1.0, PI);
        let polar = polar2(&pole, &cartesian);

        assert!(expected.radius() - polar.radius() < 1e-8);
        assert!(expected.angle() - polar.angle() < 1e-8);
    }
}
