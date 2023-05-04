use nalgebra::{Point2, Matrix2, Vector2};
use num_traits::Float;

use super::traits::RealNumber;

#[derive(PartialEq, Eq)]
pub enum Orientation {
    Clockwise,
    CounterClockwise,
    Colinear
}

pub fn orientation2d<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>) -> Orientation {
    let mat = Matrix2::new(
        a.x - c.x, a.y - c.y,
        b.x - c.x, b.y - c.y
    );

    let det = mat.determinant();

    if det < TScalar::zero() {
        return Orientation::Clockwise;
    } else if det > TScalar::zero() {
        return Orientation::CounterClockwise;
    } else {
        return Orientation::Colinear;
    }
}

#[inline]
pub fn signed_angle<TScalar: RealNumber>(v1: &Vector2<TScalar>, v2: &Vector2<TScalar>) -> TScalar {
    let mut angle = Float::atan2(v1.perp(&v2), v1.dot(&v2));

    if angle < TScalar::zero() {
        angle += TScalar::two_pi();
    }

    return angle;
}
