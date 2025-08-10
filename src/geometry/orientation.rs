use super::traits::RealNumber;
use nalgebra::{Matrix2, Point2, Vector2};

#[derive(PartialEq, Eq)]
pub enum Orientation {
    Clockwise,
    CounterClockwise,
    Colinear,
}

/// Returns orientation of triangle `a`-`b`-`c`
pub fn orientation2d<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>) -> Orientation {
    let mat = Matrix2::new(
        a.x - c.x, a.y - c.y,
        b.x - c.x, b.y - c.y
    );

    let det = mat.determinant();

    if det < R::zero() {
        Orientation::Clockwise
    } else if det > R::zero() {
        Orientation::CounterClockwise
    } else {
        Orientation::Colinear
    }
}

/// Returns signed angle between vectors in range [0; 2PI)
#[inline]
pub fn signed_angle_between_vectors<R: RealNumber>(v1: &Vector2<R>, v2: &Vector2<R>) -> R {
    let mut angle = R::atan2(v1.perp(v2), v1.dot(v2));

    if angle < R::zero() {
        angle += R::two_pi();
    }

    angle
}

/// Returns signed angle between X-axis and vector (x, y) in range [0; 4]
#[inline]
pub fn signed_diamond_angle<R: RealNumber>(y: R, x: R) -> R {
    if y >= R::zero() {
        if x >= R::zero() {
            y / (x + y)
        } else {
            R::one() - x / (-x + y)
        }
    } else if x < R::zero() {
        R::two() - y / (-x - y)
    } else {
        R::one() + R::two() + x / (x - y)
    }
}

/// Returns signed angle between vectors in range [0; 4]
#[inline]
pub fn signed_diamond_angle_between_vectors<R: RealNumber>(v1: &Vector2<R>, v2: &Vector2<R>) -> R {
    signed_diamond_angle(v1.perp(v2), v1.dot(v2))
}
