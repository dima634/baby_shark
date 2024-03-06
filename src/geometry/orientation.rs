use nalgebra::{Point2, Matrix2, Vector2};
use num_traits::{Float, cast};

use super::traits::RealNumber;

#[derive(PartialEq, Eq)]
pub enum Orientation {
    Clockwise,
    CounterClockwise,
    Colinear
}

/// Returns orientation of triangle `a`-`b`-`c`
pub fn orientation2d<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>) -> Orientation {
    let mat = Matrix2::new(
        a.x - c.x, a.y - c.y,
        b.x - c.x, b.y - c.y
    );

    let det = mat.determinant();

    if det < TScalar::zero() {
        Orientation::Clockwise
    } else if det > TScalar::zero() {
        Orientation::CounterClockwise
    } else {
        Orientation::Colinear
    }
}
 
/// Returns signed angle between vectors in range [0; 2PI)
#[inline]
pub fn signed_angle_between_vectors<TScalar: RealNumber>(v1: &Vector2<TScalar>, v2: &Vector2<TScalar>) -> TScalar {
    let mut angle = Float::atan2(v1.perp(v2), v1.dot(v2));

    if angle < TScalar::zero() {
        angle += TScalar::two_pi();
    }

    angle
}

/// Returns signed angle between X-axis and vector (x, y) in range [0; 4]
#[inline]
pub fn signed_diamond_angle<TScalar: RealNumber>(y: TScalar, x: TScalar) -> TScalar {
    if y >= TScalar::zero() {
        if x >= TScalar::zero() { 
            y / (x + y) 
        } else {
            TScalar::one() - x / (-x + y) 
        }
    } else {
        if x < TScalar::zero() {
            cast::<f64, TScalar>(2.0).unwrap() - y / (-x - y) 
        } else { 
            cast::<f64, TScalar>(3.0).unwrap() + x / (x - y)
        }
    }
}

/// Returns signed angle between vectors in range [0; 4]
#[inline]
pub fn signed_diamond_angle_between_vectors<TScalar: RealNumber>(v1: &Vector2<TScalar>, v2: &Vector2<TScalar>) -> TScalar {
    signed_diamond_angle(v1.perp(v2), v1.dot(v2))
}
