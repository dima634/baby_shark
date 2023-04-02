use nalgebra::{Point2, Matrix2};

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
