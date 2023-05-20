use nalgebra::Point2;

use crate::geometry::traits::Number;

/// 2d circle
pub struct Circle2<TScalar: Number> {
    radius: TScalar,
    center: Point2<TScalar>
}

impl<TScalar: Number> Circle2<TScalar> {
    pub fn new(radius: TScalar, center: Point2<TScalar>) -> Self { 
        return Self { radius, center };
    }

    #[inline]
    pub fn radius(&self) -> TScalar {
        return self.radius;
    }

    #[inline]
    pub fn center(&self) -> Point2<TScalar> {
        return self.center;
    }
}
