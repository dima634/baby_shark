use crate::geometry::{
    orientation::{orientation2d, Orientation},
    traits::{Number, RealNumber},
};
use nalgebra::Point2;

/// 2d triangle
#[derive(Debug)]
pub struct Triangle2<TScalar: Number> {
    a: Point2<TScalar>,
    b: Point2<TScalar>,
    c: Point2<TScalar>,
}

impl<TScalar: Number> Triangle2<TScalar> {
    pub fn new(a: Point2<TScalar>, b: Point2<TScalar>, c: Point2<TScalar>) -> Self {
        Self { a, b, c }
    }
}

impl<TScalar: RealNumber> Triangle2<TScalar> {
    #[inline]
    pub fn circumcircle_center(&self) -> Point2<TScalar> {
        circumcircle_center(&self.a, &self.b, &self.c)
    }

    #[inline]
    pub fn circumcircle_radius_squared(&self) -> TScalar {
        let c = self.circumcircle_center();
        (c - self.a).norm_squared()
    }

    #[inline]
    pub fn orientation(&self) -> Orientation {
        orientation2d(&self.a, &self.b, &self.c)
    }

    /// Checks whether point is inside of triangle's circumscribed circle
    #[inline]
    pub fn is_inside_circumcircle(&self, point: &Point2<TScalar>) -> bool {
        is_inside_circumcircle(&self.a, &self.b, &self.c, point)
    }
}

/// Returns center of triangle's circumscribed circle
pub fn circumcircle_center<TScalar: RealNumber>(
    a: &Point2<TScalar>,
    b: &Point2<TScalar>,
    c: &Point2<TScalar>,
) -> Point2<TScalar> {
    let ab = b - a;
    let ac = c - a;
    let norm_pq = ab.norm_squared();
    let norm_pr = ac.norm_squared();
    let det = ab.x * ac.y - ab.y * ac.x;
    let half_inv_det = num_traits::cast::<f64, TScalar>(0.5).unwrap() / det;
    let x = a.x + (norm_pq * ac.y - norm_pr * ab.y) * half_inv_det;
    let y = a.y + (norm_pr * ab.x - norm_pq * ac.x) * half_inv_det;

    Point2::new(x, y)
}

/// Checks whether point is inside of triangle's circumscribed circle
#[inline]
pub fn is_inside_circumcircle<TScalar: RealNumber>(
    a: &Point2<TScalar>,
    b: &Point2<TScalar>,
    c: &Point2<TScalar>,
    p: &Point2<TScalar>,
) -> bool {
    let c = circumcircle_center(a, b, c);
    let c_r_sq = (c - a).norm_squared();
    let p_sq = (c - p).norm_squared();

    p_sq < c_r_sq
}
