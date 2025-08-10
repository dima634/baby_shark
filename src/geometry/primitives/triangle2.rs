use crate::geometry::{
    orientation::{orientation2d, Orientation},
    traits::{Number, RealNumber},
};
use nalgebra::Point2;

/// 2d triangle
#[derive(Debug)]
pub struct Triangle2<I: Number> {
    a: Point2<I>,
    b: Point2<I>,
    c: Point2<I>,
}

impl<I: Number> Triangle2<I> {
    pub fn new(a: Point2<I>, b: Point2<I>, c: Point2<I>) -> Self {
        Self { a, b, c }
    }
}

impl<R: RealNumber> Triangle2<R> {
    #[inline]
    pub fn circumcircle_center(&self) -> Point2<R> {
        circumcircle_center(&self.a, &self.b, &self.c)
    }

    #[inline]
    pub fn circumcircle_radius_squared(&self) -> R {
        let c = self.circumcircle_center();
        (c - self.a).norm_squared()
    }

    #[inline]
    pub fn orientation(&self) -> Orientation {
        orientation2d(&self.a, &self.b, &self.c)
    }

    /// Checks whether point is inside of triangle's circumscribed circle
    #[inline]
    pub fn is_inside_circumcircle(&self, point: &Point2<R>) -> bool {
        is_inside_circumcircle(&self.a, &self.b, &self.c, point)
    }
}

/// Returns center of triangle's circumscribed circle
pub fn circumcircle_center<R: RealNumber>(
    a: &Point2<R>,
    b: &Point2<R>,
    c: &Point2<R>,
) -> Point2<R> {
    let ab = b - a;
    let ac = c - a;
    let norm_pq = ab.norm_squared();
    let norm_pr = ac.norm_squared();
    let det = ab.x * ac.y - ab.y * ac.x;
    let half_inv_det = R::half() / det;
    let x = a.x + (norm_pq * ac.y - norm_pr * ab.y) * half_inv_det;
    let y = a.y + (norm_pr * ab.x - norm_pq * ac.x) * half_inv_det;

    Point2::new(x, y)
}

/// Checks whether point is inside of triangle's circumscribed circle
#[inline]
pub fn is_inside_circumcircle<R: RealNumber>(
    a: &Point2<R>,
    b: &Point2<R>,
    c: &Point2<R>,
    p: &Point2<R>,
) -> bool {
    let c = circumcircle_center(a, b, c);
    let c_r_sq = (c - a).norm_squared();
    let p_sq = (c - p).norm_squared();

    p_sq < c_r_sq
}
