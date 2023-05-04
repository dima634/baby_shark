use nalgebra::Point2;

use crate::geometry::{traits::{Number, RealNumber}, orientation::{Orientation, orientation2d}};

#[derive(Debug)]
pub struct Triangle2<TScalar: Number> {
    a: Point2<TScalar>,
    b: Point2<TScalar>,
    c: Point2<TScalar>
}

impl<TScalar: Number> Triangle2<TScalar> {
    pub fn new(a: Point2<TScalar>, b: Point2<TScalar>, c: Point2<TScalar>) -> Self { 
        return Self { a, b, c } ;
    }
}

impl<TScalar: RealNumber> Triangle2<TScalar> {
    #[inline]
    pub fn circumcenter(&self) -> Point2<TScalar> {
        return circumcenter(&self.a, &self.b, &self.c);
    }

    #[inline]
    pub fn circumradius_squared(&self) -> TScalar {
        let c = self.circumcenter();
        return (c - self.a).norm_squared()
    }

    #[inline]
    pub fn orientation(&self) -> Orientation {
        return orientation2d(&self.a, &self.b, &self.c);
    }

    #[inline]
    pub fn is_inside_circum(&self, point: &Point2<TScalar>) -> bool {
        let c = self.circumcenter();
        let c_r_sq = (c - self.a).norm_squared();
        let p_sq = (c - point).norm_squared();
        return p_sq < c_r_sq;
    }
}

pub fn circumcenter<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>) -> Point2<TScalar> {
    let ab = b - a;
    let ac = c - a;
    let norm_pq = ab.norm_squared();
    let norm_pr = ac.norm_squared();
    let det = ab.x * ac.y - ab.y * ac.x;
    let half_inv_det = num_traits::cast::<f64, TScalar>(0.5).unwrap() / det;
    let x = a.x + (norm_pq * ac.y - norm_pr * ab.y) * half_inv_det;
    let y = a.y + (norm_pr * ab.x - norm_pq * ac.x) * half_inv_det;

    return Point2::new(x, y);
}

#[inline]
pub fn is_inside_circumcircle<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>, p: &Point2<TScalar>) -> bool {
    let ax = a.x - p.x;
    let ay = a.y - p.y;
    let bx = b.x - p.x;
    let by = b.y - p.y;
    let cx = c.x - p.x;
    let cy = c.y - p.y;
    let det = 
        (ax*ax + ay*ay) * (bx*cy-cx*by) -
        (bx*bx + by*by) * (ax*cy-cx*ay) +
        (cx*cx + cy*cy) * (ax*by-bx*ay);
    
    return det > TScalar::zero();
}

#[inline]
pub fn is_inside_circumcircle2<TScalar: RealNumber>(a: &Point2<TScalar>, b: &Point2<TScalar>, c: &Point2<TScalar>, p: &Point2<TScalar>) -> bool {
    let c = circumcenter(a, b, c);
    let c_r_sq = (c - a).norm_squared();
    let p_sq = (c - p).norm_squared();

    return p_sq < c_r_sq;
}
