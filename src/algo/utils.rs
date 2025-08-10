use crate::{geometry::traits::*, helpers::aliases::Vec3};
use nalgebra::{Point3, Vector3};

pub fn barycenter<'a, R, TPointsIter>(points: TPointsIter) -> Vec3<R>
where
    R: RealNumber,
    TPointsIter: Iterator<Item = &'a Vec3<R>>,
{
    let mut barycenter = Vec3::zeros();
    let mut size = 0;

    for p in points {
        barycenter += p;
        size += 1;
    }

    barycenter /= R::from_usize(size).unwrap();

    barycenter
}

pub fn tangential_relaxation<'a, R, TPointsIter>(
    points: TPointsIter,
    origin: &Vec3<R>,
    normal: &Vec3<R>,
) -> Vec3<R>
where
    R: RealNumber,
    TPointsIter: Iterator<Item = &'a Vec3<R>>,
{
    let normal_t = normal.clone().transpose();
    let barycenter = barycenter(points);

    barycenter - normal * normal_t * (barycenter - origin)
}

/// v3 * (v1 x v2)
#[inline]
pub fn triple_product<R: RealNumber>(v1: &Vector3<R>, v2: &Vector3<R>, v3: &Vector3<R>) -> R {
    v3.dot(&(v1.cross(v2)))
}

#[inline]
pub fn has_same_sign<R: RealNumber>(a: R, b: R) -> bool {
    a.signum() == b.signum() || (a.is_zero() && b.is_zero())
}

#[inline]
pub fn max<R: Number>(a: R, b: R) -> R {
    if a > b {
        a
    } else {
        b
    }
}

#[inline]
pub fn min<R: Number>(a: R, b: R) -> R {
    if a < b {
        a
    } else {
        b
    }
}

#[inline]
pub fn cwise_max<R: Number>(p1: &Point3<R>, p2: &Point3<R>) -> Point3<R> {
    Point3::new(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z))
}

#[inline]
pub fn cwise_min<R: Number>(p1: &Point3<R>, p2: &Point3<R>) -> Point3<R> {
    Point3::new(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z))
}
