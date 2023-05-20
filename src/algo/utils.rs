use nalgebra::{Point3, Vector3};
use num_traits::Float;

use crate::geometry::traits::{RealNumber, Number};

pub fn barycenter<'a, TScalar, TPointsIter>(points: TPointsIter) -> Point3<TScalar> 
where 
    TScalar: RealNumber, 
    TPointsIter: Iterator<Item = &'a Point3<TScalar>> 
{
    let mut barycenter = Point3::origin();
    let mut size = 0;

    for p in points {
        barycenter += p.coords;
        size += 1 ;
    }

    barycenter /= TScalar::from_usize(size).unwrap();

    return barycenter;
}

pub fn tangential_relaxation<'a, TScalar, TPointsIter>(points: TPointsIter, origin: &Point3<TScalar>,  normal: &Vector3<TScalar>) -> Point3<TScalar> 
where 
    TScalar: RealNumber, 
    TPointsIter: Iterator<Item = &'a Point3<TScalar>> 
{
    let normal_t = normal.clone().transpose();
    let barycenter = barycenter(points);
    
    return barycenter - normal * normal_t * (barycenter - origin);
}

/// v3 * (v1 x v2)
#[inline]
pub fn triple_product<TScalar: RealNumber>(v1: &Vector3<TScalar>, v2: &Vector3<TScalar>, v3: &Vector3<TScalar>) -> TScalar {
    return v3.dot(&(v1.cross(v2)));
}

#[inline]
pub fn has_same_sign<TScalar: RealNumber>(a: TScalar, b: TScalar) -> bool {
    return Float::signum(a) == Float::signum(b) || (a.is_zero() && b.is_zero());
}

#[inline]
pub fn max<TScalar: Number>(a: TScalar, b: TScalar) -> TScalar {
    return if a > b {a} else {b};
}

#[inline]
pub fn min<TScalar: Number>(a: TScalar, b: TScalar) -> TScalar {
    return if a < b {a} else {b};
}

#[inline]
pub fn cwise_max<TScalar: Number>(p1: &Point3<TScalar>, p2: &Point3<TScalar>) -> Point3<TScalar> {
    return Point3::new(
        max(p1.x, p2.x), 
        max(p1.y, p2.y), 
        max(p1.z, p2.z)
    );
}

#[inline]
pub fn cwise_min<TScalar: Number>(p1: &Point3<TScalar>, p2: &Point3<TScalar>) -> Point3<TScalar> {
    return Point3::new(
        min(p1.x, p2.x), 
        min(p1.y, p2.y), 
        min(p1.z, p2.z)
    );
}

#[inline]
pub fn cast<T1: Number, T2: Number>(p: &Vector3<T1>) -> Vector3<T2> {
    return Vector3::new(
        T2::from(p.x).unwrap(), 
        T2::from(p.y).unwrap(),
        T2::from(p.z).unwrap()
    );
}
