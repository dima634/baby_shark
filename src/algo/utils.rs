use nalgebra::{Point3, Vector3};
use num_traits::Float;
use crate::mesh::traits::Floating;

pub fn barycenter<'a, TScalar, TPointsIter>(points: TPointsIter) -> Point3<TScalar> 
where 
    TScalar: Floating, 
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
    TScalar: Floating, 
    TPointsIter: Iterator<Item = &'a Point3<TScalar>> 
{
    let normal_t = normal.clone().transpose();
    let barycenter = barycenter(points);
    
    return barycenter - normal * normal_t * (barycenter - origin);
}

/// v3 * (v1 x v2)
#[inline]
pub fn triple_product<TScalar: Floating>(v1: &Vector3<TScalar>, v2: &Vector3<TScalar>, v3: &Vector3<TScalar>) -> TScalar {
    return v3.dot(&(v1.cross(&v2)));
}

#[inline]
pub fn has_same_sign<TScalar: Floating>(a: TScalar, b: TScalar) -> bool {
    return Float::signum(a) == Float::signum(b) || a.is_zero() || b.is_zero();
}

#[inline]
pub fn cwise_max<TScalar: Floating>(p1: &Point3<TScalar>, p2: &Point3<TScalar>) -> Point3<TScalar> {
    return Point3::new(
        Float::max(p1.x, p2.x), 
        Float::max(p1.y, p2.y), 
        Float::max(p1.z, p2.z)
    );
}

#[inline]
pub fn cwise_min<TScalar: Floating>(p1: &Point3<TScalar>, p2: &Point3<TScalar>) -> Point3<TScalar> {
    return Point3::new(
        Float::min(p1.x, p2.x), 
        Float::min(p1.y, p2.y), 
        Float::min(p1.z, p2.z)
    );
}
