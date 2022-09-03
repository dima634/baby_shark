use nalgebra::{Point3, Vector3};

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

#[inline]
pub fn triangle_normal<TScalar: Floating>(v1: &Point3<TScalar>, v2: &Point3<TScalar>, v3: &Point3<TScalar>) -> Vector3<TScalar> {
    return (v2 - v1).cross(&(v3 - v1)).normalize();
}
