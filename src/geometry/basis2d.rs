use nalgebra::{Point3, Point2, Vector3};

use super::traits::RealNumber;

pub struct Basis2<TScalar: RealNumber> {
    axis_x: Vector3<TScalar>,
    axis_y: Vector3<TScalar>,
    origin: Point3<TScalar>
}

impl<TScalar: RealNumber> Basis2<TScalar> {
    pub fn from_normal_and_point(normal: Vector3<TScalar>, point_on_plane: Point3<TScalar>) -> Self {
        let mut axis_x: Vector3<TScalar>;
        if normal[0] != TScalar::zero() {
            axis_x = Vector3::new(- (normal[1] + normal[2]) / normal.x, TScalar::one(), TScalar::one());
        } else if normal[1] != TScalar::zero(){
            axis_x = Vector3::new(TScalar::one(), - (normal[0] + normal[2]) / normal[1], TScalar::one());
        } else {
            axis_x = Vector3::new(TScalar::one(), TScalar::one(), - (normal[0] + normal[1]) / normal[2]);
        }

        axis_x.normalize_mut();
        let axis_y = normal.cross(&axis_x).normalize();

        return Basis2 {
            axis_x,
            axis_y,
            origin: point_on_plane
        };
    }

    #[inline]
    pub fn project(&self, point: &Point3<TScalar>) -> Point2<TScalar> {
        return Point2::new(
            (point - self.origin).dot(&self.axis_x),
            (point - self.origin).dot(&self.axis_y)
        );
    }

    #[inline]
    pub fn project_back(&self, point: &Point2<TScalar>) -> Point3<TScalar> {
        return self.origin + self.axis_x * point[0] + self.axis_y * point[1];
    }
}