use std::mem::swap;
use nalgebra::{Point3, Vector3};
use nalgebra_glm::{max2, min2};
use num_traits::{Float, cast};
use crate::{algo::utils::{cwise_max, cwise_min}};
use super::traits::{HasBBox3, ClosestPoint3, HasScalarType, RealNumber, Number};

/// Infinite line. l(t) = p + v*t
pub struct Line3<TScalar: RealNumber> {
    point: Point3<TScalar>,
    direction: Vector3<TScalar>
}

impl<TScalar: RealNumber> Line3<TScalar> {
    pub fn new(point: Point3<TScalar>, direction: Vector3<TScalar>) -> Self {
        return Self { point, direction };
    }

    pub fn from_points(p1: &Point3<TScalar>, p2: &Point3<TScalar>) -> Self {
        return Self {
            direction: (p2 - p1).normalize(),
            point: *p1
        };
    }

    #[inline]
    pub fn get_point(&self) -> &Point3<TScalar> {
        return &self.point;
    }

    #[inline]
    pub fn get_direction(&self) -> &Vector3<TScalar> {
        return &self.direction;
    }

    #[inline]
    pub fn parameter_at(&self, point: &Point3<TScalar>) -> TScalar {
        return (point - self.point).dot(&self.direction);
    }

    #[inline]
    pub fn point_at(&self, t: TScalar) -> Point3<TScalar> {
        return self.point + self.direction.scale(t);
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        let dot = plane.normal.dot(&self.direction);

        if dot.is_zero() {
            return None;
        }

        return Some((plane.distance - plane.normal.dot(&self.point.coords)) / dot);
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return self.intersects_plane3_at(plane).is_some();
    }    
    
    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        let mut t_min = TScalar::neg_infinity();
        let mut t_max = TScalar::infinity();

        // For all three slabs
        for i in 0..3 {
            if Float::abs(self.direction[i]) < TScalar::epsilon() {
                // Ray is parallel to slab. No hit if origin not within slab
                if self.point[i] < aabb.min[i] || self.point[i] > aabb.max[i] {
                    return None;
                }
            } else {
                // Compute intersection t value of ray with near and far plane of slab
                let ood = TScalar::one() / self.direction[i];
                let mut t1 = (aabb.min[i] - self.point[i]) * ood;
                let mut t2 = (aabb.max[i] - self.point[i]) * ood;

                // Make t1 be intersection with near plane, t2 with far plane
                if t1 > t2 {
                    swap(&mut t1, &mut t2);
                }

                // Compute the intersection of slab intersection intervals
                if t1 > t_min {
                    t_min = t1;
                }

                if t2 < t_max { 
                    t_max = t2;
                }

                // Exit with no collision as soon as slab intersection becomes empty
                if t_min > t_max { 
                    return None;
                }
            }
        }
        
        return Some(t_min);
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        return self.intersects_box3_at(aabb).is_some();
    }
}

impl<TScalar: RealNumber> HasScalarType for Line3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Line3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let t = self.parameter_at(point);
        return self.point + self.direction.scale(t);
    }
}

/// 3D ray
pub struct Ray3<TScalar: RealNumber> { 
    line: Line3<TScalar> 
}

impl<TScalar: RealNumber> Ray3<TScalar> {
    pub fn new(point: Point3<TScalar>, direction: Vector3<TScalar>) -> Self {
        return Self { line: Line3::new(point, direction) };
    }

    #[inline]
    pub fn get_origin(&self) -> &Point3<TScalar> {
        return &self.line.point;
    }

    #[inline]
    pub fn get_direction(&self) -> &Vector3<TScalar> {
        return &self.line.direction;
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_plane3_at(plane) {
            return self.is_on_ray(t);
        }

        return None;
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return self.intersects_plane3_at(plane).is_some();
    }

    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_box3_at(aabb) {
            return self.is_on_ray(t);
        }

        return None;
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        return self.intersects_box3_at(aabb).is_some();
    }  

    fn is_on_ray(&self, t: TScalar) -> Option<TScalar> {
        if t < TScalar::zero() {
            return None;
        }

        return Some(t);
    }
}

impl<TScalar: RealNumber> HasScalarType for Ray3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Ray3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let mut t = self.line.parameter_at(point);

        if t < TScalar::zero() {
            t = TScalar::zero();
        }

        return self.line.point + self.line.direction.scale(t);
    }
}

/// 3D line segment
pub struct LineSegment3<TScalar: RealNumber> {
    line: Line3<TScalar>,
    length: TScalar
}

impl<TScalar: RealNumber> LineSegment3<TScalar> {
    pub fn new(start: &Point3<TScalar>, end: &Point3<TScalar>) -> Self { 
        return Self { 
            line: Line3::from_points(&start, &end), 
            length: (end - start).norm()
        }; 
    }
    
    #[inline]
    pub fn get_start(&self) -> &Point3<TScalar> {
        return &self.line.point;
    }

    #[inline]
    pub fn get_end(&self) -> Point3<TScalar> {
        return self.line.point_at(self.length);
    }

    #[inline]
    pub fn intersects_plane3_at(&self, plane: &Plane3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_plane3_at(plane) {
            return self.is_on_segment(t);
        }

        return None;
    }

    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return self.intersects_plane3_at(plane).is_some();
    }    
    
    #[inline]
    pub fn intersects_box3_at(&self, aabb: &Box3<TScalar>) -> Option<TScalar> {
        if let Some(t) = self.line.intersects_box3_at(aabb) {
            return self.is_on_segment(t);
        }

        return None;
    }

    #[inline]
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        return self.intersects_box3_at(aabb).is_some();
    }  

    fn is_on_segment(&self, t: TScalar) -> Option<TScalar> {
        if t < TScalar::zero() || t > self.length {
            return None;
        }

        return Some(t);
    }
}

impl<TScalar: RealNumber> HasScalarType for LineSegment3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for LineSegment3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let mut t = self.line.parameter_at(point);

        if t < TScalar::zero() {
            t = TScalar::zero();
        } else if t > self.length {
            t = self.length;
        }

        return self.line.point_at(t);
    }
}

/// n * x - d = 0
pub struct Plane3<TScalar: RealNumber> {
    normal: Vector3<TScalar>,
    distance: TScalar
}

impl<TScalar: RealNumber> Plane3<TScalar> {
    pub fn new(normal: Vector3<TScalar>, d: TScalar) -> Self { 
        return Self { normal, distance: d };
    }

    /// Given three noncollinear points (ordered ccw), compute plane equation
    pub fn from_points(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> Self {
        let normal = (b - a).cross(&(c - a)).normalize();
        let d = normal.dot(&a.coords);

        return Self { normal, distance: d };
    }

    #[inline]
    pub fn get_normal(&self) -> &Vector3<TScalar> {
        return &self.normal;
    }

    #[inline]
    pub fn get_distance(&self) -> TScalar {
        return self.distance;
    }

    /// Returns signed distance from point to plane
    #[inline]
    pub fn distance(&self, point: &Point3<TScalar>) -> TScalar {
        return (self.normal.dot(&point.coords) - self.distance) / self.normal.dot(&self.normal); 
    }

    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        // These two lines not necessary with a (center, extents) AABB representation
        let c = aabb.get_center();
        let e = aabb.max - c;
        // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
        let r = e[0]*Float::abs(self.normal[0]) + e[1]*Float::abs(self.normal[1]) + e[2]*Float::abs(self.normal[2]);
        // Compute distance of box center from plane
        let s = self.normal.dot(&c.coords) - self.distance;
        // Intersection occurs when distance s falls within [-r,+r] interval
        return Float::abs(s) <= r
    }
}

impl<TScalar: RealNumber> HasScalarType for Plane3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Plane3<TScalar> {
    /// Returns closest point on plane to given point
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let t = self.distance(point);
        return point - self.normal.scale(t); 
    }
}

/// 3D bounding box
#[derive(Clone, Copy)]
pub struct Box3<TScalar: Number> {
    min: Point3<TScalar>,
    max: Point3<TScalar>
}

impl<TScalar: Number> Box3<TScalar> {
    pub fn new(min: Point3<TScalar>, max: Point3<TScalar>) -> Self {
        return Self { min, max } ;
    }

    pub fn empty() -> Self {
        return Self {
            min: Point3::origin(), 
            max: Point3::origin() 
        } ;
    }

    #[inline]
    pub fn get_min(&self) -> &Point3<TScalar> {
        return &self.min;
    }

    #[inline]
    pub fn get_max(&self) -> &Point3<TScalar> {
        return &self.max;
    }

    #[inline]
    pub fn get_center(&self) -> Point3<TScalar> {
        return (&self.min + &self.max.coords) * cast(0.5).unwrap();
    }

    #[inline]
    pub fn size_x(&self) -> TScalar {
        return self.max.x - self.min.x;
    }

    #[inline]
    pub fn size_y(&self) -> TScalar {
        return self.max.y - self.min.y;
    }

    #[inline]
    pub fn size_z(&self) -> TScalar {
        return self.max.z - self.min.z;
    }

    #[inline]
    pub fn add_box3(&mut self, other: &Box3<TScalar>) -> &mut Self {
        self.max = cwise_max(&self.max, &other.max);
        self.min = cwise_min(&self.min, &other.min);

        return self;
    }

    /// Returns the ith box vertex in order: (x,y,z),(X,y,z),(x,Y,z),(X,Y,z),(x,y,Z),(X,y,Z),(x,Y,Z),(X,Y,Z)
    #[inline]
    pub fn vertex(&self, i: u8) -> Point3<TScalar> {
        return Point3::new(
            self.min.x + TScalar::from(i % 2).unwrap() * self.size_x(), 
            self.min.y + TScalar::from((i / 2) % 2).unwrap() * self.size_y(), 
            self.min.z + TScalar::from(if i > 3 {1} else {0}).unwrap() * self.size_z()
        );
    }

    #[inline]
    pub fn volume(&self) -> TScalar {
        return self.size_x() * self.size_y() * self.size_z();
    }

    pub fn squared_distance(&self, point: &Point3<TScalar>) -> TScalar {
        let mut sq_distance = TScalar::zero();
        
        for i in 0..3 {
            let v = point[i];

            if v < self.min[i] {
                sq_distance += (self.min[i] - v) * (self.min[i] - v);
            }

            if v > self.max[i] {
                sq_distance += (v - self.max[i]) * (v - self.max[i]);
            }
        }

        return sq_distance;
    }

    #[inline]
    pub fn contains_point(&self, point: &Point3<TScalar>) -> bool {
        return 
            point.x >= self.min.x && point.x <= self.max.x &&
            point.y >= self.min.y && point.y <= self.max.y &&
            point.z >= self.min.z && point.z <= self.max.z;
    }

    /// Test bbox - bbox intersection
    pub fn intersects_box3(&self, other: &Box3<TScalar>) -> bool {
        if self.max[0] < other.min[0] || self.min[0] > other.max[0] { 
            return false; 
        }

        if self.max[1] < other.min[1] || self.min[1] > other.max[1] { 
            return false; 
        }

        if self.max[2] < other.min[2] || self.min[2] > other.max[2] { 
            return false; 
        }

        return true; 
    }
}

impl<TScalar: RealNumber> Box3<TScalar> {
    /// Returns the ith diagonal of box
    #[inline]
    pub fn diagonal(&self, i: u8) -> LineSegment3<TScalar> {
        return LineSegment3::new(&self.vertex(i), &self.vertex(7 - i));
    }
    
    /// Test bbox - plane intersection
    #[inline]
    pub fn intersects_plane3(&self, plane: &Plane3<TScalar>) -> bool {
        return plane.intersects_box3(self);
    }

    /// Test bbox - triangle intersection
    #[inline]
    pub fn intersects_triangle3(&self, triangle: &Triangle3<TScalar>) -> bool {
        return triangle.intersects_box3(self);
    }

    #[inline]
    pub fn intersects_sphere3(&self, sphere: &Sphere3<TScalar>) -> bool {
        return sphere.intersects_box3(self);
    }
}

impl<TScalar: RealNumber> HasScalarType for Box3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> ClosestPoint3 for Box3<TScalar> {
    #[inline]
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        return Point3::from(min2(&max2(&self.min.coords, &point.coords), &self.max.coords));
    }
}

pub type BarycentricCoordinates<TScalar> = Vector3<TScalar>;

/// 3D triangle
pub struct Triangle3<TScalar: RealNumber> {
    a: Point3<TScalar>,
    b: Point3<TScalar>,
    c: Point3<TScalar>
}

impl<TScalar: RealNumber> Triangle3<TScalar> {
    pub fn new(a: Point3<TScalar>, b: Point3<TScalar>, c: Point3<TScalar>) -> Self { 
        return Self { a, b, c } 
    }

    #[inline]
    pub fn normal(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> Vector3<TScalar> {
        let cross = (b - a).cross(&(c - a));
        debug_assert!(cross.norm_squared() > TScalar::zero(), "Degenerate face");
        return cross.normalize();
    }

    #[inline]
    pub fn is_degenerate(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> bool {
        let cross = (b - a).cross(&(c - a));
        return cross.norm_squared().is_zero();
    }

    #[inline]
    pub fn area(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> TScalar {
        return (b - a).cross(&(c - a)).norm() * TScalar::from(0.5).unwrap();
    }

    pub fn quality(a: &Point3<TScalar>, b: &Point3<TScalar>, c: &Point3<TScalar>) -> TScalar {
        let ab = b - a;
        let ac = c - a;
        let double_area = ab.cross(&ac).norm();

        if double_area.is_zero() {
            return TScalar::zero();
        }

        let bc = c - b;
        
        let ab_len = ab.norm_squared();
        let ac_len = ac.norm_squared();
        let bc_len = bc.norm_squared();
        let len_max = Float::max(Float::max(ab_len, ac_len), bc_len);
        let equilateral_triangle_aspect_ratio = TScalar::from(1.1547005383792515).unwrap();

        return equilateral_triangle_aspect_ratio * double_area / len_max;
    }

    #[inline]
    pub fn point_at(&self, barycoords: &BarycentricCoordinates<TScalar>) -> Point3<TScalar> {
        return Point3::new(
            barycoords.x * self.a.x + barycoords.y * self.b.x + barycoords.z * self.c.x,
            barycoords.x * self.a.y + barycoords.y * self.b.y + barycoords.z * self.c.y,
            barycoords.x * self.a.z + barycoords.y * self.b.z + barycoords.z * self.c.z,
        );
    }

    #[inline]
    pub fn plane(&self) -> Plane3<TScalar> {
        return Plane3::from_points(&self.a, &self.b, &self.c);
    }

    /// Test triangle - bbox intersection
    pub fn intersects_box3(&self, aabb: &Box3<TScalar>) -> bool {
        if !aabb.intersects_box3(&self.bbox()) {
            return false;
        }

        // Trivial approve - any vertex inside bbox
        if aabb.contains_point(&self.a) || aabb.contains_point(&self.b) || aabb.contains_point(&self.c) {
            return true;
        }

        let some_edge_intersects_box = 
            LineSegment3::new(&self.a, &self.b).intersects_box3(aabb) ||
            LineSegment3::new(&self.a, &self.c).intersects_box3(aabb) ||
            LineSegment3::new(&self.c, &self.b).intersects_box3(aabb);

        if some_edge_intersects_box {
            return true;
        }

        for i in 0..4 {
            if self.intersects_line_segment3(&aabb.diagonal(i)) {
                return true;
            }
        }

        return false;
    }    
    
    /// Test triangle - triangle intersection
    pub fn intersects_triangle3(&self, other: &Triangle3<TScalar>) -> bool {
        todo!()
    }

    /// Returns barycentric coordinates of line - triangle intersection point
    pub fn intersects_line3_at(&self, line: &Line3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        return internal::line_triangle_intersection_moller::<false, TScalar>(self, line);
    }

    #[inline]
    pub fn intersects_line3(&self, line: &Line3<TScalar>) -> bool  {
        return self.intersects_line3_at(line).is_some();
    }
    
    #[inline]
    pub fn intersects_line_segment3(&self, line_segment: &LineSegment3<TScalar>) -> bool  {
        return self.intersects_line_segment3_at(line_segment).is_some();
    }

    /// Face culling off
    #[inline]
    pub fn intersects_line_segment3_at(&self, line_segment: &LineSegment3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        let intersection = self.intersects_line3_at(&line_segment.line);

        match intersection {
            Some(i) => {
                let t = i.1;
                if t < TScalar::zero() || t > line_segment.length {
                    return None;
                }

                return Some(i);
            },
            None => return None,
        }
    }

    /// Face culling on
    #[inline]
    pub fn intersects_ray3_at(&self, ray: &Ray3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        let intersection = internal::line_triangle_intersection_moller::<true, TScalar>(self, &ray.line);

        match intersection {
            Some(i) => {
                let t = i.1;
                if t < TScalar::zero() {
                    return None;
                }

                return Some(i);
            },
            None => return None,
        }
    }

    #[inline]
    pub fn intersects_ray3(&self, ray: &Ray3<TScalar>) -> bool {
        return self.intersects_ray3_at(ray).is_some();
    }
}

impl<TScalar: RealNumber> HasScalarType for Triangle3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> HasBBox3 for Triangle3<TScalar> {
    #[inline]
    fn bbox(&self) -> Box3<TScalar> {
        return Box3::new(
            min2(&self.c.coords, &min2(&self.a.coords, &self.b.coords)).into(),
            max2(&self.c.coords, &max2(&self.a.coords, &self.b.coords)).into(),
        );
    }
}

impl<TScalar: RealNumber> ClosestPoint3 for Triangle3<TScalar> {
    /// Returns closest point on triangle to given point
    fn closest_point(&self, point: &Point3<TScalar>) -> Point3<TScalar> {
        let zero: TScalar = TScalar::zero();

        // Check if P in vertex region outside A
        let ab = self.b - self.a;
        let ac = self.c - self.a;
        let ap = point - self.a;
        let d1 = ab.dot(&ap);
        let d2 = ac.dot(&ap);

        // barycentric coordinates (1,0,0)
        if d1 <= zero && d2 <= zero { 
            return self.a; 
        }

        // Check if P in vertex region outside B
        let bp = point - self.b;
        let d3 = ab.dot(&bp);
        let d4 = ac.dot(&bp);

        // barycentric coordinates (0,1,0)
        if d3 >= zero && d4 <= d3 {
            return self.b; 
        }

        // Check if P in edge region of AB, if so return projection of P onto AB
        let vc = d1*d4 - d3*d2;
        if vc <= zero && d1 >= zero && d3 <= zero {
            let v = d1 / (d1 - d3);
            return self.a + ab.scale(v); // barycentric coordinates (1-v,v,0)
        }

        // Check if P in vertex region outside C
        let cp = point - self.c;
        let d5 = ab.dot(&cp);
        let d6 = ac.dot(&cp);
        
        // barycentric coordinates (0,0,1)
        if d6 >= zero && d5 <= d6 {
            return self.c; 
        }

        // Check if P in edge region of AC, if so return projection of P onto AC
        let vb = d5*d2 - d1*d6;
        if vb <= zero && d2 >= zero && d6 <= zero {
            let w = d2 / (d2 - d6);
            return self.a + ac.scale(w); // barycentric coordinates (1-w,0,w)
        }

        // Check if P in edge region of BC, if so return projection of P onto BC
        let va = d3*d6 - d5*d4;
        if va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return self.b + (self.c - self.b).scale(w); // barycentric coordinates (0,1-w,w)
        }

        // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
        let denom = TScalar::one() / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;

        return self.a + ab * v + ac * w;
    }
}

/// 3D sphere
pub struct Sphere3<TScalar: RealNumber> {
    center: Point3<TScalar>,
    radius: TScalar
}

impl<TScalar: RealNumber> Sphere3<TScalar> {
    pub fn new(center: Point3<TScalar>, radius: TScalar) -> Self { 
        return Self { center, radius };
    }

    #[inline]
    pub fn intersects_box3(&self, bbox: &Box3<TScalar>) -> bool {
        return bbox.squared_distance(&self.center) <= self.radius * self.radius;
    }
}

impl<TScalar: RealNumber> HasScalarType for Sphere3<TScalar> {
    type ScalarType = TScalar;
}

impl<TScalar: RealNumber> HasBBox3 for Sphere3<TScalar> {
    #[inline]
    fn bbox(&self) -> Box3<Self::ScalarType> {
        return Box3::new(
            self.center.coords.add_scalar(-self.radius).into(), 
            self.center.coords.add_scalar(self.radius).into()
        );
    }
}

pub(super) mod internal {
    use nalgebra::Vector3;
    use num_traits::Float;

    use crate::{algo::utils::{triple_product, has_same_sign}, geometry::traits::RealNumber};
    use super::{Triangle3, Line3, BarycentricCoordinates};

    #[allow(dead_code)]
    pub fn line_triangle_intersection<TScalar: RealNumber>(triangle: Triangle3<TScalar>, line: &Line3<TScalar>) -> Option<BarycentricCoordinates<TScalar>> {
        let pa = triangle.a - line.point;
        let pb = triangle.b - line.point;
        let pc = triangle.c - line.point;

        // Test if pq is inside the edges bc, ca and ab. Done by testing
        // that the signed tetrahedral volumes, computed using scalar triple
        // products, all have same sign
        let mut u = triple_product(&line.direction, &pc, &pb);
        let mut v = triple_product(&line.direction, &pa, &pc);
        let mut w = triple_product(&line.direction, &pb, &pa);

        if !has_same_sign(u, v) || !has_same_sign(u, w) {
            return None;
        }

        // Compute the barycentric coordinates (u, v, w) determining the
        // intersection point r, r = u*a + v*b + w*c
        let denom = TScalar::one() / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom; // w = 1.0f - u - v;

        return Some(BarycentricCoordinates::new(u, v, w));
    }

    /// Based on: https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    pub fn line_triangle_intersection_moller<const FACE_CULLING: bool, TScalar: RealNumber>(triangle: &Triangle3<TScalar>, line: &Line3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        let edge1 = triangle.b - triangle.a;
        let edge2 = triangle.c - triangle.a;

        let pvec = line.direction.cross(&edge2);
        let det = edge1.dot(&pvec);

        if FACE_CULLING {
            if det < TScalar::epsilon() {
                return None;
            }

            let tvec = line.point - triangle.a;
            let mut u = tvec.dot(&pvec);

            if u < TScalar::zero() || u > det {
                return None;
            }

            let qvec = tvec.cross(&edge1);
            let mut v = line.direction.dot(&qvec);

            if v < TScalar::zero() || v + u > det {
                return None;
            }

            let mut t = edge2.dot(&qvec);
            let inv_det = TScalar::one() / det;

            t *= inv_det;
            u *= inv_det;
            v *= inv_det;
            let w = TScalar::one() - u - v;

            return Some((Vector3::new(w, u, v), t));
        } else {
            if Float::abs(det) < TScalar::epsilon() {
                return None;
            }

            let inv_det = TScalar::one() / det;

            let tvec = line.point - triangle.a;
            let u = tvec.dot(&pvec) * inv_det;

            if u < TScalar::zero() || u > TScalar::one() {
                return None;
            }

            let qvec = tvec.cross(&edge1);
            let v = line.direction.dot(&qvec) * inv_det;

            if v < TScalar::zero() || v + u > TScalar::one() {
                return None;
            }

            let t = edge2.dot(&qvec) * inv_det;
            let w = TScalar::one() - u - v;

            return Some((Vector3::new(w, u, v), t));
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};
    use num_traits::Float;
    use crate::geometry::{primitives::Ray3, traits::ClosestPoint3};

    use super::{Line3, Triangle3, LineSegment3};

    #[test]
    fn line_closest_point() {
        let line = Line3::<f32>::new(Point3::origin(), Vector3::x_axis().xyz());

        let point1 = Point3::new(1.0, 1.0, 0.0);
        assert_eq!(Point3::new(1.0, 0.0, 0.0), line.closest_point(&point1));

        let point2 = Point3::new(0.0, 1.0, 0.0);
        assert_eq!(Point3::new(0.0, 0.0, 0.0), line.closest_point(&point2));

        let point2 = Point3::new(0.25, 5.0, 0.0);
        assert_eq!(Point3::new(0.25, 0.0, 0.0), line.closest_point(&point2));
    }

    #[test]
    fn line_segments_triangle_intersection() {
        let triangle = Triangle3::<f32>::new(
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0)
        );

        // Segment intersects triangle
        let segment1 = LineSegment3::<f32>::new(
            &Point3::new(2.5, 2.5, -1.0),
            &Point3::new(2.5, 2.5, 1.0)
        );

        let expected1 = Point3::<f32>::new(2.5, 2.5, 0.0);
        let mut intersection = triangle.intersects_line_segment3_at(&segment1);

        assert!(intersection.is_some());
        assert_eq!(expected1, triangle.point_at(&intersection.unwrap().0));

        // Segment outside triangle
        let segment2 = LineSegment3::<f32>::new(
            &Point3::new(2.5, 2.5, -2.0),
            &Point3::new(2.5, 2.5, -1.0)
        );

        intersection = triangle.intersects_line_segment3_at(&segment2);

        assert!(intersection.is_none());
    }

    #[test]
    fn line_triangle_intersection() {
        let triangle = Triangle3::<f32>::new(
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0)
        );

        // Segment intersects triangle 
        let line1 = Line3::<f32>::from_points(
            &Point3::new(2.5, 2.5, -1.0),
            &Point3::new(2.5, 2.5, 1.0)
        );

        let expected1 = Point3::<f32>::new(2.5, 2.5, 0.0);
        let mut intersection = triangle.intersects_line3_at(&line1);

        assert!(intersection.is_some());
        assert_eq!(expected1, triangle.point_at(&intersection.unwrap().0));

        // Segment outside triangle (but line intersects)
        let line2 = Line3::<f32>::from_points(
            &Point3::new(2.5, 2.5, -2.0),
            &Point3::new(2.5, 2.5, -1.0)
        );

        let expected2 = Point3::<f32>::new(2.5, 2.5, 0.0);
        intersection = triangle.intersects_line3_at(&line2);

        assert!(intersection.is_some());
        assert_eq!(expected2, triangle.point_at(&intersection.unwrap().0));
    }

    #[test]
    fn ray_triangle_intersection() {
        let triangle = Triangle3::<f32>::new(
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0)
        );

        // Ray face culled
        let ray1 = Ray3::<f32>::new(
            Point3::new(2.5, 2.5, -1.0),
            Vector3::new(0.0, 0.0, 1.0)
        );

        let mut intersection = triangle.intersects_ray3_at(&ray1);

        assert!(intersection.is_none());

        // Ray intersect triangle
        let ray2 = Ray3::<f32>::new(
            Point3::new(0.0, 5.0, 1.0),
            Vector3::new(0.0, 0.0, -1.0)
        );

        let expected2 = Point3::<f32>::new(0.0, 5.0, 0.0);
        intersection = triangle.intersects_ray3_at(&ray2);

        assert!(intersection.is_some());
        assert_eq!(expected2, triangle.point_at(&intersection.unwrap().0));

        
        // Ray outside triangle
        let ray2 = Ray3::<f32>::new(
            Point3::new(2.5, 2.5, 1.0),
            Vector3::new(0.0, 0.0, 1.0)
        );

        intersection = triangle.intersects_ray3_at(&ray2);

        assert!(intersection.is_none());
    }

    #[test]
    fn triangle_quality() {
        let equilateral_quality = Triangle3::quality(
            &Point3::new(-1.0, 1.5, 0.0), 
            &Point3::new(1.0, -2.0, 0.0), 
            &Point3::new(3.0, 1.5, 0.0)
        );

        assert!((1.0 - equilateral_quality).abs() < 0.01);
    }
}
