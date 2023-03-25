use std::mem::swap;

use nalgebra::{Point3, Vector3};
use nalgebra_glm::{min2, max2};
use num_traits::Float;

use crate::{
    geometry::traits::{
        RealNumber, 
        ClosestPoint3,
        HasBBox3, 
        HasScalarType, 
        IntersectsTriangle3, 
        Number, 
        IntersectsPlane3
    }, 
    algo::utils::{has_same_sign, triple_product}
};

use super::{box3::Box3, ray3::Ray3, line_segment3::LineSegment3, line3::Line3, plane3::{Plane3, Plane3Plane3Intersection}};

pub type BarycentricCoordinates<TScalar> = Vector3<TScalar>;

/// 3D triangle
pub struct Triangle3<TScalar: Number> {
    a: Point3<TScalar>,
    b: Point3<TScalar>,
    c: Point3<TScalar>
}

impl<TScalar: RealNumber> Triangle3<TScalar> {
    pub fn new(a: Point3<TScalar>, b: Point3<TScalar>, c: Point3<TScalar>) -> Self { 
        return Self { a, b, c } 
    }

    #[inline]
    pub fn p1(&self) -> &Point3<TScalar> {
        return &self.a;
    }

    #[inline]
    pub fn p2(&self) -> &Point3<TScalar> {
        return &self.b;
    }

    #[inline]
    pub fn p3(&self) -> &Point3<TScalar> {
        return &self.c;
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

    #[inline]
    pub fn get_normal(&self) -> Vector3<TScalar> {
        return Triangle3::normal(&self.a, &self.b, &self.c);
    }

    #[inline]
    pub fn get_quality(&self) -> TScalar {
        return Triangle3::quality(&self.a, &self.b, &self.c);
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
    pub fn intersects_triangle3(&self, _other: &Triangle3<TScalar>) -> bool {
        todo!()
    }

    /// Returns barycentric coordinates of line - triangle intersection point
    pub fn intersects_line3_at(&self, line: &Line3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        return line_triangle_intersection_moller::<false, TScalar>(self, line);
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
        let intersection = self.intersects_line3_at(line_segment.get_line());

        match intersection {
            Some(i) => {
                let t = i.1;
                if !line_segment.is_on_segment(t) {
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
        let intersection = line_triangle_intersection_moller::<true, TScalar>(self, ray.get_line());

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

#[derive(PartialEq, Debug)]
pub enum Triangle3Triangle3Intersection<TScalar: RealNumber> {
    LineSegment(LineSegment3<TScalar>),
    Point(Point3<TScalar>),
    Coplanar
}

impl<TScalar: RealNumber> IntersectsTriangle3 for Triangle3<TScalar> {
    type Output = Triangle3Triangle3Intersection<TScalar>;

    // http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
    fn intersects_triangle3_at(&self, other: &Triangle3<Self::ScalarType>) -> Option<Self::Output> {
        let p1 = self.plane();
        let p2 = other.plane();

        let (d0t1, d1t1, d2t1) = distances_from_triangle_to_plane(self, &p2);

        // Reject as trivial if all points of triangle 1 are on same side of triangle 2 plane
        if (d0t1 > TScalar::zero() &&  d1t1 > TScalar::zero() &&  d2t1 > TScalar::zero()) ||
           (d0t1 < TScalar::zero() &&  d1t1 < TScalar::zero() &&  d2t1 < TScalar::zero())
        {
            return None;
        }
    
        let (d0t2, d1t2, d2t2) = distances_from_triangle_to_plane(other, &p1);

        if (d0t2 > TScalar::zero() &&  d1t2 > TScalar::zero() &&  d2t2 > TScalar::zero()) ||
           (d0t2 < TScalar::zero() &&  d1t2 < TScalar::zero() &&  d2t2 < TScalar::zero())
        {
            return None;
        }

        let line = p1.intersects_plane3_at(&p2).unwrap();

        return match line {
            Plane3Plane3Intersection::Line(line) => {
                let (t1t1, t2t1) = calculate_line_intervals(self, &line, d0t1, d1t1, d2t1);
                let (t1t2, t2t2) = calculate_line_intervals(other, &line, d0t2, d1t2, d2t2);
    
                let intervals_do_not_overlap =
                    (t2t1 < t1t2) ||
                    (t2t2 < t1t1);

                if  intervals_do_not_overlap {
                    return None;
                }
    
                // Intersection is intervals overlap
                let t_min = Float::max(t1t1, t1t2);
                let t_max = Float::min(t2t1, t2t2);

                // Is interval a point (zero length)?
                if t_min == t_max {
                    return Some(Triangle3Triangle3Intersection::Point(line.point_at(t_min)));
                }

                let segment = LineSegment3::from_line_and_t(&line, Float::max(t1t1, t1t2), Float::min(t2t1, t2t2));
                return Some(Triangle3Triangle3Intersection::LineSegment(segment));
            },
            Plane3Plane3Intersection::Plane => Some(Triangle3Triangle3Intersection::Coplanar)
        }
    }
}

#[allow(dead_code)]
fn line_triangle_intersection<TScalar: RealNumber>(triangle: Triangle3<TScalar>, line: &Line3<TScalar>) -> Option<BarycentricCoordinates<TScalar>> {
    let pa = triangle.a - line.get_point();
    let pb = triangle.b - line.get_point();
    let pc = triangle.c - line.get_point();

    // Test if pq is inside the edges bc, ca and ab. Done by testing
    // that the signed tetrahedral volumes, computed using scalar triple
    // products, all have same sign
    let mut u = triple_product(line.get_direction(), &pc, &pb);
    let mut v = triple_product(line.get_direction(), &pa, &pc);
    let mut w = triple_product(line.get_direction(), &pb, &pa);

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
fn line_triangle_intersection_moller<const FACE_CULLING: bool, TScalar: RealNumber>(triangle: &Triangle3<TScalar>, line: &Line3<TScalar>) -> Option<(BarycentricCoordinates<TScalar>, TScalar)> {
        let edge1 = triangle.b - triangle.a;
        let edge2 = triangle.c - triangle.a;

        let pvec = line.get_direction().cross(&edge2);
        let det = edge1.dot(&pvec);

        if FACE_CULLING {
            if det < TScalar::epsilon() {
                return None;
            }

            let tvec = line.get_point() - triangle.a;
            let mut u = tvec.dot(&pvec);

            if u < TScalar::zero() || u > det {
                return None;
            }

            let qvec = tvec.cross(&edge1);
            let mut v = line.get_direction().dot(&qvec);

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

            let tvec = line.get_point() - triangle.a;
            let u = tvec.dot(&pvec) * inv_det;

            if u < TScalar::zero() || u > TScalar::one() {
                return None;
            }

            let qvec = tvec.cross(&edge1);
            let v = line.get_direction().dot(&qvec) * inv_det;

            if v < TScalar::zero() || v + u > TScalar::one() {
                return None;
            }

            let t = edge2.dot(&qvec) * inv_det;
            let w = TScalar::one() - u - v;

            return Some((Vector3::new(w, u, v), t));
        }
    }

#[inline]
fn distances_from_triangle_to_plane<TScalar: RealNumber>(triangle: &Triangle3<TScalar>, plane: &Plane3<TScalar>) -> (TScalar, TScalar, TScalar) {
    return (
        plane.distance_to_point(&triangle.a),
        plane.distance_to_point(&triangle.b),
        plane.distance_to_point(&triangle.c)
    );
}

fn calculate_line_intervals<TScalar: RealNumber>(triangle: &Triangle3<TScalar>, line: &Line3<TScalar>, d0: TScalar, d1: TScalar, d2: TScalar) -> (TScalar, TScalar) {
    let p0 = line.get_direction().dot(&(triangle.a - line.get_point()));
    let p1 = line.get_direction().dot(&(triangle.b - line.get_point()));
    let p2 = line.get_direction().dot(&(triangle.c - line.get_point()));

    let mut t1;
    let mut t2;

    if d0 * d2 > TScalar::zero() {
        t1 = calculate_line_parameter(p0, p1, d0, d1);
        t2 = calculate_line_parameter(p2, p1, d2, d1);
    } else if d0 * d1 > TScalar::zero() {
        t1 = calculate_line_parameter(p0, p2, d0, d2);
        t2 = calculate_line_parameter(p1, p2, d1, d2);
    } else if d1 * d2 > TScalar::zero() || d0 != TScalar::zero() {
        t1 = calculate_line_parameter(p0, p1, d0, d1);
        t2 = calculate_line_parameter(p0, p2, d0, d2);     
    } else if d1 != TScalar::zero() {
        t1 = calculate_line_parameter(p1, p0, d1, d0);
        t2 = calculate_line_parameter(p1, p2, d1, d2);           
    } else if d2 != TScalar::zero() {
        t1 = calculate_line_parameter(p2, p0, d2, d0);
        t2 = calculate_line_parameter(p2, p1, d2, d1);          
    } else {
        panic!("WTF");
    }

    if t1 > t2 {
        swap(&mut t1, &mut t2);
    }

    return (t1, t2);
}

#[inline]
fn calculate_line_parameter<TScalar: RealNumber>(p0: TScalar, p1: TScalar, d0: TScalar, d1: TScalar) -> TScalar {
    return p0 + (p1 - p0) * (d0 / (d0 - d1));
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};
    use num_traits::Float;

    use crate::geometry::{
        primitives::{
            line3::Line3, 
            triangle3::Triangle3, 
            line_segment3::LineSegment3, 
            ray3::Ray3
        }, 
        traits::{
            ClosestPoint3, 
            IntersectsTriangle3
        }
    };

    use super::Triangle3Triangle3Intersection;

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

    #[test]
    fn triangle_triangle_intersection() {
        use Triangle3Triangle3Intersection::LineSegment;
        use Triangle3Triangle3Intersection::Point;
        use Triangle3Triangle3Intersection::Coplanar;

        let t1 = Triangle3::new(
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0)
        );
        // Test intersection against itself
        let t1t1_expected = Coplanar;
        let t1t1_actual = t1.intersects_triangle3_at(&t1);
        assert!(t1t1_actual.is_some());
        assert_eq!(t1t1_expected, t1t1_actual.unwrap());

        // Intersection at edge
        let t2 = Triangle3::new(
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, -1.0)
        );
        let t1t2_expected = LineSegment3::new(
            &Point3::new(0.0, 0.0, 0.0),
            &Point3::new(0.0, 1.0, 0.0)
        );
        let t1t2_actual = t1.intersects_triangle3_at(&t2);
        assert!(t1t2_actual.is_some());
        assert_eq!(LineSegment(t1t2_expected), t1t2_actual.unwrap());

        // Intersection at point on edge
        let t3 = Triangle3::new(
            Point3::new(0.0, 0.5, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0)
        );
        let t1t3_expected = Point3::new(0.0, 0.5, 0.0);
        let t1t3_actual = t1.intersects_triangle3_at(&t3);
        assert!(t1t3_actual.is_some());
        assert_eq!(Point(t1t3_expected), t1t3_actual.unwrap());

        // Intersection at point on triangle
        let t4 = Triangle3::new(
            Point3::new(0.2, 0.2, 0.0),
            Point3::new(0.2, 0.0, 1.0),
            Point3::new(0.2, 1.0, 1.0)
        );
        let t1t4_expected = Point3::new(0.2, 0.2, 0.0);
        let t1t4_actual = t1.intersects_triangle3_at(&t4);
        assert!(t1t4_actual.is_some());
        assert_eq!(Point(t1t4_expected), t1t4_actual.unwrap());

        // No intersection but coplanar
        let t5 = Triangle3::new(
            Point3::new(5.0, 1.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0)
        );
        let t1t5_actual = t1.intersects_triangle3_at(&t5);
        assert!(t1t5_actual.is_some());
        assert_eq!(Coplanar, t1t5_actual.unwrap());

        // No intersection
        let t6 = Triangle3::new(
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, 0.0, -1.0)
        );
        assert!(t1.intersects_triangle3_at(&t6).is_none());

        // Line segment intersection
        let t7 = Triangle3::new(
            Point3::new(0.5, 5.0, -1.0),
            Point3::new(0.5, -5.0, -1.0),
            Point3::new(0.5, 0.0, 5.0)
        );
        let t1t7_expected = LineSegment(
            LineSegment3::new(
                &Point3::new(0.5, 0.5, 0.0),
                &Point3::new(0.5, 0.0, 0.0)
            )
        );
        let t1t7_actual = t1.intersects_triangle3_at(&t7);
        assert!(t1t7_actual.is_some());
        assert_eq!(t1t7_expected, t1t7_actual.unwrap());
    }
}
