use crate::{
    mesh::{traits::Mesh, polygon_soup::data_structure::PolygonSoup}, 
    geometry::{
        traits::IntersectsTriangle3, 
        primitives::triangle3::Triangle3Triangle3Intersection
    }
};

pub struct Intersections<'a, TMesh: Mesh> {
    mesh: &'a mut TMesh,
    triangulated: PolygonSoup<TMesh::ScalarType>
}

impl<'a, TMesh: Mesh> Intersections<'a, TMesh> {
    fn compute_intersection_candidates(&self, face: &TMesh::FaceDescriptor) -> Vec<TMesh::FaceDescriptor> {
        return self.mesh.faces().collect();
    }

    fn resolve(&mut self) {
        let faces: Vec<_> = self.mesh.faces().collect();

        for face in faces {
            let triangle = self.mesh.face_positions(&face);
            let candidates = self.compute_intersection_candidates(&face);
            let mut intersection_points = vec![
                *triangle.p1(),
                *triangle.p2(),
                *triangle.p3()
            ];

            for candidate in candidates {
                let can_tri = self.mesh.face_positions(&candidate);
                let intersection = triangle.intersects_triangle3_at(&can_tri);

                if let Some(intersection) = intersection {
                    // Collect all intersection points
                    match intersection {
                        Triangle3Triangle3Intersection::LineSegment(segment) => {
                            intersection_points.push(*segment.get_start());
                            intersection_points.push(segment.get_end());
                        },
                        Triangle3Triangle3Intersection::Point(point) => intersection_points.push(point),
                        Triangle3Triangle3Intersection::Coplanar => todo!(),
                    }
                }
            }

            // Triangulate
            let basis = triangle.basis();
            let points2d = intersection_points.iter().map(|p| {
                let p2d = basis.project(p);
                
            });
        }
    }
}


