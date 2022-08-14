use nalgebra::Point3;
use crate::mesh::traits::Mesh;
use super::{prelude::CornerTableF, connectivity::{corner::DefaultCorner, vertex::VertexF}};

pub fn create_unit_square_mesh() -> CornerTableF {
    let vertices = vec![
        Point3::<f32>::new(0.0, 1.0, 0.0),
        Point3::<f32>::new(0.0, 0.0, 0.0),
        Point3::<f32>::new(1.0, 0.0, 0.0),
        Point3::<f32>::new(1.0, 1.0, 0.0)
    ];

    let indices = vec![0, 1, 2, 2, 3, 0];

    return CornerTableF::from_vertices_and_indices(&vertices, &indices);
}

pub fn create_single_face_mesh() -> CornerTableF {
    let vertices = vec![
        Point3::<f32>::new(0.0, 1.0, 0.0),
        Point3::<f32>::new(0.0, 0.0, 0.0),
        Point3::<f32>::new(1.0, 0.0, 0.0)
    ];

    let indices = vec![0, 1, 2];

    return CornerTableF::from_vertices_and_indices(&vertices, &indices);
}

pub fn assert_mesh_equals(mesh: &CornerTableF, expected_corners: &Vec<DefaultCorner>, expected_vertices: &Vec<VertexF>) {
    assert_eq!(expected_corners.len(), mesh.corners.len());
    assert_eq!(expected_vertices.len(), mesh.vertices.len());

    for i in 0..expected_corners.len() {
        assert_eq!(expected_corners[i], mesh.corners[i]);
    }

    for i in 0..expected_vertices.len() {
        assert_eq!(expected_vertices[i], mesh.vertices[i]);
    }
}
