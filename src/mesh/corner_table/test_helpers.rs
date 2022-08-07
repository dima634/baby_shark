use nalgebra::Point3;
use crate::mesh::traits::Mesh;
use super::prelude::CornerTableF;

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
