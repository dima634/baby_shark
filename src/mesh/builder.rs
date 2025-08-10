use super::traits::FromIndexed;
use crate::geometry::traits::*;
use crate::helpers::aliases::Vec3;

pub fn cube<T: FromIndexed>(
    origin: Vec3<T::Scalar>,
    x_size: T::Scalar,
    y_size: T::Scalar,
    z_size: T::Scalar,
) -> T {
    let vertices = [
        origin,
        Vec3::new(origin.x,          origin.y + y_size,  origin.z),
        Vec3::new(origin.x + x_size, origin.y + y_size,  origin.z),
        Vec3::new(origin.x + x_size, origin.y,           origin.z),

        Vec3::new(origin.x,          origin.y,           origin.z + z_size),
        Vec3::new(origin.x,          origin.y + y_size,  origin.z + z_size),
        Vec3::new(origin.x + x_size, origin.y + y_size,  origin.z + z_size),
        Vec3::new(origin.x + x_size, origin.y,           origin.z + z_size)
    ];

    let faces = [
        0, 1, 2, 0, 2, 3,
        6, 5, 4, 7, 6, 4,
        0, 4, 5, 0, 5, 1,
        0, 7, 4, 0, 3, 7,
        3, 6, 7, 3, 2, 6,
        1, 5, 6, 1, 6, 2
    ];

    T::from_vertex_and_face_slices(&vertices, &faces)
}

pub fn cylinder<T: FromIndexed>(
    height: T::Scalar,
    radius: T::Scalar,
    num_segments: usize,
    num_sections: usize,
) -> T {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Generate vertices
    for i in 0..=num_sections {
        let y = T::Scalar::usize(i) * height / T::Scalar::usize(num_sections);

        for j in 0..num_segments {
            let theta = T::Scalar::usize(j) * T::Scalar::two() * T::Scalar::pi() / T::Scalar::usize(num_segments);
            let x = radius * theta.cos();
            let z = radius * theta.sin();
            vertices.push(Vec3::new(x, y, z));
        }
    }

    // Generate side faces
    for i in 0..num_sections {
        for j in 0..num_segments {
            let next_j = (j + 1) % num_segments;
            let current = i * num_segments + j;
            let next = i * num_segments + next_j;
            let above = (i + 1) * num_segments + j;
            let above_next = (i + 1) * num_segments + next_j;

            indices.push(current);
            indices.push(above);
            indices.push(next);

            indices.push(next);
            indices.push(above);
            indices.push(above_next);
        }
    }

    // Generate top and bottom faces
    let top_center_index = vertices.len();
    vertices.push(Vec3::new(
        T::Scalar::zero(),
        height,
        T::Scalar::zero(),
    ));
    let bottom_center_index = vertices.len();
    vertices.push(Vec3::new(
        T::Scalar::zero(),
        T::Scalar::zero(),
        T::Scalar::zero(),
    ));

    for j in 0..num_segments {
        let next_j = (j + 1) % num_segments;

        // Top face
        let top_current = num_sections * num_segments + j;
        let top_next = num_sections * num_segments + next_j;
        indices.push(top_center_index);
        indices.push(top_next);
        indices.push(top_current);

        // Bottom face
        let bottom_current = j;
        let bottom_next = next_j;
        indices.push(bottom_center_index);
        indices.push(bottom_current);
        indices.push(bottom_next);
    }

    T::from_vertex_and_face_slices(&vertices, &indices)
}
