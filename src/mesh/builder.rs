use super::traits::Mesh;
use crate::helpers::aliases::Vec3;
use nalgebra_glm::pi;
use num_traits::*;

pub fn cube<T: Mesh>(
    origin: Vec3<T::ScalarType>,
    x_size: T::ScalarType,
    y_size: T::ScalarType,
    z_size: T::ScalarType,
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

    T::from_vertices_and_indices(&vertices, &faces)
}

pub fn cylinder<T: Mesh>(
    height: T::ScalarType,
    radius: T::ScalarType,
    num_segments: usize,
    num_sections: usize,
) -> T {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Generate vertices
    for i in 0..=num_sections {
        let y = T::ScalarType::from_usize(i).unwrap() * height
            / T::ScalarType::from_usize(num_sections).unwrap();
        for j in 0..num_segments {
            let theta = T::ScalarType::from_usize(j).unwrap()
                * T::ScalarType::from_f64(2.0).unwrap()
                * pi::<T::ScalarType>()
                / T::ScalarType::from_usize(num_segments).unwrap();
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
        T::ScalarType::zero(),
        height,
        T::ScalarType::zero(),
    ));
    let bottom_center_index = vertices.len();
    vertices.push(Vec3::new(
        T::ScalarType::zero(),
        T::ScalarType::zero(),
        T::ScalarType::zero(),
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

    T::from_vertices_and_indices(&vertices, &indices)
}
