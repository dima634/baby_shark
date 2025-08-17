use crate::geometry::traits::*;
use crate::helpers::aliases::Vec3;
use crate::io::*;

pub fn cube<T: CreateBuilder>(
    origin: Vec3<T::Scalar>,
    x_size: T::Scalar,
    y_size: T::Scalar,
    z_size: T::Scalar,
) -> T::Mesh {
    let mut builder = T::builder(BuildMode::Indexed);

    builder.add_vertex(origin                                                    ).unwrap();
    builder.add_vertex(Vec3::new(origin.x,          origin.y + y_size,  origin.z)).unwrap();
    builder.add_vertex(Vec3::new(origin.x + x_size, origin.y + y_size,  origin.z)).unwrap();
    builder.add_vertex(Vec3::new(origin.x + x_size, origin.y,           origin.z)).unwrap();
    builder.add_vertex(Vec3::new(origin.x,          origin.y,           origin.z + z_size)).unwrap();
    builder.add_vertex(Vec3::new(origin.x,          origin.y + y_size,  origin.z + z_size)).unwrap();
    builder.add_vertex(Vec3::new(origin.x + x_size, origin.y + y_size,  origin.z + z_size)).unwrap();
    builder.add_vertex(Vec3::new(origin.x + x_size, origin.y,           origin.z + z_size)).unwrap();
    
    builder.add_face_indexed(0, 1, 2).unwrap();
    builder.add_face_indexed(0, 2, 3).unwrap();
    builder.add_face_indexed(6, 5, 4).unwrap();
    builder.add_face_indexed(7, 6, 4).unwrap();
    builder.add_face_indexed(0, 4, 5).unwrap();
    builder.add_face_indexed(0, 5, 1).unwrap();
    builder.add_face_indexed(0, 7, 4).unwrap();
    builder.add_face_indexed(0, 3, 7).unwrap();
    builder.add_face_indexed(3, 6, 7).unwrap();
    builder.add_face_indexed(3, 2, 6).unwrap();
    builder.add_face_indexed(1, 5, 6).unwrap();
    builder.add_face_indexed(1, 6, 2).unwrap();

    builder.finish().unwrap()
}

pub fn cylinder<T: CreateBuilder>(
    height: T::Scalar,
    radius: T::Scalar,
    num_segments: usize,
    num_sections: usize,
) -> T::Mesh {
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

    // Generate side faces
    for i in 0..num_sections {
        for j in 0..num_segments {
            let next_j = (j + 1) % num_segments;
            let current = i * num_segments + j;
            let next = i * num_segments + next_j;
            let above = (i + 1) * num_segments + j;
            let above_next = (i + 1) * num_segments + next_j;

            indices.push([current, above, next]);
            indices.push([next, above, above_next]);
        }
    }

    for j in 0..num_segments {
        let next_j = (j + 1) % num_segments;

        // Top face
        let top_current = num_sections * num_segments + j;
        let top_next = num_sections * num_segments + next_j;
        indices.push([top_center_index, top_next, top_current]);

        // Bottom face
        let bottom_current = j;
        let bottom_next = next_j;
        indices.push([bottom_center_index, bottom_current, bottom_next]);
    }

    let mut builder = T::builder(BuildMode::Indexed);
    builder.add_vertices(vertices.into_iter()).unwrap();
    builder.add_faces_indexed(indices.into_iter()).unwrap();
    builder.finish().unwrap()
}
