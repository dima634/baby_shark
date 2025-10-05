use crate::geometry::basis2d::Basis2;
use crate::geometry::traits::*;
use crate::helpers::aliases::Vec3;
use crate::io::*;

pub fn cube<T: Builder>(
    origin: Vec3<T::Scalar>,
    x_size: T::Scalar,
    y_size: T::Scalar,
    z_size: T::Scalar,
) -> T::Mesh {
    #[rustfmt::skip]
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
        [0, 1, 2],
        [0, 2, 3],
        [6, 5, 4],
        [7, 6, 4],
        [0, 4, 5],
        [0, 5, 1],
        [0, 7, 4],
        [0, 3, 7],
        [3, 6, 7],
        [3, 2, 6],
        [1, 5, 6],
        [1, 6, 2],
    ];

    let mut builder = T::builder_indexed();
    builder
        .add_vertices(vertices.into_iter())
        .expect("should add vertices");
    builder
        .add_faces(faces.into_iter())
        .expect("should add faces");
    builder.finish().expect("should build mesh") // Safe to unwrap because data is for sure valid
}

pub fn cylinder<T: Builder>(
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
            let theta = T::Scalar::usize(j) * T::Scalar::two() * T::Scalar::pi()
                / T::Scalar::usize(num_segments);
            let x = radius * theta.cos();
            let z = radius * theta.sin();
            vertices.push(Vec3::new(x, y, z));
        }
    }

    // Generate top and bottom faces
    let top_center_index = vertices.len();
    vertices.push(Vec3::new(T::Scalar::zero(), height, T::Scalar::zero()));
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

    let mut builder = T::builder_indexed();
    builder
        .add_vertices(vertices.into_iter())
        .expect("should add vertices");
    builder
        .add_faces(indices.into_iter())
        .expect("should add faces");
    builder.finish().expect("should build mesh") // Safe to unwrap because data is for sure valid
}

pub fn cylinder_by_endpoints<T: Builder>(
    point1: &Vec3<T::Scalar>,
    point2: &Vec3<T::Scalar>,
    radius: T::Scalar,
    num_segments: usize,
    num_sections: usize,
) -> Option<T::Mesh> {
    // Calculate the cylinder axis vector
    let axis = point2 - point1;
    let height = axis.magnitude();
    
    // Create an orthonormal basis
    let axis_normalized = axis / height;
    let basis = Basis2::from_normal_and_point(axis_normalized, (*point1).into());
    
    let mut builder = T::builder_indexed();

    // Generate vertices along the cylinder
    for i in 0..=num_sections {
        let t = T::Scalar::usize(i) / T::Scalar::usize(num_sections);
        let center_point = point1 + axis * t;

        for j in 0..num_segments {
            let theta = T::Scalar::usize(j) * T::Scalar::two() * T::Scalar::pi()
                / T::Scalar::usize(num_segments);
            let cos_theta = theta.cos();
            let sin_theta = theta.sin();
            
            // Calculate point on the circle
            let offset = basis.u() * (radius * cos_theta) + basis.v() * (radius * sin_theta);
            let vertex = center_point + offset;
            builder.add_vertex(vertex).ok()?;
        }
    }

    // Add center points for the caps
    let bottom_center_index = builder.add_vertex(*point1).ok()?;
    let top_center_index = builder.add_vertex(*point2).ok()?;

    // Generate side faces
    for i in 0..num_sections {
        for j in 0..num_segments {
            let next_j = (j + 1) % num_segments;
            let current = i * num_segments + j;
            let next = i * num_segments + next_j;
            let above = (i + 1) * num_segments + j;
            let above_next = (i + 1) * num_segments + next_j;

            builder.add_face(above, current, next).ok()?;
            builder.add_face(above, next, above_next).ok()?;
        }
    }

    // Generate cap faces
    for j in 0..num_segments {
        let next_j = (j + 1) % num_segments;

        // Bottom cap (point1)
        let bottom_current = j;
        let bottom_next = next_j;
        builder.add_face(bottom_current, bottom_center_index, bottom_next).ok()?;

        // Top cap (point2)
        let top_current = num_sections * num_segments + j;
        let top_next = num_sections * num_segments + next_j;
        builder.add_face(top_next, top_center_index, top_current).ok()?;
    }

    builder.finish().ok()
}
