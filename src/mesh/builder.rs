use nalgebra::Point3;

use super::traits::Mesh;

pub fn cube<T: Mesh>(origin: Point3<T::ScalarType>, x_size: T::ScalarType, y_size: T::ScalarType, z_size: T::ScalarType) -> T {
    let vertices = [
        origin,
        Point3::new(origin.x,          origin.y + y_size,  origin.z),
        Point3::new(origin.x + x_size, origin.y + y_size,  origin.z),
        Point3::new(origin.x + x_size, origin.y,           origin.z),

        Point3::new(origin.x,          origin.y,           origin.z + z_size),
        Point3::new(origin.x,          origin.y + y_size,  origin.z + z_size),
        Point3::new(origin.x + x_size, origin.y + y_size,  origin.z + z_size),
        Point3::new(origin.x + x_size, origin.y,           origin.z + z_size)
    ];

    let faces = [
        0, 1, 2, 0, 2, 3,
        6, 5, 4, 7, 6, 4,
        0, 4, 5, 0, 5, 1,
        0, 7, 4, 0, 3, 7,
        3, 6, 7, 3, 2, 6,
        1, 5, 6, 1, 6, 2
    ];

    return T::from_vertices_and_indices(&vertices, &faces);
}
