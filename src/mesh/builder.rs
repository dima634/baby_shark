use crate::helpers::aliases::Vec3;

use super::traits::Mesh;

pub fn cube<T: Mesh>(origin: Vec3<T::ScalarType>, x_size: T::ScalarType, y_size: T::ScalarType, z_size: T::ScalarType) -> T {
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

    T::from_slices(&vertices, &faces)
}
