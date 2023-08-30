use nalgebra::Vector3;

/// Check if all bits in mask are zeroes
pub fn is_mask_empty<const SIZE: usize>(mask: &[usize]) -> bool {
    let mut value = 0;

    for i in 0..SIZE {
        value |= mask[i];
    }

    return value == 0;
}

pub fn box_indices(start: usize, end: usize) -> impl Iterator<Item = Vector3<usize>> {
    return (start..end).flat_map(move |x| {
        (start..end).flat_map(move |y| (start..end).map(move |z| Vector3::new(x, y, z)))
    });
}
