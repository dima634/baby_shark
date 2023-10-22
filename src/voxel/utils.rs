use nalgebra::Vector3;

/// Check if all bits in mask are zeroes
pub fn is_mask_empty<const SIZE: usize>(mask: &[usize]) -> bool {
    let mut value = 0;

    for i in 0..SIZE {
        value |= mask[i];
    }

    return value == 0;
}

/// Check if all bits in mask are ones
pub fn is_mask_full<const SIZE: usize>(mask: &[usize]) -> bool {
    for i in 0..SIZE {
        if mask[i] != usize::MAX {
            return false;
        }
    }

    true
}

pub fn box_indices(start: isize, end: isize) -> impl Iterator<Item = Vector3<isize>> {
    return (start..end).flat_map(move |x| {
        (start..end).flat_map(move |y| (start..end).map(move |z| Vector3::new(x, y, z)))
    });
}

pub struct BoxIndices {
    size: isize,
    x: isize,
    y: isize,
    z: isize,
}
