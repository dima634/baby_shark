use std::ops::Range;

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

pub struct GridIter {
    log2: isize,
    two_log2: isize,
    range: Range<isize>,
}

impl GridIter {
    pub fn from_origin_and_size(start: isize, log2: isize) -> Self {
        let dim = 1 << log2;
        let dim_squared = dim * dim;
        let range = start..(dim_squared * dim);
        let two_log2 = log2 + log2;

        Self {
            log2,
            two_log2,
            range,
        }
    }
}

impl Iterator for GridIter {
    type Item = Vector3<isize>;

    fn next(&mut self) -> Option<Self::Item> {
        let next_flat = self.range.next()?;

        let x = next_flat & self.log2;
        let y = (next_flat >> self.log2) & self.log2;
        let z = next_flat >> self.two_log2;

        Some(Vector3::new(x, y, z))
    }
}

mod tests {
    use nalgebra::Vector3;

    use super::GridIter;

    #[test]
    fn test_grid_iter() {
        let mut it = GridIter::from_origin_and_size(0, 1);

        assert_eq!(it.next(), Some(Vector3::new(0, 0, 0)));
        assert_eq!(it.next(), Some(Vector3::new(1, 0, 0)));
        assert_eq!(it.next(), Some(Vector3::new(0, 1, 0)));
        assert_eq!(it.next(), Some(Vector3::new(1, 1, 0)));
        assert_eq!(it.next(), Some(Vector3::new(0, 0, 1)));
        assert_eq!(it.next(), Some(Vector3::new(1, 0, 1)));
        assert_eq!(it.next(), Some(Vector3::new(0, 1, 1)));
        assert_eq!(it.next(), Some(Vector3::new(1, 1, 1)));
        assert_eq!(it.next(), None);
    }
}
