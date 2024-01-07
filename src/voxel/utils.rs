use std::ops::Range;

use nalgebra::Vector3;

use crate::helpers::aliases::Vec3i;

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

pub const CUBE_OFFSETS: [Vec3i; 8] = [
    Vec3i::new(0, 0, 0),
    Vec3i::new(1, 0, 0),
    Vec3i::new(0, 1, 0),
    Vec3i::new(1, 1, 0),
    Vec3i::new(0, 0, 1),
    Vec3i::new(1, 0, 1),
    Vec3i::new(0, 1, 1),
    Vec3i::new(1, 1, 1),
];

#[cfg(test)]
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
