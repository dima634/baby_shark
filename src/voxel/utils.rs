use std::{cmp::Ordering, ops::Range};

use crate::helpers::aliases::Vec3i;

#[inline]
pub fn box_indices(start: isize, end: isize) -> impl Iterator<Item = Vec3i> {
    region(Vec3i::new(start, start, start), Vec3i::new(end, end, end))
}

pub fn region(start: Vec3i, end: Vec3i) -> impl Iterator<Item = Vec3i> {
    (start.x..end.x).flat_map(move |x| {
        (start.y..end.y).flat_map(move |y| (start.z..end.z).map(move |z| Vec3i::new(x, y, z)))
    })
}

pub fn region_boundary(start: Vec3i, end: Vec3i) -> impl Iterator<Item = Vec3i> {
    region(start, Vec3i::new(start.x + 1, end.y, end.z))
        .chain(region(Vec3i::new(end.x - 1, start.y, start.z), end))
        .chain(region(
            Vec3i::new(start.x + 1, start.y, start.z),
            Vec3i::new(end.x - 1, end.y, start.z + 1),
        ))
        .chain(region(
            Vec3i::new(start.x + 1, start.y, end.z - 1),
            Vec3i::new(end.x - 1, end.y, end.z),
        ))
        .chain(region(
            Vec3i::new(start.x + 1, start.y, start.z + 1),
            Vec3i::new(end.x - 1, start.y + 1, end.z - 1),
        ))
        .chain(region(
            Vec3i::new(start.x + 1, end.y - 1, start.z + 1),
            Vec3i::new(end.x - 1, end.y, end.z - 1),
        ))
}

#[inline]
pub fn partial_min<T: PartialOrd>(a: T, b: T) -> T {
    match a.partial_cmp(&b) {
        Some(Ordering::Less) => a,
        None => {
            debug_assert!(false, "Partial comparison failed");
            b
        }
        _ => b,
    }
}

#[inline]
pub fn partial_max<T: PartialOrd>(a: T, b: T) -> T {
    match a.partial_cmp(&b) {
        Some(Ordering::Greater) => a,
        None => {
            debug_assert!(false, "Partial comparison failed");
            b
        }
        _ => b,
    }
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
    type Item = Vec3i;

    fn next(&mut self) -> Option<Self::Item> {
        let next_flat = self.range.next()?;

        let x = next_flat & self.log2;
        let y = (next_flat >> self.log2) & self.log2;
        let z = next_flat >> self.two_log2;

        Some(Vec3i::new(x, y, z))
    }
}

//         7 ________ 6 
//         /|       /|  
//       /  |     /  |  
//   4 /_______ /    |  
//    |     |  |5    |  
//    |    3|__|_____|2 
//    |    /   |    /   
//    |  /     |  /     
//    |/_______|/       
//   0          1       
//
pub const CUBE_OFFSETS: [Vec3i; 8] = [
    Vec3i::new(0, 0, 0),
    Vec3i::new(1, 0, 0),
    Vec3i::new(1, 1, 0),
    Vec3i::new(0, 1, 0),
    Vec3i::new(0, 0, 1),
    Vec3i::new(1, 0, 1),
    Vec3i::new(1, 1, 1),
    Vec3i::new(0, 1, 1),
];

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use nalgebra::Vector3;

    use crate::helpers::aliases::Vec3i;

    use super::{region_boundary, GridIter};

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

    #[test]
    fn test_region_boundary() {
        let s = Vec3i::new(0, 0, 0);
        let e = Vec3i::new(3, 3, 3);
        let boundary = BTreeSet::from_iter(region_boundary(s, e).map(|v| v.data.0));

        let expected = BTreeSet::from_iter(
            [
                Vec3i::new(0, 0, 0),
                Vec3i::new(0, 0, 1),
                Vec3i::new(0, 0, 2),
                Vec3i::new(0, 1, 0),
                Vec3i::new(0, 1, 1),
                Vec3i::new(0, 1, 2),
                Vec3i::new(0, 2, 0),
                Vec3i::new(0, 2, 1),
                Vec3i::new(0, 2, 2),
                Vec3i::new(2, 0, 0),
                Vec3i::new(2, 0, 1),
                Vec3i::new(2, 0, 2),
                Vec3i::new(2, 1, 0),
                Vec3i::new(2, 1, 1),
                Vec3i::new(2, 1, 2),
                Vec3i::new(2, 2, 0),
                Vec3i::new(2, 2, 1),
                Vec3i::new(2, 2, 2),
                Vec3i::new(1, 0, 0),
                Vec3i::new(1, 0, 1),
                Vec3i::new(1, 0, 2),
                Vec3i::new(1, 2, 0),
                Vec3i::new(1, 2, 1),
                Vec3i::new(1, 2, 2),
                Vec3i::new(1, 1, 0),
                Vec3i::new(1, 1, 2),
            ]
            .map(|v| v.data.0),
        );

        assert_eq!(region_boundary(s, e).count(), 9 + 9 + 3 + 3 + 1 + 1);
        assert_eq!(boundary, expected);

        let s = Vec3i::new(0, 0, 0);
        let e = Vec3i::new(4, 4, 4);
        assert_eq!(region_boundary(s, e).count(), 16 + 16 + 8 + 8 + 4 + 4);
    }
}
