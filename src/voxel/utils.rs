use crate::helpers::aliases::Vec3i;
use std::cmp::Ordering;

#[cfg(test)]
#[inline]
pub fn box_indices(start: isize, end: isize) -> impl Iterator<Item = Vec3i> {
    region(Vec3i::new(start, start, start), Vec3i::new(end, end, end))
}

#[cfg(test)]
pub fn region(start: Vec3i, end: Vec3i) -> impl Iterator<Item = Vec3i> {
    (start.x..end.x).flat_map(move |x| {
        (start.y..end.y).flat_map(move |y| (start.z..end.z).map(move |z| Vec3i::new(x, y, z)))
    })
}

#[cfg(test)]
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

#[inline]
pub fn option_min_by<T>(a: Option<T>, b: Option<T>, cmp: impl Fn(&T, &T) -> Ordering) -> Option<T> {
    match (a, b) {
        (Some(a), Some(b)) => Some(match cmp(&a, &b) {
            Ordering::Less | Ordering::Equal => a,
            Ordering::Greater => b,
        }),
        (Some(v), None) | (None, Some(v)) => Some(v),
        (None, None) => None,
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
    use super::region_boundary;
    use crate::helpers::aliases::Vec3i;
    use std::collections::BTreeSet;

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
