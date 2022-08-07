use std::collections::HashMap;
use std::hash::Hash;
use nalgebra::Point3;
use crate::{algo::float_hash::{combine_hash, hash_float}, mesh::traits::Floating};

///
/// Hashable type for nalgebra Point3
/// 
struct HashablePoint<TScalar: Floating>(Point3<TScalar>);

impl<TScalar: Floating> PartialEq for HashablePoint<TScalar> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.0 == other.0;
    }
}

impl<'a, TScalar: Floating> Eq for HashablePoint<TScalar> {}

impl<TScalar: Floating> Hash for HashablePoint<TScalar> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let mut hash = hash_float(self.0[0]);
        hash = combine_hash(hash, hash_float(self.0[1]));
        hash = combine_hash(hash, hash_float(self.0[2]));

        //println!("{}", hash);

        state.write_i32(hash);
    }
}

///
/// Stores 3d point -> index map
/// 
pub struct PointIndexMap<TScalar: Floating> {
    map: HashMap<HashablePoint<TScalar>, usize>
}

impl<TScalar: Floating> PointIndexMap<TScalar> {
    pub fn new() -> Self {
        return Self {
            map: HashMap::new()
        }; 
    }
    
    pub fn with_capacity(capacity: usize) -> Self {
        return Self {
            map: HashMap::with_capacity(capacity)
        };
    }

    /// Returns index of point
    #[inline]
    pub fn get_index(&self, point: Point3<TScalar>) -> Option<&usize> {
        let hashable = HashablePoint(point);
        return self.map.get(&hashable);
    }

    /// Inserts new point with specified index to map 
    #[inline]
    pub fn insert(&mut self, point: Point3<TScalar>, index: usize) {
        let hashable = HashablePoint(point);
        self.map.insert(hashable, index);
    }
}
