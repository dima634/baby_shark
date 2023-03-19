use std::collections::HashMap;
use std::hash::Hash;
use nalgebra::Point3;
use crate::{algo::float_hash::{combine_hash, hash_float}, geometry::traits::RealNumber};

///
/// Hashable type for nalgebra Point3
/// 
struct HashablePoint<TScalar: RealNumber>(Point3<TScalar>);

impl<TScalar: RealNumber> PartialEq for HashablePoint<TScalar> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.0 == other.0;
    }
}

impl<TScalar: RealNumber> Eq for HashablePoint<TScalar> {}

impl<TScalar: RealNumber> Hash for HashablePoint<TScalar> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let mut hash = hash_float(self.0[0]);
        hash = combine_hash(hash, hash_float(self.0[1]));
        hash = combine_hash(hash, hash_float(self.0[2]));

        state.write_i32(hash);
    }
}

///
/// Stores 3d point -> index map
/// 
pub struct PointIndexMap<TScalar: RealNumber> {
    map: HashMap<HashablePoint<TScalar>, usize>
}

impl<TScalar: RealNumber> PointIndexMap<TScalar> {
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

impl<TScalar: RealNumber> Default for PointIndexMap<TScalar> {
    #[inline]
    fn default() -> Self {
        return Self::new();
    }
}
