use std::collections::HashMap;
use std::hash::Hash;
use nalgebra::Point;
use crate::{algo::float_hash::{combine_hash, hash_float}, geometry::traits::RealNumber};

///
/// Hashable type for nalgebra Point3
/// 
pub struct HashablePoint<const D: usize, TScalar: RealNumber>(Point<TScalar, D>);

impl<const D: usize, TScalar: RealNumber> Into<Point<TScalar, D>> for HashablePoint<D, TScalar> {
    #[inline]
    fn into(self) -> Point<TScalar, D> {
        return self.0;
    }
}

impl<const D: usize, TScalar: RealNumber> From<Point<TScalar, D>> for HashablePoint<D, TScalar> {
    #[inline]
    fn from(value: Point<TScalar, D>) -> Self {
        return Self(value);
    }
}

impl<const D: usize, TScalar: RealNumber> PartialEq for HashablePoint<D, TScalar> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.0 == other.0;
    }
}

impl<const D: usize, TScalar: RealNumber> Eq for HashablePoint<D, TScalar> {}

impl<const D: usize, TScalar: RealNumber> Hash for HashablePoint<D, TScalar> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let mut hash = hash_float(self.0[0]);

        for dim in self.0.iter().skip(1) {
            hash = combine_hash(hash, hash_float(*dim));
        }

        state.write_i32(hash);
    }
}

///
/// Stores 3d point -> index map
/// 
pub struct PointIndexMap<const D: usize, TScalar: RealNumber> {
    map: HashMap<HashablePoint<D, TScalar>, usize>
}

impl<const D: usize, TScalar: RealNumber> PointIndexMap<D, TScalar> {
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
    pub fn get_index(&self, point: Point<TScalar, D>) -> Option<&usize> {
        let hashable = HashablePoint(point);
        return self.map.get(&hashable);
    }

    /// Inserts new point with specified index to map 
    #[inline]
    pub fn insert(&mut self, point: Point<TScalar, D>, index: usize) {
        let hashable = HashablePoint(point);
        self.map.insert(hashable, index);
    }
}

impl<const D: usize, TScalar: RealNumber> Default for PointIndexMap<D, TScalar> {
    #[inline]
    fn default() -> Self {
        return Self::new();
    }
}
