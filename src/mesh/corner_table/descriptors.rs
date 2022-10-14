
use std::hash::Hash;

use crate::geometry::traits::RealNumber;

use super::corner_table::CornerTable;

/// 
/// Edge descriptor for corner table.
/// Edge is saved as a corner opposite to it that has smaller index.
/// 
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct EdgeRef {
    corner_index: usize
}

impl EdgeRef {
    pub fn new<TScalar: RealNumber>(corner_index: usize, corner_table: &CornerTable<TScalar>) -> Self { 
        let corner = &corner_table.corners[corner_index];
        return Self { 
            corner_index: corner_index.min(corner.get_opposite_corner_index().unwrap_or(usize::MAX))
        };
    }

    /// Returns corner index of `this` edge reference
    pub fn get_corner_index(&self) -> usize {
        return self.corner_index;
    }
}
