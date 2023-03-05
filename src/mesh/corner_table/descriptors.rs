
use std::{hash::Hash, fmt::{Display, Debug}};

use crate::{geometry::traits::RealNumber, mesh::traits::Edge};

use super::corner_table::CornerTable;

/// 
/// Edge descriptor for corner table.
/// Edge is saved as a corner opposite to it that has smaller index.
/// 
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
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

impl Display for EdgeRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "{}", self.corner_index);
    }
}

impl Debug for EdgeRef {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "corner_index: {}", &self.corner_index);
    }
}
