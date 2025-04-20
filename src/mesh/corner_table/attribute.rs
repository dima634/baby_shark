use crate::geometry::traits::RealNumber;

use super::{CornerTable, EdgeId};
use std::ops::{Index, IndexMut};

#[derive(Debug)]
pub struct EdgeAttribute<T> {
    data: Vec<T>,
}

impl<T: Default> Default for EdgeAttribute<T> {
    #[inline]
    fn default() -> Self {
        Self { data: Vec::new() }
    }
}

impl<T> Index<EdgeId> for EdgeAttribute<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: EdgeId) -> &Self::Output {
        assert!(index.corner().is_valid(), "Invalid edge");
        &self.data[index.corner().index()]
    }
}

impl<T> IndexMut<EdgeId> for EdgeAttribute<T> {
    #[inline]
    fn index_mut(&mut self, index: EdgeId) -> &mut Self::Output {
        assert!(index.corner().is_valid(), "Invalid edge");
        &mut self.data[index.corner().index()]
    }
}

impl<S: RealNumber> CornerTable<S> {
    #[inline]
    pub fn create_edge_attribute<T: Default + Clone>(&self) -> EdgeAttribute<T> {
        EdgeAttribute {
            data: vec![T::default(); self.corners.len()],
        }
    }
}
