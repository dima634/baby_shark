use super::{corner::CornerId, CornerTable};
use crate::geometry::traits::RealNumber;
use std::{fmt::Debug, hash::Hash};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EdgeId(CornerId); // corner opposite to the edge that has smaller index.

impl EdgeId {
    /// Returns corner opposite to the edge.
    #[inline]
    pub fn corner(&self) -> CornerId {
        self.0
    }

    pub(super) fn new<TScalar: RealNumber>(
        mut corner: CornerId,
        corner_table: &CornerTable<TScalar>,
    ) -> Self {
        if let Some(opposite) = corner_table[corner].opposite_corner() {
            corner = corner.min(opposite);
        }

        Self(corner)
    }
}
