use super::*;
use std::{fmt::Debug, hash::Hash};

/// Oriented edge
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EdgeId(CornerId);

impl EdgeId {
    /// Returns corner opposite to the edge.
    #[inline]
    pub fn corner(&self) -> CornerId {
        self.0
    }

    #[inline]
    pub(super) fn new(corner: CornerId) -> Self {
        Self(corner)
    }
}
