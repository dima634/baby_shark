use crate::helpers::aliases::Vec3;
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
    pub fn face(&self) -> FaceId {
        self.0.face()
    }

    #[inline]
    pub(super) fn new(corner: CornerId) -> Self {
        Self(corner)
    }
}

impl<S: RealNumber> CornerTable<S> {
    #[inline]
    pub fn edge_exists(&self, edge: EdgeId) -> bool {
        !self[edge.corner()].is_deleted()
    }

    #[inline]
    pub fn edge_positions(&self, edge: EdgeId) -> (Vec3<S>, Vec3<S>) {
        let mut walker = CornerWalker::from_corner(self, edge.corner());
        (
            *walker.move_to_next().vertex().position(),
            *walker.move_to_next().vertex().position(),
        )
    }

    /// Returns vertices of the edge (v1, v2), edge points in following direction v1 -> v2
    #[inline]
    pub fn edge_vertices(&self, edge: EdgeId) -> (VertexId, VertexId) {
        let mut walker = CornerWalker::from_corner(self, edge.corner());
        (
            walker.move_to_next().corner().vertex(),
            walker.move_to_next().corner().vertex(),
        )
    }
    
    #[inline]
    pub fn opposite_edge(&self, edge: EdgeId) -> Option<EdgeId> {
        self[edge.corner()]
            .opposite_corner()
            .map(|corner| EdgeId::new(corner))
    }

    #[inline]
    pub fn previous_edge(&self, edge: EdgeId) -> EdgeId {
        EdgeId::new(edge.corner().previous())
    }

    #[inline]
    pub fn is_edge_on_boundary(&self, edge: EdgeId) -> bool {
        self.opposite_edge(edge).is_none()
    }

    #[inline]
    pub fn edge_faces(&self, edge: EdgeId) -> (FaceId, Option<FaceId>) {
        let corner = edge.corner();
        (corner.face(), self[corner].opposite_corner().map(|c| c.face()))
    }

    /// Returns edge length
    #[inline]
    pub fn edge_length(&self, edge: EdgeId) -> S {
        let (v1, v2) = self.edge_positions(edge);
        (v1 - v2).norm()
    }

    /// Returns edge length
    #[inline]
    pub fn edge_length_squared(&self, edge: EdgeId) -> S {
        let (v1, v2) = self.edge_positions(edge);
        (v1 - v2).norm_squared()
    }
}
