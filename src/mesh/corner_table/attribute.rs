use super::*;
use crate::geometry::traits::RealNumber;
use std::ops::{Index, IndexMut};

/// An attribute associated with oriented edges of mesh.
#[derive(Debug)]
pub struct EdgeAttribute<T> {
    data: Vec<T>,
}

impl<T> Default for EdgeAttribute<T> {
    #[inline]
    fn default() -> Self {
        Self { data: vec![] }
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

impl<T: Clone> EdgeAttribute<T> {
    #[inline]
    pub fn fill(&mut self, value: T) {
        self.data.fill(value);
    }
}

impl<S: RealNumber> CornerTable<S> {
    /// Creates a new edge attribute with the same number of elements as the number of edges in the mesh.
    /// It is not guaranteed that the attribute will stay valid if the mesh is modified.
    #[inline]
    pub fn create_edge_attribute<T: Default + Clone>(&self) -> EdgeAttribute<T> {
        EdgeAttribute {
            data: vec![T::default(); self.corners.len()],
        }
    }
}

#[derive(Debug)]
pub struct VertexAttribute<T> {
    data: Vec<T>,
}

impl<T> Index<VertexId> for VertexAttribute<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: VertexId) -> &Self::Output {
        &self.data[index.index()]
    }
}

impl<T> IndexMut<VertexId> for VertexAttribute<T> {
    #[inline]
    fn index_mut(&mut self, index: VertexId) -> &mut Self::Output {
        &mut self.data[index.index()]
    }
}

impl<T: Clone> VertexAttribute<T> {
    #[inline]
    pub fn fill(&mut self, value: T) {
        self.data.fill(value);
    }
}

impl<S: RealNumber> CornerTable<S> {
    /// Creates a new vertex attribute with the same number of elements as the number of vertices in the mesh.
    /// It is not guaranteed that the attribute will stay valid if the mesh is modified.
    #[inline]
    pub fn create_vertex_attribute<T: Default + Clone>(&self) -> VertexAttribute<T> {
        VertexAttribute {
            data: vec![T::default(); self.vertices.len()],
        }
    }
}
