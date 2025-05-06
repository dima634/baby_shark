use super::*;
use crate::helpers::aliases::Vec3;
use bitflags::bitflags;
use std::{
    fmt::Debug,
    ops::{Index, IndexMut},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VertexId(u32);

impl VertexId {
    #[inline]
    pub(super) fn new(index: u32) -> Self {
        Self(index)
    }

    #[inline]
    pub(super) fn index(&self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Clone)]
pub struct Vertex<S: RealNumber> {
    corner: CornerId,
    position: Vec3<S>,
    flags: VertexFlags,
}

impl<S: RealNumber> Vertex<S> {
    #[inline]
    pub fn new(corner: CornerId, position: Vec3<S>) -> Self {
        Self {
            corner,
            position,
            flags: VertexFlags::default(),
        }
    }
}

impl<S: RealNumber> Vertex<S> {
    #[inline]
    pub fn position(&self) -> &Vec3<S> {
        &self.position
    }

    #[inline]
    pub fn position_mut(&mut self) -> &mut Vec3<S> {
        &mut self.position
    }

    #[inline]
    pub fn set_position(&mut self, point: Vec3<S>) -> &mut Self {
        self.position = point;
        self
    }

    #[inline]
    pub fn corner(&self) -> CornerId {
        self.corner
    }

    #[inline]
    pub fn set_corner(&mut self, corner: CornerId) -> &mut Self {
        self.corner = corner;
        self
    }

    #[inline]
    pub fn is_deleted(&self) -> bool {
        self.flags.contains(VertexFlags::IS_DELETED)
    }

    #[inline]
    pub fn set_deleted(&mut self, deleted: bool) {
        self.flags.set(VertexFlags::IS_DELETED, deleted);
    }
}

impl<S: RealNumber> PartialEq for Vertex<S> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.corner == other.corner && self.position == other.position
    }
}
impl<S: RealNumber> Eq for Vertex<S> {}

impl<S: RealNumber> Index<VertexId> for CornerTable<S> {
    type Output = Vertex<S>;

    #[inline]
    fn index(&self, index: VertexId) -> &Self::Output {
        &self.vertices[index.0 as usize]
    }
}

impl<S: RealNumber> IndexMut<VertexId> for CornerTable<S> {
    #[inline]
    fn index_mut(&mut self, index: VertexId) -> &mut Self::Output {
        &mut self.vertices[index.0 as usize]
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
    struct VertexFlags: u32 {
        const IS_DELETED = 1;
    }
}

impl Default for VertexFlags {
    #[inline]
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<S: RealNumber> CornerTable<S> {
    /// Returns the number of vertices in the mesh.
    #[inline]
    pub fn count_vertices(&self) -> usize {
        self.vertices.iter().filter(|v| !v.is_deleted()).count()
    }

    #[inline]
    pub fn vertex_position(&self, vertex: VertexId) -> &Vec3<S> {
        self[vertex].position()
    }

    pub fn vertex_normal(&self, vertex: VertexId) -> Option<Vec3<S>> {
        let mut sum = Vec3::zeros();

        self.faces_around_vertex(vertex, |face_index| {
            sum += self.face_normal(face_index).unwrap_or(Vec3::zeros());
        });

        if sum.norm_squared() == S::zero() {
            return None;
        }

        Some(sum.normalize())
    }

    pub fn is_vertex_on_boundary(&self, vertex: VertexId) -> bool {
        let mut walker = CornerWalker::from_vertex(self, vertex);
        walker.move_to_next();
        let started_at = walker.corner_id();

        loop {
            if walker.corner().opposite_corner().is_none() {
                return true;
            }

            walker.move_to_opposite().move_to_previous();

            if started_at == walker.corner_id() {
                break;
            }
        }

        false
    }

    #[inline]
    pub fn vertex_degree(&self, vertex: VertexId) -> usize {
        let mut valence = 0;
        self.vertices_around_vertex(vertex, |_| valence += 1);
        valence
    }
}
