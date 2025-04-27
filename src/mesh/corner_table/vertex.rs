use super::*;
use crate::{geometry::traits::RealNumber, helpers::aliases::Vec3};
use bitflags::bitflags;
use std::{
    fmt::Debug,
    ops::{Index, IndexMut},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VertexId(usize);

impl VertexId {
    #[inline]
    pub(super) fn new(index: usize) -> Self {
        Self(index)
    }

    #[inline]
    pub fn new_invalid() -> Self {
        Self(usize::MAX)
    }

    #[inline]
    pub(super) fn index(&self) -> usize {
        self.0
    }
}

#[derive(Debug, Clone)]
pub struct Vertex<TScalarType: RealNumber> {
    corner: CornerId,
    position: Vec3<TScalarType>,
    flags: VertexFlags,
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
    #[inline]
    pub fn new(corner: CornerId, position: Vec3<TScalarType>) -> Self {
        Self {
            corner,
            position,
            flags: VertexFlags::default(),
        }
    }
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
    #[inline]
    pub fn position(&self) -> &Vec3<TScalarType> {
        &self.position
    }

    #[inline]
    pub fn position_mut(&mut self) -> &mut Vec3<TScalarType> {
        &mut self.position
    }

    #[inline]
    pub fn set_position(&mut self, point: Vec3<TScalarType>) -> &mut Self {
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

impl<TScalarType: RealNumber> PartialEq for Vertex<TScalarType> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.corner == other.corner && self.position == other.position
    }
}
impl<TScalarType: RealNumber> Eq for Vertex<TScalarType> {}

impl<TScalar: RealNumber> Index<VertexId> for CornerTable<TScalar> {
    type Output = Vertex<TScalar>;

    #[inline]
    fn index(&self, index: VertexId) -> &Self::Output {
        &self.vertices[index.0]
    }
}

impl<TScalar: RealNumber> IndexMut<VertexId> for CornerTable<TScalar> {
    #[inline]
    fn index_mut(&mut self, index: VertexId) -> &mut Self::Output {
        &mut self.vertices[index.0]
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
