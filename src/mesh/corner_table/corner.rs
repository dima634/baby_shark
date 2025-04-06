use super::face::FaceId;
use super::vertex::VertexId;
use super::CornerTable;
use super::{flags, traits::Flags};
use crate::geometry::traits::RealNumber;
use std::cell::{Ref, RefCell, RefMut};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut, Index, IndexMut};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CornerId(usize);

impl CornerId {
    #[inline]
    pub(super) fn new(index: usize) -> Self {
        debug_assert!(
            index != usize::MAX,
            "CornerId cannot be created with invalid index"
        );
        Self(index)
    }

    #[inline]
    pub(super) fn is_valid(&self) -> bool {
        self.0 != usize::MAX
    }

    #[inline]
    pub(super) fn from_option(corner: Option<CornerId>) -> Self {
        corner.unwrap_or(Self(usize::MAX))
    }

    #[inline]
    pub fn next(&self) -> CornerId {
        if (self.0 % 3) == 2 {
            CornerId(self.0 - 2)
        } else {
            CornerId(self.0 + 1)
        }
    }

    #[inline]
    pub fn previous(&self) -> CornerId {
        if (self.0 % 3) == 0 {
            CornerId(self.0 + 2)
        } else {
            CornerId(self.0 - 1)
        }
    }

    #[inline]
    pub fn face(&self) -> FaceId {
        FaceId::new(self.0 / 3)
    }
}

#[derive(Debug)]
pub struct Corner {
    opposite_corner: CornerId,
    vertex: VertexId,
    flags: RefCell<flags::Flags>,
}

impl Corner {
    pub fn new(opposite_corner: Option<CornerId>, vertex: VertexId) -> Self {
        Self {
            opposite_corner: CornerId::from_option(opposite_corner),
            flags: RefCell::new(flags::Flags::default()),
            vertex,
        }
    }

    #[inline]
    pub fn opposite_corner(&self) -> Option<CornerId> {
        if self.opposite_corner.is_valid() {
            Some(self.opposite_corner)
        } else {
            None
        }
    }

    #[inline]
    pub fn set_opposite_corner(&mut self, corner: Option<CornerId>) -> &mut Self {
        self.opposite_corner = CornerId::from_option(corner);
        self
    }

    #[inline]
    pub fn vertex(&self) -> VertexId {
        self.vertex
    }

    #[inline]
    pub fn set_vertex(&mut self, vertex: VertexId) -> &mut Self {
        self.vertex = vertex;
        self
    }
}

impl Flags for Corner {
    #[inline]
    fn flags(&self) -> impl Deref<Target = flags::Flags> {
        Ref::map(self.flags.borrow(), |flags| flags)
    }

    #[inline]
    fn flags_mut(&self) -> impl DerefMut<Target = flags::Flags> {
        RefMut::map(self.flags.borrow_mut(), |flags| flags)
    }
}

impl PartialEq for Corner {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.opposite_corner == other.opposite_corner && self.vertex == other.vertex
    }
}
impl Eq for Corner {}

impl<TScalar: RealNumber> Index<CornerId> for CornerTable<TScalar> {
    type Output = Corner;

    #[inline]
    fn index(&self, index: CornerId) -> &Self::Output {
        assert!(index.is_valid());
        &self.corners[index.0]
    }
}

impl<TScalar: RealNumber> IndexMut<CornerId> for CornerTable<TScalar> {
    #[inline]
    fn index_mut(&mut self, index: CornerId) -> &mut Self::Output {
        assert!(index.is_valid());
        &mut self.corners[index.0]
    }
}
