use std::{cell::{Ref, RefCell, RefMut}, fmt::Debug, ops::{Deref, DerefMut, Index, IndexMut}};
use crate::{helpers::aliases::Vec3, geometry::traits::RealNumber};
use super::{traits::Flags, *};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VertexId(usize);

impl VertexId {
    #[inline]
    pub(super) fn new(index: usize) -> Self {
        debug_assert!(index != usize::MAX, "VertexId cannot be created with invalid index");
        Self(index)
    }

    #[inline]
    pub fn new_invalid() -> Self {
        Self(usize::MAX)
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.0 != usize::MAX
    }

    #[inline]
    pub(super) fn index(&self) -> usize {
        self.0
    }
}

#[derive(Debug, Clone)]
pub struct Vertex<TScalarType: RealNumber> {
    corner_index: usize,
    position: Vec3<TScalarType>,
    flags: RefCell<flags::Flags>
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
    #[inline]
    pub fn new(corner_index: usize, position: Vec3<TScalarType>, flags: flags::Flags) -> Self { 
        Self { 
            corner_index, 
            position, 
            flags: RefCell::new(flags)
        }
    }
}

impl<TScalarType: RealNumber> Default for Vertex<TScalarType> {
    fn default() -> Self {
        Self {
            corner_index: usize::max_value(), 
            position: Vec3::zeros(), 
            flags: Default::default() 
        }
    }
}

impl<TScalarType: RealNumber> Flags for Vertex<TScalarType> {
    #[inline]
    fn flags(&self) -> impl Deref<Target = flags::Flags> {
        Ref::map(self.flags.borrow(), |flags| flags)
    }
    
    #[inline]
    fn flags_mut(&self) -> impl DerefMut<Target = flags::Flags> {
        RefMut::map(self.flags.borrow_mut(), |flags| flags)
    }
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
    #[inline]
    pub fn position(&self) -> &Vec3<TScalarType> {
        &self.position
    }

    #[inline]
    pub fn set_position(&mut self, point: Vec3<TScalarType>) -> &mut Self {
        self.position = point;
        self
    }

    #[inline]
    pub fn corner_index(&self) ->  usize {
        self.corner_index
    }

    #[inline]
    pub fn set_corner_index(&mut self, index: usize) -> &mut Self {
        self.corner_index = index;
        self
    }
}

impl<TScalarType: RealNumber> PartialEq for Vertex<TScalarType> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.corner_index == other.corner_index && self.position == other.position
    }
}
impl<TScalarType: RealNumber> Eq for Vertex<TScalarType> {}

impl<TScalar: RealNumber> Index<VertexId> for CornerTable<TScalar> {
    type Output = Vertex<TScalar>;

    #[inline]
    fn index(&self, index: VertexId) -> &Self::Output {
        debug_assert!(index.is_valid());
        &self.vertices[index.0]
    }
}

impl<TScalar: RealNumber> IndexMut<VertexId> for CornerTable<TScalar> {
    #[inline]
    fn index_mut(&mut self, index: VertexId) -> &mut Self::Output {
        debug_assert!(index.is_valid());
        &mut self.vertices[index.0]
    }
}

/// Aliases
pub type VertexF = Vertex<f32>;
pub type VertexD = Vertex<f64>;
