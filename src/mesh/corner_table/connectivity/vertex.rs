use std::{cell::{Ref, RefCell, RefMut}, fmt::Debug, ops::{Deref, DerefMut}};
use tabled::Tabled;
use crate::{helpers::{display::display_refcell, aliases::Vec3}, geometry::traits::RealNumber};
use super::{traits::Flags, flags};

///
/// Default implementation for Vertex trait
/// 
#[derive(Tabled)]
pub struct Vertex<TScalarType: RealNumber> {
    corner_index: usize,
    position: Vec3<TScalarType>,

    #[tabled(display_with = "display_refcell")]
    flags: RefCell<flags::Flags>
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
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
    pub fn get_position(&self) -> &Vec3<TScalarType> {
        &self.position
    }

    #[inline]
    pub fn set_position(&mut self, point: Vec3<TScalarType>) -> &mut Self {
        self.position = point;
        self
    }

    #[inline]
    pub fn get_corner_index(&self) ->  usize {
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

impl<T: RealNumber> Debug for Vertex<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Vertex")
            .field("corner_index", &self.corner_index)
            .field("position", &self.position)
            .field("flags", &self.flags.borrow().bits())
            .finish()
    }
}

/// Aliases
pub type VertexF = Vertex<f32>;
pub type VertexD = Vertex<f64>;
