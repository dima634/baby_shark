use std::cell::UnsafeCell;
use tabled::Tabled;
use crate::{helpers::{display::display_unsafecell, aliases::Vec3}, geometry::traits::RealNumber};
use super::{traits::Flags, flags};

///
/// Default implementation for Vertex trait
/// 
#[derive(Debug, Tabled)]
pub struct Vertex<TScalarType: RealNumber> {
    corner_index: usize,
    position: Vec3<TScalarType>,

    #[tabled(display_with = "display_unsafecell")]
    flags: UnsafeCell<flags::Flags>
}

impl<TScalarType: RealNumber> Vertex<TScalarType> {
    pub fn new(corner_index: usize, position: Vec3<TScalarType>, flags: flags::Flags) -> Self { 
        Self { 
            corner_index, 
            position, 
            flags: UnsafeCell::new(flags)
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
    fn get_flags(&self) -> &UnsafeCell<flags::Flags> {
        &self.flags
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
        self.corner_index  == other.corner_index && 
            self.position      == other.position
    }
}
impl<TScalarType: RealNumber> Eq for Vertex<TScalarType> {}

/// Aliases
pub type VertexF = Vertex<f32>;
pub type VertexD = Vertex<f64>;
