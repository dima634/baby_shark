use std::cell::UnsafeCell;
use nalgebra::Point3;
use tabled::Tabled;
use crate::mesh::{traits::Floating};
use crate::helpers::display::display_unsafecell;
use super::{traits::Flags, flags};

///
/// Default implementation for Vertex trait
/// 
#[derive(Debug, Tabled)]
pub struct Vertex<TScalarType: Floating> {
    corner_index: usize,
    position: Point3<TScalarType>,

    #[tabled(display_with = "display_unsafecell")]
    flags: UnsafeCell<flags::Flags>
}

impl<TScalarType: Floating> Vertex<TScalarType> {
    pub fn new(corner_index: usize, position: Point3<TScalarType>, flags: flags::Flags) -> Self { 
        return Self { 
            corner_index, 
            position, 
            flags: UnsafeCell::new(flags)
        };
    }
}

impl<TScalarType: Floating> Default for Vertex<TScalarType> {
    fn default() -> Self {
        return Self {
            corner_index: usize::max_value(), 
            position: Default::default(), 
            flags: Default::default() 
        };
    }
}

impl<TScalarType: Floating> Flags for Vertex<TScalarType> {
    #[inline]
    fn get_flags(&self) -> &UnsafeCell<flags::Flags> {
        return &self.flags;
    }
}

impl<TScalarType: Floating> Vertex<TScalarType> {
    #[inline]
    pub fn get_position(&self) -> &Point3<TScalarType> {
        return &self.position;
    }

    #[inline]
    pub fn set_position(&mut self, point: Point3<TScalarType>) -> &mut Self {
        self.position = point;
        return self;
    }

    #[inline]
    pub fn get_corner_index(&self) ->  usize {
        return self.corner_index;
    }

    #[inline]
    pub fn set_corner_index(&mut self, index:  usize) -> &mut Self {
        self.corner_index = index;
        return self;
    }
}

impl<TScalarType: Floating> PartialEq for Vertex<TScalarType> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return 
            self.corner_index  == other.corner_index && 
            self.position      == other.position;
    }
}
impl<TScalarType: Floating> Eq for Vertex<TScalarType> {}

/// Aliases
pub type VertexF = Vertex<f32>;
pub type VertexD = Vertex<f64>;
