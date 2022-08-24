use std::cell::UnsafeCell;
use nalgebra::Point3;
use tabled::Tabled;
use crate::mesh::{self, traits::Floating};
use crate::helpers::display::display_unsafecell;
use super::{traits::{Vertex, Flags}, flags};

///
/// Default implementation for Vertex trait
/// 
#[derive(Debug, Tabled)]
pub struct DefaultVertex<TScalarType: Floating> {
    corner_index: usize,
    position: Point3<TScalarType>,

    #[tabled(display_with = "display_unsafecell")]
    flags: UnsafeCell<flags::Flags>,
    index: usize
}

impl<TScalarType: Floating> DefaultVertex<TScalarType> {
    pub fn new(corner_index: usize, position: Point3<TScalarType>, flags: flags::Flags, index: usize) -> Self { 
        return Self { 
            corner_index, 
            position, 
            flags: UnsafeCell::new(flags), 
            index 
        };
    }
}

impl<TScalarType: Floating> Default for DefaultVertex<TScalarType> {
    fn default() -> Self {
        return Self {
            index: usize::max_value(), 
            corner_index: usize::max_value(), 
            position: Default::default(), 
            flags: Default::default() 
        };
    }
}

impl<TScalarType: Floating> Flags for DefaultVertex<TScalarType> {
    #[inline]
    fn get_flags(&self) -> &UnsafeCell<flags::Flags> {
        return &self.flags;
    }
}

impl<TScalarType: Floating> mesh::traits::Vertex for DefaultVertex<TScalarType> {
    type ScalarType = TScalarType;

    #[inline]
    fn get_position(&self) -> &Point3<Self::ScalarType> {
        return &self.position;
    }

    #[inline]
    fn set_position(&mut self, point: Point3<Self::ScalarType>) -> &mut Self {
        self.position = point;
        return self;
    }
}

impl<TScalarType: Floating> Vertex for DefaultVertex<TScalarType> {
    #[inline]
    fn get_corner_index(&self) ->  usize {
        return self.corner_index;
    }

    #[inline]
    fn set_corner_index(&mut self, index:  usize) -> &mut Self {
        self.corner_index = index;
        return self;
    }
}

impl<TScalarType: Floating> PartialEq for DefaultVertex<TScalarType> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        let flags_equal: bool;
        unsafe {
            flags_equal = (*self.flags.get()) == (*other.flags.get());
        }

        return 
            self.corner_index  == other.corner_index && 
            self.position      == other.position &&
            self.index         == other.index &&
            flags_equal;
    }
}
impl<TScalarType: Floating> Eq for DefaultVertex<TScalarType> {}

/// Aliases
pub type VertexF = DefaultVertex<f32>;
pub type VertexD = DefaultVertex<f64>;
