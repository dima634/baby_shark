use nalgebra::Point3;
use tabled::Tabled;
use crate::mesh::{self, traits::Floating};
use super::{traits::{TopologyPrimitive, Vertex, TopologyFlags}, flags};

///
/// Default implementation for Vertex trait
/// 
#[derive(PartialEq, Eq, Debug, Tabled)]
pub struct DefaultVertex<TScalarType: Floating> {
    corner_index: usize,
    position: Point3<TScalarType>,

    flags: flags::TopologyFlags,
    index: usize
}

impl<TScalarType: Floating> DefaultVertex<TScalarType> {
    pub fn new(corner_index: usize, position: Point3<TScalarType>, flags: flags::TopologyFlags, index: usize) -> Self { 
        return Self { 
            corner_index, 
            position, 
            flags, 
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

impl<TScalarType: Floating> TopologyFlags for DefaultVertex<TScalarType> {
    #[inline]
    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &flags::TopologyFlags {
        return &self.flags;
    }
}

impl<TScalarType: Floating> TopologyPrimitive for DefaultVertex<TScalarType> {
    #[inline]
    fn get_index(&self) ->  usize {
        return self.index;
    }

    fn set_index(&mut self, index:  usize) -> &mut Self {
        self.index = index;
        return self;
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

/// Aliases
pub type VertexF = DefaultVertex<f32>;
pub type VertexD = DefaultVertex<f64>;
