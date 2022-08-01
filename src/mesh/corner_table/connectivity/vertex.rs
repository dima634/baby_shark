use nalgebra::{Point3, Scalar};
use num_traits::Float;
use super::{traits::{TopologyPrimitive, Vertex, TopologyFlags}, flags};

///
/// Default implementation for Vertex trait
/// 
struct DefaultVertex<TScalarType: Float + Scalar > {
    corner_index: usize,
    position: Point3<TScalarType>,

    flags: flags::TopologyFlags,
    index: usize
}

impl<TScalarType: Float + Scalar> Default for DefaultVertex<TScalarType> {
    fn default() -> Self {
        return Self {
            index: usize::max_value(), 
            corner_index: usize::max_value(), 
            position: Default::default(), 
            flags: Default::default() 
        };
    }
}

impl<TScalarType: Float + Scalar> TopologyFlags for DefaultVertex<TScalarType> {
    #[inline]
    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &flags::TopologyFlags {
        return &self.flags;
    }
}

impl<TScalarType: Float + Scalar> TopologyPrimitive for DefaultVertex<TScalarType> {
    #[inline]
    fn get_index(&self) ->  usize {
        return self.index;
    }

    fn set_index(&mut self, index:  usize) -> &mut Self {
        self.index = index;
        return self;
    }
}

impl<TScalarType: Float + Scalar> Vertex for DefaultVertex<TScalarType> {
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

