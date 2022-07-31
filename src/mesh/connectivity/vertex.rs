use nalgebra::{Point3, Scalar};
use num_traits::{Float, PrimInt};
use super::{traits::{TopologyPrimitive, Vertex, TopologyFlags}, flags};

///
/// Default implementation for Vertex trait
/// 
struct DefaultVertex<TVertexIndexType, TCornerIndexType, TScalarType> 
where 
    TVertexIndexType: PrimInt, 
    TCornerIndexType: PrimInt, 
    TScalarType: Float + Scalar 
{
    corner_index: TCornerIndexType,
    position: Point3<TScalarType>,

    flags: flags::TopologyFlags,
    index: TVertexIndexType
}

impl<TVertexIndexType, TCornerIndexType, TScalarType> Default for DefaultVertex<TVertexIndexType, TCornerIndexType, TScalarType>
where 
    TVertexIndexType: PrimInt, 
    TCornerIndexType: PrimInt, 
    TScalarType: Float + Scalar  
{
    fn default() -> Self {
        return Self {
            index: TVertexIndexType::max_value(), 
            corner_index: TCornerIndexType::max_value(), 
            position: Default::default(), 
            flags: Default::default() 
        };
    }
}

impl<TVertexIndexType, TCornerIndexType, TScalarType> TopologyFlags for DefaultVertex<TVertexIndexType, TCornerIndexType, TScalarType>
where 
    TVertexIndexType: PrimInt, 
    TCornerIndexType: PrimInt, 
    TScalarType: Float + Scalar  
{
    #[inline]
    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &flags::TopologyFlags {
        return &self.flags;
    }
}

impl<TVertexIndexType, TCornerIndexType, TScalarType> TopologyPrimitive for DefaultVertex<TVertexIndexType, TCornerIndexType, TScalarType>
where 
    TVertexIndexType: PrimInt, 
    TCornerIndexType: PrimInt, 
    TScalarType: Float + Scalar  
{
    type IndexType = TVertexIndexType;

    #[inline]
    fn get_index(&self) -> Self::IndexType {
        return self.index;
    }

    fn set_index(&mut self, index: Self::IndexType) -> &mut Self {
        self.index = index;
        return self;
    }
}

impl<TVertexIndexType, TCornerIndexType, TScalarType> Vertex for DefaultVertex<TVertexIndexType, TCornerIndexType, TScalarType>
where 
    TVertexIndexType: PrimInt, 
    TCornerIndexType: PrimInt, 
    TScalarType: Float + Scalar  
{
    type ScalarType = TScalarType;
    type CornerIndexType = TCornerIndexType;

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
    fn get_corner_index(&self) -> Self::CornerIndexType {
        return self.corner_index;
    }

    #[inline]
    fn set_corner_index(&mut self, index: Self::CornerIndexType) -> &mut Self {
        self.corner_index = index;
        return self;
    }
}

