use num_traits::PrimInt;

use super::{traits::{TopologyFlags, TopologyPrimitive, Corner}, flags};

///
/// Default implementation for Corner trait
/// 
struct DefaultCorner<TCornerIndexType: PrimInt, TVertexIndexType: PrimInt, TFaceIndexType: PrimInt> {
    next_corner_index: TCornerIndexType,
    opposite_corner_index: TCornerIndexType,
    face_index: TFaceIndexType,
    vertex_index: TVertexIndexType,

    index: TCornerIndexType,
    flags: flags::TopologyFlags
}

impl<TCornerIndexType, TVertexIndexType, TFaceIndexType> Default for DefaultCorner<TCornerIndexType, TVertexIndexType, TFaceIndexType>
where
    TCornerIndexType: PrimInt,
    TVertexIndexType: PrimInt,
    TFaceIndexType: PrimInt
{
    fn default() -> Self {
        return Self { 
            next_corner_index:      TCornerIndexType::max_value(), 
            opposite_corner_index:  TCornerIndexType::max_value(), 
            face_index:             TFaceIndexType::max_value(), 
            vertex_index:           TVertexIndexType::max_value(), 
            index:                  TCornerIndexType::max_value(), 
            flags:                  Default::default() 
        };
    }
}

impl<TCornerIndexType, TVertexIndexType, TFaceIndexType> TopologyFlags for DefaultCorner<TCornerIndexType, TVertexIndexType, TFaceIndexType>
where
    TCornerIndexType: PrimInt,
    TVertexIndexType: PrimInt,
    TFaceIndexType: PrimInt
{
    #[inline]
    fn get_flags_mut(&mut self) -> &mut super::flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &super::flags::TopologyFlags {
        return &self.flags;
    }
}

impl<TCornerIndexType, TVertexIndexType, TFaceIndexType> TopologyPrimitive for DefaultCorner<TCornerIndexType, TVertexIndexType, TFaceIndexType>
where
    TCornerIndexType: PrimInt,
    TVertexIndexType: PrimInt,
    TFaceIndexType: PrimInt
{
    type IndexType = TCornerIndexType;

    #[inline]
    fn get_index(&self) -> Self::IndexType {
        return self.index;
    }

    #[inline]
    fn set_index(&mut self, index: Self::IndexType) -> &mut Self {
        self.index = index;
        return self;
    }
}

impl<TCornerIndexType, TVertexIndexType, TFaceIndexType> Corner for DefaultCorner<TCornerIndexType, TVertexIndexType, TFaceIndexType>
where
    TCornerIndexType: PrimInt,
    TVertexIndexType: PrimInt,
    TFaceIndexType: PrimInt
{
    type FaceIndexType = TFaceIndexType;
    type VertexIndexType = TVertexIndexType;

    #[inline]
    fn get_next_corner_index(&self) -> Self::IndexType {
        return self.next_corner_index;
    }

    #[inline]
    fn set_next_corner_index(&mut self, index: Self::IndexType) -> &Self {
        self.next_corner_index = index;
        return self;
    }

    #[inline]
    fn get_opposite_corner_index(&self) -> Self::IndexType {
        return self.opposite_corner_index;
    }

    #[inline]
    fn set_opposite_corner_index(&mut self, index: Self::IndexType) -> &mut Self {
        self.opposite_corner_index = index;
        return self;
    }

    #[inline]
    fn get_face_index(&self) -> Self::FaceIndexType {
        return self.face_index;
    }

    #[inline]
    fn set_face_index(&mut self, index: Self::FaceIndexType) -> &mut Self {
        self.face_index = index;
        return self;
    }

    #[inline]
    fn get_vertex_index(&self) -> Self::VertexIndexType {
        return self.vertex_index;
    }

    #[inline]
    fn set_vertex_index(&mut self, index: Self::VertexIndexType) -> &mut Self {
        self.vertex_index = index;
        return self;
    }
}
