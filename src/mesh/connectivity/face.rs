use num_traits::PrimInt;

use super::{flags, traits::{TopologyFlags, Face, TopologyPrimitive}};

///
/// Default implementation of Face trait
/// 
struct DefaultFace<TFaceIndexType: PrimInt, TCornerIndexType: PrimInt> {
    corner_index: TCornerIndexType,

    flags: flags::TopologyFlags,
    index: TFaceIndexType
}

impl<TFaceIndexType: PrimInt, TCornerIndexType: PrimInt> Default for DefaultFace<TFaceIndexType, TCornerIndexType> {
    fn default() -> Self {
        return Self {
            corner_index: TCornerIndexType::max_value(),
            index: TFaceIndexType::max_value(),
            flags: Default::default() 
        };
    }
}

impl<TFaceIndexType: PrimInt, TCornerIndexType: PrimInt> TopologyFlags for DefaultFace<TFaceIndexType, TCornerIndexType> {
    #[inline]
    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags {
        return &mut self.flags;
    }

    #[inline]
    fn get_flags(&self) -> &flags::TopologyFlags {
        return &self.flags;
    }
}

impl<TFaceIndexType: PrimInt, TCornerIndexType: PrimInt> TopologyPrimitive for DefaultFace<TFaceIndexType, TCornerIndexType> {
    type IndexType = TFaceIndexType;

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

impl<TFaceIndexType: PrimInt, TCornerIndexType: PrimInt> Face for DefaultFace<TFaceIndexType, TCornerIndexType> {
    type CornerIndexType = TCornerIndexType;

    #[inline]
    fn get_corner_index(&self) -> Self::CornerIndexType {
        return self.corner_index;
    }

    fn set_corner_index(&mut self, index: Self::CornerIndexType) -> &mut Self {
        self.corner_index = index;
        return self;
    }
}
