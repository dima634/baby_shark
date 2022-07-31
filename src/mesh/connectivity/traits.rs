use nalgebra::{Point3, Scalar};
use num_traits::{Float, PrimInt};

use super::flags;

pub trait TopologyFlags {
    #[inline]
    fn is_deleted(&self) -> bool {
        return self.get_flags().contains(flags::TopologyFlags::IS_DELETED);
    }

    #[inline]
    fn set_deleted(&mut self, deleted: bool) -> &mut Self {
        self.get_flags_mut().set(flags::TopologyFlags::IS_DELETED, deleted);
        return self;
    }

    #[inline]
    fn is_visited(&self) -> bool {
        return self.get_flags().contains(flags::TopologyFlags::IS_VISITED);
    }

    #[inline]
    fn set_visited(&mut self, visited: bool) -> &mut Self {
        self.get_flags_mut().set(flags::TopologyFlags::IS_VISITED, visited);
        return self;
    }

    fn get_flags_mut(&mut self) -> &mut flags::TopologyFlags;
    fn get_flags(&self) -> &flags::TopologyFlags;
}

pub trait TopologyPrimitive: Default + TopologyFlags {
    type IndexType: PrimInt;

    fn get_index(&self) -> Self::IndexType;
    fn set_index(&mut self, index: Self::IndexType) -> &mut Self;
}

pub trait Vertex: TopologyPrimitive {
    type ScalarType: Float + Scalar;
    type CornerIndexType: PrimInt;

    fn get_position(&self) -> &Point3<Self::ScalarType>;
    fn set_position(&mut self, point: Point3<Self::ScalarType>) -> &mut Self;

    fn get_corner_index(&self) -> Self::CornerIndexType;
    fn set_corner_index(&mut self, index: Self::CornerIndexType) -> &mut Self;
}

pub trait Corner: TopologyPrimitive {
    type FaceIndexType: PrimInt;
    type VertexIndexType: PrimInt;

    fn get_next_corner_index(&self) -> Self::IndexType;
    fn set_next_corner_index(&mut self, index: Self::IndexType) -> &Self;

    fn get_opposite_corner_index(&self) -> Self::IndexType;
    fn set_opposite_corner_index(&mut self, index: Self::IndexType) -> &mut Self;

    fn get_face_index(&self) -> Self::FaceIndexType;
    fn set_face_index(&mut self, index: Self::FaceIndexType) -> &mut Self;

    fn get_vertex_index(&self) -> Self::VertexIndexType;
    fn set_vertex_index(&mut self, index: Self::VertexIndexType) -> &mut Self;
}

pub trait Face: TopologyPrimitive {
    type CornerIndexType: PrimInt;

    fn get_corner_index(&self) -> Self::CornerIndexType;
    fn set_corner_index(&mut self, index: Self::CornerIndexType) -> &mut Self;
}
