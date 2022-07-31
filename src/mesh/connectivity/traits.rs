use nalgebra::{Point3, Scalar};
use num_traits::{Float, PrimInt};

trait TopologyEntity: Default {
    type IndexType: PrimInt;

    fn is_deleted(&self) -> bool;
    fn set_deleted(&mut self, deleted: bool) -> &mut Self;

    fn is_visited(&self) -> bool;
    fn set_visited(&mut self, visited: bool) -> &mut Self;

    fn is_valid(&self) -> bool;
    fn set_valid(&mut self, valid: bool) -> &mut Self;

    fn get_index(&self) -> Self::IndexType;
    fn set_index(&mut self, index: Self::IndexType) -> &mut Self;
}

trait Vertex: TopologyEntity {
    type ScalarType: Float + Scalar;

    fn get_position(&self) -> &Point3<Self::ScalarType>;
    fn set_position(&mut self) -> &mut Self;

    fn get_corner_index(&self) -> Self::IndexType;
    fn set_corner_index(&mut self, index: Self::IndexType) -> &mut Self;
}

trait Corner: TopologyEntity {
    fn get_next_corner_index(&self) -> Self::IndexType;
    fn set_next_corner_index(&self, index: Self::IndexType) -> &Self;

    fn get_opposite_corner_index(&self) -> Self::IndexType;
    fn set_opposite_corner_index(&mut self, index: Self::IndexType) -> &mut Self;

    fn get_face_index(&self) -> Self::IndexType;
    fn set_face_index(&mut self, index: Self::IndexType) -> &mut Self;

    fn get_vertex_index(&self) -> Self::IndexType;
    fn set_vertex_index(&mut self, index: Self::IndexType) -> &mut Self;
}

trait Face: TopologyEntity {
    fn get_corner_index(&self) -> Self::IndexType;
    fn set_corner_index(&mut self, index: Self::IndexType) -> &mut Self;
}
