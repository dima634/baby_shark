use crate::geometry::{primitives::triangle3::Triangle3, traits::RealNumber};
use std::hash::Hash;

pub trait Triangles {
    type Scalar: RealNumber;

    fn triangles(&self) -> impl Iterator<Item = Triangle3<Self::Scalar>>;
}

pub trait TriangleMesh {
    type Scalar: RealNumber;
    type VertexId: Eq + PartialEq + PartialOrd + Ord + Hash;

    fn position(&self, vertex: Self::VertexId) -> [Self::Scalar; 3];
    fn vertices(&self) -> impl Iterator<Item = Self::VertexId>;
    fn faces(&self) -> impl Iterator<Item = [Self::VertexId; 3]>;
}

/// Contains constants which defines what is good mesh
pub mod stats {
    pub const IDEAL_INTERIOR_VERTEX_VALENCE: usize = 6;
    pub const IDEAL_BOUNDARY_VERTEX_VALENCE: usize = 4;

    /// Constant used to preallocate memory for some algorithms, it is not limiting anything
    pub const MAX_VERTEX_VALENCE: usize = 10;
}
