use crate::geometry::{primitives::triangle3::Triangle3, traits::RealNumber};

pub trait Triangles {
    type Scalar: RealNumber;

    fn triangles(&self) -> impl Iterator<Item = Triangle3<Self::Scalar>>;
}

pub enum PolygonData<R: RealNumber, V: Iterator<Item = [R; 3]>, F: Iterator<Item = [u32; 3]>> {
    Indexed(V, F),
    Soup(V),
}

/// Contains constants which defines what is good mesh
pub mod stats {
    pub const IDEAL_INTERIOR_VERTEX_VALENCE: usize = 6;
    pub const IDEAL_BOUNDARY_VERTEX_VALENCE: usize = 4;

    /// Constant used to preallocate memory for some algorithms, it is not limiting anything
    pub const MAX_VERTEX_VALENCE: usize = 10;
}
