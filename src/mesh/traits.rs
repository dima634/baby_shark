use crate::{
    geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
    helpers::aliases::Vec3,
};

pub trait FromSoup {
    type Scalar: RealNumber; // TODO: move scalar to supertrait

    /// # Arguments
    /// * `triangles` - iterator of triangles, each triangle is represented as a vector of 3 vertices
    fn from_triangles_soup(triangles: impl Iterator<Item = Vec3<Self::Scalar>>) -> Self;
}

pub trait Triangles {
    type Scalar: RealNumber;

    fn triangles(&self) -> impl Iterator<Item = Triangle3<Self::Scalar>>;
}

/// Triangular mesh
pub trait FromIndexed {
    type Scalar: RealNumber;

    /// Creates mesh from vertices and face indices
    fn from_vertex_and_face_iters(
        vertices: impl Iterator<Item = Vec3<Self::Scalar>>,
        faces: impl Iterator<Item = usize>,
    ) -> Self;

    /// Creates mesh from vertices and face indices saved in slices
    #[inline]
    fn from_vertex_and_face_slices(vertices: &[Vec3<Self::Scalar>], faces: &[usize]) -> Self
    where
        Self: Sized,
    {
        Self::from_vertex_and_face_iters(vertices.iter().cloned(), faces.iter().cloned())
    }
}

/// Contains constants which defines what is good mesh
pub mod stats {
    pub const IDEAL_INTERIOR_VERTEX_VALENCE: usize = 6;
    pub const IDEAL_BOUNDARY_VERTEX_VALENCE: usize = 4;

    /// Constant used to preallocate memory for some algorithms, it is not limiting anything
    pub const MAX_VERTEX_VALENCE: usize = 10;
}
