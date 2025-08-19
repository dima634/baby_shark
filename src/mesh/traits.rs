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

mod sealed {
    pub trait IndexType {}

    // Zero-cost conversions to usize (on 32/64-bit platforms)
    // `I.try_into().unwrap()` is optimized as `I as usize`
    impl IndexType for usize {}
    impl IndexType for u32 {}
    impl IndexType for u16 {}
    // Runtime-checked conversion - included for ergonomics since i32 is the
    // default integer literal type in Rust. Negative values will panic.
    impl IndexType for i32 {}
}

/// Triangular mesh
pub trait FromIndexed {
    type Scalar: RealNumber;

    /// Creates mesh from vertices and face indices
    fn from_vertex_and_face_iters<V, I>(
        vertices: impl Iterator<Item = V>,
        faces: impl Iterator<Item = I>,
    ) -> Self
    where
        V: Into<[Self::Scalar; 3]>,
        I: TryInto<usize> + sealed::IndexType,
        I::Error: std::fmt::Debug;

    /// Creates mesh from vertices and face indices saved in slices
    #[inline]
    fn from_vertex_and_face_slices<V, I>(vertices: &[V], faces: &[I]) -> Self
    where
        Self: Sized,
        V: Clone + Into<[Self::Scalar; 3]>,
        I: Copy + TryInto<usize> + sealed::IndexType,
        I::Error: std::fmt::Debug,
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
