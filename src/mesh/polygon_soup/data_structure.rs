use super::traversal::{EdgesIter, FacesIter};
use crate::{
    geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
    helpers::aliases::Vec3,
    mesh::traits::{TriangleMesh, Triangles},
};

/// Polygon soup
#[derive(Debug)]
pub struct PolygonSoup<S: RealNumber> {
    pub(super) vertices: Vec<Vec3<S>>, // TODO: hide this field
}

impl<TScalar: RealNumber> PolygonSoup<TScalar> {
    #[inline]
    pub fn new() -> Self {
        Default::default()
    }

    #[inline]
    pub fn from_vertices(vertices: Vec<Vec3<TScalar>>) -> Self {
        Self { vertices }
    }

    pub fn add_face(&mut self, v1: Vec3<TScalar>, v2: Vec3<TScalar>, v3: Vec3<TScalar>) {
        self.vertices.push(v1);
        self.vertices.push(v2);
        self.vertices.push(v3);
    }

    #[inline]
    pub fn concat(&mut self, other: PolygonSoup<TScalar>) {
        self.vertices.extend(other.vertices);
    }

    pub fn from_triangles_soup(triangles: impl Iterator<Item = Vec3<TScalar>>) -> Self {
        let mut vertices: Vec<_> = triangles.collect();
        vertices.resize(vertices.len() - vertices.len() % 3, Vec3::zeros());
        Self { vertices }
    }

    pub fn from_vertex_and_face_iters(
        vertices: impl Iterator<Item = Vec3<TScalar>>,
        faces: impl Iterator<Item = usize>,
    ) -> Self {
        let num_faces = faces.size_hint().1.unwrap_or(0);
        let mut soup = Vec::with_capacity(num_faces * 3);
        let vertices: Vec<_> = vertices.collect();

        for vertex_index in faces {
            soup.push(vertices[vertex_index]);
        }

        Self { vertices: soup }
    }

    pub fn from_vertex_and_face_slices(vertices: &[Vec3<TScalar>], faces: &[usize]) -> Self
    where
        Self: Sized,
    {
        let mut soup = Vec::with_capacity(faces.len());

        for &vertex_index in faces {
            soup.push(vertices[vertex_index]);
        }

        Self { vertices: soup }
    }
}

impl<S: RealNumber> Default for PolygonSoup<S> {
    #[inline]
    fn default() -> Self {
        Self { vertices: vec![] }
    }
}

impl<S: RealNumber> Triangles for PolygonSoup<S> {
    type Scalar = S;

    fn triangles(&self) -> impl Iterator<Item = Triangle3<Self::Scalar>> {
        self.vertices
            .chunks_exact(3)
            .map(|chunk| Triangle3::new(chunk[0], chunk[1], chunk[2]))
    }
}

impl<R: RealNumber> TriangleMesh for PolygonSoup<R> {
    type Scalar = R;
    type VertexId = usize;

    #[inline]
    fn position(&self, vertex: Self::VertexId) -> [Self::Scalar; 3] {
        self.vertices[vertex].into()
    }

    #[inline]
    fn vertices(&self) -> impl Iterator<Item = Self::VertexId> {
        0..self.vertices.len()
    }

    fn faces(&self) -> impl Iterator<Item = [Self::VertexId; 3]> {
        (0..self.vertices.len() / 3).map(|i| {
            let base = i * 3;
            [base, base + 1, base + 2]
        })
    }
}

impl<S: RealNumber> PolygonSoup<S> {
    #[inline]
    pub fn faces(&self) -> FacesIter<'_, S> {
        FacesIter::new(self)
    }

    #[inline]
    pub fn vertices(&self) -> impl Iterator<Item = &Vec3<S>> {
        self.vertices.iter()
    }

    #[inline]
    pub fn edges(&self) -> EdgesIter<'_, S> {
        EdgesIter::new(self)
    }
}

impl<TScalar: RealNumber> From<Vec<Vec3<TScalar>>> for PolygonSoup<TScalar> {
    #[inline]
    fn from(value: Vec<Vec3<TScalar>>) -> Self {
        Self::from_vertices(value)
    }
}
