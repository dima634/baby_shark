use crate::{mesh::traits::Mesh, geometry::{traits::RealNumber, primitives::triangle3::Triangle3}, helpers::aliases::Vec3};
use super::traversal::{FacesIter, VerticesIter, EdgesIter};

///
/// Polygon soup
/// 
#[derive(Debug)]
pub struct PolygonSoup<TScalar: RealNumber> {
   pub(super) vertices: Vec<Vec3<TScalar>>
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
}

impl<TScalar: RealNumber> Default for PolygonSoup<TScalar> {
    fn default() -> Self {
        Self { vertices: Vec::new() }
    }
}

impl<TScalar: RealNumber> Mesh for PolygonSoup<TScalar> {
    type ScalarType = TScalar;

    type EdgeDescriptor = usize;
    type VertexDescriptor = usize;
    type FaceDescriptor = usize;

    type FacesIter<'iter> = FacesIter<'iter, TScalar>;
    type VerticesIter<'iter> = VerticesIter<'iter, TScalar>;
    type EdgesIter<'iter> = EdgesIter<'iter, TScalar>;

    fn from_iters(vertices: impl Iterator<Item = Vec3<Self::ScalarType>>, faces: impl Iterator<Item = usize>) -> Self {
        let num_faces = faces.size_hint().1.unwrap_or(0);
        let mut soup = Vec::with_capacity(num_faces * 3);
        let vertices: Vec<_> = vertices.collect();

        for vertex_index in faces {
            soup.push(vertices[vertex_index]);
        }

        Self {
            vertices: soup
        }
    }

    fn from_slices(vertices: &[Vec3<Self::ScalarType>], faces: &[usize]) -> Self where Self: Sized {
        let mut soup = Vec::with_capacity(faces.len());

        for &vertex_index in faces {
            soup.push(vertices[vertex_index]);
        }
        
        Self { vertices: soup }
    }

    #[inline]
    fn faces(& self) -> Self::FacesIter<'_> {
        return FacesIter::new(self);
    }

    #[inline]
    fn vertices(&self) -> Self::VerticesIter<'_> {
        return VerticesIter::new(self);
    }

    #[inline]
    fn edges(&self) -> Self::EdgesIter<'_> {
        return EdgesIter::new(self);
    }

    #[inline]
    fn face_positions(&self, face: &Self::FaceDescriptor) -> Triangle3<TScalar> {
        Triangle3::new(self.vertices[*face], self.vertices[face + 1], self.vertices[face + 2])
    }

    #[inline]
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Vec3<Self::ScalarType>, Vec3<Self::ScalarType>) {
        let v2 = if edge % 3 == 2 { edge - 2 } else { edge + 1 };
        (self.vertices[*edge], self.vertices[v2])
    }

    #[inline]
    fn edge_vertices(&self, _edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor) {
        todo!()
    }

    #[inline]
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Vec3<Self::ScalarType> {
        &self.vertices[*vertex]
    }

    #[inline]
    fn vertex_normal(&self, _vertex: &Self::VertexDescriptor) -> Option<Vec3<Self::ScalarType>> {
        todo!()
    }

    fn face_vertices(&self, _face: &Self::FaceDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor, Self::VertexDescriptor) {
        todo!()
    }
}

impl<TScalar: RealNumber> From<Vec<Vec3<TScalar>>> for PolygonSoup<TScalar> {
    #[inline]
    fn from(value: Vec<Vec3<TScalar>>) -> Self {
        Self::from_vertices(value)
    }
}
