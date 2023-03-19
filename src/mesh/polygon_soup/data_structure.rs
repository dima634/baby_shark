use nalgebra::Point3;
use crate::{mesh::traits::{Mesh}, geometry::{traits::RealNumber, primitives::triangle3::Triangle3}};
use super::traversal::{FacesIter, VerticesIter, EdgesIter};

///
/// Polygon soup
/// 
pub struct PolygonSoup<TScalar: RealNumber> {
   pub(super) vertices: Vec<Point3<TScalar>>
}

impl<TScalar: RealNumber> PolygonSoup<TScalar> {
    #[inline]
    pub fn new() -> Self { 
        return Default::default();
    }

    pub fn add_face(&mut self, v1: Point3<TScalar>, v2: Point3<TScalar>, v3: Point3<TScalar>) {
        self.vertices.push(v1);
        self.vertices.push(v2);
        self.vertices.push(v3);
    }
}

impl<TScalar: RealNumber> Default for PolygonSoup<TScalar> {
    fn default() -> Self {
        return Self { vertices: Vec::new() };
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

    fn from_vertices_and_indices(vertices: &[Point3<Self::ScalarType>], faces: &[usize]) -> Self {
        let mut soup = Vec::with_capacity(faces.len());

        for vertex_index in faces {
            soup.push(vertices[*vertex_index]);
        }

        return Self {
            vertices: soup
        };
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
        return Triangle3::new(self.vertices[*face], self.vertices[face + 1], self.vertices[face + 2]);
    }

    #[inline]
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        let v2 = if edge % 3 == 2 { edge - 2 } else { edge + 1 };
        return (self.vertices[*edge], self.vertices[v2]);
    }

    #[inline]
    fn edge_vertices(&self, _edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor) {
        todo!()
    }

    #[inline]
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Point3<Self::ScalarType> {
        return &self.vertices[*vertex];
    }

    #[inline]
    fn vertex_normal(&self, _vertex: &Self::VertexDescriptor) -> nalgebra::Vector3<Self::ScalarType> {
        todo!()
    }

    fn face_vertices(&self, _face: &Self::FaceDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor, Self::VertexDescriptor) {
        todo!()
    }
}
