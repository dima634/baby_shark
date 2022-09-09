use nalgebra::Point3;
use crate::mesh::traits::{Floating, Mesh};
use super::traversal::{FacesIter, VerticesIter, EdgesIter};

pub struct PolygonSoup<TScalar: Floating> {
   pub(super) vertices: Vec<Point3<TScalar>>
}

impl<TScalar: Floating> PolygonSoup<TScalar> {
    pub fn new() -> Self { 
        return Self { 
            vertices: Vec::new() 
        } ;
    }

    pub fn add_face(&mut self, v1: Point3<TScalar>, v2: Point3<TScalar>, v3: Point3<TScalar>) {
        self.vertices.push(v1);
        self.vertices.push(v2);
        self.vertices.push(v3);
    }
}

impl<TScalar: Floating> Mesh for PolygonSoup<TScalar> {
    type ScalarType = TScalar;

    type EdgeDescriptor = usize;
    type VertexDescriptor = usize;
    type FaceDescriptor = usize;

    type FacesIter<'iter> = FacesIter<'iter, TScalar>;
    type VerticesIter<'iter> = VerticesIter<'iter, TScalar>;
    type EdgesIter<'iter> = EdgesIter<'iter, TScalar>;

    fn from_vertices_and_indices(vertices: &Vec<Point3<Self::ScalarType>>, faces: &Vec<usize>) -> Self {
        let mut soup = Vec::with_capacity(faces.len());

        for i in 0..faces.len() {
            soup.push(vertices[i]);
        }

        return Self {
            vertices: soup
        };
    }

    #[inline]
    fn faces<'a>(&'a self) -> Self::FacesIter<'a> {
        return FacesIter::new(self);
    }

    #[inline]
    fn vertices<'a>(&'a self) -> Self::VerticesIter<'a> {
        return VerticesIter::new(self);
    }

    #[inline]
    fn edges<'a>(&'a self) -> Self::EdgesIter<'a> {
        return EdgesIter::new(self);
    }

    #[inline]
    fn face_positions(&self, face: &Self::FaceDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        return (self.vertices[*face], self.vertices[face + 1], self.vertices[face + 2]);
    }

    #[inline]
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        let v2 = if edge % 3 == 2 { edge - 2 } else { edge + 1 };
        return (self.vertices[*edge], self.vertices[v2]);
    }

    #[inline]
    fn get_edge_vertices(&self, edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor) {
        todo!()
    }

    #[inline]
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Point3<Self::ScalarType> {
        return &self.vertices[*vertex];
    }

    #[inline]
    fn vertex_normal(&self, vertex: &Self::VertexDescriptor) -> nalgebra::Vector3<Self::ScalarType> {
        todo!()
    }
}
