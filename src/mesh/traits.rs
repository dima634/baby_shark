use nalgebra::{Point3, UnitVector3, RealField};
use num_traits::Float;

pub trait Floating: RealField + Float {}
impl<T> Floating for T where T: RealField + Float {}

pub trait Edge {
    type VertexDescriptor;

    fn start(&self) -> Self::VertexDescriptor;
    fn end(&self) -> Self::VertexDescriptor;
}

pub trait Vertex {
    type ScalarType: Floating;

    fn get_position(&self) -> &Point3<Self::ScalarType>;
    fn set_position(&mut self, point: Point3<Self::ScalarType>) -> &mut Self;
}

///
/// Triangular mesh
/// 
pub trait Mesh {
    type ScalarType: Floating;

    type EdgeDescriptor;
    type VertexDescriptor;
    type FaceDescriptor;

    type FacesIter<'iter>: Iterator<Item = Self::FaceDescriptor> where Self: 'iter;
    type VerticesIter<'iter>: Iterator<Item = Self::VertexDescriptor> where Self: 'iter;
    type EdgesIter<'iter>: Iterator<Item = Self::EdgeDescriptor> where Self: 'iter;

    /// Creates mesh from vertices and face indices
    fn from_vertices_and_indices(vertices: &Vec<Point3<Self::ScalarType>>, faces: &Vec<usize>) -> Self;

    /// Iterator over mesh faces
    fn faces<'a>(&'a self) -> Self::FacesIter<'a>;
    /// Iterator over mesh vertices
    fn vertices<'a>(&'a self) -> Self::VerticesIter<'a>;
    /// Iterator over mesh edges
    fn edges<'a>(&'a self) -> Self::EdgesIter<'a>;

    /// Returns positions of face vertices in ccw order
    fn face_positions(&self, face: &Self::FaceDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>, Point3<Self::ScalarType>);
    /// Returns face normal
    fn face_normal(&self, face: &Self::FaceDescriptor) -> UnitVector3<Self::ScalarType>;

    /// Returns edge length
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>);
    /// Returns edge length
    fn edge_length(&self, edge: &Self::EdgeDescriptor) -> Self::ScalarType;
}

///
/// Triangular mesh that supports editing operations
/// 
pub trait EditableMesh: Mesh {
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor);
    fn is_edge_collapse_safe(&mut self, edge: &Self::EdgeDescriptor) -> bool;

    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor);
    fn is_edge_flip_safe(&mut self, edge: &Self::EdgeDescriptor) -> bool;

    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Point3<Self::ScalarType>);
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Point3<Self::ScalarType>);
}

/// Contains constants which defines what is good mesh
pub mod mesh_stats {
    pub const IDEAL_INTERIOR_VERTEX_VALENCE: usize = 6;
    pub const IDEAL_BOUNDARY_VERTEX_VALENCE: usize = 4;

    /// Constant used to preallocate memory for some algorithms, it is not limiting anything
    pub const MAX_VERTEX_VALENCE: usize = 10;
}
