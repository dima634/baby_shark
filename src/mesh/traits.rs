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
pub trait Mesh<'a> {
    type ScalarType: Floating;

    type EdgeDescriptor: 'a;
    type VertexDescriptor: 'a;
    type FaceDescriptor: 'a;

    type FacesIter: Iterator<Item = &'a Self::FaceDescriptor>;
    type VerticesIter: Iterator<Item = &'a Self::VertexDescriptor>;
    type EdgesIter: Iterator<Item = &'a Self::EdgeDescriptor>;

    /// Creates mesh from vertices and face indices
    fn from_vertices_and_indices(vertices: &Vec<Point3<Self::ScalarType>>, faces: &Vec<usize>) -> Self;

    /// Iterator over mesh faces
    fn faces(&'a self) -> Self::FacesIter;
    /// Iterator over mesh vertices
    fn vertices(&'a self) -> Self::VerticesIter;
    /// Iterator over mesh edges
    fn edges(&'a self) -> Self::EdgesIter;

    /// Returns positions of face vertices in ccw order
    fn face_positions(&self, face: &Self::FaceDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>, Point3<Self::ScalarType>);
    /// Returns face normal
    fn face_normal(&self, face: &Self::FaceDescriptor) -> UnitVector3<Self::ScalarType>;
}

// pub trait EditableMesh {  
//     type EdgeDescriptor;
//     type VertexDescriptor;

//     fn collapse_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn split_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn flip_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn shift_vertex(&mut self, vertex: Self::VertexDescriptor) -> Result<(), ()>;
// }
