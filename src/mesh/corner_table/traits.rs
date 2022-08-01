use nalgebra::{Scalar, Point3};
use num_traits::Float;

use super::connectivity::traits::{Vertex, Face, Corner};

pub trait CornerTable {
    type ScalarType: Float + Scalar;

    type CornerType: Corner;
    type FaceType: Face;
    type VertexType: Vertex;

    fn get_vertex(&self, vertex_index: usize) -> &Self::VertexType;
    fn get_vertex_mut(&mut self, vertex_index: usize) -> &mut Self::VertexType;
    
    fn get_face(&self, face_index: usize) -> &Self::FaceType;
    fn get_face_mut(&mut self, face_index: usize) -> &mut Self::FaceType;

    fn get_corner(&self, corner_index: usize) -> &Self::CornerType;
    fn get_corner_mut(&mut self, corner_index: usize) -> &mut Self::CornerType;
 
    fn build(vertices: Vec<Point3<Self::ScalarType>>, faces: Vec<usize>);
}
