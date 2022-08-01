use super::{connectivity::traits::{Corner, Face, Vertex}, traits};


pub struct CornerTable<TCorner: Corner, TFace: Face, TVertex: Vertex> {
    faces: Vec<TFace>,
    vertices: Vec<TVertex>,
    corners: Vec<TCorner>
}

impl<TCorner, TFace, TVertex> CornerTable<TCorner, TFace, TVertex> 
where 
    TCorner: Corner, 
    TFace: Face, 
    TVertex: Vertex 
{
    fn new<>() -> Self {
        return Self {
            corners: Vec::new(),
            faces: Vec::new(),
            vertices: Vec::new()
        };
    }
}

impl<TCorner, TFace, TVertex> traits::CornerTable for CornerTable<TCorner, TFace, TVertex> 
where 
    TCorner: Corner, 
    TFace: Face, 
    TVertex: Vertex 
{
    type ScalarType = TVertex::ScalarType;

    type CornerType = TCorner;
    type FaceType = TFace;
    type VertexType = TVertex;

    #[inline]
    fn get_vertex(&self, vertex_index:  usize) -> &Self::VertexType {
        return self.vertices.get(vertex_index).unwrap();
    }

    #[inline]
    fn get_vertex_mut(&mut self, vertex_index:  usize) -> &mut Self::VertexType {
        todo!()
    }

    #[inline]
    fn get_face(&self, face_index:  usize) -> &Self::FaceType {
        todo!()
    }

    #[inline]
    fn get_face_mut(&mut self, face_index:  usize) -> &mut Self::FaceType {
        todo!()
    }

    #[inline]
    fn get_corner(&self, corner_index:  usize) -> &Self::CornerType {
        todo!()
    }

    #[inline]
    fn get_corner_mut(&mut self, corner_index:  usize) -> &mut Self::CornerType {
        todo!()
    }

    fn build(vertices: Vec<nalgebra::Point3<Self::ScalarType>>, faces: Vec< usize>) {
        todo!()
    }
}
