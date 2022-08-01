
pub trait Mesh {
    type EdgeDescriptor;
    type VertexDescriptor;
    type FaceDescriptor;

    type FacesIter: Iterator<Item = Self::FaceDescriptor>;
    type VerticesIter: Iterator<Item = Self::VertexDescriptor>;
    type EdgesIter: Iterator<Item = Self::EdgeDescriptor>;

    fn faces(&self) -> Self::FacesIter;
    fn vertices(&self) -> Self::VerticesIter;
    fn edges(&self) -> Self::EdgesIter;
}

// pub trait EditableMesh {  
//     type EdgeDescriptor;
//     type VertexDescriptor;

//     fn collapse_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn split_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn flip_edge(&mut self, edge: Self::EdgeDescriptor) -> Result<(), ()>;
//     fn shift_vertex(&mut self, vertex: Self::VertexDescriptor) -> Result<(), ()>;
// }
