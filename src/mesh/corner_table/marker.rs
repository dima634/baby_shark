use crate::{geometry::traits::RealNumber, mesh::traits::{Marker, Mesh}};
use super::{CornerTable, traits::Flags};

/// Implementation of [Marker] API for [CornerTable] 
pub struct CornerTableMarker<TScalar: RealNumber> {
    corner_table: *const CornerTable<TScalar>
}

impl<TScalar: RealNumber> CornerTableMarker<TScalar> {
    pub fn new(corner_table: &CornerTable<TScalar>) -> Self { 
        Self { corner_table }
    }
}

impl<TScalar: RealNumber> Marker<CornerTable<TScalar>> for CornerTableMarker<TScalar> {

    //
    // Face
    //

    #[inline]
    fn mark_face(&mut self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor, marked: bool) {
        let first_corner = face.corner();
        unsafe { (*self.corner_table)[first_corner].set_marked_1(marked); }
    }

    #[inline]
    fn is_face_marked(&self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor) -> bool {
        let first_corner = face.corner();
        unsafe { (*self.corner_table)[first_corner].is_marked_1()}
    }

    //
    // Vertex
    //

    #[inline]
    fn mark_vertex(&mut self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor, marked: bool) {
        unsafe { (*self.corner_table)[*vertex].set_marked_1(marked); }
    }

    #[inline]
    fn is_vertex_marked(&self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor) -> bool {
        unsafe { (*self.corner_table)[*vertex].is_marked_1()}
    }

    //
    // Edge
    // 

    #[inline]
    fn mark_edge(&mut self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor, marked: bool)  {
        unsafe { 
            let corner = &(*self.corner_table)[edge.corner()];
            corner.set_marked_2(marked);
    
            if let Some(opposite) = corner.opposite_corner() {
                (*self.corner_table)[opposite].set_marked_2(marked); 
            }
        }
    }

    #[inline]
    fn is_edge_marked(&self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor) -> bool {
        unsafe { (*self.corner_table)[edge.corner()].is_marked_2()}
    }
}
