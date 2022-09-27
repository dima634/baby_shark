use crate::{geometry::traits::RealNumber, mesh::traits::{Marker, Mesh}};

use super::{corner_table::CornerTable, connectivity::traits::Flags};

/// Implementation of [Marker] API for [CornerTable] 
pub struct CornerTableMarker<TScalar: RealNumber> {
    corner_table: *const CornerTable<TScalar>
}

impl<TScalar: RealNumber> CornerTableMarker<TScalar> {
    pub fn new(corner_table: &CornerTable<TScalar>) -> Self { 
        return Self { corner_table } ;
    }
}

impl<TScalar: RealNumber> Marker<CornerTable<TScalar>> for CornerTableMarker<TScalar> {

    //
    // Face
    //

    #[inline]
    fn mark_face(&self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor) {
        unsafe { (*self.corner_table).corners[*face].set_marked_1(true); }
    }

    #[inline]
    fn unmark_face(&self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor) {
        unsafe { (*self.corner_table).corners[*face].set_marked_1(false); }
    }

    #[inline]
    fn is_face_marked(&self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor) -> bool {
        unsafe { return (*self.corner_table).corners[*face].is_marked_1(); }
    }

    //
    // Vertex
    //

    #[inline]
    fn mark_vertex(&self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor) {
        unsafe { (*self.corner_table).vertices[*vertex].set_marked_1(true); }
    }

    #[inline]
    fn unmark_vertex(&self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor) {
        unsafe { (*self.corner_table).vertices[*vertex].set_marked_1(false); }
    }

    #[inline]
    fn is_vertex_marked(&self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor) -> bool {
        unsafe { return (*self.corner_table).vertices[*vertex].is_marked_1(); }
    }

    //
    // Edge
    // 

    #[inline]
    fn mark_edge(&self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor)  {
        unsafe { (*self.corner_table).corners[*edge].set_marked_2(true); }
    }

    #[inline]
    fn unmark_edge(&self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor) {
        unsafe { (*self.corner_table).corners[*edge].set_marked_2(false); }
    }

    #[inline]
    fn is_edge_marked(&self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor) -> bool {
        unsafe { return (*self.corner_table).corners[*edge].is_marked_2(); }
    }
}
