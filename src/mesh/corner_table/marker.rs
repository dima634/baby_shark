use crate::{geometry::traits::RealNumber, mesh::traits::{Marker, Mesh}};

use super::{table::CornerTable, connectivity::{traits::Flags, corner}};

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
        let first_corner = corner::first_corner_from_corner(*face);
        unsafe { (*self.corner_table).corners[first_corner].set_marked_1(marked); }
    }

    #[inline]
    fn is_face_marked(&self, face: &<CornerTable<TScalar> as Mesh>::FaceDescriptor) -> bool {
        let first_corner = corner::first_corner_from_corner(*face);
        unsafe { (*self.corner_table).corners[first_corner].is_marked_1()}
    }

    //
    // Vertex
    //

    #[inline]
    fn mark_vertex(&mut self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor, marked: bool) {
        unsafe { (*self.corner_table).vertices[*vertex].set_marked_1(marked); }
    }

    #[inline]
    fn is_vertex_marked(&self, vertex: &<CornerTable<TScalar> as Mesh>::VertexDescriptor) -> bool {
        unsafe { (*self.corner_table).vertices[*vertex].is_marked_1()}
    }

    //
    // Edge
    // 

    #[inline]
    fn mark_edge(&mut self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor, marked: bool)  {
        unsafe { 
            let corner = &(*self.corner_table).corners[edge.get_corner_index()];
            corner.set_marked_2(marked);
    
            if let Some(opposite) = corner.get_opposite_corner_index() {
                (*self.corner_table).corners[opposite].set_marked_2(marked); 
            }
        }
    }

    #[inline]
    fn is_edge_marked(&self, edge: &<CornerTable<TScalar> as Mesh>::EdgeDescriptor) -> bool {
        unsafe { (*self.corner_table).corners[edge.get_corner_index()].is_marked_2()}
    }
}
