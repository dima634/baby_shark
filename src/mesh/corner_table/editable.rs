use nalgebra::Point3;
use num_traits::cast;
use crate::{mesh::{traits::{EditableMesh, TopologicalMesh, Mesh}}, geometry::traits::RealNumber};
use super::{corner_table::CornerTable, traversal::{CornerWalker, collect_corners_around_vertex}, connectivity::traits::Flags};


impl<TScalar: RealNumber> CornerTable<TScalar> {
    /// Splits inner edge opposite to corner at given position
    fn split_inner_edge(&mut self, corner_index: usize, at: &Point3<TScalar>) {
        // New corner indices
        let c6_idx = self.corners.len();
        let c7_idx = c6_idx + 1;
        let c8_idx = c6_idx + 2;

        let c9_idx = c6_idx + 3;
        let c10_idx = c6_idx + 4;
        let c11_idx = c6_idx + 5;
        
        // Existing corners and vertices that needs to be updated
        let mut walker = CornerWalker::from_corner(self, corner_index);
        let c0_idx = walker.get_previous_corner_index();
        let v1_idx = walker.get_corner().get_vertex_index();
        let c2_idx = walker.next().get_corner_index();
        let v2_idx = walker.get_corner().get_vertex_index();
        let c3_idx = walker.swing_right().get_corner_index();
        let v3_idx = walker.next().get_corner().get_vertex_index();
        let c5_idx = walker.next().get_corner_index();

        // Shift existing vertex
        let old_vertex_position = *walker.next().get_vertex().get_position();
        self.shift_vertex(&v2_idx, at);
        self.get_vertex_mut(v2_idx).unwrap().set_corner_index(c2_idx);

        // New vertex, instead of shifted
        let new_vertex_index = self.vertices.len();
        let new_vertex = self.create_vertex();
        new_vertex.set_corner_index(c7_idx);
        new_vertex.set_position(old_vertex_position);

        // Update vertex index of existing corners
        for corner_index in collect_corners_around_vertex(self, v2_idx) {
            if corner_index != c3_idx && corner_index != c2_idx {
                self.get_corner_mut(corner_index).unwrap().set_vertex_index(new_vertex_index);
            }
        }

        // Create new faces
        self.create_face_from_vertices(v1_idx, new_vertex_index, v2_idx);
        self.create_face_from_vertices(new_vertex_index, v3_idx, v2_idx);

        // Update opposites
        if let Some(c0_opposite_idx) = self.get_corner(c0_idx).unwrap().get_opposite_corner_index() {
            self.set_opposite_relationship(c0_opposite_idx, c8_idx);
        }

        if let Some(c5_opposite_idx) = self.get_corner(c5_idx).unwrap().get_opposite_corner_index() {
            self.set_opposite_relationship(c5_opposite_idx, c11_idx);
        }

        self.set_opposite_relationship(c0_idx, c7_idx);
        self.set_opposite_relationship(c5_idx, c9_idx);
        self.set_opposite_relationship(c6_idx, c10_idx);
    }

    /// Splits boundary edge opposite to corner at given position
    fn split_boundary_edge(&mut self, corner_index: usize, at: &Point3<TScalar>) {
        // New corner indices
        let c3_idx = self.corners.len();
        let c4_idx = c3_idx + 1;
        let c5_idx = c3_idx + 2;
        
        // Existing corners and vertices that needs to be updated
        let mut walker = CornerWalker::from_corner(self, corner_index);
        let c0_idx = walker.get_previous_corner_index();
        let v1_idx = walker.get_corner().get_vertex_index();
        let c2_idx = walker.next().get_corner_index();
        let v2_idx = walker.get_corner().get_vertex_index();

        // Shift existing vertex
        let old_vertex_position = *walker.get_vertex().get_position();
        self.shift_vertex(&v2_idx, at);
        self.get_vertex_mut(v2_idx).unwrap().set_corner_index(c2_idx);

        // New vertex, instead of shifted
        let new_vertex_index = self.vertices.len();
        let new_vertex = self.create_vertex();
        new_vertex.set_corner_index(c4_idx);
        new_vertex.set_position(old_vertex_position);

        // Update vertex index of existing corners
        for corner_index in collect_corners_around_vertex(self, v2_idx) {
            if corner_index != c3_idx && corner_index != c2_idx {
                self.get_corner_mut(corner_index).unwrap().set_vertex_index(new_vertex_index);
            }
        }

        // Create new face
        self.create_face_from_vertices(v1_idx, new_vertex_index, v2_idx);

        // Update opposites
        if let Some(c0_opposite_idx) = self.get_corner(c0_idx).unwrap().get_opposite_corner_index() {
            self.set_opposite_relationship(c0_opposite_idx, c5_idx);
        }

        self.set_opposite_relationship(c0_idx, c4_idx);
    }
}

impl<TScalar: RealNumber> EditableMesh for CornerTable<TScalar> {
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor) {
        let mut walker = CornerWalker::from_corner(self, *edge);

        // Skip collapse on boundary for now
        // TODO: implement collapse on boundary
        let (e_start, e_end) = self.get_edge_vertices(edge);
        if self.is_vertex_on_boundary(&e_start) || self.is_vertex_on_boundary(&e_end) {
            return;
        }

        // Collect corners of faces that is going to be removed, 
        // vertices of collapsed edge and corners that going to be opposite after collapse
        let c24_idx = walker.get_corner_index();
        let v7_idx = walker.get_corner().get_vertex_index();

        let c25_idx = walker.next().get_corner_index();
        let v8_idx = walker.get_corner().get_vertex_index();
        let c21_idx = walker.get_corner().get_opposite_corner_index().unwrap();

        let c26_idx = walker.next().get_corner_index();
        let c28_idx = walker.get_corner().get_opposite_corner_index().unwrap();

        let c9_idx = walker.next().opposite().get_corner_index();
        let v3_idx = walker.get_corner().get_vertex_index();

        let c10_idx = walker.next().get_corner_index();
        let v9_idx = walker.get_corner().get_vertex_index();
        let c6_idx = walker.get_corner().get_opposite_corner_index().unwrap();
    
        let c11_idx = walker.next().get_corner_index();
        let c13_idx = walker.get_corner().get_opposite_corner_index().unwrap();

        // Make sure vertices are not referencing deleted corners
        let c27_idx = walker.set_current_corner(c28_idx).next().get_corner().get_next_corner_index();
        let c29_idx = walker.get_corner_index();
        let c7_idx = walker.set_current_corner(c6_idx).get_corner().get_next_corner_index();

        self.get_vertex_mut(v7_idx).unwrap().set_corner_index(c27_idx);
        self.get_vertex_mut(v3_idx).unwrap().set_corner_index(c7_idx);

        // Delete corners
        self.get_corner_mut(c24_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c25_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c26_idx).unwrap().set_deleted(true);

        self.get_corner_mut(c9_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c10_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c11_idx).unwrap().set_deleted(true);

        // Remove vertex on edge end
        let v9 = self.get_vertex_mut(v9_idx).unwrap();
        let v9_position = *v9.get_position();
        v9.set_deleted(true);

        // Update vertex for corners around removed one
        for corner_index in collect_corners_around_vertex(self, v9_idx) {
            self.get_corner_mut(corner_index).unwrap().set_vertex_index(v8_idx);
        }

        // Shift vertex on other side of edge
        let v8 = self.get_vertex_mut(v8_idx).unwrap();
        v8.set_position((v8.get_position() + v9_position.coords) / cast(2.0).unwrap())
          .set_corner_index(c29_idx);

        // Setup new opposites
        self.set_opposite_relationship(c28_idx, c21_idx);
        self.set_opposite_relationship(c6_idx, c13_idx);
    }

    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor) {
        todo!()
    }

    #[inline]
    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Point3<Self::ScalarType>) {
        let corner = self.get_corner(*edge).unwrap();

        match corner.get_opposite_corner_index() {
            Some(_) => self.split_inner_edge(*edge, at),
            None => self.split_boundary_edge(*edge, at),
        }
    }

    #[inline]
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Point3<Self::ScalarType>) {
        self.get_vertex_mut(*vertex).unwrap().set_position(*to);
    }

    #[inline]
    fn edge_exist(&self, edge: &Self::EdgeDescriptor) -> bool {
        return self.get_corner(*edge).unwrap().is_deleted();
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::mesh::{
        corner_table::{test_helpers::{create_unit_square_mesh, assert_mesh_equals, create_single_face_mesh, create_unit_cross_square_mesh, create_collapse_edge_sample_mesh}, 
        connectivity::{vertex::VertexF, corner::Corner}}, 
        traits::EditableMesh
    };

    #[test]
    fn split_inner_edge1() {
        let mut mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(1, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(2, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 2
            VertexF::new(4, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(7, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default())  // 4
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(1, Some(7), 0, Default::default()), // 0
            Corner::new(2, Some(4), 1, Default::default()), // 1
            Corner::new(0, None,    2, Default::default()), // 2
    
            Corner::new(4, None,    2, Default::default()), // 3
            Corner::new(5, Some(1), 3, Default::default()), // 4
            Corner::new(3, Some(9), 0, Default::default()), // 5
            
            Corner::new(7, Some(10), 1, Default::default()), // 6
            Corner::new(8, Some(0),  4, Default::default()), // 7
            Corner::new(6, None,     2, Default::default()), // 8
            
            Corner::new(10, Some(5), 4, Default::default()), // 9
            Corner::new(11, Some(6), 3, Default::default()), // 10
            Corner::new(9,  None,    2, Default::default()), // 11
        ];

        mesh.split_edge(&1, &Point3::<f32>::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_inner_edge2() {
        let mut mesh = create_unit_cross_square_mesh();

        let expected_vertices = vec![
            VertexF::new(10, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(9, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(11, Point3::<f32>::new(0.75, 0.75, 0.0), Default::default()), // 4
            VertexF::new(13, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default())  // 5
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(1, Some(4),  0, Default::default()), // 0
            Corner::new(2, Some(14), 1, Default::default()), // 1
            Corner::new(0, None,     5, Default::default()), // 2
    
            Corner::new(4, Some(17), 1, Default::default()), // 3
            Corner::new(5, Some(0),  2, Default::default()), // 4
            Corner::new(3, None,     5, Default::default()), // 5
            
            Corner::new(7, Some(10), 2, Default::default()), // 6
            Corner::new(8, Some(15), 3, Default::default()), // 7
            Corner::new(6, None,     4, Default::default()), // 8
            
            Corner::new(10, Some(13), 3, Default::default()), // 9
            Corner::new(11, Some(6),  0, Default::default()), // 10
            Corner::new(9,  None,     4, Default::default()), // 11
            
            Corner::new(13, Some(16), 0, Default::default()), // 12
            Corner::new(14, Some(9),  5, Default::default()), // 13
            Corner::new(12, Some(1),  4, Default::default()), // 14
            
            Corner::new(16, Some(7),  5, Default::default()), // 15
            Corner::new(17, Some(12), 2, Default::default()), // 16
            Corner::new(15, Some(3),  4, Default::default()), // 17
        ];

        mesh.split_edge(&10, &Point3::<f32>::new(0.75, 0.75, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_boundary_edge() {
        let mut mesh = create_single_face_mesh();

        let expected_vertices = vec![
            VertexF::new(0, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(1, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(2, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 2
            VertexF::new(4, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 3
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(1, Some(4), 0, Default::default()), // 0
            Corner::new(2, None,    1, Default::default()), // 1
            Corner::new(0, None,    2, Default::default()), // 2
    
            Corner::new(4, None,    1, Default::default()), // 3
            Corner::new(5, Some(0), 3, Default::default()), // 4
            Corner::new(3, None,    2, Default::default()), // 5
        ];

        mesh.split_edge(&1, &Point3::<f32>::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge() {
        let mut mesh = create_collapse_edge_sample_mesh();

        let expected_vertices = vec![
            VertexF::new(28, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Point3::<f32>::new(0.0, 0.5, 0.0), Default::default()), // 1
            VertexF::new(6, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(7, Point3::<f32>::new(0.5, 0.0, 0.0), Default::default()), // 3
            VertexF::new(15, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 4
            VertexF::new(18, Point3::<f32>::new(1.0, 0.5, 0.0), Default::default()), // 5
            VertexF::new(21, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 6
            VertexF::new(27, Point3::<f32>::new(0.5, 1.0, 0.0), Default::default()), // 7
            VertexF::new(29, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 8
            VertexF::new(26, Point3::<f32>::new(0.75, 0.5, 0.0), Default::default()), // 9
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(1, Some(4),  0, Default::default()), // 0
            Corner::new(2, Some(27), 1, Default::default()), // 1
            Corner::new(0, None,     8, Default::default()), // 2
    
            Corner::new(4, Some(7), 1, Default::default()), // 3
            Corner::new(5, Some(0), 2, Default::default()), // 4
            Corner::new(3, None,    8, Default::default()), // 5
    
            Corner::new(7, Some(13), 2, Default::default()), // 6
            Corner::new(8, Some(3),  3, Default::default()), // 7
            Corner::new(6, None,     8, Default::default()), // 8
    
            Corner::new(10, Some(24), 3, Default::default()), // 9
            Corner::new(11, Some(6),  8, Default::default()), // 10
            Corner::new(9,  Some(13), 8, Default::default()), // 11
    
            Corner::new(13, Some(16), 3, Default::default()), // 12
            Corner::new(14, Some(6),  4, Default::default()), // 13
            Corner::new(12, None,     8, Default::default()), // 14
    
            Corner::new(16, Some(19), 4, Default::default()), // 15
            Corner::new(17, Some(12), 5, Default::default()), // 16
            Corner::new(15, None,     8, Default::default()), // 17
    
            Corner::new(19, Some(22), 5, Default::default()), // 18
            Corner::new(20, Some(15), 6, Default::default()), // 19
            Corner::new(18, None,     8, Default::default()), // 20
    
            Corner::new(22, Some(28), 6, Default::default()), // 21
            Corner::new(23, Some(18), 7, Default::default()), // 22
            Corner::new(21, None,     8, Default::default()), // 23
    
            Corner::new(25, Some(9),  7, Default::default()), // 24
            Corner::new(26, Some(21), 8, Default::default()), // 25
            Corner::new(24, Some(28), 8, Default::default()), // 26
    
            Corner::new(28, Some(1),  7, Default::default()), // 27
            Corner::new(29, Some(21), 0, Default::default()), // 28
            Corner::new(27, None,     8, Default::default()), // 29
        ];

        mesh.collapse_edge(&24);

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }
}
