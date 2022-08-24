use nalgebra::Point3;
use crate::{mesh::traits::EditableMesh};
use super::{connectivity::traits::{Corner, Vertex}, corner_table::CornerTable, traversal::{CornerWalker, corners_around_vertex}};


impl<TCorner: Corner, TVertex: Vertex> CornerTable<TCorner, TVertex> {
    /// Splits inner edge opposite to corner at given position
    fn split_inner_edge(&mut self, corner_index: usize, at: &Point3<TVertex::ScalarType>) {
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
        for corner_index in corners_around_vertex(self, v2_idx) {
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
    fn split_boundary_edge(&mut self, corner_index: usize, at: &Point3<TVertex::ScalarType>) {
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
        for corner_index in corners_around_vertex(self, v2_idx) {
            if corner_index != c2_idx {
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

impl<TCorner: Corner, TVertex: Vertex> EditableMesh for CornerTable<TCorner, TVertex> {
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor) {
        todo!()
    }

    fn is_edge_collapse_safe(&mut self, edge: &Self::EdgeDescriptor) -> bool {
        todo!()
    }

    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor) {
        todo!()
    }

    fn is_edge_flip_safe(&mut self, edge: &Self::EdgeDescriptor) -> bool {
        todo!()
    }

    #[inline]
    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Point3<Self::ScalarType>) {
        match self.get_corner(*edge).unwrap().get_opposite_corner_index() {
            Some(_) => self.split_inner_edge(*edge, at),
            None => self.split_boundary_edge(*edge, at),
        }
    }

    #[inline]
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Point3<Self::ScalarType>) {
        self.get_vertex_mut(*vertex).unwrap().set_position(*to);
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::mesh::{
        corner_table::{test_helpers::{create_unit_square_mesh, assert_mesh_equals, create_single_face_mesh, create_unit_cross_square_mesh}, 
        connectivity::{vertex::VertexF, corner::DefaultCorner}}, 
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
            DefaultCorner::new(1, Some(7), 0, Default::default()), // 0
            DefaultCorner::new(2, Some(4), 1, Default::default()), // 1
            DefaultCorner::new(0, None,    2, Default::default()), // 2
    
            DefaultCorner::new(4, None,    2, Default::default()), // 3
            DefaultCorner::new(5, Some(1), 3, Default::default()), // 4
            DefaultCorner::new(3, Some(9), 0, Default::default()), // 5
            
            DefaultCorner::new(7, Some(10), 1, Default::default()), // 6
            DefaultCorner::new(8, Some(0),  4, Default::default()), // 7
            DefaultCorner::new(6, None,     2, Default::default()), // 8
            
            DefaultCorner::new(10, Some(5), 4, Default::default()), // 9
            DefaultCorner::new(11, Some(6), 3, Default::default()), // 10
            DefaultCorner::new(9,  None,    2, Default::default()), // 11
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
            DefaultCorner::new(1, Some(4),  0, Default::default()), // 0
            DefaultCorner::new(2, Some(14), 1, Default::default()), // 1
            DefaultCorner::new(0, None,     5, Default::default()), // 2
    
            DefaultCorner::new(4, Some(17), 1, Default::default()), // 3
            DefaultCorner::new(5, Some(0),  2, Default::default()), // 4
            DefaultCorner::new(3, None,     5, Default::default()), // 5
            
            DefaultCorner::new(7, Some(10), 2, Default::default()), // 6
            DefaultCorner::new(8, Some(15), 3, Default::default()), // 7
            DefaultCorner::new(6, None,     4, Default::default()), // 8
            
            DefaultCorner::new(10, Some(13), 3, Default::default()), // 9
            DefaultCorner::new(11, Some(6),  0, Default::default()), // 10
            DefaultCorner::new(9,  None,     4, Default::default()), // 11
            
            DefaultCorner::new(13, Some(16), 0, Default::default()), // 12
            DefaultCorner::new(14, Some(9),  5, Default::default()), // 13
            DefaultCorner::new(12, Some(1),  4, Default::default()), // 14
            
            DefaultCorner::new(16, Some(7),  5, Default::default()), // 15
            DefaultCorner::new(17, Some(12), 2, Default::default()), // 16
            DefaultCorner::new(15, Some(3),  4, Default::default()), // 17
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
            DefaultCorner::new(1, Some(4), 0, Default::default()), // 0
            DefaultCorner::new(2, None,    1, Default::default()), // 1
            DefaultCorner::new(0, None,    2, Default::default()), // 2
    
            DefaultCorner::new(4, None,    1, Default::default()), // 3
            DefaultCorner::new(5, Some(0), 3, Default::default()), // 4
            DefaultCorner::new(3, None,    2, Default::default()), // 5
        ];

        mesh.split_edge(&1, &Point3::<f32>::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }
}
