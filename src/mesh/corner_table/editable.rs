use nalgebra::Point3;
use crate::{mesh::traits::EditableMesh, geometry::traits::RealNumber};
use super::{table::CornerTable, traversal::{CornerWalker, collect_corners_around_vertex}, connectivity::{traits::Flags, corner}};

/// Set corner for wing vertex of collapsed edge
#[inline]
fn set_corner_for_wing_vertex<TScalar: RealNumber>(
    corner_table: &mut CornerTable<TScalar>, 
    vertex_index: usize, 
    opposite_corner_left: Option<usize>,
    opposite_corner_right: Option<usize>
) {
    if let Some(corner) = opposite_corner_left {
        corner_table.vertices[vertex_index].set_corner_index(corner::previous(corner));
    } else if let Some(corner) = opposite_corner_right {
        corner_table.vertices[vertex_index].set_corner_index(corner::next(corner));
    } else {
        corner_table.vertices[vertex_index].set_deleted(true);
    }
}

/// Make corners opposite to each other
#[inline]
fn make_corners_opposite<TScalar: RealNumber>(
    corner_table: &mut CornerTable<TScalar>, 
    c1: Option<usize>,
    c2: Option<usize>
) {
    if let Some(c1_idx) = c1 {
        corner_table.corners[c1_idx].set_opposite_corner_index(c2);
    }
    
    if let Some(c2_idx) = c2 {
        corner_table.corners[c2_idx].set_opposite_corner_index(c1);
    }
}

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
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Point3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());

        // Collect corners of faces that is going to be removed, 
        // vertices of collapsed edge and corners that going to be opposite after collapse
        let c24_idx = walker.get_corner_index();
        let v7_idx = walker.get_corner().get_vertex_index();

        let c25_idx = walker.next().get_corner_index();
        let v8_idx = walker.get_corner().get_vertex_index();
        let c21_idx = walker.get_corner().get_opposite_corner_index();

        let c26_idx = walker.next().get_corner_index();
        let c28_idx = walker.get_corner().get_opposite_corner_index();
        let v9_idx = walker.get_corner().get_vertex_index();

        walker.next();

        let mut c6_idx = None;
        let mut c13_idx = None;

        // If not boundary edge
        if walker.get_corner().get_opposite_corner_index().is_some() {
            let c9_idx = walker.opposite().get_corner_index();
            let v3_idx = walker.get_corner().get_vertex_index();

            let c10_idx = walker.next().get_corner_index();
            c6_idx = walker.get_corner().get_opposite_corner_index();
        
            let c11_idx = walker.next().get_corner_index();
            c13_idx = walker.get_corner().get_opposite_corner_index();

            // Make sure vertices are not referencing deleted corners
            set_corner_for_wing_vertex(self, v3_idx, c13_idx, c6_idx);

            // Delete face
            self.get_corner_mut(c9_idx).unwrap().set_deleted(true);
            self.get_corner_mut(c10_idx).unwrap().set_deleted(true);
            self.get_corner_mut(c11_idx).unwrap().set_deleted(true);
        }

        // Make sure vertices are not referencing deleted corners
        set_corner_for_wing_vertex(self, v7_idx, c28_idx, c21_idx);

        // Delete face
        self.get_corner_mut(c24_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c25_idx).unwrap().set_deleted(true);
        self.get_corner_mut(c26_idx).unwrap().set_deleted(true);

        // Remove vertex on edge end
        let v9 = self.get_vertex_mut(v9_idx).unwrap();
        v9.set_deleted(true);

        // Update vertex for corners around removed one
        for corner_index in collect_corners_around_vertex(self, v9_idx) {
            self.get_corner_mut(corner_index).unwrap().set_vertex_index(v8_idx);
        }

        // Shift vertex on other side of edge
        self.get_vertex_mut(v8_idx).unwrap().set_position(*at);
        set_corner_for_wing_vertex(self, v8_idx, c6_idx.or(c21_idx), c28_idx.or(c13_idx));

        // Setup new opposites
        make_corners_opposite(self, c28_idx, c21_idx);
        make_corners_opposite(self, c6_idx, c13_idx);
    }

    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());

        // Face 1
        let c1_idx = walker.get_corner_index();
        let v1_idx = walker.get_corner().get_vertex_index();

        let c2_idx = walker.next().get_corner_index();
        let c2 = walker.get_corner();
        let v2_idx = c2.get_vertex_index();
        let c2_opp = c2.get_opposite_corner_index();

        let c0_idx = walker.next().get_corner_index();
        let c0 = walker.get_corner();
        let v0_idx = c0.get_vertex_index();
        let c0_opp = c0.get_opposite_corner_index();

        // Face 2
        let c4_idx = walker.next().opposite().get_corner_index();
        let v3_idx = walker.get_corner().get_vertex_index();

        let c5_idx = walker.next().get_corner_index();
        let c5_opp = walker.get_corner().get_opposite_corner_index();

        let c3_idx = walker.next().get_corner_index();
        let c3_opp = walker.get_corner().get_opposite_corner_index();

        // Update corners
        self.corners[c0_idx].set_vertex_index(v1_idx);
        make_corners_opposite(self, Some(c0_idx), c5_opp);
        self.corners[c1_idx].set_vertex_index(v2_idx);
        make_corners_opposite(self, Some(c1_idx), Some(c4_idx));
        self.corners[c2_idx].set_vertex_index(v3_idx);
        make_corners_opposite(self, Some(c2_idx), c0_opp);

        self.corners[c3_idx].set_vertex_index(v3_idx);
        make_corners_opposite(self, Some(c3_idx), c2_opp);
        self.corners[c4_idx].set_vertex_index(v0_idx);
        self.corners[c5_idx].set_vertex_index(v1_idx);
        make_corners_opposite(self, Some(c5_idx), c3_opp);

        // Make sure vertices are referencing correct corners
        self.vertices[v0_idx].set_corner_index(c4_idx);
        self.vertices[v1_idx].set_corner_index(c0_idx);
        self.vertices[v2_idx].set_corner_index(c1_idx);
        self.vertices[v3_idx].set_corner_index(c2_idx);
    }

    #[inline]
    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Point3<Self::ScalarType>) {
        let corner_index = edge.get_corner_index();
        let corner = &self.corners[corner_index];

        match corner.get_opposite_corner_index() {
            Some(_) => self.split_inner_edge(corner_index, at),
            None => self.split_boundary_edge(corner_index, at),
        }
    }

    #[inline]
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Point3<Self::ScalarType>) {
        self.get_vertex_mut(*vertex).unwrap().set_position(*to);
    }

    #[inline]
    fn edge_exist(&self, edge: &Self::EdgeDescriptor) -> bool {
        return !self.corners[edge.get_corner_index()].is_deleted();
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::mesh::{
        corner_table::{
            test_helpers::{
                create_unit_square_mesh, 
                assert_mesh_equals, 
                create_single_face_mesh, 
                create_unit_cross_square_mesh, 
                create_collapse_edge_sample_mesh1, 
                create_flip_edge_sample_mesh, 
                create_collapse_edge_sample_mesh2, 
                create_collapse_edge_sample_mesh3
            }, 
        connectivity::{vertex::VertexF, corner::Corner}, descriptors::EdgeRef}, 
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
            Corner::new(Some(7), 0, Default::default()), // 0
            Corner::new(Some(4), 1, Default::default()), // 1
            Corner::new(None,    2, Default::default()), // 2
    
            Corner::new(None,    2, Default::default()), // 3
            Corner::new(Some(1), 3, Default::default()), // 4
            Corner::new(Some(9), 0, Default::default()), // 5
            
            Corner::new(Some(10), 1, Default::default()), // 6
            Corner::new(Some(0),  4, Default::default()), // 7
            Corner::new(None,     2, Default::default()), // 8
            
            Corner::new(Some(5), 4, Default::default()), // 9
            Corner::new(Some(6), 3, Default::default()), // 10
            Corner::new(None,    2, Default::default()), // 11
        ];

        mesh.split_edge(&EdgeRef::new(1, &mesh), &Point3::<f32>::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_inner_edge2() {
        let mut mesh = create_unit_cross_square_mesh();

        let expected_vertices = vec![
            VertexF::new(10, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(7, Point3::<f32>::new(0.75, 0.75, 0.0), Default::default()), // 3
            VertexF::new(11, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 4
            VertexF::new(13, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default())  // 5
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  0, Default::default()), // 0
            Corner::new(Some(9),  1, Default::default()), // 1
            Corner::new(None,     4, Default::default()), // 2

            Corner::new(Some(7),  1, Default::default()), // 3
            Corner::new(Some(0),  2, Default::default()), // 4
            Corner::new(None,     4, Default::default()), // 5
        
            Corner::new(Some(10), 2, Default::default()), // 6
            Corner::new(Some(3),  3, Default::default()), // 7
            Corner::new(Some(13), 4, Default::default()), // 8
         
            Corner::new(Some(1),  3, Default::default()), // 9
            Corner::new(Some(6),  0, Default::default()), // 10
            Corner::new(Some(15), 4, Default::default()), // 11
            
            Corner::new(Some(16), 2, Default::default()), // 12
            Corner::new(Some(8),  5, Default::default()), // 13
            Corner::new(None,     3, Default::default()), // 14
            
            Corner::new(Some(11), 5, Default::default()), // 15
            Corner::new(Some(12), 0, Default::default()), // 16
            Corner::new(None,     3, Default::default()), // 17
        ];

        mesh.split_edge(&EdgeRef::new(6, &mesh), &Point3::<f32>::new(0.75, 0.75, 0.0));

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
            // opposite, vertex, flags
            Corner::new(Some(4), 0, Default::default()), // 0
            Corner::new(None,    1, Default::default()), // 1
            Corner::new(None,    2, Default::default()), // 2
    
            Corner::new(None,    1, Default::default()), // 3
            Corner::new(Some(0), 3, Default::default()), // 4
            Corner::new(None,    2, Default::default()), // 5
        ];

        mesh.split_edge(&EdgeRef::new(1, &mesh), &Point3::<f32>::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge() {
        let mut mesh = create_collapse_edge_sample_mesh1();

        let expected_vertices = vec![
            VertexF::new(28, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Point3::<f32>::new(0.0, 0.5, 0.0), Default::default()), // 1
            VertexF::new(6, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(12, Point3::<f32>::new(0.5, 0.0, 0.0), Default::default()), // 3
            VertexF::new(15, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 4
            VertexF::new(18, Point3::<f32>::new(1.0, 0.5, 0.0), Default::default()), // 5
            VertexF::new(21, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 6
            VertexF::new(27, Point3::<f32>::new(0.5, 1.0, 0.0), Default::default()), // 7
            VertexF::new(29, Point3::<f32>::new(0.25, 0.5, 0.0), Default::default()), // 8
            VertexF::new(23, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 9
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  0, Default::default()), // 0
            Corner::new(Some(27), 1, Default::default()), // 1
            Corner::new(None,     9, Default::default()), // 2
    
            Corner::new(Some(7), 1, Default::default()), // 3
            Corner::new(Some(0), 2, Default::default()), // 4
            Corner::new(None,    9, Default::default()), // 5
    
            Corner::new(Some(13), 2, Default::default()), // 6
            Corner::new(Some(3),  3, Default::default()), // 7
            Corner::new(None,     9, Default::default()), // 8
    
            Corner::new(Some(24), 3, Default::default()), // 9
            Corner::new(Some(6),  9, Default::default()), // 10
            Corner::new(Some(13), 9, Default::default()), // 11
    
            Corner::new(Some(16), 3, Default::default()), // 12
            Corner::new(Some(6),  4, Default::default()), // 13
            Corner::new(None,     9, Default::default()), // 14
    
            Corner::new(Some(19), 4, Default::default()), // 15
            Corner::new(Some(12), 5, Default::default()), // 16
            Corner::new(None,     9, Default::default()), // 17
    
            Corner::new(Some(22), 5, Default::default()), // 18
            Corner::new(Some(15), 6, Default::default()), // 19
            Corner::new(None,     9, Default::default()), // 20
    
            Corner::new(Some(28), 6, Default::default()), // 21
            Corner::new(Some(18), 7, Default::default()), // 22
            Corner::new(None,     9, Default::default()), // 23
    
            Corner::new(Some(9),  7, Default::default()), // 24
            Corner::new(Some(21), 9, Default::default()), // 25
            Corner::new(Some(28), 9, Default::default()), // 26
    
            Corner::new(Some(1),  7, Default::default()), // 27
            Corner::new(Some(21), 0, Default::default()), // 28
            Corner::new(None,      9, Default::default()), // 29
        ];

        mesh.collapse_edge(&EdgeRef::new(9, &mesh), &Point3::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge_with_one_vertex_on_boundary() {
        let mut mesh = create_collapse_edge_sample_mesh2();

        let expected_vertices = vec![
            VertexF::new(0,  Point3::<f32>::new(0.5, 0.0, 0.0), Default::default()), // 0
            VertexF::new(3, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6, Point3::<f32>::new(1.0, 0.5, 0.0), Default::default()), // 2
            VertexF::new(9, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(10, Point3::<f32>::new(0.5, 1.0, 0.0), Default::default()), // 4
            VertexF::new(11, Point3::<f32>::new(0.5, 0.5, 0.0), Default::default()), // 5
            VertexF::new(17, Point3::<f32>::new(0.75, 0.5, 0.0), Default::default()), // 6
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  0, Default::default()), // 0
            Corner::new(None,     1, Default::default()), // 1
            Corner::new(None,     5, Default::default()), // 2
    
            Corner::new(Some(7), 1, Default::default()), // 3
            Corner::new(Some(0), 2, Default::default()), // 4
            Corner::new(None,    5, Default::default()), // 5
    
            Corner::new(Some(10), 2, Default::default()), // 6
            Corner::new(Some(3),  3, Default::default()), // 7
            Corner::new(None,     5, Default::default()), // 8
    
            Corner::new(None,     3, Default::default()), // 9
            Corner::new(Some(6),  4, Default::default()), // 10
            Corner::new(None,     5, Default::default()), // 11
    
            Corner::new(Some(16), 4, Default::default()), // 12
            Corner::new(Some(9),  5, Default::default()), // 13
            Corner::new(None,     5, Default::default()), // 14
    
            Corner::new(Some(1),  5, Default::default()), // 15
            Corner::new(Some(12), 0, Default::default()), // 16
            Corner::new(None,     5, Default::default()), // 17
        ];

        mesh.collapse_edge(&EdgeRef::new(12, &mesh), &Point3::new(0.5, 0.5, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_boundary_edge() {
        let mut mesh = create_collapse_edge_sample_mesh3();

        let expected_vertices = vec![
            VertexF::new(0,  Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(6,  Point3::<f32>::new(2.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6,  Point3::<f32>::new(3.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(7,  Point3::<f32>::new(4.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(2,  Point3::<f32>::new(2.0, 1.0, 0.0), Default::default()), // 4
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(7),  0, Default::default()), // 0
            Corner::new(None,     1, Default::default()), // 1
            Corner::new(None,     4, Default::default()), // 2
    
            Corner::new(Some(7), 1, Default::default()), // 3
            Corner::new(Some(0), 1, Default::default()), // 4
            Corner::new(None,    4, Default::default()), // 5
    
            Corner::new(None,    1, Default::default()), // 6
            Corner::new(Some(0), 3, Default::default()), // 7
            Corner::new(None,    4, Default::default()), // 8
        ];

        mesh.collapse_edge(&EdgeRef::new(5, &mesh), &Point3::new(2.0, 0.0, 0.0));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn flip_edge() {
        let mut mesh = create_flip_edge_sample_mesh();

        let expected_vertices = vec![
            VertexF::new(4, Point3::<f32>::new(0.5, 1.0, 0.0), Default::default()), // 0
            VertexF::new(0, Point3::<f32>::new(0.0, 0.5, 0.0), Default::default()), // 1
            VertexF::new(1, Point3::<f32>::new(0.5, 0.0, 0.0), Default::default()), // 2
            VertexF::new(2, Point3::<f32>::new(1.0, 0.5, 0.0), Default::default()), // 3
            VertexF::new(13, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default()), // 4
            VertexF::new(16, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()), // 5
            VertexF::new(7, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()), // 6
            VertexF::new(10, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()), // 7
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(10), 1, Default::default()), // 0
            Corner::new(Some(4),  2, Default::default()), // 1
            Corner::new(Some(7),  3, Default::default()), // 2
        
            Corner::new(Some(16), 3, Default::default()), // 3
            Corner::new(Some(1),  0, Default::default()), // 4
            Corner::new(Some(13), 1, Default::default()), // 5

            Corner::new(None,    1, Default::default()), // 6
            Corner::new(Some(2), 6, Default::default()), // 7
            Corner::new(None,    2, Default::default()), // 8

            Corner::new(None,    2, Default::default()), // 9
            Corner::new(Some(0), 7, Default::default()), // 10
            Corner::new(None,    3, Default::default()), // 11

            Corner::new(None,    3, Default::default()), // 12
            Corner::new(Some(5), 4, Default::default()), // 13
            Corner::new(None,    0, Default::default()), // 14

            Corner::new(None,    0, Default::default()), // 15
            Corner::new(Some(3), 5, Default::default()), // 16
            Corner::new(None,    1,  Default::default()), // 17
        ];

        mesh.flip_edge(&EdgeRef::new(1, &mesh));

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }
}
