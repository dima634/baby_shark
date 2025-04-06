use crate::{
    mesh::traits::{EditableMesh, SplitFaceAtPoint}, 
    geometry::traits::RealNumber, helpers::aliases::Vec3};
use super::{
    corner, traits::Flags, traversal::{collect_corners_around_vertex, CornerWalker}, vertex::VertexId, CornerTable
};

/// Set corner for wing vertex of collapsed edge
#[inline]
fn set_corner_for_wing_vertex<TScalar: RealNumber>(
    corner_table: &mut CornerTable<TScalar>, 
    vertex_id: VertexId, 
    opposite_corner_left: Option<usize>,
    opposite_corner_right: Option<usize>
) {
    if let Some(corner) = opposite_corner_left {
        corner_table[vertex_id].set_corner_index(corner::previous(corner));
    } else if let Some(corner) = opposite_corner_right {
        corner_table[vertex_id].set_corner_index(corner::next(corner));
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
        debug_assert!(!corner_table.corners[c1_idx].is_deleted());
        corner_table.corners[c1_idx].set_opposite_corner_index(c2);
    }
    
    if let Some(c2_idx) = c2 {
        debug_assert!(!corner_table.corners[c2_idx].is_deleted());
        corner_table.corners[c2_idx].set_opposite_corner_index(c1);
    }
}

impl<TScalar: RealNumber> CornerTable<TScalar> {
    /// Splits inner edge opposite to corner at given position
    fn split_inner_edge(&mut self, corner_index: usize, at: &Vec3<TScalar>) {
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
        let v1_idx = walker.get_corner().vertex();
        let c2_idx = walker.next().get_corner_index();
        let v2_idx = walker.get_corner().vertex();
        let c3_idx = walker.swing_right().get_corner_index();
        let v3_idx = walker.next().get_corner().vertex();
        let c5_idx = walker.next().get_corner_index();

        // Shift existing vertex
        let old_vertex_position = *walker.next().vertex().position();
        self.shift_vertex(&v2_idx, at);
        self[v2_idx].set_corner_index(c2_idx);

        // New vertex, instead of shifted
        let (new_vertex_id, new_vertex) = self.create_vertex();
        new_vertex.set_corner_index(c7_idx);
        new_vertex.set_position(old_vertex_position);

        // Update vertex index of existing corners
        for corner_index in collect_corners_around_vertex(self, v2_idx) {
            if corner_index != c3_idx && corner_index != c2_idx {
                self.get_corner_mut(corner_index).unwrap().set_vertex(new_vertex_id);
            }
        }

        // Create new faces
        self.create_face_from_vertices(v1_idx, new_vertex_id, v2_idx);
        self.create_face_from_vertices(new_vertex_id, v3_idx, v2_idx);

        // Update opposites
        if let Some(c0_opposite_idx) = self.get_corner(c0_idx).unwrap().opposite_corner_index() {
            self.set_opposite_relationship(c0_opposite_idx, c8_idx);
        }

        if let Some(c5_opposite_idx) = self.get_corner(c5_idx).unwrap().opposite_corner_index() {
            self.set_opposite_relationship(c5_opposite_idx, c11_idx);
        }

        self.set_opposite_relationship(c0_idx, c7_idx);
        self.set_opposite_relationship(c5_idx, c9_idx);
        self.set_opposite_relationship(c6_idx, c10_idx);
    }

    /// Splits boundary edge opposite to corner at given position
    fn split_boundary_edge(&mut self, corner_index: usize, at: &Vec3<TScalar>) {
        // New corner indices
        let c3_idx = self.corners.len();
        let c4_idx = c3_idx + 1;
        let c5_idx = c3_idx + 2;
        
        // Existing corners and vertices that needs to be updated
        let mut walker = CornerWalker::from_corner(self, corner_index);
        let c0_idx = walker.get_previous_corner_index();
        let v1_idx = walker.get_corner().vertex();
        let c2_idx = walker.next().get_corner_index();
        let v2_idx = walker.get_corner().vertex();

        // Shift existing vertex
        let old_vertex_position = *walker.vertex().position();
        self.shift_vertex(&v2_idx, at);
        self[v2_idx].set_corner_index(c2_idx);

        // New vertex, instead of shifted
        let (new_vertex_id, new_vertex) = self.create_vertex();
        new_vertex.set_corner_index(c4_idx);
        new_vertex.set_position(old_vertex_position);

        // Update vertex index of existing corners
        for corner_index in collect_corners_around_vertex(self, v2_idx) {
            if corner_index != c3_idx && corner_index != c2_idx {
                self.get_corner_mut(corner_index).unwrap().set_vertex(new_vertex_id);
            }
        }

        // Create new face
        self.create_face_from_vertices(v1_idx, new_vertex_id, v2_idx);

        // Update opposites
        if let Some(c0_opposite_idx) = self.get_corner(c0_idx).unwrap().opposite_corner_index() {
            self.set_opposite_relationship(c0_opposite_idx, c5_idx);
        }

        self.set_opposite_relationship(c0_idx, c4_idx);
    }
}

impl<TScalar: RealNumber> EditableMesh for CornerTable<TScalar> {
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Vec3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());

        // Collect corners of faces that is going to be removed, 
        // vertices of collapsed edge and corners that going to be opposite after collapse
        let c24_idx = walker.get_corner_index();
        let v7_idx = walker.get_corner().vertex();

        let c25_idx = walker.next().get_corner_index();
        let v8_idx = walker.get_corner().vertex();
        let c21_idx = walker.get_corner().opposite_corner_index();

        let c26_idx = walker.next().get_corner_index();
        let c28_idx = walker.get_corner().opposite_corner_index();
        let v9_idx = walker.get_corner().vertex();

        walker.next();

        let mut c6_idx = None;
        let mut c13_idx = None;

        let is_boundary_edge = walker.get_corner().opposite_corner_index().is_none();
    
        if !is_boundary_edge {
            let c9_idx = walker.opposite().get_corner_index();
            let v3_idx = walker.get_corner().vertex();

            let c10_idx = walker.next().get_corner_index();
            c6_idx = walker.get_corner().opposite_corner_index();
        
            let c11_idx = walker.next().get_corner_index();
            c13_idx = walker.get_corner().opposite_corner_index();

            // Update vertex for corners around removed one
            for corner_index in collect_corners_around_vertex(self, v9_idx) {
                self.corners[corner_index].set_vertex(v8_idx);
            }
            
            // Make sure vertices are not referencing deleted corners
            set_corner_for_wing_vertex(self, v3_idx, c13_idx, c6_idx);

            // Delete face
            self.corners[c9_idx].set_deleted(true);
            self.corners[c10_idx].set_deleted(true);
            self.corners[c11_idx].set_deleted(true);
        } else {
            // Update vertex for corners around removed one
            for corner_index in collect_corners_around_vertex(self, v9_idx) {
                self.corners[corner_index].set_vertex(v8_idx);
            }
        }
        // Make sure vertices are not referencing deleted corners
        set_corner_for_wing_vertex(self, v7_idx, c28_idx, c21_idx);

        // Delete face
        self.corners[c24_idx].set_deleted(true);
        self.corners[c25_idx].set_deleted(true);
        self.corners[c26_idx].set_deleted(true);

        // Remove vertex on edge end
        self[v9_idx].set_deleted(true);


        // Shift vertex on other side of edge
        self[v8_idx].set_position(*at);
        set_corner_for_wing_vertex(self, v8_idx, c6_idx.or(c21_idx), c28_idx.or(c13_idx));

        // Setup new opposites
        make_corners_opposite(self, c28_idx, c21_idx);
        make_corners_opposite(self, c6_idx, c13_idx);
    }

    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());

        // Face 1
        let c1_idx = walker.get_corner_index();
        let v1_idx = walker.get_corner().vertex();

        let c2_idx = walker.next().get_corner_index();
        let c2 = walker.get_corner();
        let v2_idx = c2.vertex();
        let c2_opp = c2.opposite_corner_index();

        let c0_idx = walker.next().get_corner_index();
        let c0 = walker.get_corner();
        let v0_idx = c0.vertex();
        let c0_opp = c0.opposite_corner_index();

        // Face 2
        let c4_idx = walker.next().opposite().get_corner_index();
        let v3_idx = walker.get_corner().vertex();

        let c5_idx = walker.next().get_corner_index();
        let c5_opp = walker.get_corner().opposite_corner_index();

        let c3_idx = walker.next().get_corner_index();
        let c3_opp = walker.get_corner().opposite_corner_index();

        // Update corners
        self.corners[c0_idx].set_vertex(v1_idx);
        make_corners_opposite(self, Some(c0_idx), c5_opp);
        self.corners[c1_idx].set_vertex(v2_idx);
        make_corners_opposite(self, Some(c1_idx), Some(c4_idx));
        self.corners[c2_idx].set_vertex(v3_idx);
        make_corners_opposite(self, Some(c2_idx), c0_opp);

        self.corners[c3_idx].set_vertex(v3_idx);
        make_corners_opposite(self, Some(c3_idx), c2_opp);
        self.corners[c4_idx].set_vertex(v0_idx);
        self.corners[c5_idx].set_vertex(v1_idx);
        make_corners_opposite(self, Some(c5_idx), c3_opp);

        // Make sure vertices are referencing correct corners
        self[v0_idx].set_corner_index(c4_idx);
        self[v1_idx].set_corner_index(c0_idx);
        self[v2_idx].set_corner_index(c1_idx);
        self[v3_idx].set_corner_index(c2_idx);
    }

    #[inline]
    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Vec3<Self::ScalarType>) {
        let corner_index = edge.get_corner_index();
        let corner = &self.corners[corner_index];

        match corner.opposite_corner_index() {
            Some(_) => self.split_inner_edge(corner_index, at),
            None => self.split_boundary_edge(corner_index, at),
        }
    }

    #[inline]
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Vec3<Self::ScalarType>) {
        self[*vertex].set_position(*to);
    }

    #[inline]
    fn edge_exist(&self, edge: &Self::EdgeDescriptor) -> bool {
        !self.corners[edge.get_corner_index()].is_deleted()
    }
}

impl<TScalar: RealNumber> SplitFaceAtPoint for CornerTable<TScalar> {
    fn split_face(&mut self, face: &Self::FaceDescriptor, point: Vec3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, *face);

        // Splitted face
        let c0_idx = walker.get_corner_index();
        let v0_idx = walker.get_corner().vertex();
        let c0_opp = walker.get_corner().opposite_corner_index();

        let c1_idx = walker.next().get_corner_index();
        let v1_idx = walker.get_corner().vertex();
        let c1_opp = walker.get_corner().opposite_corner_index();
        
        let c2_idx = walker.next().get_corner_index();
        let v2_idx = walker.get_corner().vertex();
        
        // Create new vertex at split point
        let (new_vertex_id, new_vertex) = self.create_vertex();
        new_vertex.set_corner_index(c2_idx);
        new_vertex.set_position(point);

        // New faces required for split
        let c3_idx = self.create_face_from_vertices(v1_idx, v2_idx, new_vertex_id);
        let c4_idx = corner::next(c3_idx);
        let c5_idx = corner::next(c4_idx);

        let c6_idx = self.create_face_from_vertices(v2_idx, v0_idx, new_vertex_id);
        let c7_idx = corner::next(c6_idx);
        let c8_idx = corner::next(c7_idx);

        // Corners relationship between internal faces
        self.set_opposite_relationship(c0_idx, c4_idx);
        self.set_opposite_relationship(c3_idx, c7_idx);
        self.set_opposite_relationship(c6_idx, c1_idx);

        // Corners relationship with external faces
        make_corners_opposite(self, Some(c8_idx), c1_opp);
        make_corners_opposite(self, Some(c5_idx), c0_opp);

        self.corners[c2_idx].set_vertex(new_vertex_id);
        self[v2_idx].set_corner_index(c4_idx);
    }
}

#[cfg(test)]
mod tests {
    use crate::{helpers::aliases::Vec3f, mesh::{
        corner_table::{
            corner::Corner, descriptors::EdgeRef, test_helpers::{
                assert_mesh_eq, create_collapse_edge_sample_mesh1, create_collapse_edge_sample_mesh2, create_collapse_edge_sample_mesh3, create_flip_edge_sample_mesh, create_single_face_mesh, create_unit_cross_square_mesh, create_unit_square_mesh
            }, vertex::{VertexF, VertexId}}, 
        traits::{EditableMesh, SplitFaceAtPoint}
    }};

    #[test]
    fn split_inner_edge1() {
        let mut mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(1, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(2, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 2
            VertexF::new(4, Vec3f::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(7, Vec3f::new(1.0, 0.0, 0.0), Default::default())  // 4
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(Some(7), VertexId::new(0)), // 0
            Corner::new(Some(4), VertexId::new(1)), // 1
            Corner::new(None,    VertexId::new(2)), // 2
            Corner::new(None,    VertexId::new(2)), // 3
            Corner::new(Some(1), VertexId::new(3)), // 4
            Corner::new(Some(9), VertexId::new(0)), // 5
            Corner::new(Some(10), VertexId::new(1)), // 6
            Corner::new(Some(0),  VertexId::new(4)), // 7
            Corner::new(None,     VertexId::new(2)), // 8
            Corner::new(Some(5), VertexId::new(4)), // 9
            Corner::new(Some(6), VertexId::new(3)), // 10
            Corner::new(None,    VertexId::new(2)), // 11
        ];

        mesh.split_edge(&EdgeRef::new(1, &mesh), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_inner_edge2() {
        let mut mesh = create_unit_cross_square_mesh();

        let expected_vertices = vec![
            VertexF::new(10, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(7, Vec3f::new(0.75, 0.75, 0.0), Default::default()), // 3
            VertexF::new(11, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 4
            VertexF::new(13, Vec3f::new(1.0, 1.0, 0.0), Default::default())  // 5
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  VertexId::new(0)), // 0
            Corner::new(Some(9),  VertexId::new(1)), // 1
            Corner::new(None,     VertexId::new(4)), // 2
            Corner::new(Some(7),  VertexId::new(1)), // 3
            Corner::new(Some(0),  VertexId::new(2)), // 4
            Corner::new(None,     VertexId::new(4)), // 5
            Corner::new(Some(10), VertexId::new(2)), // 6
            Corner::new(Some(3),  VertexId::new(3)), // 7
            Corner::new(Some(13), VertexId::new(4)), // 8
            Corner::new(Some(1),  VertexId::new(3)), // 9
            Corner::new(Some(6),  VertexId::new(0)), // 10
            Corner::new(Some(15), VertexId::new(4)), // 11
            Corner::new(Some(16), VertexId::new(2)), // 12
            Corner::new(Some(8),  VertexId::new(5)), // 13
            Corner::new(None,     VertexId::new(3)), // 14
            Corner::new(Some(11), VertexId::new(5)), // 15
            Corner::new(Some(12), VertexId::new(0)), // 16
            Corner::new(None,     VertexId::new(3)), // 17
        ];

        mesh.split_edge(&EdgeRef::new(6, &mesh), &Vec3f::new(0.75, 0.75, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_boundary_edge() {
        let mut mesh = create_single_face_mesh();

        let expected_vertices = vec![
            VertexF::new(0, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(1, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(2, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 2
            VertexF::new(4, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 3
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4), VertexId::new(0)), // 0
            Corner::new(None,    VertexId::new(1)), // 1
            Corner::new(None,    VertexId::new(2)), // 2
            Corner::new(None,    VertexId::new(1)), // 3
            Corner::new(Some(0), VertexId::new(3)), // 4
            Corner::new(None,    VertexId::new(2)), // 5
        ];

        mesh.split_edge(&EdgeRef::new(1, &mesh), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge() {
        let mut mesh = create_collapse_edge_sample_mesh1();

        let expected_vertices = vec![
            VertexF::new(28, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(3, Vec3f::new(0.0, 0.5, 0.0), Default::default()), // 1
            VertexF::new(6, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(12, Vec3f::new(0.5, 0.0, 0.0), Default::default()), // 3
            VertexF::new(15, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 4
            VertexF::new(18, Vec3f::new(1.0, 0.5, 0.0), Default::default()), // 5
            VertexF::new(21, Vec3f::new(1.0, 1.0, 0.0), Default::default()), // 6
            VertexF::new(27, Vec3f::new(0.5, 1.0, 0.0), Default::default()), // 7
            VertexF::new(29, Vec3f::new(0.25, 0.5, 0.0), Default::default()), // 8
            VertexF::new(23, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 9
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  VertexId::new(0)), // 0
            Corner::new(Some(27), VertexId::new(1)), // 1
            Corner::new(None,     VertexId::new(9)), // 2
            Corner::new(Some(7), VertexId::new(1)), // 3
            Corner::new(Some(0), VertexId::new(2)), // 4
            Corner::new(None,    VertexId::new(9)), // 5
            Corner::new(Some(13), VertexId::new(2)), // 6
            Corner::new(Some(3),  VertexId::new(3)), // 7
            Corner::new(None,     VertexId::new(9)), // 8
            Corner::new(Some(24), VertexId::new(3)), // 9
            Corner::new(Some(6),  VertexId::new(9)), // 10
            Corner::new(Some(13), VertexId::new(9)), // 11
            Corner::new(Some(16), VertexId::new(3)), // 12
            Corner::new(Some(6),  VertexId::new(4)), // 13
            Corner::new(None,     VertexId::new(9)), // 14
            Corner::new(Some(19), VertexId::new(4)), // 15
            Corner::new(Some(12), VertexId::new(5)), // 16
            Corner::new(None,     VertexId::new(9)), // 17
            Corner::new(Some(22), VertexId::new(5)), // 18
            Corner::new(Some(15), VertexId::new(6)), // 19
            Corner::new(None,     VertexId::new(9)), // 20
            Corner::new(Some(28), VertexId::new(6)), // 21
            Corner::new(Some(18), VertexId::new(7)), // 22
            Corner::new(None,     VertexId::new(9)), // 23
            Corner::new(Some(9),  VertexId::new(7)), // 24
            Corner::new(Some(21), VertexId::new(9)), // 25
            Corner::new(Some(28), VertexId::new(9)), // 26
            Corner::new(Some(1),  VertexId::new(7)), // 27
            Corner::new(Some(21), VertexId::new(0)), // 28
            Corner::new(None,     VertexId::new(9)), // 29
        ];

        mesh.collapse_edge(&EdgeRef::new(9, &mesh), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge_with_one_vertex_on_boundary() {
        let mut mesh = create_collapse_edge_sample_mesh2();

        let expected_vertices = vec![
            VertexF::new(0,  Vec3f::new(0.5, 0.0, 0.0), Default::default()), // 0
            VertexF::new(3, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6, Vec3f::new(1.0, 0.5, 0.0), Default::default()), // 2
            VertexF::new(9, Vec3f::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(10, Vec3f::new(0.5, 1.0, 0.0), Default::default()), // 4
            VertexF::new(11, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 5
            VertexF::new(17, Vec3f::new(0.75, 0.5, 0.0), Default::default()), // 6
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(4),  VertexId::new(0)), // 0
            Corner::new(None,     VertexId::new(1)), // 1
            Corner::new(None,     VertexId::new(5)), // 2
            Corner::new(Some(7), VertexId::new(1)), // 3
            Corner::new(Some(0), VertexId::new(2)), // 4
            Corner::new(None,    VertexId::new(5)), // 5
            Corner::new(Some(10), VertexId::new(2)), // 6
            Corner::new(Some(3),  VertexId::new(3)), // 7
            Corner::new(None,     VertexId::new(5)), // 8
            Corner::new(None,     VertexId::new(3)), // 9
            Corner::new(Some(6),  VertexId::new(4)), // 10
            Corner::new(None,     VertexId::new(5)), // 11
            Corner::new(Some(16), VertexId::new(4)), // 12
            Corner::new(Some(9),  VertexId::new(5)), // 13
            Corner::new(None,     VertexId::new(5)), // 14
            Corner::new(Some(1),  VertexId::new(5)), // 15
            Corner::new(Some(12), VertexId::new(0)), // 16
            Corner::new(None,     VertexId::new(5)), // 17
        ];

        mesh.collapse_edge(&EdgeRef::new(12, &mesh), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_boundary_edge() {
        let mut mesh = create_collapse_edge_sample_mesh3();

        let expected_vertices = vec![
            VertexF::new(0,  Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(6,  Vec3f::new(2.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(6,  Vec3f::new(3.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(7,  Vec3f::new(4.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(2,  Vec3f::new(2.0, 1.0, 0.0), Default::default()), // 4
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(7),  VertexId::new(0)), // 0
            Corner::new(None,     VertexId::new(1)), // 1
            Corner::new(None,     VertexId::new(4)), // 2
            Corner::new(Some(7), VertexId::new(1)), // 3
            Corner::new(Some(0), VertexId::new(1)), // 4
            Corner::new(None,    VertexId::new(4)), // 5
            Corner::new(None,    VertexId::new(1)), // 6
            Corner::new(Some(0), VertexId::new(3)), // 7
            Corner::new(None,    VertexId::new(4)), // 8
        ];

        mesh.collapse_edge(&EdgeRef::new(5, &mesh), &Vec3f::new(2.0, 0.0, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn flip_edge() {
        let mut mesh = create_flip_edge_sample_mesh();

        let expected_vertices = vec![
            VertexF::new(4, Vec3f::new(0.5, 1.0, 0.0), Default::default()), // 0
            VertexF::new(0, Vec3f::new(0.0, 0.5, 0.0), Default::default()), // 1
            VertexF::new(1, Vec3f::new(0.5, 0.0, 0.0), Default::default()), // 2
            VertexF::new(2, Vec3f::new(1.0, 0.5, 0.0), Default::default()), // 3
            VertexF::new(13, Vec3f::new(1.0, 1.0, 0.0), Default::default()), // 4
            VertexF::new(16, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 5
            VertexF::new(7, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 6
            VertexF::new(10, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 7
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(10), VertexId::new(1)), // 0
            Corner::new(Some(4),  VertexId::new(2)), // 1
            Corner::new(Some(7),  VertexId::new(3)), // 2
            Corner::new(Some(16), VertexId::new(3),), // 3
            Corner::new(Some(1),  VertexId::new(0),), // 4
            Corner::new(Some(13), VertexId::new(1),), // 5
            Corner::new(None,     VertexId::new(1)), // 6
            Corner::new(Some(2),  VertexId::new(6)), // 7
            Corner::new(None,     VertexId::new(2)), // 8
            Corner::new(None,     VertexId::new(2)), // 9
            Corner::new(Some(0),  VertexId::new(7)), // 10
            Corner::new(None,     VertexId::new(3)), // 11
            Corner::new(None,     VertexId::new(3)), // 12
            Corner::new(Some(5),  VertexId::new(4)), // 13
            Corner::new(None,     VertexId::new(0)), // 14
            Corner::new(None,     VertexId::new(0)), // 15
            Corner::new(Some(3),  VertexId::new(5)), // 16
            Corner::new(None,     VertexId::new(1)), // 17
        ];

        mesh.flip_edge(&EdgeRef::new(1, &mesh));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_face() {
        let mut mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Vec3f::new(0.0, 1.0, 0.0), Default::default()), // 0
            VertexF::new(1, Vec3f::new(0.0, 0.0, 0.0), Default::default()), // 1
            VertexF::new(7, Vec3f::new(1.0, 0.0, 0.0), Default::default()), // 2
            VertexF::new(4, Vec3f::new(1.0, 1.0, 0.0), Default::default()), // 3
            VertexF::new(2, Vec3f::new(0.5, 0.5, 0.0), Default::default()), // 4
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(7),  VertexId::new(0)), // 0
            Corner::new(Some(9),  VertexId::new(1)), // 1
            Corner::new(None,     VertexId::new(4)), // 2
            Corner::new(None,     VertexId::new(2)), // 3
            Corner::new(Some(11), VertexId::new(3)), // 4
            Corner::new(None,     VertexId::new(0)), // 5
            Corner::new(Some(10), VertexId::new(1)), // 6
            Corner::new(Some(0),  VertexId::new(2)), // 7
            Corner::new(None,     VertexId::new(4)), // 8
            Corner::new(Some(1),  VertexId::new(2)), // 9
            Corner::new(Some(6),  VertexId::new(0)), // 10
            Corner::new(Some(4),  VertexId::new(4)), // 11
        ];

        mesh.split_face(&0, Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }
}
