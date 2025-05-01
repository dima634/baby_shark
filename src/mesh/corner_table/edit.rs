use super::{traversal::collect_corners_around_vertex, *};
use crate::helpers::aliases::Vec3;

// Topology editing operations
impl<S: RealNumber> CornerTable<S> {
    pub fn collapse_edge(&mut self, edge: EdgeId, at: &Vec3<S>) {
        if self[edge.corner()].is_deleted() {
            return;
        }

        let mut walker = CornerWalker::from_corner(self, edge.corner());

        // Collect corners of faces that is going to be removed,
        // vertices of collapsed edge and corners that going to be opposite after collapse
        let c24_idx = walker.corner_id();
        let v7_idx = walker.corner().vertex();

        let c25_idx = walker.move_to_next().corner_id();
        let v8_idx = walker.corner().vertex();
        let c21_idx = walker.corner().opposite_corner();

        let c26_idx = walker.move_to_next().corner_id();
        let c28_idx = walker.corner().opposite_corner();
        let v9_idx = walker.corner().vertex();

        walker.move_to_next();

        let mut c6_idx = None;
        let mut c13_idx = None;

        let is_boundary_edge = walker.corner().opposite_corner().is_none();

        if !is_boundary_edge {
            let c9_idx = walker.move_to_opposite().corner_id();
            let v3_idx = walker.corner().vertex();

            let c10_idx = walker.move_to_next().corner_id();
            c6_idx = walker.corner().opposite_corner();

            let c11_idx = walker.move_to_next().corner_id();
            c13_idx = walker.corner().opposite_corner();

            // Update vertex for corners around removed one
            for corner_index in collect_corners_around_vertex(self, v9_idx) {
                self[corner_index].set_vertex(v8_idx);
            }

            // Make sure vertices are not referencing deleted corners
            self.set_corner_for_wing_vertex(v3_idx, c13_idx, c6_idx);

            // Delete face
            self[c9_idx].set_deleted(true);
            self[c10_idx].set_deleted(true);
            self[c11_idx].set_deleted(true);
        } else {
            // Update vertex for corners around removed one
            for corner_id in collect_corners_around_vertex(self, v9_idx) {
                self[corner_id].set_vertex(v8_idx);
            }
        }

        // Make sure vertices are not referencing deleted corners
        self.set_corner_for_wing_vertex(v7_idx, c28_idx, c21_idx);

        // Delete face
        self[c24_idx].set_deleted(true);
        self[c25_idx].set_deleted(true);
        self[c26_idx].set_deleted(true);

        // Remove vertex on edge end
        self[v9_idx].set_deleted(true);

        // Shift vertex on other side of edge
        self[v8_idx].set_position(*at);
        self.set_corner_for_wing_vertex(v8_idx, c6_idx.or(c21_idx), c28_idx.or(c13_idx));

        // Setup new opposites
        self.make_corners_opposite(c28_idx, c21_idx);
        self.make_corners_opposite(c6_idx, c13_idx);
    }

    pub fn flip_edge(&mut self, edge: EdgeId) {
        if self[edge.corner()].is_deleted() {
            return;
        }

        let mut walker = CornerWalker::from_corner(self, edge.corner());

        // Face 1
        let c1_idx = walker.corner_id();
        let v1_idx = walker.corner().vertex();

        let c2_idx = walker.move_to_next().corner_id();
        let c2 = walker.corner();
        let v2_idx = c2.vertex();
        let c2_opp = c2.opposite_corner();

        let c0_idx = walker.move_to_next().corner_id();
        let c0 = walker.corner();
        let v0_idx = c0.vertex();
        let c0_opp = c0.opposite_corner();

        // Face 2
        let c4_idx = walker.move_to_next().move_to_opposite().corner_id();
        let v3_idx = walker.corner().vertex();

        let c5_idx = walker.move_to_next().corner_id();
        let c5_opp = walker.corner().opposite_corner();

        let c3_idx = walker.move_to_next().corner_id();
        let c3_opp = walker.corner().opposite_corner();

        // Update corners
        self[c0_idx].set_vertex(v1_idx);
        self.make_corners_opposite(Some(c0_idx), c5_opp);
        self[c1_idx].set_vertex(v2_idx);
        self.make_corners_opposite(Some(c1_idx), Some(c4_idx));
        self[c2_idx].set_vertex(v3_idx);
        self.make_corners_opposite(Some(c2_idx), c0_opp);

        self[c3_idx].set_vertex(v3_idx);
        self.make_corners_opposite(Some(c3_idx), c2_opp);
        self[c4_idx].set_vertex(v0_idx);
        self[c5_idx].set_vertex(v1_idx);
        self.make_corners_opposite(Some(c5_idx), c3_opp);

        // Make sure vertices are referencing correct corners
        self[v0_idx].set_corner(c4_idx);
        self[v1_idx].set_corner(c0_idx);
        self[v2_idx].set_corner(c1_idx);
        self[v3_idx].set_corner(c2_idx);
    }

    pub fn split_edge(&mut self, edge: EdgeId, at: &Vec3<S>) {
        if self[edge.corner()].is_deleted() {
            return;
        }

        let corner_id = edge.corner();
        let corner = &self[corner_id];

        match corner.opposite_corner() {
            Some(_) => self.split_inner_edge(corner_id, at),
            None => self.split_boundary_edge(corner_id, at),
        }
    }

    pub fn split_face(&mut self, face: FaceId, point: Vec3<S>) {
        if self[face.corner()].is_deleted() {
            return;
        }

        let mut walker = CornerWalker::from_corner(self, face.corner());

        // Splitted face
        let c0_idx = walker.corner_id();
        let v0_idx = walker.corner().vertex();
        let c0_opp = walker.corner().opposite_corner();

        let c1_idx = walker.move_to_next().corner_id();
        let v1_idx = walker.corner().vertex();
        let c1_opp = walker.corner().opposite_corner();

        let c2_idx = walker.move_to_next().corner_id();
        let v2_idx = walker.corner().vertex();

        // Create new vertex at split point
        let new_vertex_id = self.create_vertex(Some(c2_idx), point);

        // New faces required for split
        let (c3_idx, c4_idx, c5_idx) =
            self.create_isolated_face_from_vertices(v1_idx, v2_idx, new_vertex_id);
        let (c6_idx, c7_idx, c8_idx) =
            self.create_isolated_face_from_vertices(v2_idx, v0_idx, new_vertex_id);

        // Corners relationship between internal faces
        self.set_opposite_relationship(c0_idx, c4_idx);
        self.set_opposite_relationship(c3_idx, c7_idx);
        self.set_opposite_relationship(c6_idx, c1_idx);

        // Corners relationship with external faces
        self.make_corners_opposite(Some(c8_idx), c1_opp);
        self.make_corners_opposite(Some(c5_idx), c0_opp);

        self[c2_idx].set_vertex(new_vertex_id);
        self[v2_idx].set_corner(c4_idx);
    }
}

// Helper functions
impl<S: RealNumber> CornerTable<S> {
    /// Splits inner edge opposite to corner at given position
    fn split_inner_edge(&mut self, corner_id: CornerId, at: &Vec3<S>) {
        // Existing corners and vertices that needs to be updated
        let mut walker = CornerWalker::from_corner(self, corner_id);
        let c0_idx = walker.previous_corner_id();
        let v1_idx = walker.corner().vertex();
        let c2_idx = walker.move_to_next().corner_id();
        let v2_idx = walker.corner().vertex();
        let c3_idx = walker.swing_right().corner_id();
        let v3_idx = walker.move_to_next().corner().vertex();
        let c5_idx = walker.move_to_next().corner_id();

        // Shift existing vertex
        let old_vertex_position = *walker.move_to_next().vertex().position();
        self[v2_idx].set_position(*at);
        self[v2_idx].set_corner(c2_idx);

        // New vertex, instead of shifted
        let new_vertex_id = self.create_vertex(None, old_vertex_position);

        // Create new faces
        let (c6_idx, c7_idx, c8_idx) =
            self.create_isolated_face_from_vertices(v1_idx, new_vertex_id, v2_idx);
        let (c9_idx, c10_idx, c11_idx) =
            self.create_isolated_face_from_vertices(new_vertex_id, v3_idx, v2_idx);
        self[new_vertex_id].set_corner(c7_idx);

        // Update vertex index of existing corners
        for corner_index in collect_corners_around_vertex(self, v2_idx) {
            if corner_index != c3_idx && corner_index != c2_idx {
                self[corner_index].set_vertex(new_vertex_id);
            }
        }

        // Update opposites
        if let Some(c0_opposite_idx) = self[c0_idx].opposite_corner() {
            self.set_opposite_relationship(c0_opposite_idx, c8_idx);
        }

        if let Some(c5_opposite_idx) = self[c5_idx].opposite_corner() {
            self.set_opposite_relationship(c5_opposite_idx, c11_idx);
        }

        self.set_opposite_relationship(c0_idx, c7_idx);
        self.set_opposite_relationship(c5_idx, c9_idx);
        self.set_opposite_relationship(c6_idx, c10_idx);
    }

    /// Splits boundary edge opposite to corner at given position
    fn split_boundary_edge(&mut self, corner_id: CornerId, at: &Vec3<S>) {
        // Existing corners and vertices that needs to be updated
        let mut walker = CornerWalker::from_corner(self, corner_id);
        let c0_idx = walker.previous_corner_id();
        let v1_idx = walker.corner().vertex();
        let c2_idx = walker.move_to_next().corner_id();
        let v2_idx = walker.corner().vertex();

        // Shift existing vertex
        let old_vertex_position = *walker.vertex().position();
        self[v2_idx].set_position(*at);
        self[v2_idx].set_corner(c2_idx);

        // New vertex, instead of shifted
        let new_vertex_id = self.create_vertex(None, old_vertex_position);

        // Create new face
        let (c3_idx, c4_idx, c5_idx) =
            self.create_isolated_face_from_vertices(v1_idx, new_vertex_id, v2_idx);
        self[new_vertex_id].set_corner(c4_idx);

        // Update vertex index of existing corners
        for corner_id in collect_corners_around_vertex(self, v2_idx) {
            if corner_id != c3_idx && corner_id != c2_idx {
                self[corner_id].set_vertex(new_vertex_id);
            }
        }

        // Update opposites
        if let Some(c0_opposite) = self[c0_idx].opposite_corner() {
            self.set_opposite_relationship(c0_opposite, c5_idx);
        }

        self.set_opposite_relationship(c0_idx, c4_idx);
    }

    /// Makes give corners opposite to each other
    #[inline]
    fn set_opposite_relationship(&mut self, corner1_id: CornerId, corner2_id: CornerId) {
        self[corner1_id].set_opposite_corner(Some(corner2_id));
        self[corner2_id].set_opposite_corner(Some(corner1_id));
    }

    /// Set corner for wing vertex of collapsed edge
    #[inline]
    fn set_corner_for_wing_vertex(
        &mut self,
        vertex_id: VertexId,
        opposite_corner_left: Option<CornerId>,
        opposite_corner_right: Option<CornerId>,
    ) {
        if let Some(corner) = opposite_corner_left {
            self[vertex_id].set_corner(corner.previous());
        } else if let Some(corner) = opposite_corner_right {
            self[vertex_id].set_corner(corner.next());
        }
    }

    /// Make corners opposite to each other
    #[inline]
    fn make_corners_opposite(&mut self, c1: Option<CornerId>, c2: Option<CornerId>) {
        if let Some(c1) = c1 {
            debug_assert!(!self[c1].is_deleted());
            self[c1].set_opposite_corner(c2);
        }

        if let Some(c2) = c2 {
            debug_assert!(!self[c2].is_deleted());
            self[c2].set_opposite_corner(c1);
        }
    }

    fn create_isolated_face_from_vertices(
        &mut self,
        v1: VertexId,
        v2: VertexId,
        v3: VertexId,
    ) -> (CornerId, CornerId, CornerId) {
        let (c1, _) = self.create_corner(v1);
        let (c2, _) = self.create_corner(v2);
        let (c3, _) = self.create_corner(v3);
        (c1, c2, c3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        helpers::aliases::Vec3f,
        mesh::corner_table::test_helpers::*,
    };

    #[test]
    fn split_inner_edge1() {
        let mut mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(5), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(1), Vec3f::new(0.0, 0.0, 0.0)), // 1
            Vertex::new(CornerId::new(2), Vec3f::new(0.5, 0.5, 0.0)), // 2
            Vertex::new(CornerId::new(4), Vec3f::new(1.0, 1.0, 0.0)), // 3
            Vertex::new(CornerId::new(7), Vec3f::new(1.0, 0.0, 0.0)), // 4
        ];

        let expected_corners = vec![
            // next, opposite, vertex, index, flags
            Corner::new(Some(CornerId::new(7)), VertexId::new(0)), // 0
            Corner::new(Some(CornerId::new(4)), VertexId::new(1)), // 1
            Corner::new(None, VertexId::new(2)),                   // 2
            Corner::new(None, VertexId::new(2)),                   // 3
            Corner::new(Some(CornerId::new(1)), VertexId::new(3)), // 4
            Corner::new(Some(CornerId::new(9)), VertexId::new(0)), // 5
            Corner::new(Some(CornerId::new(10)), VertexId::new(1)), // 6
            Corner::new(Some(CornerId::new(0)), VertexId::new(4)), // 7
            Corner::new(None, VertexId::new(2)),                   // 8
            Corner::new(Some(CornerId::new(5)), VertexId::new(4)), // 9
            Corner::new(Some(CornerId::new(6)), VertexId::new(3)), // 10
            Corner::new(None, VertexId::new(2)),                   // 11
        ];

        mesh.split_edge(EdgeId::new(CornerId::new(1)), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_inner_edge2() {
        let mut mesh = create_unit_cross_square_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(10), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(3), Vec3f::new(0.0, 0.0, 0.0)),  // 1
            Vertex::new(CornerId::new(6), Vec3f::new(1.0, 0.0, 0.0)),  // 2
            Vertex::new(CornerId::new(7), Vec3f::new(0.75, 0.75, 0.0)), // 3
            Vertex::new(CornerId::new(11), Vec3f::new(0.5, 0.5, 0.0)), // 4
            Vertex::new(CornerId::new(13), Vec3f::new(1.0, 1.0, 0.0)), // 5
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(4)), VertexId::new(0)), // 0
            Corner::new(Some(CornerId::new(9)), VertexId::new(1)), // 1
            Corner::new(None, VertexId::new(4)),                   // 2
            Corner::new(Some(CornerId::new(7)), VertexId::new(1)), // 3
            Corner::new(Some(CornerId::new(0)), VertexId::new(2)), // 4
            Corner::new(None, VertexId::new(4)),                   // 5
            Corner::new(Some(CornerId::new(10)), VertexId::new(2)), // 6
            Corner::new(Some(CornerId::new(3)), VertexId::new(3)), // 7
            Corner::new(Some(CornerId::new(13)), VertexId::new(4)), // 8
            Corner::new(Some(CornerId::new(1)), VertexId::new(3)), // 9
            Corner::new(Some(CornerId::new(6)), VertexId::new(0)), // 10
            Corner::new(Some(CornerId::new(15)), VertexId::new(4)), // 11
            Corner::new(Some(CornerId::new(16)), VertexId::new(2)), // 12
            Corner::new(Some(CornerId::new(8)), VertexId::new(5)), // 13
            Corner::new(None, VertexId::new(3)),                   // 14
            Corner::new(Some(CornerId::new(11)), VertexId::new(5)), // 15
            Corner::new(Some(CornerId::new(12)), VertexId::new(0)), // 16
            Corner::new(None, VertexId::new(3)),                   // 17
        ];

        mesh.split_edge(EdgeId::new(CornerId::new(6)), &Vec3f::new(0.75, 0.75, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_boundary_edge() {
        let mut mesh = create_single_face_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(0), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(1), Vec3f::new(0.0, 0.0, 0.0)), // 1
            Vertex::new(CornerId::new(2), Vec3f::new(0.5, 0.5, 0.0)), // 2
            Vertex::new(CornerId::new(4), Vec3f::new(1.0, 0.0, 0.0)), // 3
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(4)), VertexId::new(0)), // 0
            Corner::new(None, VertexId::new(1)),                   // 1
            Corner::new(None, VertexId::new(2)),                   // 2
            Corner::new(None, VertexId::new(1)),                   // 3
            Corner::new(Some(CornerId::new(0)), VertexId::new(3)), // 4
            Corner::new(None, VertexId::new(2)),                   // 5
        ];

        mesh.split_edge(EdgeId::new(CornerId::new(1)), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge() {
        let mut mesh = create_collapse_edge_sample_mesh1();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(28), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(3), Vec3f::new(0.0, 0.5, 0.0)),  // 1
            Vertex::new(CornerId::new(6), Vec3f::new(0.0, 0.0, 0.0)),  // 2
            Vertex::new(CornerId::new(12), Vec3f::new(0.5, 0.0, 0.0)), // 3
            Vertex::new(CornerId::new(15), Vec3f::new(1.0, 0.0, 0.0)), // 4
            Vertex::new(CornerId::new(18), Vec3f::new(1.0, 0.5, 0.0)), // 5
            Vertex::new(CornerId::new(21), Vec3f::new(1.0, 1.0, 0.0)), // 6
            Vertex::new(CornerId::new(27), Vec3f::new(0.5, 1.0, 0.0)), // 7
            Vertex::new(CornerId::new(29), Vec3f::new(0.25, 0.5, 0.0)), // 8
            Vertex::new(CornerId::new(23), Vec3f::new(0.5, 0.5, 0.0)), // 9
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(4)), VertexId::new(0)), // 0
            Corner::new(Some(CornerId::new(27)), VertexId::new(1)), // 1
            Corner::new(None, VertexId::new(9)),                   // 2
            Corner::new(Some(CornerId::new(7)), VertexId::new(1)), // 3
            Corner::new(Some(CornerId::new(0)), VertexId::new(2)), // 4
            Corner::new(None, VertexId::new(9)),                   // 5
            Corner::new(Some(CornerId::new(13)), VertexId::new(2)), // 6
            Corner::new(Some(CornerId::new(3)), VertexId::new(3)), // 7
            Corner::new(None, VertexId::new(9)),                   // 8
            Corner::new(Some(CornerId::new(24)), VertexId::new(3)), // 9
            Corner::new(Some(CornerId::new(6)), VertexId::new(9)), // 10
            Corner::new(Some(CornerId::new(13)), VertexId::new(9)), // 11
            Corner::new(Some(CornerId::new(16)), VertexId::new(3)), // 12
            Corner::new(Some(CornerId::new(6)), VertexId::new(4)), // 13
            Corner::new(None, VertexId::new(9)),                   // 14
            Corner::new(Some(CornerId::new(19)), VertexId::new(4)), // 15
            Corner::new(Some(CornerId::new(12)), VertexId::new(5)), // 16
            Corner::new(None, VertexId::new(9)),                   // 17
            Corner::new(Some(CornerId::new(22)), VertexId::new(5)), // 18
            Corner::new(Some(CornerId::new(15)), VertexId::new(6)), // 19
            Corner::new(None, VertexId::new(9)),                   // 20
            Corner::new(Some(CornerId::new(28)), VertexId::new(6)), // 21
            Corner::new(Some(CornerId::new(18)), VertexId::new(7)), // 22
            Corner::new(None, VertexId::new(9)),                   // 23
            Corner::new(Some(CornerId::new(9)), VertexId::new(7)), // 24
            Corner::new(Some(CornerId::new(21)), VertexId::new(9)), // 25
            Corner::new(Some(CornerId::new(28)), VertexId::new(9)), // 26
            Corner::new(Some(CornerId::new(1)), VertexId::new(7)), // 27
            Corner::new(Some(CornerId::new(21)), VertexId::new(0)), // 28
            Corner::new(None, VertexId::new(9)),                   // 29
        ];

        mesh.collapse_edge(EdgeId::new(CornerId::new(9)), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_edge_with_one_vertex_on_boundary() {
        let mut mesh = create_collapse_edge_sample_mesh2();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(0), Vec3f::new(0.5, 0.0, 0.0)), // 0
            Vertex::new(CornerId::new(3), Vec3f::new(1.0, 0.0, 0.0)), // 1
            Vertex::new(CornerId::new(6), Vec3f::new(1.0, 0.5, 0.0)), // 2
            Vertex::new(CornerId::new(9), Vec3f::new(1.0, 1.0, 0.0)), // 3
            Vertex::new(CornerId::new(10), Vec3f::new(0.5, 1.0, 0.0)), // 4
            Vertex::new(CornerId::new(11), Vec3f::new(0.5, 0.5, 0.0)), // 5
            Vertex::new(CornerId::new(17), Vec3f::new(0.75, 0.5, 0.0)), // 6
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(4)), VertexId::new(0)), // 0
            Corner::new(None, VertexId::new(1)),                   // 1
            Corner::new(None, VertexId::new(5)),                   // 2
            Corner::new(Some(CornerId::new(7)), VertexId::new(1)), // 3
            Corner::new(Some(CornerId::new(0)), VertexId::new(2)), // 4
            Corner::new(None, VertexId::new(5)),                   // 5
            Corner::new(Some(CornerId::new(10)), VertexId::new(2)), // 6
            Corner::new(Some(CornerId::new(3)), VertexId::new(3)), // 7
            Corner::new(None, VertexId::new(5)),                   // 8
            Corner::new(None, VertexId::new(3)),                   // 9
            Corner::new(Some(CornerId::new(6)), VertexId::new(4)), // 10
            Corner::new(None, VertexId::new(5)),                   // 11
            Corner::new(Some(CornerId::new(16)), VertexId::new(4)), // 12
            Corner::new(Some(CornerId::new(9)), VertexId::new(5)), // 13
            Corner::new(None, VertexId::new(5)),                   // 14
            Corner::new(Some(CornerId::new(1)), VertexId::new(5)), // 15
            Corner::new(Some(CornerId::new(12)), VertexId::new(0)), // 16
            Corner::new(None, VertexId::new(5)),                   // 17
        ];

        mesh.collapse_edge(EdgeId::new(CornerId::new(12)), &Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn collapse_boundary_edge() {
        let mut mesh = create_collapse_edge_sample_mesh3();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(0), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(6), Vec3f::new(2.0, 0.0, 0.0)), // 1
            Vertex::new(CornerId::new(6), Vec3f::new(3.0, 0.0, 0.0)), // 2
            Vertex::new(CornerId::new(7), Vec3f::new(4.0, 1.0, 0.0)), // 3
            Vertex::new(CornerId::new(2), Vec3f::new(2.0, 1.0, 0.0)), // 4
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(7)), VertexId::new(0)), // 0
            Corner::new(None, VertexId::new(1)),                   // 1
            Corner::new(None, VertexId::new(4)),                   // 2
            Corner::new(Some(CornerId::new(7)), VertexId::new(1)), // 3
            Corner::new(Some(CornerId::new(0)), VertexId::new(1)), // 4
            Corner::new(None, VertexId::new(4)),                   // 5
            Corner::new(None, VertexId::new(1)),                   // 6
            Corner::new(Some(CornerId::new(0)), VertexId::new(3)), // 7
            Corner::new(None, VertexId::new(4)),                   // 8
        ];

        mesh.collapse_edge(EdgeId::new(CornerId::new(5)), &Vec3f::new(2.0, 0.0, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn flip_edge() {
        let mut mesh = create_flip_edge_sample_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(4), Vec3f::new(0.5, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(0), Vec3f::new(0.0, 0.5, 0.0)), // 1
            Vertex::new(CornerId::new(1), Vec3f::new(0.5, 0.0, 0.0)), // 2
            Vertex::new(CornerId::new(2), Vec3f::new(1.0, 0.5, 0.0)), // 3
            Vertex::new(CornerId::new(13), Vec3f::new(1.0, 1.0, 0.0)), // 4
            Vertex::new(CornerId::new(16), Vec3f::new(0.0, 1.0, 0.0)), // 5
            Vertex::new(CornerId::new(7), Vec3f::new(0.0, 0.0, 0.0)), // 6
            Vertex::new(CornerId::new(10), Vec3f::new(1.0, 0.0, 0.0)), // 7
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(10)), VertexId::new(1)), // 0
            Corner::new(Some(CornerId::new(4)), VertexId::new(2)),  // 1
            Corner::new(Some(CornerId::new(7)), VertexId::new(3)),  // 2
            Corner::new(Some(CornerId::new(16)), VertexId::new(3)), // 3
            Corner::new(Some(CornerId::new(1)), VertexId::new(0)),  // 4
            Corner::new(Some(CornerId::new(13)), VertexId::new(1)), // 5
            Corner::new(None, VertexId::new(1)),                    // 6
            Corner::new(Some(CornerId::new(2)), VertexId::new(6)),  // 7
            Corner::new(None, VertexId::new(2)),                    // 8
            Corner::new(None, VertexId::new(2)),                    // 9
            Corner::new(Some(CornerId::new(0)), VertexId::new(7)),  // 10
            Corner::new(None, VertexId::new(3)),                    // 11
            Corner::new(None, VertexId::new(3)),                    // 12
            Corner::new(Some(CornerId::new(5)), VertexId::new(4)),  // 13
            Corner::new(None, VertexId::new(0)),                    // 14
            Corner::new(None, VertexId::new(0)),                    // 15
            Corner::new(Some(CornerId::new(3)), VertexId::new(5)),  // 16
            Corner::new(None, VertexId::new(1)),                    // 17
        ];

        mesh.flip_edge(EdgeId::new(CornerId::new(1)));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn split_face() {
        let mut mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            Vertex::new(CornerId::new(5), Vec3f::new(0.0, 1.0, 0.0)), // 0
            Vertex::new(CornerId::new(1), Vec3f::new(0.0, 0.0, 0.0)), // 1
            Vertex::new(CornerId::new(7), Vec3f::new(1.0, 0.0, 0.0)), // 2
            Vertex::new(CornerId::new(4), Vec3f::new(1.0, 1.0, 0.0)), // 3
            Vertex::new(CornerId::new(2), Vec3f::new(0.5, 0.5, 0.0)), // 4
        ];

        let expected_corners = vec![
            // opposite, vertex, flags
            Corner::new(Some(CornerId::new(7)), VertexId::new(0)), // 0
            Corner::new(Some(CornerId::new(9)), VertexId::new(1)), // 1
            Corner::new(None, VertexId::new(4)),                   // 2
            Corner::new(None, VertexId::new(2)),                   // 3
            Corner::new(Some(CornerId::new(11)), VertexId::new(3)), // 4
            Corner::new(None, VertexId::new(0)),                   // 5
            Corner::new(Some(CornerId::new(10)), VertexId::new(1)), // 6
            Corner::new(Some(CornerId::new(0)), VertexId::new(2)), // 7
            Corner::new(None, VertexId::new(4)),                   // 8
            Corner::new(Some(CornerId::new(1)), VertexId::new(2)), // 9
            Corner::new(Some(CornerId::new(6)), VertexId::new(0)), // 10
            Corner::new(Some(CornerId::new(4)), VertexId::new(4)), // 11
        ];

        mesh.split_face(FaceId::new(0), Vec3f::new(0.5, 0.5, 0.0));

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }
}
