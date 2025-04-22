use super::{
    CornerTable,
    corner::{Corner, CornerId},
    vertex::{Vertex, VertexId},
    edge::EdgeId,
    face::FaceId,
    flags::clear_visited,
    traits::Flags,
};
use crate::{
    geometry::traits::RealNumber,
    mesh::traits::{mesh_stats::MAX_VERTEX_VALENCE, Position},
};

///
/// Can be used to traverse corner table topology
///
pub struct CornerWalker<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    corner: CornerId,
}

impl<'a, TScalar: RealNumber> CornerWalker<'a, TScalar> {
    /// Creates walker starting at given corner
    #[inline]
    pub fn from_corner(table: &'a CornerTable<TScalar>, corner: CornerId) -> Self {
        debug_assert!(!table[corner].is_deleted());
        Self {
            table,
            corner,
        }
    }

    /// Creates walker starting at random corner of given vertex
    #[inline]
    pub fn from_vertex(table: &'a CornerTable<TScalar>, vertex_id: VertexId) -> Self {
        Self::from_corner(table, table[vertex_id].corner())
    }

    /// Jumps to given corner
    #[inline]
    pub fn set_current_corner(&mut self, corner: CornerId) -> &mut Self {
        self.corner = corner;
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Moves to next corner
    #[allow(clippy::should_implement_trait)]
    #[inline]
    pub fn move_to_next(&mut self) -> &mut Self {
        self.corner = self.corner.next();
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Moves to opposite corner if exist, otherwise corner stays still
    #[inline]
    pub fn move_to_opposite(&mut self) -> &mut Self {
        if let Some(opposite) = self.corner().opposite_corner() {
            self.corner = opposite;
        } else {
            debug_assert!(false, "Moving to not existing corner");
        }

        debug_assert!(!self.table[self.corner].is_deleted());

        self
    }

    /// Moves to previous corner. Shorthand for next().next()
    #[inline]
    pub fn move_to_previous(&mut self) -> &mut Self {
        self.corner = self.corner_id().previous();
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Swings to right around corner vertex
    #[inline]
    pub fn swing_right(&mut self) -> &mut Self {
        return self.move_to_previous().move_to_opposite().move_to_previous();
    }

    pub fn try_swing_right(&mut self) -> bool {
        self.move_to_previous();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_previous();
            true
        } else {
            self.move_to_next();
            false
        }
    }

    /// Swings to left around corner vertex
    #[inline]
    pub fn swing_left(&mut self) -> &mut Self {
        return self.move_to_next().move_to_opposite().move_to_next();
    }

    // TODO
    // /// Returns `true` if it is possible to [`Self::swing_right()`] (corner is not on the border), `false` otherwise
    // #[inline]
    // pub fn can_swing_right(&self) -> bool {
    //     return self.previous_corner().opposite_corner().is_some();
    // }

    // /// Returns `true` if it is possible to [`Self::swing_left()`] (corner is not on the border), `false` otherwise
    // #[inline]
    // pub fn can_swing_left(&self) -> bool {
    //     return self.next_corner().opposite_corner().is_some();
    // }

    ///
    /// Trying to swing left and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing left walker stays at starting position.
    ///
    #[inline]
    pub fn swing_left_or_stay(&mut self) -> bool {
        self.move_to_next();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_next();
            true
        } else {
            self.move_to_previous();
            false
        }
    }

    ///
    /// Trying to swing right and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing right walker stays at starting position.
    ///
    #[inline]
    pub fn swing_right_or_stay(&mut self) -> bool {
        self.move_to_previous();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_previous();
            true
        } else {
            self.move_to_next();
            false
        }
    }

    /// Returns next corner
    #[inline]
    pub fn next_corner(&self) -> &Corner {
        &self.table[self.corner.next()]
    }

    /// Returns previous corner index
    #[inline]
    pub fn previous_corner_id(&self) -> CornerId {
        self.corner_id().previous()
    }

    /// Returns previous corner
    #[inline]
    pub fn previous_corner(&self) -> &Corner {
        &self.table[self.corner.previous()]
    }

    /// Returns opposite corner
    #[inline]
    pub fn opposite_corner(&self) -> Option<&Corner> {
        self.corner().opposite_corner().map(|corner| &self.table[corner])
    }

    /// Returns current corner
    #[inline]
    pub fn corner(&self) -> &Corner {
        &self.table[self.corner]
    }

    /// Returns current corner index
    #[inline]
    pub fn corner_id(&self) -> CornerId {
        self.corner
    }

    /// Returns vertex of current corner
    #[inline]
    pub fn vertex(&self) -> &Vertex<TScalar> {
        &self.table[self.corner().vertex()]
    }
}

impl<'a, TScalar: RealNumber> Position<'a, CornerTable<TScalar>> for CornerWalker<'a, TScalar> {
    fn from_vertex_on_face(
        mesh: &'a CornerTable<TScalar>,
        face: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor,
        vertex: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor,
    ) -> Self {
        let mut walker = CornerWalker::from_corner(mesh, face.corner());

        if walker.corner().vertex() == *vertex {
            return walker;
        }

        walker.move_to_next();

        if walker.corner().vertex() == *vertex {
            return walker;
        }

        walker.move_to_next();

        if walker.corner().vertex() == *vertex {
            return walker;
        }

        unreachable!("Input must be invalid or non-manifold");
    }

    #[inline]
    fn from_edge_on_face(
        mesh: &'a CornerTable<TScalar>,
        face: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor,
        edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor,
    ) -> Self {
        let corner = if *face == edge.corner().face() {
            edge.corner()
        } else {
            mesh[edge.corner()]
                .opposite_corner()
                .unwrap()
        };

        return CornerWalker::from_corner(mesh, corner);
    }

    #[inline]
    fn set_from_vertex_on_face(
        &mut self,
        face: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor,
        vertex: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor,
    ) -> &mut Self {
        self.set_current_corner(face.corner());

        while self.corner().vertex() != *vertex {
            self.move_to_next();
        }

        self
    }

    #[inline]
    fn set_from_edge_on_face(
        &mut self,
        face: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor,
        edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor,
    ) -> &mut Self {
        if *face == edge.corner().face() {
            self.set_current_corner(edge.corner());
        } else {
            let corner = self.table[edge.corner()]
                .opposite_corner()
                .unwrap();
            self.set_current_corner(corner);
        };

        self
    }

    #[inline]
    fn next(&mut self) -> &mut Self {
        return self.move_to_next();
    }

    #[inline]
    fn get_vertex(&self) -> <CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor {
        return self.corner().vertex();
    }

    #[inline]
    fn from_edge(
        mesh: &'a CornerTable<TScalar>,
        edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor,
    ) -> Self {
        return CornerWalker::from_corner(mesh, edge.corner());
    }

    #[inline]
    fn opposite(&mut self) -> &mut Self {
        return self.move_to_opposite();
    }
}

///
/// Iterator over faces of corner table. Face is returned as one of its corners.
///
pub struct CornerTableFacesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    face_index: usize,
}

impl<'a, TScalar: RealNumber> CornerTableFacesIter<'a, TScalar> {
    pub fn new(table: &'a CornerTable<TScalar>) -> Self {
        Self {
            table,
            face_index: 0,
        }
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableFacesIter<'a, TScalar> {
    type Item = FaceId;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let corner_index = self.face_index * 3;
            if corner_index >= self.table.corners.len() {
                return None;
            }

            if self.table.corners[corner_index].is_deleted() {
                self.face_index += 1;
            } else {
                break;
            }
        }

        let face = FaceId::new(self.face_index);
        self.face_index += 1;
        Some(face)
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        let size = self.table.corners.len() / 3;
        (0, Some(size))
    }
}

///
/// Iterator over vertices of mesh
///
pub struct CornerTableVerticesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    vertex_index: usize,
}

impl<'a, TScalar: RealNumber> CornerTableVerticesIter<'a, TScalar> {
    pub fn new(table: &'a CornerTable<TScalar>) -> Self {
        Self {
            table,
            vertex_index: 0,
        }
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableVerticesIter<'a, TScalar> {
    type Item = VertexId;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.vertex_index >= self.table.vertices.len() {
                return None;
            }

            if self.table.vertices[self.vertex_index].is_deleted() {
                self.vertex_index += 1;
            } else {
                break;
            }
        }

        let id = VertexId::new(self.vertex_index);
        self.vertex_index += 1;
        Some(id)
    }
}

///
/// Iterator over edges of mesh. Edge is returned as corner opposite to it. Uses `is_visited` flag
///
pub struct CornerTableEdgesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    corner_index: usize,
}

impl<'a, TScalar: RealNumber> CornerTableEdgesIter<'a, TScalar> {
    pub fn new(table: &'a CornerTable<TScalar>) -> Self {
        clear_visited(table.corners.iter());
        Self {
            table,
            corner_index: 0,
        }
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableEdgesIter<'a, TScalar> {
    type Item = EdgeId;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.corner_index >= self.table.corners.len() {
                return None;
            }

            let corner = &self.table.corners[self.corner_index];

            if corner.is_visited() || corner.is_deleted() {
                self.corner_index += 1;
            } else {
                break;
            }
        }

        // Visit current
        let corner = &self.table.corners[self.corner_index];
        corner.set_visited(true);

        // Visit opposite, it is referencing same edge as current
        if let Some(opposite_index) = corner.opposite_corner() {
            self.table[opposite_index].set_visited(true);
        }

        // Move to next
        let edge = EdgeId::new(CornerId::new(self.corner_index), self.table);
        self.corner_index += 1;
        Some(edge)
    }
}

/// Iterates over corners that are adjacent to given vertex
pub fn corners_around_vertex<TScalar: RealNumber, TFunc: FnMut(CornerId)>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
    mut visit: TFunc,
) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_id);
    walker.move_to_previous();
    let started_at = walker.corner_id();
    let mut border_reached = false;

    loop {
        visit(walker.corner_id().next());
        walker.move_to_previous();

        if walker.corner().opposite_corner().is_none() {
            border_reached = true;
            break;
        }

        walker.move_to_opposite();

        if started_at == walker.corner_id() {
            break;
        }
    }

    walker.set_current_corner(started_at);

    if border_reached && walker.corner().opposite_corner().is_some() {
        walker.move_to_opposite();

        loop {
            visit(walker.previous_corner_id());
            walker.move_to_next();

            if walker.corner().opposite_corner().is_none() {
                break;
            }

            walker.move_to_opposite();
        }
    }
}

pub fn collect_corners_around_vertex<TScalar: RealNumber>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
) -> Vec<CornerId> {
    let mut corners = Vec::with_capacity(MAX_VERTEX_VALENCE);
    corners_around_vertex(corner_table, vertex_id, |corner_id| {
        corners.push(corner_id)
    });

    corners
}

/// Iterates over one-ring vertices of vertex
pub fn vertices_around_vertex<TScalar: RealNumber, TFunc: FnMut(&VertexId)>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
    mut visit: TFunc,
) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_id);
    walker.move_to_previous();
    let started_at = walker.corner_id();
    let mut border_reached = false;

    loop {
        visit(&walker.corner().vertex());

        walker.move_to_previous();

        if walker.corner().opposite_corner().is_none() {
            border_reached = true;
            break;
        }

        walker.move_to_opposite();

        if started_at == walker.corner_id() {
            break;
        }
    }

    if border_reached {
        walker.set_current_corner(started_at).move_to_previous();
        loop {
            visit(&walker.corner().vertex());

            walker.move_to_next();

            if walker.corner().opposite_corner().is_none() {
                break;
            }

            walker.move_to_opposite();
        }
    }
}

/// Iterates over one-ring faces of vertex. Face is returned as one of it`s corners.
pub fn faces_around_vertex<TScalar: RealNumber, TFunc: FnMut(&FaceId)>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
    mut visit: TFunc,
) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_id);
    walker.move_to_previous();
    let started_at = walker.corner_id();
    let mut border_reached = false;

    loop {
        visit(&walker.corner_id().face());

        walker.move_to_previous();

        if walker.corner().opposite_corner().is_none() {
            border_reached = true;
            break;
        }

        walker.move_to_opposite();

        if started_at == walker.corner_id() {
            break;
        }
    }

    walker.set_current_corner(started_at);

    if border_reached && walker.corner().opposite_corner().is_some() {
        walker.move_to_opposite();

        loop {
            visit(&walker.corner_id().face());

            walker.move_to_next();

            if walker.corner().opposite_corner().is_none() {
                break;
            }

            walker.move_to_opposite();
        }
    }
}

/// Iterates over edges incident to vertex. Edge is represented by opposite corner index.
pub fn edges_around_vertex<TScalar: RealNumber, TFunc: FnMut(&EdgeId)>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
    mut visit: TFunc,
) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_id);
    walker.move_to_next();
    let started_at = walker.corner_id();
    let mut border_reached = false;

    loop {
        visit(&EdgeId::new(walker.corner_id(), corner_table));

        if walker.corner().opposite_corner().is_none() {
            border_reached = true;
            break;
        }

        walker.move_to_opposite().move_to_previous();

        if started_at == walker.corner_id() {
            break;
        }
    }

    if border_reached {
        walker.set_current_corner(started_at);
        walker.move_to_next();

        loop {
            visit(&EdgeId::new(walker.corner_id(), corner_table));

            if walker.corner().opposite_corner().is_none() {
                break;
            }

            walker.move_to_opposite().move_to_next();
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::{
        corner_table::{
            corner::CornerId, 
            edge::EdgeId, 
            face::FaceId,
            vertex::VertexId,
            test_helpers::{create_unit_cross_square_mesh, create_unit_square_mesh}, 
            traversal::{corners_around_vertex, faces_around_vertex, vertices_around_vertex},
        },
        traits::Mesh,
    };

    #[test]
    fn edges_iterator() {
        let mesh = create_unit_square_mesh();
        let expected_edges: Vec<EdgeId> = vec![
            EdgeId::new(CornerId::new(0), &mesh),
            EdgeId::new(CornerId::new(1), &mesh),
            EdgeId::new(CornerId::new(2), &mesh),
            EdgeId::new(CornerId::new(3), &mesh),
            EdgeId::new(CornerId::new(5), &mesh),
        ];

        assert_eq!(expected_edges.len(), mesh.edges().count());

        let pairs = mesh.edges().zip(expected_edges.iter());

        for pair in pairs {
            assert_eq!(pair.0, *pair.1);
        }
    }

    // Corners iter macro

    #[test]
    fn corners_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners = [11, 2, 5, 8].map(|el| CornerId::new(el)).to_vec();
        let mut corners = Vec::new();

        corners_around_vertex(&mesh, VertexId::new(4), |corner_index| {
            corners.push(corner_index)
        });

        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners = [10, 0].map(|el| CornerId::new(el)).to_vec();
        let mut corners = Vec::new();

        corners_around_vertex(&mesh, VertexId::new(0), |corner_index| {
            corners.push(corner_index)
        });

        assert_eq!(corners, expected_corners);
    }

    // Vertices iter macro

    #[test]
    fn vertices_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices = [0, 1, 2, 3].map(|el| VertexId::new(el)).to_vec();
        let mut vertices = Vec::new();
        vertices_around_vertex(&mesh, VertexId::new(4), |vertex_index| vertices.push(*vertex_index));

        assert_eq!(vertices, expected_vertices);
    }

    #[test]
    fn vertices_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices = [3, 4, 1].map(|el| VertexId::new(el)).to_vec();
        let mut vertices = Vec::new();
        vertices_around_vertex(&mesh, VertexId::new(0), |vertex_index| vertices.push(*vertex_index));

        assert_eq!(vertices, expected_vertices);
    }

    // Faces iter macro

    #[test]
    fn test_faces_around_internal_vertex() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces = [3, 0, 1, 2].map(|el| FaceId::new(el)).to_vec();
        let mut faces = Vec::new();
        faces_around_vertex(&mesh, VertexId::new(4), |face_index| faces.push(*face_index));

        assert_eq!(faces, expected_faces);
    }

    #[test]
    fn test_faces_around_boundary_vertex() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces = [3, 0].map(|el| FaceId::new(el)).to_vec();
        let mut faces = Vec::new();
        faces_around_vertex(&mesh, VertexId::new(0), |face_index| faces.push(*face_index));

        assert_eq!(faces, expected_faces);
    }
}
