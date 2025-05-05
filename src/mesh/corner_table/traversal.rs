use super::*;
use crate::mesh::traits::stats::MAX_VERTEX_VALENCE;

/// Iterators over corners, vertices and edges of mesh
impl<S: RealNumber> CornerTable<S> {
    #[inline]
    pub fn faces(&self) -> impl Iterator<Item = FaceId> + '_ {
        self.corners.iter()
            .enumerate()
            .step_by(3)
            .filter(|(_, corner)| !corner.is_deleted())
            .map(|(index, _)| FaceId::new(index / 3))
    }

    #[inline]
    pub fn vertices(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.vertices.iter()
            .enumerate()
            .filter(|(_, vertex)| !vertex.is_deleted())
            .map(|(index, _)| VertexId::new(index))
    }

    #[inline]
    pub fn edges(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.corners.iter()
            .enumerate()
            .filter(|(_, corner)| !corner.is_deleted())
            .map(|(index, _)| EdgeId::new(CornerId::new(index)))
    }

    #[inline]
    pub fn unique_edges(&self) -> impl Iterator<Item = EdgeId> + '_ {
        let mut visited = self.create_edge_attribute();
        let mut edge_iter = self.edges();

        core::iter::from_fn(move || {
            loop {
                let edge = edge_iter.next()?;
                if visited[edge] {
                    continue;
                }

                visited[edge] = true;
                if let Some(opposite) = self.opposite_edge(edge) {
                    visited[opposite] = true;
                }

                return Some(edge);
            }
        })
    }
}

pub enum IncidentEdge {
    Incoming(EdgeId),
    Outgoing(EdgeId),
}

impl IncidentEdge {
    #[inline]
    pub fn id(&self) -> EdgeId {
        match self {
            Self::Incoming(edge_id) => *edge_id,
            Self::Outgoing(edge_id) => *edge_id,
        }
    }

    #[inline]
    pub fn is_outgoing(&self) -> bool {
        matches!(self, Self::Outgoing(_))
    }
}

/// Topological queries
impl<S: RealNumber> CornerTable<S> {
    /// Visits vertices adjacent to the vertex. Order is not guaranteed.
    pub fn vertices_around_vertex<F: FnMut(VertexId)>(&self, vertex_id: VertexId, mut visit: F) {
        let mut walker = CornerWalker::from_vertex(self, vertex_id);
        walker.move_to_previous();
        let started_at = walker.corner_id();
        let mut border_reached = false;

        loop {
            visit(walker.corner().vertex());

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
                visit(walker.corner().vertex());

                walker.move_to_next();

                if walker.corner().opposite_corner().is_none() {
                    break;
                }

                walker.move_to_opposite();
            }
        }
    }

    #[inline]
    pub fn faces_around_vertex<F: FnMut(FaceId)>(&self, vertex_id: VertexId, mut visit: F) {
        let mut walker = CornerWalker::from_vertex(self, vertex_id);
        walker.move_to_previous();
        let started_at = walker.corner_id();
        let mut border_reached = false;

        loop {
            visit(walker.corner_id().face());

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
                visit(walker.corner_id().face());

                walker.move_to_next();

                if walker.corner().opposite_corner().is_none() {
                    break;
                }

                walker.move_to_opposite();
            }
        }
    }

    /// Iterates over incident edges of vertex
    #[inline]
    pub fn edges_around_vertex<F: FnMut(IncidentEdge)>(&self, vertex_id: VertexId, mut visit: F) {
        let mut walker = CornerWalker::from_vertex(self, vertex_id);
        walker.move_to_next();
        let started_at = walker.corner_id();
        let mut border_reached = false;

        loop {
            visit(IncidentEdge::Incoming(EdgeId::new(walker.corner_id())));

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
                visit(IncidentEdge::Outgoing(EdgeId::new(walker.corner_id())));

                if walker.corner().opposite_corner().is_none() {
                    break;
                }

                walker.move_to_opposite().move_to_next();
            }
        }
    }

    /// Iterates over edges "opposite" to the vertex. Order is not guaranteed.
    pub fn vertex_bounding_edges<F: FnMut(EdgeId)>(&self, vertex: VertexId, mut visit: F) {
        let mut walker = CornerWalker::from_vertex(self, vertex);
        walker.move_to_next();
        let started_at = walker.corner_id();
        let mut border_reached = false;

        loop {
            visit(EdgeId::new(walker.corner_id().previous()));

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

            if walker.corner().opposite_corner().is_none() {
                return;
            }

            walker.move_to_opposite();

            loop {
                visit(EdgeId::new(walker.corner_id().previous()));
                walker.move_to_next();

                if walker.corner().opposite_corner().is_none() {
                    break;
                }

                walker.move_to_opposite();
            }
        }
    }

    /// Iterates over corners that are adjacent to given vertex
    pub fn corners_around_vertex<F: FnMut(CornerId)>(
        &self,
        vertex_id: VertexId,
        mut visit: F,
    ) {
        let mut walker = CornerWalker::from_vertex(self, vertex_id);
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
}

// TODO: think about removing this function and using mut iterator instead
pub fn collect_corners_around_vertex<TScalar: RealNumber>(
    corner_table: &CornerTable<TScalar>,
    vertex_id: VertexId,
) -> Vec<CornerId> {
    let mut corners = Vec::with_capacity(MAX_VERTEX_VALENCE);
    corner_table.corners_around_vertex(vertex_id, |corner_id| corners.push(corner_id));

    corners
}

#[cfg(test)]
mod tests {
    use crate::mesh::corner_table::{
        test_helpers::{create_unit_cross_square_mesh, create_unit_square_mesh},
        *,
    };

    #[test]
    fn edges_iterator() {
        let mesh = create_unit_square_mesh();
        let expected_edges: Vec<EdgeId> = vec![
            EdgeId::new(CornerId::new(0)),
            EdgeId::new(CornerId::new(1)),
            EdgeId::new(CornerId::new(2)),
            EdgeId::new(CornerId::new(3)),
            EdgeId::new(CornerId::new(4)),
            EdgeId::new(CornerId::new(5)),
        ];
        let actual_edges = mesh.edges().collect::<Vec<_>>();

        assert_eq!(expected_edges, actual_edges);
        assert_eq!(mesh.unique_edges().count(), 5);
    }

    // Corners iter macro

    #[test]
    fn corners_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners = [11, 2, 5, 8].map(|el| CornerId::new(el)).to_vec();
        let mut corners = Vec::new();

        mesh.corners_around_vertex(VertexId::new(4), |corner_index| corners.push(corner_index));

        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners = [10, 0].map(|el| CornerId::new(el)).to_vec();
        let mut corners = Vec::new();

        mesh.corners_around_vertex(VertexId::new(0), |corner_index| corners.push(corner_index));

        assert_eq!(corners, expected_corners);
    }

    // Vertices iter macro

    #[test]
    fn vertices_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices = [0, 1, 2, 3].map(|el| VertexId::new(el)).to_vec();
        let mut vertices = Vec::new();
        mesh.vertices_around_vertex(VertexId::new(4), |vertex_index| {
            vertices.push(vertex_index)
        });

        assert_eq!(vertices, expected_vertices);
    }

    #[test]
    fn vertices_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices = [3, 4, 1].map(|el| VertexId::new(el)).to_vec();
        let mut vertices = Vec::new();
        mesh.vertices_around_vertex(VertexId::new(0), |vertex_index| {
            vertices.push(vertex_index)
        });

        assert_eq!(vertices, expected_vertices);
    }

    #[test]
    fn test_faces_around_internal_vertex() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces = [3, 0, 1, 2].map(|el| FaceId::new(el)).to_vec();
        let mut faces = Vec::new();
        mesh.faces_around_vertex(VertexId::new(4), |face_index| faces.push(face_index));

        assert_eq!(faces, expected_faces);
    }

    #[test]
    fn test_faces_around_boundary_vertex() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces = [3, 0].map(|el| FaceId::new(el)).to_vec();
        let mut faces = Vec::new();
        mesh.faces_around_vertex(VertexId::new(0), |face_index| faces.push(face_index));

        assert_eq!(faces, expected_faces);
    }
}
