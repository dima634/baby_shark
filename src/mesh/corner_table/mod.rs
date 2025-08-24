mod attribute;
mod boundary;
mod builder;
mod corner;
mod edge;
mod edit;
mod face;
mod traversal;
mod vertex;
mod walker;

#[cfg(test)]
mod test_helpers;

pub use attribute::{EdgeAttribute, VertexAttribute};
pub use boundary::BoundaryRing;
pub use corner::CornerId;
pub use edge::EdgeId;
pub use face::FaceId;
pub use traversal::IncidentEdge;
pub use vertex::VertexId;
pub use walker::CornerWalker;

pub type CornerTableF = CornerTable<f32>;
pub type CornerTableD = CornerTable<f64>;

use super::traits::{TriangleMesh, Triangles};
use crate::geometry::{primitives::triangle3::Triangle3, traits::RealNumber};
use corner::*;
use vertex::*;

#[derive(Debug, Clone)]
pub struct CornerTable<S: RealNumber> {
    vertices: Vec<Vertex<S>>,
    corners: Vec<Corner>,
}

impl<S: RealNumber> Triangles for CornerTable<S> {
    type Scalar = S;

    fn triangles(&self) -> impl Iterator<Item = Triangle3<Self::Scalar>> {
        self.faces().map(|face| self.face_positions(face))
    }
}

impl<R: RealNumber> TriangleMesh for CornerTable<R> {
    type Scalar = R;
    type VertexId = VertexId;

    #[inline]
    fn position(&self, vertex: Self::VertexId) -> [Self::Scalar; 3] {
        self[vertex].position().clone().into()
    }

    #[inline]
    fn vertices(&self) -> impl Iterator<Item = Self::VertexId> {
        self.vertices()
    }

    fn faces(&self) -> impl Iterator<Item = [Self::VertexId; 3]> {
        self.faces().map(|face| self.face_vertices(face).into())
    }
}

impl<S: RealNumber> CornerTable<S> {
    /// Checks if the topology of the mesh is valid:
    /// * Each corner has a valid opposite corner.
    /// * Each vertex has a valid corner.
    /// * Each corner has a valid vertex.
    /// Useful for debugging.
    #[allow(dead_code)]
    fn validate_topology(&self) -> bool {
        for corner_idx in 0..self.corners.len() {
            let corner = &self.corners[corner_idx];
            let corner_id = CornerId::new(corner_idx as u32);

            if corner.is_deleted() {
                continue;
            }

            if !self[corner.vertex()].is_deleted() {
                return false;
            }

            if let Some(opp_corner_id) = corner.opposite_corner() {
                let opposite = &self[opp_corner_id];
                let valid = opposite.vertex() != corner.vertex()
                    && opp_corner_id != corner_id
                    && !opposite.is_deleted()
                    && opposite.opposite_corner() == Some(corner_id);

                if !valid {
                    return false;
                }
            }
        }

        for vertex_idx in 0..self.vertices.len() {
            let vertex = &self.vertices[vertex_idx];

            if vertex.is_deleted() {
                continue;
            }

            let corner = &self[vertex.corner()];
            let valid = !corner.is_deleted() && corner.vertex() == VertexId::new(vertex_idx as u32);

            if !valid {
                return false;
            }
        }

        true
    }
}
