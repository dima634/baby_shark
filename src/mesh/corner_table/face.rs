use super::*;
use crate::{geometry::primitives::triangle3::Triangle3, helpers::aliases::Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceId(u32);

impl FaceId {
    #[inline]
    pub fn corners(&self) -> (CornerId, CornerId, CornerId) {
        let base = self.0 * 3;
        (
            CornerId::new(base as usize),
            CornerId::new((base + 1) as usize),
            CornerId::new((base + 2) as usize),
        )
    }

    /// Returns the first corner of the face.
    #[inline]
    pub fn corner(&self) -> CornerId {
        CornerId::new((self.0 * 3) as usize)
    }

    #[inline]
    pub(super) fn new(index: usize) -> Self {
        Self(index as u32)
    }
}

impl<S: RealNumber> CornerTable<S> {
    /// Returns the number of faces in the mesh.
    #[inline]
    pub fn count_faces(&self) -> usize {
        self.corners.iter().filter(|c| !c.is_deleted()).count() / 3
    }

    #[inline]
    pub fn face_vertices(&self, face: FaceId) -> (VertexId, VertexId, VertexId) {
        let mut walker = CornerWalker::from_corner(self, face.corner());
        (
            walker.corner().vertex(),
            walker.move_to_next().corner().vertex(),
            walker.move_to_next().corner().vertex(),
        )
    }

    #[inline]
    pub fn face_edges(&self, face: FaceId) -> (EdgeId, EdgeId, EdgeId) {
        let (c1, c2, c3) = face.corners();
        (EdgeId::new(c1), EdgeId::new(c2), EdgeId::new(c3))
    }

    /// Returns positions of face vertices in ccw order
    #[inline]
    pub fn face_positions(&self, face: FaceId) -> Triangle3<S> {
        let (v1, v2, v3) = self.face_vertices(face);
        Triangle3::new(
            self[v1].position().clone(),
            self[v2].position().clone(),
            self[v3].position().clone(),
        )
    }

    /// Returns face normal
    #[inline]
    pub fn face_normal(&self, face: FaceId) -> Option<Vec3<S>> {
        let triangle = self.face_positions(face);
        triangle.get_normal()
    }
}
