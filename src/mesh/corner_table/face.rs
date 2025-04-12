use super::corner::CornerId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceId(usize);

impl FaceId {
    #[inline]
    pub fn corners(&self) -> (CornerId, CornerId, CornerId) {
        let base = self.0 * 3;
        (
            CornerId::new(base),
            CornerId::new(base + 1),
            CornerId::new(base + 2),
        )
    }

    /// Returns the first corner of the face.
    #[inline]
    pub fn corner(&self) -> CornerId {
        CornerId::new(self.0 * 3)
    }

    #[inline]
    pub(super) fn new(index: usize) -> Self {
        Self(index)
    }
}
