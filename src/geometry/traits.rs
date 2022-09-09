use crate::mesh::traits::Floating;
use super::primitives::Box3;


/// 3D bounding box
pub trait HasBBox3 {
    type ScalarType: Floating;
    fn bbox(&self) -> Box3<Self::ScalarType>;
}
