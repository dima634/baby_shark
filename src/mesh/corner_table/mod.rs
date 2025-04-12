pub use corner::CornerId;
pub use vertex::VertexId;
pub use face::FaceId;
pub use edge::EdgeId;

use corner::*;
use vertex::*;

use crate::geometry::traits::RealNumber;

pub mod create;
pub mod prelude;
pub mod traversal;

mod marker;
mod edit;
mod edge;
mod property_maps;
mod vertex;
mod corner;
mod face;
mod traits;
mod flags;

#[cfg(test)]
mod test_helpers;

pub struct CornerTable<TScalar: RealNumber> {
    vertices: Vec<Vertex<TScalar>>,
    corners: Vec<Corner>,
}
