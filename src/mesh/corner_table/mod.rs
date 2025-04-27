pub mod create;
pub mod traversal;

mod edit;
mod edge;
mod property_maps;
mod vertex;
mod corner;
mod face;
mod attribute;
mod boundary;

#[cfg(test)]
mod test_helpers;

pub use corner::CornerId;
pub use vertex::VertexId;
pub use face::FaceId;
pub use edge::EdgeId;
pub use attribute::EdgeAttribute;
pub use boundary::BoundaryRing;

use corner::*;
use vertex::*;
use crate::geometry::traits::RealNumber;

#[derive(Debug, Clone)]
pub struct CornerTable<TScalar: RealNumber> {
    vertices: Vec<Vertex<TScalar>>,
    corners: Vec<Corner>,
}

pub type CornerTableF = CornerTable<f32>;
pub type CornerTableD = CornerTable<f64>;
