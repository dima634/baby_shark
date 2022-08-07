use super::{corner_table::CornerTable, connectivity::{corner::DefaultCorner, vertex::{VertexF, VertexD}}};

pub type CornerTableF = CornerTable<DefaultCorner, VertexF>;
pub type CornerTableD = CornerTable<DefaultCorner, VertexD>;
