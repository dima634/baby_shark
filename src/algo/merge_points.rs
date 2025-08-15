use crate::{data_structures::vertex_index_map::PointIndexMap, geometry::traits::RealNumber};
use nalgebra::{SVector, Scalar};

pub struct IndexedVertices<const D: usize, S: Scalar> {
    /// Unique points
    pub points: Vec<SVector<S, D>>,
    /// Vertex indices
    pub indices: Vec<usize>,
}

/// Merges exactly coincident points
pub fn merge_points<const D: usize, S: RealNumber, V: Into<[S; D]>>(
    points: impl Iterator<Item = V>,
) -> IndexedVertices<D, S> {
    let num_points = points.size_hint().1.unwrap_or(0);
    let num_unique_vertices = num_points / 3; // Just a guess
    let mut vertex_index_map = PointIndexMap::<D, S>::with_capacity(num_unique_vertices);

    // Storages for merged vertices and indices
    let mut indices = Vec::with_capacity(num_points);
    let mut merged_vertices = Vec::with_capacity(num_unique_vertices);

    for vertex in points {
        let vertex : SVector<S,D> = vertex.into().into();
        let index = *vertex_index_map.entry(vertex).or_insert_with(|| {
            let vert_idx = merged_vertices.len();
            merged_vertices.push(vertex);
            vert_idx
        });
        indices.push(index);
    }

    IndexedVertices {
        indices,
        points: merged_vertices,
    }
}
