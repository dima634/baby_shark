use crate::{data_structures::vertex_index_map::PointIndexMap, geometry::traits::RealNumber};
use nalgebra::{SVector, Scalar};

pub struct IndexedVertices<const D: usize, S: Scalar> {
    /// Unique points
    pub points: Vec<SVector<S, D>>,
    /// Vertex indices
    pub indices: Vec<usize>,
}

/// Merges exactly coincident points
pub fn merge_points<const D: usize, S: RealNumber>(
    points: impl Iterator<Item = SVector<S, D>>,
) -> IndexedVertices<D, S> {
    let num_points = points.size_hint().1.unwrap_or(0);
    let num_unique_vertices = num_points / 3; // Just a guess
    let mut vertex_index_map = PointIndexMap::<D, S>::with_capacity(num_unique_vertices);

    // Storages for merged vertices and indices
    let mut indices = Vec::with_capacity(num_points);
    let mut merged_vertices = Vec::with_capacity(num_unique_vertices);

    for vertex in points {
        let index = vertex_index_map.get_index(vertex);
        if let Some(index) = index {
            // Insert old vertex
            indices.push(*index);
        } else {
            // Insert new vertex and index
            let vert_idx = merged_vertices.len();
            merged_vertices.push(vertex);
            vertex_index_map.insert(vertex, vert_idx);
            indices.push(vert_idx);
        }
    }

    IndexedVertices {
        indices,
        points: merged_vertices,
    }
}
