use nalgebra::{Scalar, Point};
use crate::{data_structures::vertex_index_map::PointIndexMap, geometry::traits::RealNumber};

pub struct IndexedVertices<const D: usize, TScalar: Scalar> {
    /// Unique points
    pub points: Vec<Point<TScalar, D>>,
    /// Vertex indices
    pub indices: Vec<usize>
}

///
/// Merges exactly coincident points
/// 
pub fn merge_points<const D: usize, TScalar: RealNumber>(vertices: &Vec<Point<TScalar, D>>) -> IndexedVertices<D, TScalar>
{
    let mut vertex_index_map = PointIndexMap::<D, TScalar>::with_capacity(vertices.len());

    // Storages for merged vertices and indices
    let mut indices = Vec::with_capacity(vertices.len());
    let mut merged_vertices = Vec::with_capacity(vertices.len());

    for vertex in vertices {
        let index = vertex_index_map.get_index(*vertex);
        if let Some(index) = index {
            // Insert old vertex
            indices.push(*index);
        } else {
            // Insert new vertex and index
            let vert_idx = merged_vertices.len();
            merged_vertices.push(*vertex);
            vertex_index_map.insert(*vertex, vert_idx);
            indices.push(vert_idx);
        }
    }

    return IndexedVertices {
        indices,
        points: merged_vertices
    };
}
