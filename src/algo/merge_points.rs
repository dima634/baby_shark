use nalgebra::{Point3, Scalar};
use num_traits::Float;
use crate::data_structures::vertex_index_map::PointIndexMap;

pub struct MergedVertices<TScalar: Scalar> {
    /// Unique points
    pub points: Vec<Point3<TScalar>>,
    /// Vertex indices
    pub indices: Vec<usize>
}

///
/// Merges exactly coincident points
/// 
pub fn merge_points<TScalar: Scalar + Float>(vertices: &Vec<Point3<TScalar>>) -> MergedVertices<TScalar>
{
    let mut vertex_index_map = PointIndexMap::<TScalar>::with_capacity(vertices.len());

    // Storages for merged vertices and indices
    let mut indices = Vec::with_capacity(vertices.len());
    let mut merged_vertices = Vec::with_capacity(vertices.len());

    for vertex in vertices {
        let index = vertex_index_map.get_index(*vertex);
        if index.is_some() {
            // Insert old vertex
            indices.push(*index.unwrap());
        } else {
            // Insert new vertex and index
            let vert_idx = merged_vertices.len();
            merged_vertices.push(*vertex);
            vertex_index_map.insert(*vertex, vert_idx);
            indices.push(vert_idx);
        }
    }

    return MergedVertices {
        indices,
        points: merged_vertices
    };
}
