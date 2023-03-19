use nalgebra::{Point3, Scalar};
use crate::{data_structures::vertex_index_map::PointIndexMap, geometry::traits::RealNumber};

pub struct IndexedVertices<TScalar: Scalar> {
    /// Unique points
    pub points: Vec<Point3<TScalar>>,
    /// Vertex indices
    pub indices: Vec<usize>
}

///
/// Merges exactly coincident points
/// 
pub fn merge_points<TScalar: RealNumber>(vertices: &Vec<Point3<TScalar>>) -> IndexedVertices<TScalar>
{
    let mut vertex_index_map = PointIndexMap::<TScalar>::with_capacity(vertices.len());

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
