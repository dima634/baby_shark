mod boundary;
mod deform;
mod region;

use crate::{
    algo::merge_points::merge_points,
    helpers::aliases::Vec3,
    mesh::{
        corner_table::*,
        traits::{Mesh as MeshTrait, VertexProperties},
    },
};
use wasm_bindgen::prelude::*;

/// Triangular mesh
#[wasm_bindgen]
pub struct Mesh(CornerTableD);

impl Mesh {
    #[inline]
    fn inner(&self) -> &CornerTableD {
        &self.0
    }

    #[inline]
    fn inner_mut(&mut self) -> &mut CornerTableD {
        &mut self.0
    }
}

#[wasm_bindgen]
impl Mesh {
    /// Creates a new mesh from a polygon soup.
    pub fn from_vertices(position: &[f64]) -> Mesh {
        let vertices = vec_of_floats_to_vec3s(position);
        let indexed = merge_points(&vertices);
        Self(CornerTableD::from_vertex_and_face_slices(
            &indexed.points,
            &indexed.indices,
        ))
    }

    /// Creates a new mesh from list of vertex positions and face indices (each triangle is defined by 3 indices of vertices).
    pub fn from_vertices_and_faces(position: &[f64], faces: &[usize]) -> Mesh {
        let vertices = vec_of_floats_to_vec3s(position);
        Self(CornerTableD::from_vertex_and_face_slices(&vertices, &faces))
    }

    /// Returns array of vertex positions. Each vertex is represented by 3 consecutive floats (x, y, z).
    pub fn position(&self) -> Vec<f64> {
        self.0
            .vertices()
            .map(|v| self.0.vertex_position(&v).clone())
            .flat_map(|v| [v.x, v.y, v.z])
            .collect()
    }

    /// Returns array of face indices.
    pub fn index(&self) -> Vec<usize> {
        let vertex_to_idx = self.vertex_to_index();
        let mut indices = Vec::new();

        for face in self.0.faces() {
            let (v1, v2, v3) = self.0.face_vertices(&face);
            indices.push(vertex_to_idx[v1]);
            indices.push(vertex_to_idx[v2]);
            indices.push(vertex_to_idx[v3]);
        }

        indices
    }

    /// Converts vertex indices to vertex ids.
    fn vertex_indices_to_ids(
        &self,
        vertex_indices: impl ExactSizeIterator<Item = usize>,
    ) -> Vec<VertexId> {
        let index_to_vertex = self.index_to_vertex();
        let mut vertex_ids = Vec::with_capacity(vertex_indices.len());

        for idx in vertex_indices {
            vertex_ids.push(index_to_vertex[idx]);
        }

        vertex_ids
    }

    /// Converts vertex ids to vertex indices.
    fn vertex_ids_to_indices(
        &self,
        vertex_ids: impl ExactSizeIterator<Item = VertexId>,
    ) -> Vec<usize> {
        let vertex_to_idx = self.vertex_to_index();
        let mut vertex_indices = Vec::with_capacity(vertex_ids.len());

        for id in vertex_ids {
            vertex_indices.push(vertex_to_idx[id]);
        }

        vertex_indices
    }

    fn vertex_to_index(&self) -> <CornerTableD as VertexProperties>::VertexPropertyMap<usize> {
        let mut vertex_to_idx = self.0.create_vertex_properties_map();

        for (idx, vert) in self.0.vertices().enumerate() {
            vertex_to_idx[vert] = idx;
        }

        vertex_to_idx
    }

    fn index_to_vertex(&self) -> Vec<VertexId> {
        let mut index_to_vertex = Vec::new();

        for vert in self.0.vertices() {
            index_to_vertex.push(vert);
        }

        index_to_vertex
    }
}

fn vec_of_floats_to_vec3s(floats: &[f64]) -> Vec<Vec3<f64>> {
    floats
        .chunks(3)
        .map(|chunk| Vec3::new(chunk[0], chunk[1], chunk[2]))
        .collect()
}
