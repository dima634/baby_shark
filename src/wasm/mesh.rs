use crate::{
    helpers::aliases::Vec3,
    mesh::{corner_table::prelude::CornerTableD, traits::{Mesh as MeshTrait, VertexProperties}},
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Mesh(CornerTableD);

impl Mesh {
    #[inline]
    pub fn inner(&self) -> &CornerTableD {
        &self.0
    }

    #[inline]
    pub fn inner_mut(&mut self) -> &mut CornerTableD {
        &mut self.0
    }
}

#[wasm_bindgen]
impl Mesh {
    pub fn from_vertices(position: &[f64]) -> Mesh {
        let vertices = vec_of_floats_to_vec3s(position);
        Self(CornerTableD::from_vertices(&vertices))
    }

    pub fn from_vertices_and_faces(position: &[f64], faces: &[usize]) -> Mesh {
        let vertices = vec_of_floats_to_vec3s(position);
        Self(CornerTableD::from_vertices_and_indices(&vertices, &faces))
    }

    pub fn vertex_ids(&self) -> Vec<usize> {
        self.0.vertices().collect()
    }

    pub fn position(&self) -> Vec<f64> {
        self.0.vertices()
            .map(|v| self.0.vertex_position(&v).clone())
            .flat_map(|v| [v.x, v.y, v.z])
            .collect()
    }

    pub fn index(&self) -> Vec<usize> {
        let mut vertex_to_idx = self.0.create_vertex_properties_map();

        for (idx, vert) in self.0.vertices().enumerate() {
            vertex_to_idx[vert] = idx;
        }

        let mut indices = Vec::new();

        for face in self.0.faces() {
            let (v1, v2, v3) = self.0.face_vertices(&face);
            indices.push(vertex_to_idx[v1]);
            indices.push(vertex_to_idx[v2]);
            indices.push(vertex_to_idx[v3]);
        }

        indices
    }

    pub fn extend_region(&self, vertex_region: &[usize], rings: usize) -> Vec<usize> {
        let mut region = vertex_region.iter().copied().collect();
        crate::extend_region(&self.0, &mut region, rings);
        region.into_iter().collect()
    }

    pub fn vertex_indices(&self) -> Vec<usize> {
        let vertices_count = self.0.vertices().count();
        let mut indices = Vec::new();
        indices.resize(vertices_count, usize::MAX);

        for (idx, vert) in self.0.vertices().enumerate() {
            indices[vert] = idx;
        }

        indices
    }
}

fn vec_of_floats_to_vec3s(floats: &[f64]) -> Vec<Vec3<f64>> {
    floats.chunks(3)
        .map(|chunk| Vec3::new(chunk[0], chunk[1], chunk[2]))
        .collect()
}
