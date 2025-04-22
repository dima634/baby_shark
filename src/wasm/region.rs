use super::Mesh;
use std::collections::HashSet;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl Mesh {
    pub fn extend_region(&self, vertex_region: &[usize], rings: usize) -> Vec<usize> {
        let mut region =
            HashSet::from_iter(self.vertex_indices_to_ids(vertex_region.into_iter().copied()));
        crate::extend_region(&self.0, &mut region, rings);
        self.vertex_ids_to_indices(region.into_iter())
    }

    pub fn region_boundary(&self, vertex_region: &[usize]) -> Vec<usize> {
        let region =
            HashSet::from_iter(self.vertex_indices_to_ids(vertex_region.into_iter().copied()));
        self.vertex_ids_to_indices(crate::region_boundary(&self.0, &region).into_iter())
    }
}
