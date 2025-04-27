use super::Mesh;
use std::collections::HashSet;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl Mesh {
    /// Extends region by the given number of neighboring vertices rings.
    /// # Arguments
    /// * `vertex_region` - A list of vertex indices to extend the region from.
    /// * `rings` - The number of rings to extend the region by.
    pub fn extend_region(&self, vertex_region: &[usize], rings: usize) -> Vec<usize> {
        let mut region =
            HashSet::from_iter(self.vertex_indices_to_ids(vertex_region.into_iter().copied()));
        crate::extend_region(&self.0, &mut region, rings);
        self.vertex_ids_to_indices(region.into_iter())
    }

    /// Returns vertices on the boundary of a region (i.e., vertices that have neighbors outside the region).
    pub fn region_boundary(&self, vertex_region: &[usize]) -> Vec<usize> {
        let region =
            HashSet::from_iter(self.vertex_indices_to_ids(vertex_region.into_iter().copied()));
        self.vertex_ids_to_indices(crate::region_boundary(&self.0, &region).into_iter())
    }
}
