use super::Mesh;
use nalgebra as na;
use std::collections::{HashMap, HashSet};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PreparedDeform {
    inner: crate::PreparedDeform<f64>,
    handle: Vec<usize>,
}

#[wasm_bindgen]
impl PreparedDeform {
    pub fn new(
        mesh: &mut Mesh,
        handle_region: &[usize],
        region_of_interest: &[usize],
    ) -> Result<PreparedDeform, String> {
        let handle =
            HashSet::from_iter(mesh.vertex_indices_to_ids(handle_region.into_iter().copied()));
        let region_of_interest =
            HashSet::from_iter(mesh.vertex_indices_to_ids(region_of_interest.into_iter().copied()));

        crate::prepare_deform(mesh.inner_mut(), &handle, &region_of_interest)
            .map(|prepared| PreparedDeform {
                inner: prepared,
                handle: handle_region.to_vec(),
            })
            .map_err(|err| err.to_string())
    }

    pub fn set_max_iters(&mut self, max_iters: usize) {
        self.inner.set_max_iters(max_iters);
    }

    pub fn deform(&self, mesh: &mut Mesh, target: &[f64]) -> Result<Mesh, String> {
        if target.len() != 3 * self.handle.len() {
            return Err("Target positions do not match handle size".to_string());
        }

        let handle_ids = mesh.vertex_indices_to_ids(self.handle.iter().copied());
        let target = HashMap::from_iter(
            handle_ids.into_iter().zip(
                target
                    .chunks_exact(3)
                    .map(|chunk| na::Vector3::new(chunk[0], chunk[1], chunk[2])),
            ),
        );

        let result = self
            .inner
            .deform(mesh.inner_mut(), &target)
            .map(|mesh| Mesh(mesh))
            .map_err(|err| err.to_string());

        result
    }
}
