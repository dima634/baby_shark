use std::collections::HashSet;
use super::mesh::*;
use wasm_bindgen::prelude::*;
use nalgebra as na;

#[wasm_bindgen]
pub struct PreparedDeform(crate::PreparedDeform);

#[wasm_bindgen]
impl PreparedDeform {
    pub fn new(
        mesh: &mut Mesh,
        handle: &[usize],
        region_of_interest: &[usize],
    ) -> Option<PreparedDeform> {
        let handle = HashSet::from_iter(handle.iter().copied());
        let region_of_interest = HashSet::from_iter(region_of_interest.iter().copied());
        let prep_deform =
            crate::prepare_deform(mesh.inner_mut(), &handle, &region_of_interest).ok()?;
        Some(PreparedDeform(prep_deform))
    }

    pub fn deform(&self, mesh: &mut Mesh, transform: &[f64]) {
        assert!(transform.len() == 16);
        let transform = na::Matrix4::from_column_slice(transform);
        self.0.deform(mesh.inner_mut(), transform);
    }
}
