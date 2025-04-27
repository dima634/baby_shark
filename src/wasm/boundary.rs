use super::Mesh;
use crate::mesh::{traits::Mesh as MeshTrait, *};
use std::ops::ControlFlow;
use wasm_bindgen::prelude::*;

/// A ring of boundary edges in a mesh.
#[wasm_bindgen]
#[derive(Debug, Clone, Copy)]
pub struct BoundaryRing(corner_table::BoundaryRing);

/// List of boundary rings in a mesh.
#[wasm_bindgen]
#[derive(Debug)]
pub struct BoundaryRings(Vec<BoundaryRing>);

#[wasm_bindgen]
impl BoundaryRings {
    /// Returns the number of boundary rings in the list
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Get the boundary ring at the given index
    pub fn get(&self, index: usize) -> Option<BoundaryRing> {
        self.0.get(index).copied()
    }
}

#[wasm_bindgen]
impl Mesh {
    /// Returns list of boundary rings in the mesh
    pub fn boundaries(&self) -> BoundaryRings {
        let rings = self
            .inner()
            .boundary_rings()
            .into_iter()
            .map(BoundaryRing)
            .collect();
        BoundaryRings(rings)
    }

    /// Returns vertices of the boundary `ring`
    pub fn boundary_ring_vertices(&self, ring: BoundaryRing) -> Vec<usize> {
        let mut vertices = Vec::new();

        self.inner().boundary_edges(ring.0, |edge| {
            let (v1, _) = self.inner().edge_vertices(&edge);
            vertices.push(v1);
            ControlFlow::Continue(())
        });

        self.vertex_ids_to_indices(vertices.into_iter())
    }
}
