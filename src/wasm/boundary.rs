use std::ops::ControlFlow;
use wasm_bindgen::prelude::*;
use crate::mesh::{traits::Mesh as MeshTrait, *};
use super::Mesh;

#[wasm_bindgen]
#[derive(Debug, Clone, Copy)]
pub struct BoundaryRing(corner_table::BoundaryRing);

#[wasm_bindgen]
#[derive(Debug)]
pub struct BoundaryRings(Vec<BoundaryRing>);

#[wasm_bindgen]
impl BoundaryRings {
    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn get(&self, index: usize) -> Option<BoundaryRing> {
        self.0.get(index).copied()
    }
}

#[wasm_bindgen]
impl Mesh {
    pub fn boundaries(&self) -> BoundaryRings {
        let rings = self.inner()
            .boundary_rings()
            .into_iter()
            .map(BoundaryRing)
            .collect();
        BoundaryRings(rings)
    }

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