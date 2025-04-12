use std::collections::HashSet;
use super::{corner_table::{prelude::CornerTableD, VertexId}, traits::TopologicalMesh};

/// Extends the region of a vertices by including all vertices within a given number of rings.
pub fn extend_region(mesh: &CornerTableD, vertex_region: &mut HashSet<VertexId>, rings: usize) {
    if rings == 0 {
        return;
    }

    // (vertex_id, depth)
    let mut current_ring = Vec::from_iter(vertex_region.iter().map(|vert| (*vert, 0)));
    let mut next_ring = Vec::new();

    loop {
        while let Some((vertex, depth)) = current_ring.pop() {
            if depth >= rings {
                continue;
            }

            mesh.vertices_around_vertex(&vertex, |&neighbor| {
                if vertex_region.insert(neighbor) {
                    next_ring.push((neighbor, depth + 1));
                }
            });
        }

        if next_ring.is_empty() {
            break;
        }

        std::mem::swap(&mut current_ring, &mut next_ring);
    }
}