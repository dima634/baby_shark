use std::collections::HashSet;
use super::{corner_table::prelude::CornerTableD, traits::TopologicalMesh};

/// Extends the region of a vertices by including all vertices within a given number of rings.
pub fn extend_region(mesh: &CornerTableD, vertex_region: &mut HashSet<usize>, rings: usize) {
    if rings == 0 {
        return;
    }

    // (vertex_id, depth)
    let mut stack = Vec::from_iter(vertex_region.iter().map(|vert| (*vert, 0)));

    while let Some((vertex, depth)) = stack.pop() {
        if depth >= rings {
            continue;
        }

        mesh.vertices_around_vertex(&vertex, |&neighbor| {
            if vertex_region.insert(neighbor) {
                stack.push((neighbor, depth + 1));
            }
        });
    }
}