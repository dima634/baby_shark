use super::{
    corner_table::*,
    traits::TopologicalMesh,
};
use std::collections::HashSet;

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

/// Returns vertices on the boundary of a region (i.e., vertices that have neighbors outside the region).
pub fn region_boundary(mesh: &CornerTableD, vertex_region: &HashSet<VertexId>) -> Vec<VertexId> {
    let mut boundary = Vec::new();

    for vert in vertex_region {
        let mut added = false;
        mesh.vertices_around_vertex(vert, |neighbor| {
            if !added && !vertex_region.contains(&neighbor) {
                boundary.push(*vert);
                added = true;
            }
        });
    }

    boundary
}
