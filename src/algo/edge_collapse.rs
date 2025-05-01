use crate::{
    geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
    helpers::aliases::Vec3,
    mesh::corner_table::*,
};
use num_traits::cast;
use std::collections::BTreeSet;

/// Returns `true` when edge collapse is topologically and geometrically safe, `false` otherwise
#[inline]
pub fn is_safe<S: RealNumber>(
    mesh: &CornerTable<S>,
    edge: EdgeId,
    collapse_at: &Vec3<S>,
    min_quality: S,
) -> bool {
    is_topologically_safe(mesh, edge) && is_geometrically_safe(mesh, edge, collapse_at, min_quality)
}

/// Returns `true` when edge collapse is topologically safe, `false` otherwise
pub fn is_topologically_safe<S: RealNumber>(mesh: &CornerTable<S>, edge: EdgeId) -> bool {
    // Was collapsed?
    if !mesh.edge_exists(edge) {
        return false;
    }

    // Count common vertices of edge vertices
    let (e_start, e_end) = mesh.edge_vertices(edge);
    let mut e_start_neighbors = BTreeSet::new(); // TODO: nested loop
    mesh.vertices_around_vertex(e_start, |vertex| {
        e_start_neighbors.insert(vertex);
    });
    let mut common_neighbors_count = 0;
    mesh.vertices_around_vertex(e_end, |vertex| {
        if e_start_neighbors.contains(&vertex) {
            common_neighbors_count += 1;
        }
    });

    // Is topologically safe?
    common_neighbors_count == 2
}

///
/// Returns `true` when edge collapse is geometrically safe, `false` otherwise.
/// Collapse is not safe when face normals are flipped or quality faces becomes too bad.
///
pub fn is_geometrically_safe<S: RealNumber>(
    mesh: &CornerTable<S>,
    edge: EdgeId,
    new_position: &Vec3<S>,
    min_quality: S,
) -> bool {
    // Check new normals (geometrical safety)
    let (e_start, e_end) = mesh.edge_vertices(edge);
    let (f1, f2) = mesh.edge_faces(edge);

    check_faces_after_collapse(mesh, f1, f2, e_start, new_position, min_quality)
        && check_faces_after_collapse(mesh, f1, f2, e_end, new_position, min_quality)
}

fn check_faces_after_collapse<S: RealNumber>(
    mesh: &CornerTable<S>,
    removed_face1: FaceId,
    removed_face2: Option<FaceId>,
    collapsed_vertex: VertexId,
    new_position: &Vec3<S>,
    min_quality_perc: S,
) -> bool {
    let mut bad_collapse = false;

    mesh.faces_around_vertex(collapsed_vertex, |face| {
        if face == removed_face1 || Some(face) == removed_face2 {
            return;
        }

        // TODO: iterate over neighboring vertices to avoid match statement
        let (v1, v2, v3) = match mesh.face_vertices(face) {
            (v1, v2, v3) if v1 == collapsed_vertex => (v1, v2, v3),
            (v1, v2, v3) if v2 == collapsed_vertex => (v2, v3, v1),
            (v1, v2, v3) if v3 == collapsed_vertex => (v3, v1, v2),
            _ => unreachable!("we are iteration over faces around collapse vertex so such face must contain it"),
        };
        let (v1, v2, v3) = (
            mesh[v1].position(),
            mesh[v2].position(),
            mesh[v3].position(),
        );

        let new_quality = Triangle3::quality(new_position, v2, v3);

        if new_quality <= S::epsilon() {
            bad_collapse = true;
            return;
        }

        let old_quality = Triangle3::quality(v1, v2, v3);

        // Quality become too bad?
        if new_quality < old_quality * min_quality_perc {
            bad_collapse = true;
            return;
        }

        let Some(old_normal) = Triangle3::normal(v1, v2, v3) else {
            bad_collapse = true;
            return;
        };
        let Some(new_normal) = Triangle3::normal(new_position, v2, v3) else {
            bad_collapse = true;
            return;
        };

        // Normal flipped?
        if old_normal.dot(&new_normal) < cast(0.7).unwrap() {
            bad_collapse = true;
        }
    });

    !bad_collapse
}

pub fn will_collapse_affect_boundary<S: RealNumber>(mesh: &CornerTable<S>, edge: EdgeId) -> bool {
    let mut boundary_affected = mesh.is_edge_on_boundary(edge);
    if boundary_affected {
        return true;
    }

    // TODO: allow collapse towards boundary

    let (v1, v2) = mesh.edge_vertices(edge);

    for vertex in [v1, v2] {
        mesh.edges_around_vertex(vertex, |edge| {
            if mesh.is_edge_on_boundary(edge) {
                boundary_affected = true;
            }
        });

        if boundary_affected {
            return true;
        }
    }

    false
}
