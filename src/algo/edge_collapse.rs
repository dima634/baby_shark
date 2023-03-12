use std::collections::BTreeSet;

use nalgebra::Point3;
use num_traits::cast;

use crate::{mesh::traits::{TopologicalMesh, EditableMesh, Position}, geometry::primitives::Triangle3};

/// Returns `true` when edge collapse is topologically safe, `false` otherwise
pub fn is_topologically_safe<TMesh: TopologicalMesh + EditableMesh>(mesh: &TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
    // Was collapsed?
    if !mesh.edge_exist(edge) {
        return false;
    }

    // Count common vertices of edge vertices
    let (e_start, e_end) = mesh.edge_vertices(edge);
    let mut e_start_neighbors = BTreeSet::new();
    mesh.vertices_around_vertex(&e_start, |vertex| { e_start_neighbors.insert(*vertex); });
    let mut common_neighbors_count = 0;
    mesh.vertices_around_vertex(&e_end, |vertex|
    {
        if e_start_neighbors.contains(vertex) {
            common_neighbors_count += 1;
        }
    });

    // Is topologically safe?
    return common_neighbors_count == 2 
}

///
/// Returns `true` when edge collapse is geometrically safe, `false` otherwise.
/// Collapse is not safe when face normals are flipped or quality faces becomes too bad.
/// 
pub fn is_geometrically_safe<TMesh: TopologicalMesh + EditableMesh>(
    mesh: &TMesh, 
    edge: &TMesh::EdgeDescriptor, 
    new_position: &Point3<TMesh::ScalarType>,
    min_quality: TMesh::ScalarType
) -> bool {
    // Check new normals (geometrical safety)
    let (e_start, e_end) = mesh.edge_vertices(edge);
    return check_faces_after_collapse(mesh, &e_start, new_position, min_quality) && 
           check_faces_after_collapse(mesh, &e_end, new_position, min_quality);
}

/// Returns `true` when edge collapse is topologically and geometrically safe, `false` otherwise
#[inline]
pub fn is_safe<TMesh: TopologicalMesh + EditableMesh>(
    mesh: &TMesh, 
    edge: &TMesh::EdgeDescriptor, 
    collapse_at: &Point3<TMesh::ScalarType>,
    min_quality: TMesh::ScalarType
) -> bool {
    return is_topologically_safe(mesh, &edge) && is_geometrically_safe(mesh, &edge, collapse_at, min_quality);
}

fn check_faces_after_collapse<TMesh: TopologicalMesh + EditableMesh>(
    mesh: &TMesh, 
    collapsed_vertex: &TMesh::VertexDescriptor, 
    new_position: &Point3<TMesh::ScalarType>,
    min_quality: TMesh::ScalarType
) -> bool {
    let mut bad_collapse = false;

    mesh.faces_around_vertex(&collapsed_vertex, |face| {
        let mut pos = TMesh::Position::from_vertex_on_face(mesh, &face, &collapsed_vertex);

        let v1 = mesh.vertex_position(&pos.get_vertex());
        let v2 = mesh.vertex_position(&pos.next().get_vertex());
        let v3 = mesh.vertex_position(&pos.next().get_vertex());

        let old_quality = Triangle3::quality(v1, v2, v3);
        let new_quality = Triangle3::quality(new_position, v2, v3);

        // Quality become too bad?
        if new_quality < old_quality * min_quality {
            bad_collapse = true;
            return;
        }

        let old_normal = Triangle3::normal(v1, v2, v3);
        let new_normal = Triangle3::normal(new_position, v2, v3);

        // Normal flipped?
        if old_normal.dot(&new_normal) < cast(0.7).unwrap() {
            bad_collapse = true;
        }
    });

    return !bad_collapse;
}