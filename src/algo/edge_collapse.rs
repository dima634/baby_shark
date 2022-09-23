use std::collections::BTreeSet;

use nalgebra::Point3;
use num_traits::cast;

use crate::{mesh::traits::{TopologicalMesh, EditableMesh, Position}, geometry::primitives::Triangle3};

/// Returns `true` when edge collapse is topologically safe, `false` otherwise
pub fn is_topologically_safe<TMesh: TopologicalMesh + EditableMesh>(mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
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

/// Returns `true` when edge collapse is geometrically safe (normals of new faces are not flipped), `false` otherwise
pub fn is_geometrically_safe<TMesh: TopologicalMesh + EditableMesh>(mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
    // Check new normals (geometrical safety)
    let (e_start, e_end) = mesh.edge_vertices(edge);
    let new_position = (mesh.vertex_position(&e_start) + mesh.vertex_position(&e_end).coords) * cast(0.5).unwrap();
    return check_faces_normals_after_collapse(mesh, &e_start, &new_position) && 
           check_faces_normals_after_collapse(mesh, &e_end, &new_position);
}

/// Returns `true` when edge collapse is topologically and geometrically safe, `false` otherwise
pub fn is_safe<TMesh: TopologicalMesh + EditableMesh>(mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {        
    if !is_topologically_safe(mesh, &edge) {
        return false;
    }

    return is_geometrically_safe(mesh, &edge);
}

fn check_faces_normals_after_collapse<TMesh: TopologicalMesh + EditableMesh>(
    mesh: &TMesh, 
    collapsed_vertex: &TMesh::VertexDescriptor, 
    new_position: &Point3<TMesh::ScalarType>
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
        if new_quality < old_quality * cast(0.5).unwrap() {
            bad_collapse = true;
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