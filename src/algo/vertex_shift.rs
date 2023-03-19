use nalgebra::Point3;

use crate::{
    mesh::traits::{TopologicalMesh, Position}, 
    geometry::primitives::triangle3::Triangle3
};

pub fn is_vertex_shift_safe<TMesh: TopologicalMesh>(
    vertex: &TMesh::VertexDescriptor, 
    new_position: &Point3<TMesh::ScalarType>, 
    mesh: &TMesh
) -> bool {
    let mut is_safe = true;

    mesh.faces_around_vertex(vertex, |face| {
        let mut pos = TMesh::Position::from_vertex_on_face(mesh, face, vertex);

        let v1 = pos.next().get_vertex();
        let v1_pos = mesh.vertex_position(&v1);
        
        let v2 = pos.next().get_vertex();
        let v2_pos = mesh.vertex_position(&v2);

        is_safe &= !Triangle3::is_degenerate(new_position, &v1_pos, &v2_pos);
    });

    return is_safe;
}
