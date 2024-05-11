use num_traits::cast;

use crate::{
    mesh::traits::{TopologicalMesh, Position}, 
    geometry::primitives::triangle3::Triangle3, helpers::aliases::Vec3
};

pub fn is_vertex_shift_safe<TMesh: TopologicalMesh>(
    vertex: &TMesh::VertexDescriptor, 
    old_position: &Vec3<TMesh::ScalarType>, 
    new_position: &Vec3<TMesh::ScalarType>, 
    target_edge_length_squared: TMesh::ScalarType,
    mesh: &TMesh
) -> bool {
    if (old_position - new_position).norm_squared() > target_edge_length_squared {
        return false;
    }

    let mut damages_quality = false;

    mesh.faces_around_vertex(vertex, |face| {
        let mut pos = TMesh::Position::from_vertex_on_face(mesh, face, vertex);

        let v1 = pos.next().get_vertex();
        let v1_pos = mesh.vertex_position(&v1);
        
        let v2 = pos.next().get_vertex();
        let v2_pos = mesh.vertex_position(&v2);


        let old_quality = Triangle3::quality(old_position, v1_pos, v2_pos);
        let new_quality = Triangle3::quality(new_position, v1_pos, v2_pos);

        let bad_quality =
            new_quality < (old_quality * cast(0.5).unwrap()) ||
            new_quality == cast(0).unwrap();
        damages_quality |= bad_quality;
    });

    !damages_quality
}