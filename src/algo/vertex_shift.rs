use crate::{
    geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
    helpers::aliases::Vec3,
    mesh::corner_table::*,
};
use num_traits::cast;

pub fn is_vertex_shift_safe<S: RealNumber>(
    vertex: VertexId,
    old_position: &Vec3<S>,
    new_position: &Vec3<S>,
    target_edge_length_squared: S,
    mesh: &CornerTable<S>,
) -> bool {
    if (old_position - new_position).norm_squared() > target_edge_length_squared {
        return false;
    }

    let mut damages_quality = false;

    // TODO: iterate over neighboring vertices to avoid match statement
    mesh.faces_around_vertex(vertex, |face| {
        let (v1, v2) = match mesh.face_vertices(face) {
            (v1, v2, v3) if v1 == vertex => (v2, v3),
            (v1, v2, v3) if v2 == vertex => (v3, v1),
            (v1, v2, v3) if v3 == vertex => (v1, v2),
            _ => unreachable!("we are iteration over faces around vertex so such face must contain it"),
        };

        let v1_pos = mesh[v1].position();
        let v2_pos = mesh[v2].position();

        let new_quality = Triangle3::quality(new_position, v1_pos, v2_pos);
        damages_quality |= new_quality < cast(0.001).unwrap();
    });

    !damages_quality
}
