use baby_shark::{
    io::stl::StlWriter,
    mesh::{builder::cylinder, corner_table::prelude::CornerTableD, traits::Mesh},
    *,
};
use nalgebra as na;
use std::{collections::HashSet, path::Path};

fn main() {
    let mut cylinder: CornerTableD = cylinder(10.0, 2.0, 4, 20);

    let transform = na::Matrix4::new_translation(&na::Vector3::new(0.0, 0.0, 5.0));
    let handle = &HashSet::from_iter(
        cylinder
            .vertices()
            .filter(|v| cylinder.vertex_position(v).y == 10.0),
    );
    let roi =
        &HashSet::from_iter(cylinder.vertices().filter(|v| {
            cylinder.vertex_position(v).y != 0.0 && cylinder.vertex_position(v).y != 10.0
        }));

    prepare_deform(&cylinder, handle, roi)
        .expect("failed to prepare deform")
        .deform(&mut cylinder, transform);

    StlWriter::new()
        .write_stl_to_file(&cylinder, Path::new("cylinder.stl"))
        .unwrap();
}
