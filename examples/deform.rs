use baby_shark::{
    io::stl::StlWriter, mesh::{builder::cylinder, corner_table::prelude::CornerTableD, traits::Mesh}, remeshing::incremental::IncrementalRemesher, *
};
use nalgebra as na;
use std::{collections::{BTreeSet, HashSet}, f64::consts::PI, path::Path};

fn main() {
    let mut cylinder: CornerTableD = cylinder(10.0, 2.0, 4, 15);

    // IncrementalRemesher::default()
    //     .remesh(&mut cylinder, 0.3);

    StlWriter::new()
        .write_stl_to_file(&cylinder, Path::new("orig.stl"))
        .unwrap();

    let transform = na::Matrix4::new_rotation(na::Vector3::new(0.0, PI / 2.0, 0.0));
    let handle = BTreeSet::from_iter(
        cylinder
            .vertices()
            .filter(|v| cylinder.vertex_position(v).y == 10.0),
    );
    let roi =
        BTreeSet::from_iter(cylinder.vertices().filter(|v| {
            cylinder.vertex_position(v).y != 0.0 && cylinder.vertex_position(v).y != 10.0
        }));

    let anchor = BTreeSet::from_iter(
        cylinder
            .vertices()
            .filter(|v| cylinder.vertex_position(v).y == 0.0),
    );

    //let anchor = BTreeSet::new();

    let cylinder = prepare_deform_arap(&cylinder, &handle, &roi, &anchor)
        .expect("failed to prepare deform")
        .deform_arap(&cylinder, transform);

    StlWriter::new()
        .write_stl_to_file(&cylinder, Path::new("cylinder.stl"))
        .unwrap();
}
