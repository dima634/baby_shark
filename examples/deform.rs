use std::{collections::BTreeSet, path::Path};
use nalgebra as na;
use baby_shark::{io::stl::StlWriter, mesh::{builder::cylinder, corner_table::prelude::CornerTableD, traits::Mesh}, *};

fn main() {
    let mut cylinder: CornerTableD = cylinder(10.0, 2.0, 4, 20);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&cylinder, Path::new("cylinder_orig.stl")).unwrap();

    let transform = na::Matrix4::new_translation(&na::Vector3::new(0.0, 0.0, 5.0));
    let handle = &BTreeSet::from_iter(cylinder.vertices().filter(|v| cylinder.vertex_position(v).y == 10.0));
    let roi = &BTreeSet::from_iter(cylinder.vertices().filter(|v| cylinder.vertex_position(v).y != 0.0 && cylinder.vertex_position(v).y != 10.0));

    deform(
        &mut cylinder, 
        handle, 
        roi, 
        transform
    );

    writer.write_stl_to_file(&cylinder, Path::new("cylinder.stl")).unwrap();
}
