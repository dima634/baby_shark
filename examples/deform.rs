use baby_shark::{
    exports::nalgebra as na,
    io::write_to_file,
    mesh::{builder::cylinder, corner_table::CornerTableD},
    prepare_deform,
};
use std::{
    collections::{HashMap, HashSet},
    f64::consts::PI,
    path::Path,
};

fn main() {
    let cylinder = cylinder::<CornerTableD>(10.0, 2.0, 4, 15);
    let handle = HashSet::from_iter(cylinder.vertices().filter(|&v| {
        cylinder.vertex_position(v).y == 10.0 || cylinder.vertex_position(v).y == 0.0
    }));
    let roi = HashSet::from_iter(cylinder.vertices());

    // Rotate part of the handle
    let transform = na::Matrix4::new_rotation(na::Vector3::new(0.0, PI / 2.0, 0.0));
    let mut target = HashMap::new();

    for &vert in &handle {
        let pos = cylinder.vertex_position(vert).clone();
        if pos.y != 10.0 {
            continue; // Skip the bottom vertices
        }

        let new_pos = transform.transform_point(&pos.into()).coords;
        target.insert(vert, new_pos);
    }

    let deformed_cylinder = prepare_deform(&cylinder, &handle, &roi)
        .expect("should prepare deformation")
        .deform(&cylinder, &target)
        .expect("should deform mesh");

    write_to_file(&deformed_cylinder, Path::new("deformed_cylinder.stl"))
        .expect("should write stl file");
}
