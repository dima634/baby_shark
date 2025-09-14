use baby_shark::{
    exports::nalgebra as na,
    io::write_to_file,
    mesh::{builder::cylinder, corner_table::CornerTable},
    prepare_deform,
};
use std::{
    collections::{HashMap, HashSet},
    f64::consts::PI,
    path::Path,
    time::Instant,
};

fn main() {
    let height = 20.0;
    let cylinder = cylinder::<CornerTable<f64>>(height, 2.0, 4, 30);
    write_to_file(&cylinder, Path::new("original.stl")).expect("should write stl file");

    // Lets twist upper half of the cylinder
    // Take bottom of the cylinder as handle region
    let handle_vertices_iter = cylinder
        .vertices()
        .filter(|&vert| cylinder[vert].position().y == height);
    let handle = HashSet::from_iter(handle_vertices_iter);

    // Take the lower half of the cylinder as region of interest
    let roi_vertices_iter = cylinder
        .vertices()
        .filter(|&vert| cylinder[vert].position().y >= height * 0.5);
    let region_of_interest = HashSet::from_iter(roi_vertices_iter);

    // Rotate handle region around Y axis by 180 degrees
    let transform = na::Matrix4::new_rotation(na::Vector3::new(0.0, PI, 0.0));
    let mut target_positions = HashMap::new();

    for &vert in &handle {
        let current_position = cylinder[vert].position().clone();
        let new_pos = transform.transform_point(&current_position.into()).coords;
        target_positions.insert(vert, new_pos);
    }

    let now = Instant::now();
    let twisted_cylinder = prepare_deform(&cylinder, &handle, &region_of_interest)
        .expect("should prepare deformation")
        .deform(&cylinder, &target_positions)
        .expect("should deform mesh");
    println!("Twisting took {}ms", now.elapsed().as_millis());

    write_to_file(&twisted_cylinder, Path::new("twisted.stl")).expect("should write stl file");
}
