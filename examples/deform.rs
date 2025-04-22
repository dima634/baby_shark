use baby_shark::{
    io::stl::{StlReader, StlWriter}, mesh::{builder::cylinder, corner_table::prelude::CornerTableD, traits::Mesh}, remeshing::incremental::IncrementalRemesher, *
};
use nalgebra as na;
use std::{collections::{BTreeSet, HashMap, HashSet}, f64::consts::PI, ops::ControlFlow, path::Path, time::Instant};

fn main() {
    // let mut cylinder: CornerTableD = cylinder(10.0, 2.0, 4, 15);

    // StlWriter::new()
    //     .write_stl_to_file(&cylinder, Path::new("orig.stl"))
    //     .unwrap();

    // let handle = HashSet::from_iter(
    //     cylinder
    //         .vertices()
    //         .filter(|v| cylinder.vertex_position(v).y == 10.0 || cylinder.vertex_position(v).y == 0.0),
    // );
    // let roi = HashSet::from_iter(cylinder.vertices());

    // // Rotate part of the handle
    // let transform = na::Matrix4::new_rotation(na::Vector3::new(0.0, PI / 2.0, 0.0));
    // let mut target = HashMap::new();

    // for vert in &handle {
    //     let pos = cylinder.vertex_position(vert).clone();
    //     if pos.y != 10.0 {
    //         continue; // Skip the bottom vertices
    //     }

    //     let new_pos = transform.transform_point(&pos.into()).coords;
    //     target.insert(*vert, new_pos);
    // }

    // let cylinder = prepare_deform(&cylinder, &handle, &roi)
    //     .expect("failed to prepare deform")
    //     .deform(&cylinder, &target)
    //     .expect("failed to deform");

    // StlWriter::new()
    //     .write_stl_to_file(&cylinder, Path::new("cylinder.stl"))
    //     .unwrap();

    let gingiva: CornerTableD = StlReader::default()
        .read_stl_from_file(Path::new("assets/gingiva.stl"))
        .unwrap();

    let mut handle = HashSet::new();

    for ring in gingiva.boundary_rings() {
        let mut count = 0;
        gingiva.boundary_edges(ring, |edge| {
            count += 1;
            ControlFlow::Continue(())
        });

        if count < 10 {
            continue; // Skip small rings
        }
    
        gingiva.boundary_edges(ring, |edge| {
            let (v1, _) = gingiva.edge_vertices(&edge);
            handle.insert(v1);
            ControlFlow::Continue(())
        });

        println!("Ring {:?} has {} edges", ring, count);
    }

    let prepared_deform = prepare_deform(&gingiva, &handle, &gingiva.vertices().collect()).unwrap();
    
    let mut target = HashMap::new();

    let time = Instant::now();
    let deformed = prepared_deform.deform(&gingiva, &target).unwrap();
    println!("Deform took {}ms", time.elapsed().as_millis());

    StlWriter::new()
        .write_stl_to_file(&deformed, Path::new("deformed.stl"))
        .unwrap();
}
