use std::path::Path;

use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::corner_table::prelude::CornerTableF,
    remeshing::incremental::IncrementalRemesher,
};

fn main() {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader
        .read_stl_from_file(Path::new("./assets/bunny.stl"))
        .expect("Read mesh from STL");

    let remesher = IncrementalRemesher::new()
        .with_iterations_count(5)
        .with_split_edges(true)
        .with_collapse_edges(true)
        .with_flip_edges(true)
        .with_shift_vertices(true)
        .with_project_vertices(true);

    let now = std::time::Instant::now();
    remesher.remesh(&mut mesh, 0.5f32);
    println!("Remeshing took: {:?}", now.elapsed());

    let writer = StlWriter::new();
    writer
        .write_stl_to_file(&mesh, Path::new("remeshed.stl"))
        .expect("Save mesh to STL");
}
