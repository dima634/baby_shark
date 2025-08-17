use baby_shark::{
    io::*, mesh::corner_table::CornerTableF, remeshing::incremental::IncrementalRemesher,
};
use std::path::Path;

fn main() {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader
        .read_from_file(Path::new("./assets/bunny.stl"))
        .expect("Read mesh from STL");

    let remesher = IncrementalRemesher::default();

    let now = std::time::Instant::now();
    remesher.remesh(&mut mesh, 0.5f32);
    println!("Remeshing took: {:?}", now.elapsed());

    let writer = StlWriter::new();
    writer
        .write_to_file(&mesh, Path::new("remeshed.stl"))
        .expect("Save mesh to STL");
}
