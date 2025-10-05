use baby_shark::{
    io::*, mesh::corner_table::CornerTable, remeshing::incremental::*,
};
use std::path::Path;

fn main() {
    let mut mesh: CornerTable<f32> =
        read_from_file(Path::new("./assets/bunny.stl")).expect("should read mesh from file");

    let now = std::time::Instant::now();
    remesh(&mut mesh, 0.5f32, RemeshingOptions::default());
    println!("Remeshing took: {:?}", now.elapsed());

    write_to_file(&mesh, Path::new("remeshed.obj")).expect("should save mesh to file");
}
