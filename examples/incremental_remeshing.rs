use baby_shark::{
    io::*, mesh::corner_table::CornerTableF, remeshing::incremental::IncrementalRemesher,
};
use std::path::Path;

fn main() {
    let mut mesh: CornerTableF =
        read_from_file(Path::new("./assets/bunny.stl")).expect("should read mesh from file");

    let now = std::time::Instant::now();
    let remesher = IncrementalRemesher::default();
    remesher.remesh(&mut mesh, 0.5f32);
    println!("Remeshing took: {:?}", now.elapsed());

    write_to_file(&mesh, Path::new("remeshed.obj")).expect("should save mesh to file");
}
