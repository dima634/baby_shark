use std::path::Path;

use baby_shark::{
    io::stl::{StlReader, StlWriter}, 
    remeshing::incremental::IncrementalRemesher,
    mesh::corner_table::prelude::CornerTableF
};

fn main() {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader.read_stl_from_file(Path::new("./mc.stl")).expect("Read mesh from STL");
    
    let remesher = IncrementalRemesher::new()
        .with_iterations_count(5)
        .with_split_edges(true)
        .with_collapse_edges(false)
        .with_flip_edges(true)
        .with_shift_vertices(false)
        .with_project_vertices(false);
    remesher.remesh(&mut mesh, 0.001f32);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("remeshed.stl")).expect("Save mesh to STL");
}
