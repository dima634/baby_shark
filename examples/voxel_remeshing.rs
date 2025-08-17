use baby_shark::{
    io::*, mesh::polygon_soup::data_structure::PolygonSoup, remeshing::voxel::VoxelRemesher,
};
use std::path::Path;

fn main() {
    type Mesh = PolygonSoup<f32>;

    let mut reader = StlReader::new();
    let mesh: Mesh = reader
        .read_from_file(Path::new("./assets/box.stl"))
        .expect("Read mesh");

    let mut remesher = VoxelRemesher::default().with_voxel_size(0.01);
    let remeshed = remesher.remesh(&mesh).unwrap();

    StlWriter::new()
        .write_to_file(&remeshed, Path::new("remeshed.stl"))
        .expect("Write mesh");
}
