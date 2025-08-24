use baby_shark::{
    io::{read_from_file, write_to_file}, 
    mesh::polygon_soup::data_structure::PolygonSoup, 
    remeshing::voxel::VoxelRemesher,
};
use std::path::Path;

fn main() {
    type Mesh = PolygonSoup<f32>;

    let mesh: Mesh = read_from_file(Path::new("./assets/box.stl"))
        .expect("Read mesh");

    let mut remesher = VoxelRemesher::default().with_voxel_size(0.01);
    let remeshed = remesher.remesh(&mesh).unwrap();

    write_to_file(&remeshed, Path::new("remeshed.stl"))
        .expect("Write mesh");
}
