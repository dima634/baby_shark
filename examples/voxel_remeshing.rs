use std::path::Path;

use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    remeshing::voxel::{MeshingMethod, VoxelRemesher},
};

fn main() {
    type Mesh = PolygonSoup<f32>;

    let mut reader = StlReader::new();
    let mesh: Mesh = reader
        .read_stl_from_file(Path::new("../test_files/head.stl"))
        .expect("Read mesh");

    let mut remesher = VoxelRemesher::default().with_voxel_size(0.02).with_meshing_method(MeshingMethod::FeaturePreserving);
    let remeshed = remesher.remesh(&mesh).unwrap();

    StlWriter::new()
        .write_stl_to_file(&remeshed, Path::new("remeshed.stl"))
        .expect("Write mesh");
}
