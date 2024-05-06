use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    voxel::prelude::*,
};
use std::path::Path;

fn main() {
    let voxel_size = 0.2;

    // Read bunny mesh
    let mut reader = StlReader::new();
    let bunny_mesh: PolygonSoup<f32> = reader
        .read_stl_from_file(Path::new("./assets/bunny.stl"))
        .expect("Read mesh");

    // Convert bunny mesh to volume
    let mut mesh_to_volume = MeshToVolume::default().with_voxel_size(voxel_size);
    let mut bunny_volume = mesh_to_volume.convert(&bunny_mesh).unwrap();

    // Offset the bunny
    let offset_by = 1.5;
    bunny_volume = bunny_volume.offset(offset_by);

    // Convert volume to mesh and write to STL
    let mut mesher = MarchingCubesMesher::default().with_voxel_size(voxel_size);
    let vertices = mesher.mesh(&bunny_volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("offset.stl"))
        .expect("Write mesh");
}
