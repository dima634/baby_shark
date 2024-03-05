use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    voxel::prelude::*,
};
use nalgebra_glm::Vec3;
use std::path::Path;

fn main() {
    let voxel_size = 0.2;

    // Read bunny mesh
    let mut reader = StlReader::new();
    let bunny_mesh: PolygonSoup<f32> = reader
        .read_stl_from_file(Path::new("./assets/bunny.stl"))
        .expect("Read mesh");

    // Convert bunny mesh to volume
    let mut mesh_to_sdf = MeshToVolume::default()
        .with_voxel_size(voxel_size)
        .with_narrow_band_width(1);
    let mut bunny_volume = mesh_to_sdf.convert(&bunny_mesh).unwrap();

    // Union with sphere
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);
    let sphere = builder.sphere(12.0, Vec3::new(-18.189699, -8.620899, 32.004601));
    bunny_volume = bunny_volume.union(sphere);

    // Convert volume to mesh and write to STL
    let mut mesher = MarchingCubesMesher::default().with_voxel_size(voxel_size);
    let vertices = mesher.mesh(&bunny_volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("spherehead_bunny.stl"))
        .expect("Write mesh");
}
