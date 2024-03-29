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

    // Slice bunny with vertical boxes
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);

    for x in (-25..26).step_by(3) {
        let x = x as f32;
        let slice_box = builder.cuboid(Vec3::new(x, -20.0, 0.0), Vec3::new(x + 1.0, 20.0, 50.0));
        bunny_volume = bunny_volume.subtract(slice_box);
    }

    // Convert volume to mesh and write to STL
    let mut mesher = MarchingCubesMesher::default().with_voxel_size(voxel_size);
    let vertices = mesher.mesh(&bunny_volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("sliced_bunny.stl"))
        .expect("Write mesh");
}
