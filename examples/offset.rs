use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    voxel::{meshing::ActiveVoxelsMesher, prelude::*},
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
        .with_narrow_band_width(0);
    let mut bunny_volume = mesh_to_sdf.convert(&bunny_mesh).unwrap();

    // bunny_volume = VolumeBuilder::default().with_voxel_size(voxel_size).sphere(5.0, Vec3::zeros());
    // let s = VolumeBuilder::default().with_voxel_size(voxel_size).sphere(1.0, Vec3::new(2.0, 2.0, 2.0));
    // let s2 = VolumeBuilder::default().with_voxel_size(voxel_size).sphere(1.0, Vec3::new(0.0, 0.0, 0.0));
    // let mut bunny_volume = VolumeBuilder::default().with_voxel_size(voxel_size).cuboid(Vec3::zeros(), Vec3::new(2.0, 2.0, 2.0));
    // bunny_volume = bunny_volume.union(s).subtract(s2);

    // Convert volume to mesh and write to STL
    let mut mesher = MarchingCubesMesher::default().with_voxel_size(voxel_size);
    let vertices = mesher.mesh(&bunny_volume);//.expect("Should offset");
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("original.stl"))
        .expect("Write mesh");



    bunny_volume = bunny_volume.offset(1.0);
    

    let vertices = mesher.mesh(&bunny_volume);//.expect("Should offset");
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("offset.stl"))
        .expect("Write mesh");
}
