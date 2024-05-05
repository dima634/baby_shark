use baby_shark::{
    io::stl::{StlReader, StlWriter},
    mesh::polygon_soup::data_structure::PolygonSoup,
    voxel::prelude::*,
};
use nalgebra_glm::Vec3;
use std::path::Path;

fn main() {
    let voxel_size = 0.2;
    let mut mesher = MarchingCubesMesher::default().with_voxel_size(voxel_size);

    // Read bunny mesh
    let mut reader = StlReader::new();
    let bunny_mesh: PolygonSoup<f32> = reader
        .read_stl_from_file(Path::new("./assets/bunny.stl"))
        .expect("Read mesh");

    // Convert bunny mesh to volume
    let mut mesh_to_sdf = MeshToVolume::default()
        .with_voxel_size(voxel_size)
        .with_narrow_band_width(0);
    let bunny = mesh_to_sdf.convert(&bunny_mesh).unwrap();

    let min = Vec3::new(-26.0, -26.0, 0.0);
    let max = Vec3::new(26.0, 26.0, 50.0);
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);

    let iwp = builder.iwp(min, max);
    let offset_by = -1.0;
    let offset = bunny.clone().offset(offset_by);
    let bunny_iwp = offset.clone().intersect(iwp);
    let clip_box = builder.cuboid(Vec3::new(-30.0, -30.0, -2.0), Vec3::new(30.0, -5.0, 50.0));
    let bunny = bunny.subtract(offset)
        .union(bunny_iwp)
        .subtract(clip_box);


    // Convert volume to mesh and write to STL
    let vertices = mesher.mesh(&bunny);//.expect("Should mesh");
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("result.stl"))
        .expect("Write mesh");
}
