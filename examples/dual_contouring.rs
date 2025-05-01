use baby_shark::{
    io::stl::StlWriter, mesh::polygon_soup::data_structure::PolygonSoup, voxel::prelude::*,
};
use nalgebra_glm::Vec3;
use std::path::Path;

fn main() {
    let voxel_size = 0.2;

    // Union with sphere
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);
    let cube = builder.cuboid(Vec3::zeros(), Vec3::new(10.0, 10.0, 10.0));
    let sphere = builder.sphere(3.0, Vec3::new(8.0, 8.0, 8.0));
    let bunny_volume = cube.subtract(sphere);

    // Convert volume to mesh and write to STL
    let mut mesher = DualContouringMesher::default().with_voxel_size(voxel_size);
    let vertices = mesher.mesh(&bunny_volume).unwrap();
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new("dual_contouring.stl"))
        .expect("Write mesh");
}
