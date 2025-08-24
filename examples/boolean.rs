use baby_shark::{
    io::{read_from_file, write_to_file},
    mesh::polygon_soup::data_structure::PolygonSoup,
    voxel::prelude::*,
};
use nalgebra_glm::Vec3;
use std::path::Path;

fn main() {
    let voxel_size = 0.2;

    // Read bunny mesh
    let bunny_mesh: PolygonSoup<f32> =
        read_from_file(Path::new("./assets/bunny.stl")).expect("Read mesh");

    // Convert bunny mesh to volume
    let bunny_volume = MeshToVolume::default()
        .with_voxel_size(voxel_size)
        .convert(&bunny_mesh)
        .unwrap();

    // Create a volume of boxes
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);
    let mut boxes = Volume::with_voxel_size(voxel_size);
    for x in (-25..26).step_by(3) {
        let x = x as f32;
        let next_box = builder.cuboid(Vec3::new(x, -20.0, 0.0), Vec3::new(x + 1.0, 20.0, 50.0));
        boxes = boxes.union(next_box);
    }

    // Perform boolean operations
    let intersection_volume = bunny_volume.clone().intersect(boxes.clone());
    let union_volume = bunny_volume.clone().union(boxes.clone());
    let subtraction_volume = bunny_volume.subtract(boxes);

    write_volume_to_stl(&intersection_volume, "intersection.stl");
    write_volume_to_stl(&union_volume, "union.stl");
    write_volume_to_stl(&subtraction_volume, "subtraction.stl");
}

fn write_volume_to_stl(volume: &Volume, path: &str) {
    let vertices = MarchingCubesMesher::default()
        .with_voxel_size(volume.voxel_size())
        .mesh(volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    write_to_file(&mesh, Path::new(path)).expect("Should write mesh to STL");
}
