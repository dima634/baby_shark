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
    let bunny_volume = mesh_to_volume.convert(&bunny_mesh).unwrap();

    // Offset the bunny
    for offset_by in [-2.5, -1.5, 1.5, 3.0] {
        let offset = bunny_volume.clone().offset(offset_by);
        write_volume_to_stl(&offset, format!("offset_{}.stl", offset_by).as_str());
    }
}

fn write_volume_to_stl(volume: &Volume, path: &str) {
    let vertices = MarchingCubesMesher::default()
        .with_voxel_size(volume.voxel_size())
        .mesh(volume);
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_stl_to_file(&mesh, Path::new(path))
        .expect("Should write mesh to STL");
}
