use baby_shark::{io::*, mesh::polygon_soup::data_structure::PolygonSoup, voxel::prelude::*};
use nalgebra_glm::Vec3;
use std::path::Path;

fn main() {
    let voxel_size = 0.2;

    // Read bunny mesh
    let bunny_mesh: PolygonSoup<f32> = StlReader::new()
        .read_from_file(Path::new("./assets/bunny.stl"))
        .expect("Should read read mesh from STL");

    // Convert bunny mesh to volume
    let mut mesh_to_sdf = MeshToVolume::default().with_voxel_size(voxel_size);
    let bunny = mesh_to_sdf.convert(&bunny_mesh).unwrap();

    // Lightweighting can be accomplished by combining boolean operations and offsetting.
    // First, we use offset and boolean subtraction to create a hollow inside the bunny.
    // Then, we fill the hollow with TPMS lattice using boolean intersection and union.
    let builder = VolumeBuilder::default().with_voxel_size(voxel_size);
    let min = Vec3::new(-26.0, -26.0, 0.0);
    let max = Vec3::new(26.0, 26.0, 50.0);
    let iwp = builder.iwp(min, max, 0.5); // lattice

    let offset_by = -1.0;
    let bunny_offset = bunny.clone().offset(offset_by);
    let bunny_iwp = bunny_offset.clone().intersect(iwp);
    let clip_box = builder.cuboid(Vec3::new(-30.0, -30.0, -2.0), Vec3::new(30.0, -5.0, 50.0));
    let bunny = bunny
        .subtract(bunny_offset) // create hollow inside the bunny
        .union(bunny_iwp) // fill the hollow with TPMS lattice
        .subtract(clip_box); // clip the bunny to see what is inside, just for visualization

    // Convert volume to mesh and write to STL
    let vertices = DualContouringMesher::default()
        .with_voxel_size(voxel_size)
        .mesh(&bunny)
        .expect("Should convert volume to mesh");
    let mesh = PolygonSoup::from_vertices(vertices);

    StlWriter::new()
        .write_to_file(&mesh, Path::new("result.stl"))
        .expect("Should write mesh to STL");
}
