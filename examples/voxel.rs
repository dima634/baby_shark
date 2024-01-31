use std::{path::Path, f32::consts::PI, time::Instant};

use baby_shark::{
    dynamic_vdb,
    voxel::{LeafNode, meshing::{CubesMesher, MarchingCubesMesher}, utils::{box_indices, GridIter}, Accessor, Scalar, Sdf, mesh_to_sdf::MeshToSdf}, static_vdb, io::stl::{StlWriter, StlReader}, mesh::{corner_table::prelude::CornerTableF, builder::cube, polygon_soup::data_structure::PolygonSoup, traits::Mesh}, geometry::primitives::{box3::Box3, sphere3::Sphere3}
};
use nalgebra::{Vector3, Point3};
use rand::{rngs::StdRng, SeedableRng, Rng};
use svg::node::element::Polygon;

// type MyTree = dynamic_vdb!(Scalar, 4, 3, 2);
type MyTree = dynamic_vdb!((), 4, 3, 2);
type Stat = static_vdb!(Scalar, 5, 4, 3);

fn main() {
    type Mesh = PolygonSoup<f32>;

    let now = Instant::now();

    let mut reader = StlReader::new();
    let mesh: Mesh = reader.read_stl_from_file(Path::new("lucy.stl")).expect("Read mesh");

    println!("Read mesh: {}ms", now.elapsed().as_millis());

    let now = Instant::now();
    
    let mut mesh_to_sdf = MeshToSdf::new().voxel_size(2.0);
    let sdf: Sdf = mesh_to_sdf.approximate(&mesh);

    println!("Mesh to SDF: {}ms", now.elapsed().as_millis());

    let now = Instant::now();

    let mut mc = MarchingCubesMesher::new(&sdf);
    let vertices = mc.mesh().into_iter().map(|p| (p * mesh_to_sdf.voxel_size).cast().into()).collect();
    let mc_mesh = Mesh::from_vertices(vertices);

    println!("Marching cubes: {}ms", now.elapsed().as_millis());

    // println!("{:?}", mc_mesh);

    // let mut dc = DualContouringMesher::new(sdf.grid.as_ref());
    // let vertices = dc.mesh().into_iter().map(|p| (p * mesh_to_sdf.voxel_size).cast().into()).collect();
    // let mc_mesh = Mesh::from_vertices(vertices);

    let now = Instant::now();

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mc_mesh, Path::new("mc.stl")).expect("Write mesh");

    println!("Write mesh: {}ms", now.elapsed().as_millis());

    // let mut cubes = CubesMesher::new(sdf.grid.as_ref());
    // let vertices = cubes.mesh().into_iter().map(|p| (p.cast() * mesh_to_sdf.voxel_size).cast().into()).collect();
    // let cubes_mesh = Mesh::from_vertices(vertices);

    // writer.write_stl_to_file(&cubes_mesh, Path::new("cubes.stl")).expect("Write mesh");
}

