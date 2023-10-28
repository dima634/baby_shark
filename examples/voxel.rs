use std::{path::Path, f32::consts::PI, time::Instant};

use baby_shark::{
    dynamic_vdb,
    voxel::{TreeNode, LeafNode, meshing::{CubesMeshing, marching_cubes}, utils::box_indices, Accessor, Traverse}, static_vdb, io::stl::StlWriter, mesh::{corner_table::prelude::CornerTableF, builder::cube, polygon_soup::data_structure::PolygonSoup}
};
use bitvec::vec::BitVec;
use nalgebra::Vector3;
use rand::{rngs::StdRng, SeedableRng, Rng};

type MyTree = dynamic_vdb!((), 4, 3, 2);

fn main() {
    let tree = MyTree::from_singed_scalar_field(250, -10.0, 10.0, |p| {
        let cell_size = 3.0_f32;
        let density = 0.2_f32;
        let x = 2.0 * PI * p.x / cell_size;
        let y = 2.0 * PI * p.y / cell_size;
        let z = 2.0 * PI * p.z / cell_size;


        // Tub-Primitive 
        (x.cos() + y.cos() + z.cos() - 0.51 * (x.cos() * y.cos() + y.cos() * z.cos() + z.cos() * x.cos()) - 1.0)

        // Schoen-Gyroid
        // (x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos() - density)

        // Schoen-I-WP (IWP)
        // (
        //     x.cos() * y.cos() + y.cos() * z.cos() + x.cos() * z.cos() - 
        //     0.5 * ((x * 2.0).cos() + (y * 2.0).cos() + (z * 2.0).cos() - density)
        // )
    });

    // let mut tree = MyTree::new();

    // for i in box_indices(0, 10) {
    //     tree.insert(&i);
    // }

    // tree.insert(&Vector3::new(0, 0, 0));
    
    // let mut mesher = CubesMeshing::new(&tree);
    // let mesh: CornerTableF = mesher.mesh();


    // let writer = StlWriter::new();
    // writer.write_stl_to_file(&mesh, Path::new("cubes.stl")).unwrap();

    let now = Instant::now();

    let vert = marching_cubes(&tree);
    let mesh = PolygonSoup::from_vertices(vert.into_iter().map(|v| v.into()).collect());
    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("march.stl")).unwrap();
    
    println!("Marching Cubes: {}", now.elapsed().as_millis());

    // let mesh: PolygonSoup<f32> = marching_cubes(&tree);

    // let writer = StlWriter::new();
    // writer.write_stl_to_file(&mesh, Path::new("test_marching.stl")).unwrap();

    // type Tree = static_vdb!(5, 4, 3);

    // let size = 500;
    // let mut tree = Tree::new();

    // for x in 0..size {
    //     for y in 0..size {
    //         for z in 0..size {
    //             let idx = Vector3::new(x, y, z);
    //             tree.insert(&idx);
    //         }
    //     }
    // }

    // let cached_accessor = tree.cached_accessor();

    // let s = Instant::now();
    // for x in 0..size {
    //     for y in 0..size {
    //         for z in 0..size {
    //             let idx = Vector3::new(x, y, z);
    //             let direct = tree.at(&idx);
        
    //             assert!(direct);
    //         }
    //     } 
    // }

    // println!("Direct {}", s.elapsed().as_nanos());

    // let s = Instant::now();
    // for x in 0..size {
    //     for y in 0..size {
    //         for z in 0..size {
    //             let idx = Vector3::new(x, y, z);
    //             let cached = cached_accessor.at(&idx);
        
    //             assert!(cached);
    //         }
    //     } 
    // }
    // println!("Cached {}", s.elapsed().as_nanos());
}

