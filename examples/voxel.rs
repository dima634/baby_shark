use std::{path::Path, f32::consts::PI};

use baby_shark::{
    dynamic_vdb,
    voxel::{TreeNode, LeafNode, meshing::{cubes, marching_cubes}, utils::box_indices, Accessor}, static_vdb, io::stl::StlWriter, mesh::{corner_table::prelude::CornerTableF, builder::cube, polygon_soup::data_structure::PolygonSoup}
};
use bitvec::vec::BitVec;
use nalgebra::Vector3;
use rand::{rngs::StdRng, SeedableRng, Rng};

type MyTree = dynamic_vdb!(5, 4, 3);

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
    
    let mesh: CornerTableF = cubes(&tree);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("test_cubes.stl")).unwrap();
    
    let mesh: PolygonSoup<f32> = marching_cubes(&tree);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("test_marching.stl")).unwrap();
}

