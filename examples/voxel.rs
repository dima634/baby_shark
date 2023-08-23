use std::path::Path;

use baby_shark::{
    dynamic_vdb,
    voxel::{TreeNode, LeafNode}, static_vdb, io::stl::StlWriter
};
use bitvec::vec::BitVec;
use nalgebra::Vector3;
use rand::{rngs::StdRng, SeedableRng, Rng};

type MyTree = dynamic_vdb!(5, 4, 3);

fn main() {
    let tree = MyTree::from_singed_scalar_field(100, -10.0, 10.0, |p| {
        return p.y - p.x.sin();
    });

    let ps = tree.to_polygon_soup(100, -10.0, 10.0);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&ps, Path::new("ball.stl")).expect("to work");
}

