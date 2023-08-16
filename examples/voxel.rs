use std::path::Path;

use baby_shark::{
    vdb,
    voxel::{TreeNode, LeafNode}
};
use bitvec::vec::BitVec;
use nalgebra::Vector3;
use rand::{rngs::StdRng, SeedableRng, Rng};

type MyTree = vdb!(5, 4, 3);

fn main() {
    let tree = MyTree::empty();

    tree.at(&Vector3::new(32767, 0, 0));
}

