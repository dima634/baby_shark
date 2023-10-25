use nalgebra::Vector3;

use crate::{voxel::{InternalNode, TreeNode, Tile, Accessor, Grid, Leaf, Traverse, utils::box_indices}, dynamic_vdb};


// impl<TChild: TreeNode<LeafNode = TLeaf>, TLeaf: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> MarchingCubes for InternalNode<TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
//     type Cubes<'tree> = CubesIter<'tree, TLeaf> where TLeaf: 'tree;

//     fn cubes(&self) -> Self::Cubes<'_> {
//         todo!()
//     }
// }

pub type BoolGrid = dynamic_vdb!(4, 3, 2);

pub fn intersection_grid<T: Grid>(grid: &T) -> BoolGrid {
    let mut intersection_grid = BoolGrid::new();

    for leaf in grid.leafs() {
        match leaf {
            Leaf::Tile(t) => tile(&mut intersection_grid, grid, t),
            Leaf::Node(n) => node(&mut intersection_grid, grid, n),
        }
    }

    intersection_grid
}

fn tile<T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, tile: Tile) {
    for i in 0..tile.size {
        for j in 0..tile.size {
            let left  = tile.origin + Vector3::new(0, i, j).cast();
            let right = tile.origin + Vector3::new(tile.size - 1, i, j).cast();
            
            let top    = tile.origin + Vector3::new(i, j, tile.size - 1).cast();
            let bottom = tile.origin + Vector3::new(i, j, 0).cast();
            
            let front = tile.origin + Vector3::new(i, tile.size - 1, j).cast();
            let back  = tile.origin + Vector3::new(i, 0, j).cast();

            insert_external_voxels(int_grid, src_grid, &left);
            insert_external_voxels(int_grid, src_grid, &right);
            insert_external_voxels(int_grid, src_grid, &top);
            insert_external_voxels(int_grid, src_grid, &bottom);
            insert_external_voxels(int_grid, src_grid, &front);
            insert_external_voxels(int_grid, src_grid, &back);
        }
    }
}

fn insert_external_voxels<T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, voxel: &Vector3<isize>) {
    for x in -1..1 {
        for y in -1..1 {
            for z in -1..1 {
                let external_voxel = voxel + Vector3::new(x, y, z);

                // if src_grid.at(&external_voxel) {
                //     continue;
                // }

                int_grid.insert(&external_voxel);
            }
        }
    }
}

fn node<T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, leaf: &T::LeafNode) {
    let size = T::LeafNode::resolution() as isize;
    let origin = leaf.origin();

    for x in -1..=size {
        for y in -1..=size {
            for z in -1..=size {
                let voxel = origin + Vector3::new(x, y, z);
                int_grid.insert(&voxel);
            }
        }
    }
}
