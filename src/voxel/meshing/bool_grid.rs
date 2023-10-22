use nalgebra::Vector3;

use crate::{voxel::{InternalNode, TreeNode, Tile, Accessor}, dynamic_vdb};

use super::MarchingCubes;

// impl<TChild: TreeNode<LeafNode = TLeaf>, TLeaf: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> MarchingCubes for InternalNode<TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
//     type Cubes<'tree> = CubesIter<'tree, TLeaf> where TLeaf: 'tree;

//     fn cubes(&self) -> Self::Cubes<'_> {
//         todo!()
//     }
// }

type BoolGrid = dynamic_vdb!(4, 3, 2);

struct CubesIter {
    intersection_grid: BoolGrid,
}

impl<T: TreeNode> From<T> for CubesIter {
    fn from(value: T) -> Self {
        let intersection_grid = BoolGrid::new();

        

        Self { intersection_grid }
    }
}

fn tile<'a, T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, tile: Tile) {
    for x in tile.origin.x..tile.size as isize {
        for y in tile.origin.y..tile.size as isize {
            insert_external_voxels(int_grid, src_grid, &Vector3::new(x, y, tile.origin.z));
        }
    }

    for y in tile.origin.y..tile.size as isize {
        for z in tile.origin.z..tile.size as isize {
            insert_external_voxels(int_grid, src_grid, &Vector3::new(tile.origin.x, y, z));
        }
    }

    for x in tile.origin.x..tile.size as isize {
        for z in tile.origin.z..tile.size as isize {
            insert_external_voxels(int_grid, src_grid, &Vector3::new(x, tile.origin.y, z));
        }
    }
}

fn insert_external_voxels<'a, T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, voxel: &Vector3<isize>) {
    for x in -1..1 {
        for y in -1..1 {
            for z in -1..1 {
                let external_voxel = voxel + Vector3::new(x, y, z);
                int_grid.insert(&external_voxel);
            }
        }
    }
}

fn node<'a, T: TreeNode>(int_grid: &mut BoolGrid, src_grid: &T, leaf: &T) {
    
}
