pub mod init;
pub mod internal_node;
pub mod leaf_node;
pub mod root_node;
pub mod meshing;

mod cached_accessor;

#[cfg(test)]
mod tests;
pub mod utils;

pub use internal_node::*;
pub use leaf_node::*;
pub use root_node::*;

use nalgebra::Vector3;

pub trait TreeNodeConsts {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Total number of voxels in node
    const SIZE: usize;

    fn index_key(&self, index: &Vector3<usize>) -> Vector3<usize> {
        return Vector3::new(
            index.x & !((1 << Self::BRANCHING_TOTAL) - 1),
            index.y & !((1 << Self::BRANCHING_TOTAL) - 1),
            index.z & !((1 << Self::BRANCHING_TOTAL) - 1),
        );
    }
}

pub trait Accessor {
    fn at(&self, index: &Vector3<isize>) -> bool;
    fn insert(&mut self, index: &Vector3<isize>);
    fn remove(&mut self, index: &Vector3<isize>);
    fn index_key(&self, index: &Vector3<usize>) -> Vector3<usize>;
}

pub trait TreeNode: Accessor {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Total number of voxels in node
    const SIZE: usize;

    /// Creates empty node
    fn new_inactive(origin: Vector3<isize>) -> Self;
    /// Creates active node
    fn new_active(origin: Vector3<isize>) -> Self;

    fn is_empty(&self) -> bool;

    fn voxels<F: FnMut(Vector3<isize>)>(&self, f: &mut F);
}

pub trait HasChild {
    type Child: TreeNode;
}

struct Tile {
    pub origin: Vector3<usize>,
    pub size: usize
}

enum Voxel {
    Tile(Tile),
    Voxel(Vector3<usize>),
}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf
