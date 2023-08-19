pub mod init;
pub mod internal_node;
pub mod leaf_node;
pub mod root_node;

mod utils;

#[cfg(test)]
mod tests;

use nalgebra::Vector3;

pub use internal_node::*;
pub use leaf_node::*;
pub use root_node::*;

pub trait TreeNode {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Total number of voxels in node
    const SIZE: usize;

    /// Creates empty node
    fn new_inactive(origin: Vector3<usize>) -> Self;
    /// Create active node
    fn new_active(origin: Vector3<usize>) -> Self;

    fn at(&self, index: &Vector3<usize>) -> bool;
    fn insert(&mut self, index: &Vector3<usize>);
    fn remove(&mut self, index: &Vector3<usize>);
    fn is_empty(&self) -> bool;
}

pub trait HasChild {
    type Child: TreeNode;
}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf

