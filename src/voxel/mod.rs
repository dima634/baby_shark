pub mod mesh_to_sdf;
pub mod meshing;
pub mod sdf;

mod grid_value;
mod init;
mod internal_node;
mod leaf_node;
mod root_node;
#[cfg(test)]
mod tests;
mod utils;

use std::ops::Sub;

pub use sdf::*;

use crate::helpers::aliases::Vec3i;
use internal_node::*;
use leaf_node::*;
use root_node::*;

trait GridValue: Copy + Clone + Send + Sync + PartialEq + PartialOrd + Sub<Output = Self> {}

trait Signed: GridValue {
    fn copy_sign(&mut self, other: Self);
    fn is_neg() -> bool;
}

trait Accessor {
    type Value: GridValue; // Remove Copy?

    fn at(&self, index: &Vec3i) -> Option<&Self::Value>;
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value>;
    fn insert(&mut self, index: &Vec3i, value: Self::Value);
    fn remove(&mut self, index: &Vec3i);
}

trait Visitor<T: TreeNode> {
    fn tile(&mut self, tile: Tile<T::Value>);
    fn dense(&mut self, dense: &T);
}

trait ParVisitor<T: TreeNode>: Send + Sync {
    fn tile(&self, tile: Tile<T::Value>);
    fn dense(&self, dense: &T);
}

trait TreeNode: Accessor + Send + Sync + Sized {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Number of childs/voxels in node
    const SIZE: usize;

    const IS_LEAF: bool;

    type Leaf: TreeNode<Value = Self::Value>;
    type Child: TreeNode<Value = Self::Value, Leaf = Self::Leaf>;
    type As<TValue: GridValue>: TreeNode<
        Value = TValue,
        Child = <Self::Child as TreeNode>::As<TValue>,
        Leaf = <Self::Leaf as TreeNode>::As<TValue>,
    >;

    /// Creates empty node
    fn empty(origin: Vec3i) -> Box<Self>;
    fn origin(&self) -> Vec3i;
    fn is_empty(&self) -> bool;
    fn fill(&mut self, value: Self::Value);
    fn visit_leafs<T: Visitor<Self::Leaf>>(&self, visitor: &mut T);
    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T);

    ///
    /// Checks if node is constant within tolerance.
    /// Empty nodes are not constant.
    ///
    /// Returns `None` if node is not constant, otherwise returns constant value
    ///
    fn is_constant(&self, tolerance: Self::Value) -> Option<Self::Value>;

    ///
    /// Prune all nodes where all values are within tolerance
    ///
    fn prune(&mut self, tolerance: Self::Value) -> Option<Self::Value>;

    ///
    /// Creates a copy of the node with same topology but with different values
    ///
    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where
        TNewValue: GridValue,
        TCast: Fn(Self::Value) -> TNewValue;

    /// Number of voxels in one dimension
    #[inline]
    fn resolution() -> usize {
        1 << Self::BRANCHING_TOTAL
    }

    /// Total number of voxels
    #[inline]
    fn size() -> usize {
        1 << Self::BRANCHING_TOTAL * 3
    }
}

trait FloodFill: TreeNode where Self::Value: Signed {
    fn flood_fill(&mut self);
}

struct Tile<T> {
    pub origin: Vec3i,
    pub size: usize,
    pub value: T,
}

trait Grid: TreeNode {}

impl<T: TreeNode> Grid for T {}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf
