pub mod init;
pub mod internal_node;
pub mod leaf_node;
pub mod meshing;
pub mod root_node;
pub mod utils;
pub mod sdf;

pub mod mesh_to_sdf;
mod grid_value;

#[cfg(test)]
mod tests;

pub use grid_value::*;
pub use internal_node::*;
pub use leaf_node::*;
pub use root_node::*;
pub use sdf::*;

use crate::helpers::aliases::Vec3i;

pub trait IsWithinTolerance {
    fn is_within_tolerance(&self, value: Self, tolerance: Self) -> bool;
}

pub trait GridValue: Copy + Clone + Send + Sync + PartialEq + IsWithinTolerance {}

pub trait Accessor {
    type Value: GridValue; // Remove Copy?

    fn at(&self, index: &Vec3i) -> Option<&Self::Value>;
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value>;
    fn insert(&mut self, index: &Vec3i, value: Self::Value);
    fn remove(&mut self, index: &Vec3i);
}

pub trait Visitor<T: TreeNode> {
    fn tile(&mut self, tile: Tile<T::Value>);
    fn dense(&mut self, dense: &T);
}

pub trait ParVisitor<T: TreeNode>: Send + Sync {
    fn tile(&self, tile: Tile<T::Value>);
    fn dense(&self, dense: &T);
}

pub trait TreeNode: Accessor + Send + Sync + Sized {
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
        Leaf = <Self::Leaf as TreeNode>::As<TValue>
    >;

    /// Creates empty node
    fn empty(origin: Vec3i) -> Box<Self>;
    fn origin(&self) -> Vec3i;
    fn is_empty(&self) -> bool;
    fn fill(&mut self, value: Self::Value);
    fn traverse_leafs<F: FnMut(Leaf<Self::Leaf>)>(&self, f: &mut F);
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

    ///
    // fn dilate(&mut self, value: Self::Value, neighbor_masks: [&[usize]; 6]);

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

    /// Total number of voxels
    #[inline]
    fn size_t(&self) -> usize {
        1 << Self::BRANCHING_TOTAL
    }

    ///
    /// Returns number of tiles and dense leafs -> `(tiles_count, leafs_count)`
    /// 
    fn leafs_count(&self) -> (usize, usize) {
        let mut tiles_count = 0;
        let mut leafs_count = 0;
        self.traverse_leafs(&mut |leaf| match leaf {
            Leaf::Tile(_) => tiles_count += 1,
            Leaf::Dense(_) => leafs_count += 1,
        });

        (tiles_count, leafs_count)
    }
}

pub struct Tile<T> {
    pub origin: Vec3i,
    pub size: usize,
    pub value: T,
}

pub enum Leaf<'a, T: Accessor> {
    Tile(Tile<T::Value>),
    Dense(&'a T),
}

pub trait Grid: TreeNode {}

impl<T: TreeNode> Grid for T {}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf
