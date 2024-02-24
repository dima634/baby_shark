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

use std::ops::{Neg, Sub};

use crate::helpers::aliases::Vec3i;
use internal_node::*;
use leaf_node::*;
use root_node::*;

trait Value: Copy + Clone + Send + Sync + PartialEq + PartialOrd + Sub<Output = Self> {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sign {
    Positive,
    Negative,
}

trait Signed: Value {
    fn set_sign(&mut self, sign: Sign);
    fn sign(&self) -> Sign;
    fn far() -> Self;
}

trait Accessor {
    type Value: Value; // Remove Copy?

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
    type As<TValue: Value>: TreeNode<
        Value = TValue,
        Child = <Self::Child as TreeNode>::As<TValue>,
        Leaf = <Self::Leaf as TreeNode>::As<TValue>,
    >;

    /// Creates empty node
    fn empty(origin: Vec3i) -> Box<Self>;
    fn origin(&self) -> Vec3i;
    fn is_empty(&self) -> bool;
    fn fill(&mut self, value: Self::Value);
    fn clear(&mut self);
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
        TNewValue: Value,
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

trait FloodFill
where
    Self: TreeNode,
    Self::Value: Signed,
{
    fn flood_fill(&mut self);
    fn fill_with_sign(&mut self, sign: Sign);
    fn first_value_sign(&self) -> Sign;
    fn last_value_sign(&self) -> Sign;
    fn sign_at(&self, index: &Vec3i) -> Sign;
}

trait Csg
where
    Self: TreeNode + FloodFill,
    Self::Value: Signed + Neg<Output = Self::Value>,
{
    fn union(&mut self, other: Box<Self>);
    fn subtract(&mut self, other: Box<Self>);
    fn intersect(&mut self, other: Box<Self>);
    fn flip_signs(&mut self);
}

#[derive(Debug)]
struct Tile<T> {
    pub origin: Vec3i,
    pub size: usize,
    pub value: T,
}

trait Grid: TreeNode {}

impl<T: TreeNode> Grid for T {}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf
