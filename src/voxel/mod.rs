pub mod mesh_to_volume;
pub mod meshing;
pub mod prelude;
pub mod volume;

mod fast_sweep;
mod init;
mod internal_node;
mod leaf_node;
mod root_node;
mod utils;
mod value;
mod visitors;

#[cfg(test)]
mod tests;

use crate::helpers::aliases::Vec3i;
use internal_node::*;
use leaf_node::*;
use root_node::*;
use volume::*;
use std::ops::{Neg, Sub};

trait Value:
    Default + Copy + Clone + Send + Sync + PartialEq + PartialOrd + Sub<Output = Self>
{
}

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

trait Visitor<TLeaf: TreeNode> {
    fn tile(&mut self, tile: Tile<TLeaf::Value>);
    fn dense(&mut self, dense: &TLeaf);
}

trait ValueVisitorMut<T> {
    fn value(&mut self, value: &mut T);
}

trait ParVisitor<T: TreeNode>: Send + Sync {
    fn tile(&self, tile: Tile<T::Value>);
    fn dense(&self, dense: &T);
}

trait TreeNode: Send + Sync + Sized {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Number of childs/voxels in node
    const SIZE: usize;

    const IS_LEAF: bool; // TODO: remove

    type Value: Value;
    type Leaf: TreeNode<Value = Self::Value>;
    type Child: TreeNode<Value = Self::Value, Leaf = Self::Leaf>;
    type As<TValue: Value>: TreeNode<
        Value = TValue,
        Child = <Self::Child as TreeNode>::As<TValue>,
        Leaf = <Self::Leaf as TreeNode>::As<TValue>,
    >;

    /// Returns ref to value at grid point `index`
    fn at(&self, index: &Vec3i) -> Option<&Self::Value>;
    /// Returns mut ref value at grid point `index`
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value>;
    /// Inserts value at grid point `index`
    fn insert(&mut self, index: &Vec3i, value: Self::Value);
    /// Removes value at grid point `index`
    fn remove(&mut self, index: &Vec3i);

    /// Creates empty node
    fn empty(origin: Vec3i) -> Box<Self>;
    fn origin(&self) -> Vec3i;
    fn is_empty(&self) -> bool;
    fn fill(&mut self, value: Self::Value);
    fn clear(&mut self);
    fn visit_leafs<T: Visitor<Self::Leaf>>(&self, visitor: &mut T);
    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T);
    fn visit_values_mut<T: ValueVisitorMut<Self::Value>>(&mut self, visitor: &mut T);

    /// Returns ref to leaf at grid point `index`. Creates leaf if not exists.
    fn leaf_at(&self, index: &Vec3i) -> Option<&Self::Leaf>;

    fn take_leaf_at(&mut self, index: &Vec3i) -> Option<Box<Self::Leaf>>;
    fn insert_leaf_at(&mut self, leaf: Box<Self::Leaf>); // TODO: No need to pass index

    fn remove_if<TPred>(&mut self, pred: TPred)
    where
        TPred: Fn(&Self::Value) -> bool + Copy;
    fn remove_empty_branches(&mut self);

    ///
    /// Creates a copy of the node with same topology but with different values
    ///
    fn clone_map<TNewValue, TMap>(&self, map: &TMap) -> Box<Self::As<TNewValue>>
    where
        TNewValue: Value,
        TMap: Fn(Self::Value) -> TNewValue;

    fn clone(&self) -> Box<Self>;

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
