pub mod init;
pub mod internal_node;
pub mod leaf_node;
pub mod meshing;
pub mod root_node;
pub mod utils;

mod cached_accessor;
mod grid_value;
mod traverse;

#[cfg(test)]
mod tests;

pub use grid_value::*;
pub use internal_node::*;
pub use leaf_node::*;
pub use root_node::*;

use nalgebra::Vector3;

use self::traverse::LeafsIter;

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

pub trait GridValue: Copy + PartialEq {}

pub trait Accessor {
    type Value: GridValue; // Remove Copy?

    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value>;
    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value);
    fn remove(&mut self, index: &Vector3<isize>);
}

pub trait TreeNode: Accessor {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Number of childs/voxels in node
    const SIZE: usize;

    const IS_LEAF: bool;

    type LeafNode: TreeNode;
    type Child: TreeNode;

    /// Creates empty node
    fn empty(origin: Vector3<isize>) -> Self;
    fn origin(&self) -> Vector3<isize>;
    fn is_empty(&self) -> bool;
    fn fill(&mut self, value: Self::Value);
    fn traverse_leafs<F: FnMut(Leaf<Self::LeafNode>)>(&self, f: &mut F);

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

pub struct Tile {
    pub origin: Vector3<isize>,
    pub size: usize,
}

pub enum Leaf<'a, T: Accessor> {
    Tile(Tile),
    Dense(&'a T),
}

pub trait Traverse<TLeaf: Accessor> {
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = Child<'a, TLeaf>> + 'a>;

    #[inline]
    fn leafs(&self) -> LeafsIter<'_, TLeaf>
    where
        Self: Sized,
    {
        LeafsIter::new(self)
    }
}

pub enum Child<'a, TLeafNode: Accessor> {
    Branch(&'a dyn Traverse<TLeafNode>),
    Leaf(&'a TLeafNode),
    Tile(Tile),
}

pub trait Grid: TreeNode + Traverse<Self::LeafNode> {}

impl<T: TreeNode + Traverse<Self::LeafNode>> Grid for T {}

// https://research.dreamworks.com/wp-content/uploads/2018/08/Museth_TOG13-Edited.pdf
