mod csg;
mod flood_fill;
mod tree_node;

use super::*;
use crate::{
    data_structures::bitset::*,
    helpers::{
        aliases::{Vec3i, Vec3u},
        one_of::OneOf,
    },
};
use std::{alloc::Layout, fmt::Debug, mem::ManuallyDrop};

#[derive(Debug)]
pub(super) struct InternalNode<
    TValue: Value,
    TChild: TreeNode,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
    const PARALLEL: bool,
> {
    origin: Vec3i,
    child_mask: BitArray<SIZE, BIT_SIZE>,
    value_mask: BitArray<SIZE, BIT_SIZE>,
    childs: [ChildUnion<TValue, TChild>; SIZE], // `child_mask` and `value_mask` are discriminating between branch and tile
}

impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: TreeNode,
{
    fn offset_to_global_index(&self, offset: usize) -> Vec3i {
        let mut local = Self::offset_to_local_index(offset);

        for i in 0..3 {
            local[i] <<= TChild::BRANCHING_TOTAL;
        }

        local.cast() + self.origin
    }

    fn add_branch(&mut self, offset: usize) -> &mut TChild {
        if self.child_mask.is_on(offset) {
            return unsafe { &mut self.childs[offset].branch };
        }

        self.child_mask.on(offset);
        self.value_mask.off(offset);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = TChild::empty(child_origin);
        self.childs[offset] = ChildUnion {
            branch: ManuallyDrop::new(child_node),
        };

        unsafe { &mut self.childs[offset].branch }
    }

    #[inline]
    fn remove_branch(&mut self, offset: usize) -> Option<Box<TChild>> {
        if self.child_mask.is_off(offset) {
            return None;
        }

        self.child_mask.off(offset);
        let child = unsafe { ManuallyDrop::take(&mut self.childs[offset].branch) };
        Some(child)
    }

    #[inline]
    fn remove_child(&mut self, offset: usize) -> Option<ChildOwned<TChild>> {
        if self.child_mask.is_on(offset) {
            self.child_mask.off(offset);
            let branch = unsafe { ManuallyDrop::take(&mut self.childs[offset].branch) };
            Some(ChildOwned::T1(branch))
        } else if self.value_mask.is_on(offset) {
            self.value_mask.off(offset);
            let value = unsafe { self.childs[offset].tile };
            Some(ChildOwned::T2(value))
        } else {
            None
        }
    }

    #[inline]
    fn child(&self, offset: usize) -> Option<Child<TChild>> {
        let child = &self.childs[offset];

        unsafe {
            if self.child_mask.is_on(offset) {
                return Some(OneOf::T1(child.branch.as_ref()));
            } else if self.value_mask.is_on(offset) {
                return Some(OneOf::T2(&child.tile));
            }
        }

        None
    }

    #[inline]
    fn child_mut(&mut self, offset: usize) -> Option<ChildMut<TChild>> {
        let child = &mut self.childs[offset];

        unsafe {
            if self.child_mask.is_on(offset) {
                return Some(OneOf::T1(child.branch.as_mut()));
            } else if self.value_mask.is_on(offset) {
                return Some(OneOf::T2(&mut child.tile));
            }
        }

        None
    }

    fn childs(&self) -> impl Iterator<Item = (usize, Child<TChild>)> + '_ {
        (0..SIZE).filter_map(|offset| self.child(offset).map(|child| (offset, child)))
    }

    #[inline]
    fn offset(index: &Vec3i) -> usize {
        let offset = (((index.x & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL) << (Self::BRANCHING + Self::BRANCHING))
            + (((index.y & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL) << Self::BRANCHING)
            + ((index.z & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL);

        offset as usize
    }

    #[inline]
    fn offset_to_local_index(mut offset: usize) -> Vec3u {
        debug_assert!(offset < (1 << 3 * BRANCHING));

        let x = offset >> 2 * BRANCHING;
        offset &= (1 << 2 * BRANCHING) - 1;
        let y = offset >> BRANCHING;
        let z = offset & ((1 << BRANCHING) - 1);

        Vec3u::new(x, y, z)
    }

    const fn childs_per_dim() -> usize {
        1 << BRANCHING
    }

    ///
    /// Allocates memory for the node on the heap and initializes it with default values.
    ///
    unsafe fn alloc_on_heap(origin: Vec3i) -> Box<Self> {
        let layout = Layout::new::<Self>();
        let ptr = std::alloc::alloc(layout) as *mut Self;

        if ptr.is_null() {
            std::alloc::handle_alloc_error(layout);
        }

        (*ptr).origin = origin;
        (*ptr).child_mask.off_all();
        (*ptr).value_mask.off_all();

        Box::from_raw(ptr)
    }
}

pub type Child<'node, T> = OneOf<&'node T, &'node <T as TreeNode>::Value>;
pub type ChildMut<'node, T> = OneOf<&'node mut T, &'node mut <T as TreeNode>::Value>;
pub type ChildOwned<T> = OneOf<Box<T>, <T as TreeNode>::Value>;

pub const fn internal_node_size(branching: usize) -> usize {
    1 << branching * 3
}

pub const fn internal_node_bit_size(branching: usize) -> usize {
    let size = internal_node_size(branching) / usize::BITS as usize;

    if size == 0 {
        1
    } else {
        size
    }
}

pub const fn internal_node_branching<TChild: TreeNode>(branching: usize) -> usize {
    branching + TChild::BRANCHING_TOTAL
}

union ChildUnion<TValue: Value, TChild: TreeNode> {
    branch: ManuallyDrop<Box<TChild>>,
    tile: TValue,
}

impl<TValue: Value, TChild: TreeNode> Debug for ChildUnion<TValue, TChild> {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("unknown")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alloc_internal_node() {
        const LEAF_LOG2: usize = 2;
        const INTERNAL_LOG2: usize = 3;

        type Leaf = LeafNode<
            f32,
            LEAF_LOG2,
            LEAF_LOG2,
            { leaf_node_size(LEAF_LOG2) },
            { leaf_node_bit_size(LEAF_LOG2) },
        >;
        type Internal = InternalNode<
            f32,
            Leaf,
            INTERNAL_LOG2,
            { LEAF_LOG2 + INTERNAL_LOG2 },
            { internal_node_size(INTERNAL_LOG2) },
            { internal_node_bit_size(INTERNAL_LOG2) },
            false,
        >;

        let origin = Vec3i::new(1, 2, 3);
        let node = Internal::empty(origin);
        assert_eq!(node.child_mask, BitArray::zeroes());
        assert_eq!(node.value_mask, BitArray::zeroes());
        assert_eq!(node.origin, origin);
    }
}
