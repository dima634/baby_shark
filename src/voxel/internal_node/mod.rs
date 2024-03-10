mod csg;
mod flood_fill;
mod tree_node;

use super::*;
use crate::{
    data_structures::bitset::BitArray,
    helpers::aliases::{Vec3i, Vec3u},
};
use std::alloc::Layout;

#[derive(Debug)]
pub(super) struct InternalNode<
    TValue,
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
    childs: [Option<Box<TChild>>; SIZE],
    values: [TValue; SIZE],
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

    fn add_child(&mut self, offset: usize) {
        debug_assert!(!self.child_mask.at(offset));

        self.child_mask.on(offset);
        self.value_mask.off(offset);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = TChild::empty(child_origin);
        self.childs[offset] = Some(child_node);
    }

    #[inline]
    fn remove_child_node(&mut self, offset: usize) {
        debug_assert!(self.child_mask.at(offset));

        self.child_mask.off(offset);
        self.childs[offset] = None;
    }

    #[inline]
    fn child_owned(&mut self, offset: usize) -> Box<TChild> {
        debug_assert!(self.child_mask.at(offset));

        self.child_mask.off(offset);
        unsafe { self.childs[offset].take().unwrap_unchecked() }
    }

    #[inline]
    fn replace_node_with_tile(&mut self, offset: usize, value: TChild::Value) {
        debug_assert!(self.child_mask.at(offset));

        self.remove_child_node(offset);
        self.value_mask.on(offset);
        self.values[offset] = value;
    }

    #[inline]
    fn child_node(&self, offset: usize) -> &TChild {
        let child = &self.childs[offset];
        debug_assert!(self.child_mask.at(offset));
        debug_assert!(child.is_some());

        unsafe { child.as_ref().unwrap_unchecked() }
    }

    #[inline]
    fn child_node_mut(&mut self, offset: usize) -> &mut TChild {
        let child = &mut self.childs[offset];
        debug_assert!(self.child_mask.at(offset));
        debug_assert!(child.is_some());

        unsafe { child.as_mut().unwrap_unchecked() }
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

        let child_ptr = (*ptr).childs.as_mut_ptr();
        for i in 0..SIZE {
            child_ptr.add(i).write(None);
        }

        // (*ptr).value_mask - no need to initialize because value mask is empty

        Box::from_raw(ptr)
    }
}

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
