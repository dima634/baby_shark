use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use crate::mesh::polygon_soup::data_structure::PolygonSoup;

use super::{
    utils::{box_indices, is_mask_empty},
    Accessor, HasChild, TreeNode,
};

pub struct InternalNode<
    TChild: TreeNode,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    childs: Vec<Option<TChild>>,
    child_mask: BitArray<[usize; BIT_SIZE]>,
    value_mask: BitArray<[usize; BIT_SIZE]>,
    origin: Vector3<usize>,
}

impl<
        TChild: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    #[inline]
    pub fn new() -> Self {
        return Self::new_inactive(Vector3::zeros());
    }

    /// Returns grid resolution in one dimension
    pub const fn resolution(&self) -> usize {
        return 1 << BRANCHING_TOTAL;
    }

    fn offset_to_global_index(&self, offset: usize) -> Vector3<usize> {
        let mut local = Self::offset_to_local_index(offset);

        for i in 0..3 {
            local[i] <<= TChild::BRANCHING_TOTAL;
        }

        return local + self.origin;
    }

    fn add_child(&mut self, offset: usize, active: bool) {
        debug_assert!(!self.child_mask[offset]);

        // Do we really need this?
        // self.value_mask.set(offset, false);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = if active {
            TChild::new_active(child_origin)
        } else {
            TChild::new_inactive(child_origin)
        };

        self.child_mask.set(offset, true);
        self.childs[offset] = Some(child_node);
    }

    #[inline]
    fn remove_child(&mut self, offset: usize) {
        debug_assert!(self.child_mask[offset]);

        // Do we really need this?
        // self.value_mask.set(offset, false);

        self.child_mask.set(offset, false);
        self.childs[offset] = None;
    }

    #[inline]
    fn child_mut(&mut self, offset: usize) -> &mut TChild {
        debug_assert!(self.child_mask[offset]);

        let child = &mut self.childs[offset];
        return child.as_mut().unwrap();
    }

    #[inline]
    fn offset(index: &Vector3<usize>) -> usize {
        return 
            (((index.x & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << (Self::BRANCHING + Self::BRANCHING))+
            (((index.y & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << Self::BRANCHING) +
             ((index.z & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL);
    }

    fn offset_to_local_index(mut offset: usize) -> Vector3<usize> {
        debug_assert!(offset < (1 << 3 * BRANCHING));
        let x = offset >> 2 * BRANCHING;
        offset &= (1 << 2 * BRANCHING) - 1;
        let y = offset >> BRANCHING;
        let z = offset & ((1 << BRANCHING) - 1);

        return Vector3::new(x, y, z);
    }
}

impl<
        TChild: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > HasChild for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    type Child = TChild;
}

impl<
        TChild: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Accessor for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    fn at(&self, index: &Vector3<usize>) -> bool {
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            let child = &self.childs[offset];
            return child.as_ref().unwrap().at(index);
        }

        return self.value_mask[offset];
    }

    fn insert(&mut self, index: &Vector3<usize>) {
        // Node is branch - insert voxel
        // Node is tile:
        //   if tile is active - do nothing
        //   else - add empty child and insert voxel
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            let child = self.child_mut(offset);
            child.insert(index);
        } else if !self.value_mask[offset] {
            self.add_child(offset, false);
            let child = self.child_mut(offset);
            child.insert(index);
        }
    }

    fn remove(&mut self, index: &Vector3<usize>) {
        // Node is branch - remove voxel from child, prune child if empty
        // Node is tile:
        //   active - add active child, remove voxel
        //   inactive - do nothing
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            let child = self.child_mut(offset);
            child.remove(index);

            if child.is_empty() {
                self.remove_child(offset);
                self.value_mask.set(offset, false);
            }
        } else if self.value_mask[offset] {
            self.add_child(offset, true);
            let child = self.child_mut(offset);
            child.remove(index);
        }
    }

    #[inline]
    fn index_key(&self, index: &Vector3<usize>) -> Vector3<usize> {
        Vector3::new(
            index.x & !((1 << Self::BRANCHING_TOTAL) - 1),
            index.y & !((1 << Self::BRANCHING_TOTAL) - 1),
            index.z & !((1 << Self::BRANCHING_TOTAL) - 1),
        )
    }
}

impl<
        TChild: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > TreeNode for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    fn new_inactive(origin: Vector3<usize>) -> Self {
        let mut childs = Vec::new();
        childs.resize_with(SIZE, || None);

        return Self {
            origin,
            childs,
            value_mask: Default::default(),
            child_mask: Default::default(),
        };
    }

    fn new_active(origin: Vector3<usize>) -> Self {
        let mut childs = Vec::new();
        childs.resize_with(SIZE, || None);

        return Self {
            origin,
            childs,
            value_mask: BitArray::new([usize::MAX; BIT_SIZE]),
            child_mask: Default::default(),
        };
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return is_mask_empty::<BIT_SIZE>(&self.child_mask.data)
            && is_mask_empty::<BIT_SIZE>(&self.value_mask.data);
    }
}

pub const fn internal_node_size<T: TreeNode>(branching: usize) -> usize {
    return 1 << branching * 3;
}

pub const fn internal_node_bit_size<T: TreeNode>(branching: usize) -> usize {
    return internal_node_size::<T>(branching) / usize::BITS as usize;
}

pub const fn internal_node_branching<T: TreeNode>(branching: usize) -> usize {
    return branching + T::BRANCHING_TOTAL;
}
