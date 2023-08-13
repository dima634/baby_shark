use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use super::traits::{TreeNode, HasChild};

pub struct InternalNode<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> {
    childs: [Option<Box<TChild>>; SIZE],
    child_mask: BitArray<[u8; BIT_SIZE]>,
    value_mask: BitArray<[u8; BIT_SIZE]>,
    origin: Vector3<usize>
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
    #[inline]
    fn offset(index: Vector3<usize>) -> usize {
        return 
            (((index.x & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << (Self::BRANCHING + Self::BRANCHING))+
            (((index.y & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << Self::BRANCHING) +
             ((index.z & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL);
    }
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> HasChild for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
    type Child = TChild;
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> TreeNode for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    fn at(&self, index: Vector3<usize>) -> bool {
        let offset = Self::offset(index);

        debug_assert!(!(self.value_mask[offset] & self.child_mask[offset]), "Node should has either value or child");

        if self.child_mask[offset] {
            let child = &self.childs[offset];
            return child.as_ref().unwrap().at(index);
        }

        return self.value_mask[offset];
    }

    fn insert(&mut self, index: Vector3<usize>) {
        todo!()
    }

    fn remove(&mut self, index: Vector3<usize>) {
        todo!()
    }
 
    #[inline]
    fn is_empty(&self) -> bool {
        return self.childs.iter()
            .filter(|child| child.is_some())
            .all(|child| child.as_ref().unwrap().is_empty());
    }
}

pub const fn internal_node_size<T: TreeNode>(branching: usize) -> usize {
    return 1 << branching * 3;
}

pub const fn internal_node_bit_size<T: TreeNode>(branching: usize) -> usize {
    return internal_node_size::<T>(branching) / std::mem::size_of::<u8>();
}

pub const fn internal_node_branching<T: TreeNode>(branching: usize) -> usize {
    return branching + T::BRANCHING_TOTAL;
}
