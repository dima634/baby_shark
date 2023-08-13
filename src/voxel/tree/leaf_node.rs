use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use super::traits::TreeNode;

pub struct LeafNode<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> {
    pub value_mask: BitArray<[u8; SIZE]>,
    origin: Vector3<usize>
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
    #[inline]
    fn offset(index: Vector3<usize>) -> usize {
        return
            ((index.x & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING + Self::BRANCHING) +
            ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING) + 
             (index.z & (1 << Self::BRANCHING_TOTAL) - 1);
    }

    pub fn new() -> Self {
        return Self{
            origin: Vector3::new(0, 0, 0),
            value_mask: Default::default()
        };
    }
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> TreeNode for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    #[inline]
    fn at(&self, index: Vector3<usize>) -> bool {
        return self.value_mask[Self::offset(index)];
    }

    fn insert(&mut self, index: Vector3<usize>) {
        self.value_mask.set(Self::offset(index), true);
    }

    fn remove(&mut self, index: Vector3<usize>) {
        self.value_mask.set(Self::offset(index), false);
    }

    fn is_empty(&self) -> bool {
        let mut value = 0;

        for i in 0..SIZE {
            value |= self.value_mask.data[i];
        }

        return value == 0;
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    return (1 << branching * 3) / std::mem::size_of::<u8>();
}
