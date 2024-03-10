mod csg;
mod flood_fill;
mod tree_node;

use super::*;
use crate::{data_structures::bitset::BitArray, helpers::aliases::Vec3i};

#[derive(Debug)]
pub(super) struct LeafNode<
    TValue,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    values: [TValue; SIZE],
    value_mask: BitArray<SIZE, BIT_SIZE>,
    origin: Vec3i,
}

impl<
        TValue: Value,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    #[inline]
    fn offset(index: &Vec3i) -> usize {
        let offset = ((index.x & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING + Self::BRANCHING)
            + ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING)
            + (index.z & (1 << Self::BRANCHING_TOTAL) - 1);

        offset as usize
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    1 << branching * 3
}

pub const fn leaf_node_bit_size(branching: usize) -> usize {
    let size = leaf_node_size(branching) / usize::BITS as usize;

    if size == 0 {
        1
    } else {
        size
    }
}
