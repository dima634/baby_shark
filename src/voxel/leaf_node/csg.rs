use super::*;
use crate::voxel::utils::{partial_max, partial_min};
use std::ops::Neg;

///
/// CSG operations for leaf nodes
/// This implementation assuming that nodes are flood filler prior to CSG operations
///
impl<
        TValue: Signed + Neg<Output = Self::Value>,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Csg for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    fn union(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_min(self.values[i], other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn subtract(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_max(self.values[i], -other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn intersect(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_max(self.values[i], other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn flip_signs(&mut self) {
        for i in 0..SIZE {
            self.values[i] = -self.values[i];
        }
    }
}
