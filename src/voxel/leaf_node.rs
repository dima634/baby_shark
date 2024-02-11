use std::mem::MaybeUninit;

use nalgebra::Vector3;

use crate::data_structures::bitset::BitSet;

use super::{Accessor, GridValue, ParVisitor, TreeNode};

#[derive(Debug)]
pub(super) struct LeafNode<
    TValue,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    values: [TValue; SIZE],
    value_mask: BitSet<SIZE, BIT_SIZE>,
    origin: Vector3<isize>,
}

impl<
        TValue: GridValue,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    #[inline]
    fn offset(index: &Vector3<isize>) -> usize {
        let offset = ((index.x & (1 << Self::BRANCHING_TOTAL) - 1)
            << Self::BRANCHING + Self::BRANCHING)
            + ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING)
            + (index.z & (1 << Self::BRANCHING_TOTAL) - 1);

        offset as usize
    }
}

impl<
        TValue: GridValue,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Accessor for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    type Value = TValue;

    #[inline]
    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.value_mask.at(offset) {
            Some(&self.values[offset])
        } else {
            None
        }
    }

    #[inline]
    fn at_mut(&mut self, index: &Vector3<isize>) -> Option<&mut Self::Value> {
        let offset = Self::offset(index);

        if self.value_mask.at(offset) {
            Some(&mut self.values[offset])
        } else {
            None
        }
    }

    #[inline]
    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        let offset = Self::offset(index);
        self.value_mask.on(offset);
        self.values[offset] = value;
    }

    #[inline]
    fn remove(&mut self, index: &Vector3<isize>) {
        self.value_mask.off(Self::offset(index));
    }
}

impl<
        TValue: GridValue,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > TreeNode for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = BIT_SIZE;

    const IS_LEAF: bool = true;

    type Child = Self;
    type Leaf = Self;
    type As<TNewValue: GridValue> = LeafNode<TNewValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>;

    #[inline]
    fn empty(origin: Vector3<isize>) -> Box<Self> {
        Box::new(Self {
            origin,
            value_mask: BitSet::zeroes(),
            values: unsafe { MaybeUninit::uninit().assume_init() }, // Safe because value mask is empty
        })
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.value_mask.is_empty()
    }

    #[inline]
    fn origin(&self) -> Vector3<isize> {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.value_mask = BitSet::ones();
        self.values = [value; SIZE];
    }

    fn is_constant(&self, tolerance: Self::Value) -> Option<Self::Value> {
        if self.is_empty() || !self.value_mask.is_full() {
            return None;
        }

        let first_value_offset = self.value_mask.iter().position(|v| v)?;
        let first_value = self.values[first_value_offset];

        // Check if all values are within tolerance
        for offset in (first_value_offset + 1)..SIZE {
            if !self.value_mask.at(offset) {
                continue;
            }

            let value = self.values[offset];

            if (value - first_value) > tolerance {
                return None;
            }
        }

        Some(first_value)
    }

    #[inline]
    fn prune(&mut self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Leaf node should never be pruned")
    }

    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where
        TNewValue: GridValue,
        TCast: Fn(Self::Value) -> TNewValue,
    {
        let mut new_node = LeafNode {
            origin: self.origin,
            value_mask: self.value_mask,
            values: unsafe { MaybeUninit::uninit().assume_init() }, // We  will f
        };

        for i in 0..SIZE {
            if !self.value_mask.at(i) {
                continue;
            }

            new_node.values[i] = cast(self.values[i]);
        }

        new_node
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        visitor.dense(self);
    }

    #[inline]
    fn visit_leafs<T: super::Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        visitor.dense(self);
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    1 << branching * 3
}

pub const fn leaf_node_bit_size(branching: usize) -> usize {
    leaf_node_size(branching) / usize::BITS as usize
}
