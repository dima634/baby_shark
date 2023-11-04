use std::mem::MaybeUninit;

use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use super::{
    utils::{is_mask_empty, is_mask_full},
    Accessor, GridValue, Leaf, Traverse, TreeNode,
};

pub struct LeafNode<
    TValue,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    values: [TValue; SIZE],
    value_mask: BitArray<[usize; BIT_SIZE]>,
    origin: Vector3<isize>,
}

impl<
        TValue: GridValue,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Traverse<Self> for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = super::Child<'a, Self>> + 'a> {
        unimplemented!("Leaf node has no childs")
    }
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

    #[inline]
    fn offset_to_global_index(&self, offset: usize) -> Vector3<isize> {
        let x = self.origin.x + (offset >> BRANCHING_TOTAL + BRANCHING_TOTAL) as isize;
        let n = offset & ((1 << BRANCHING_TOTAL + BRANCHING_TOTAL) - 1);
        let y = self.origin.y + (n >> BRANCHING_TOTAL) as isize;
        let z = self.origin.z + (n & (1 << BRANCHING_TOTAL) - 1) as isize;

        Vector3::new(x, y, z)
    }

    #[inline]
    pub fn empty() -> Self {
        return Self {
            origin: Vector3::new(0, 0, 0),
            value_mask: Default::default(),
            values: unsafe { MaybeUninit::uninit().assume_init() }, // Safe because value mask is empty
        };
    }

    #[inline]
    pub fn origin(&self) -> &Vector3<isize> {
        return &self.origin;
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

    #[inline(always)]
    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.value_mask[offset] {
            Some(&self.values[offset])
        } else {
            None
        }
    }

    #[inline]
    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        let offset = Self::offset(index);
        self.value_mask.set(offset, true);
        self.values[offset] = value;
    }

    #[inline]
    fn remove(&mut self, index: &Vector3<isize>) {
        self.value_mask.set(Self::offset(index), false);
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
    type LeafNode = Self;

    #[inline]
    fn empty(origin: Vector3<isize>) -> Self {
        return Self {
            origin,
            value_mask: Default::default(),
            values: unsafe { MaybeUninit::uninit().assume_init() }, // Safe because value mask is empty
        };
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return is_mask_empty::<BIT_SIZE>(&self.value_mask.data);
    }

    #[inline]
    fn traverse_leafs<F: FnMut(super::Leaf<Self::LeafNode>)>(&self, f: &mut F) {
        f(Leaf::Dense(self));
    }

    #[inline]
    fn origin(&self) -> Vector3<isize> {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.value_mask.data = [usize::MAX; BIT_SIZE];
        self.values = [value; SIZE];
    }

    fn is_constant(&self, tolerance: Self::Value) -> Option<Self::Value> {
        if self.is_empty() || !is_mask_full::<BIT_SIZE>(&self.value_mask.data) {
            return None;
        }

        let first_value_offset = self.value_mask.iter().position(|v| *v)?;
        let first_value = self.values[first_value_offset];

        // Check if all values are within tolerance
        for offset in (first_value_offset + 1)..SIZE {
            if !self.value_mask[offset] {
                continue;
            }

            let value = &self.values[offset];

            if !value.is_within_tolerance(first_value, tolerance) {
                return None;
            }
        }

        Some(first_value)
    }

    #[inline]
    fn prune(&mut self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Leaf node should never be pruned")
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    1 << branching * 3
}

pub const fn leaf_node_bit_size(branching: usize) -> usize {
    leaf_node_size(branching) / usize::BITS as usize
}
