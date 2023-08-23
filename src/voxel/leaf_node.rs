use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use crate::{mesh::{traits::Mesh, polygon_soup::data_structure::PolygonSoup, builder}, geometry::traits::RealNumber};

use super::{TreeNode, utils::{is_mask_empty, box_indices}};

pub struct LeafNode<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> {
    value_mask: BitArray<[usize; SIZE]>,
    origin: Vector3<usize>
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
    #[inline]
    fn offset(index: &Vector3<usize>) -> usize {
        return
            ((index.x & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING + Self::BRANCHING) +
            ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING) + 
             (index.z & (1 << Self::BRANCHING_TOTAL) - 1);
    }

    #[inline]
    pub fn empty() -> Self {
        return Self{
            origin: Vector3::new(0, 0, 0),
            value_mask: Default::default()
        };
    }

    #[inline]
    pub fn origin(&self) -> &Vector3<usize> {
        return &self.origin;
    }

    #[inline]
    pub const fn resolution(&self) -> usize {
        return 1 << BRANCHING;
    }
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> TreeNode for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    #[inline]
    fn new_inactive(origin: Vector3<usize>) -> Self {
        return Self {
            origin,
            value_mask: Default::default()
        }
    }

    #[inline]
    fn new_active(origin: Vector3<usize>) -> Self {
        return Self {
            origin,
            value_mask: BitArray::new([usize::MAX; SIZE])
        }
    }

    #[inline]
    fn at(&self, index: &Vector3<usize>) -> bool {
        return self.value_mask[Self::offset(index)];
    }

    #[inline]
    fn insert(&mut self, index: &Vector3<usize>) {
        self.value_mask.set(Self::offset(index), true);
    }

    #[inline]
    fn remove(&mut self, index: &Vector3<usize>) {
        self.value_mask.set(Self::offset(index), false);
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return is_mask_empty::<SIZE>(&self.value_mask.data);
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    return (1 << branching * 3) / usize::BITS as usize;
}
