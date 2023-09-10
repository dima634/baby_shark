use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use crate::{
    geometry::traits::RealNumber,
    mesh::{builder, polygon_soup::data_structure::PolygonSoup, traits::Mesh},
};

use super::{
    utils::{box_indices, is_mask_empty},
    Accessor, TreeNode,
};

pub struct LeafNode<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> {
    value_mask: BitArray<[usize; SIZE]>,
    origin: Vector3<isize>,
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize>
    LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE>
{
    #[inline]
    fn offset(index: &Vector3<isize>) -> usize {
        let offset =
            ((index.x & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING + Self::BRANCHING) +
            ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING) + 
             (index.z & (1 << Self::BRANCHING_TOTAL) - 1);

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
        };
    }

    #[inline]
    pub fn origin(&self) -> &Vector3<isize> {
        return &self.origin;
    }

    #[inline]
    pub const fn resolution(&self) -> usize {
        return 1 << BRANCHING;
    }
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> Accessor
    for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE>
{
    #[inline]
    fn at(&self, index: &Vector3<isize>) -> bool {
        return self.value_mask[Self::offset(index)];
    }

    #[inline]
    fn insert(&mut self, index: &Vector3<isize>) {
        self.value_mask.set(Self::offset(index), true);
    }

    #[inline]
    fn remove(&mut self, index: &Vector3<isize>) {
        self.value_mask.set(Self::offset(index), false);
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

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> TreeNode
    for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE>
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    #[inline]
    fn new_inactive(origin: Vector3<isize>) -> Self {
        return Self {
            origin,
            value_mask: Default::default(),
        };
    }

    #[inline]
    fn new_active(origin: Vector3<isize>) -> Self {
        return Self {
            origin,
            value_mask: BitArray::new([usize::MAX; SIZE]),
        };
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return is_mask_empty::<SIZE>(&self.value_mask.data);
    }

    fn voxels<F: FnMut(Vector3<isize>)>(&self, mut f: F) {
        for offset in 0..SIZE {
            if self.value_mask[offset] {
                f(self.offset_to_global_index(offset));
            }
        }
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    return (1 << branching * 3) / usize::BITS as usize;
}
