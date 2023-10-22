use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use crate::{
    geometry::traits::RealNumber,
    mesh::{builder, polygon_soup::data_structure::PolygonSoup, traits::Mesh},
};

use super::{
    utils::{box_indices, is_mask_empty},
    Accessor, TreeNode, Leaf, Traverse,
};

pub struct LeafNode<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> {
    value_mask: BitArray<[usize; SIZE]>,
    origin: Vector3<isize>,
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> Traverse<Self> for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = super::Child<'a, Self>> + 'a> {
        unimplemented!("Leaf node has no childs")
    }
}

// impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> Node<Self> for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE> {
//     #[inline]
//     fn childs_count(&self) -> usize {
//         unimplemented!("Leaf node has no childs")
//     }

//     // fn child(&self, index: usize) -> Leaf<Self> {
//     //     unimplemented!("Leaf node has no childs")
//     // }

//     #[inline]
//     fn is_leaf(&self) -> bool {
//         true
//     }

//     #[inline]
//     fn next(&self, _: &Vector3<isize>) -> Option<&dyn Node<Self>> {
//         unreachable!("Leaf node has no childs")
//     }

//     #[inline]
//     fn total_branching(&self) -> usize {
//         Self::BRANCHING_TOTAL
//     }

//     #[inline]
//     fn at_if_leaf(&self, index: &Vector3<isize>) -> Option<bool> {
//         Some(self.at(index))
//     }
// }

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
}

impl<const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize> TreeNode
    for LeafNode<BRANCHING, BRANCHING_TOTAL, SIZE>
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    const IS_LEAF: bool = true;

    type LeafNode = Self;

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

    #[inline]
    fn traverse_leafs<F: FnMut(super::Leaf<Self::LeafNode>)>(&self, f: &mut F) {
        f(Leaf::Node(self));
    }

    #[inline]
    fn origin(&self) -> Vector3<isize> {
        self.origin
    }
}

pub const fn leaf_node_size(branching: usize) -> usize {
    return (1 << branching * 3) / usize::BITS as usize;
}
