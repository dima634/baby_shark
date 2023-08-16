use std::mem::MaybeUninit;

use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use super::{TreeNode, HasChild};

pub struct InternalNode<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> {
    childs: [Option<Box<TChild>>; SIZE],
    child_mask: BitArray<[u8; BIT_SIZE]>,
    value_mask: BitArray<[u8; BIT_SIZE]>,
    origin: Vector3<usize>
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
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

    fn offset_to_global_index(&self, offset: usize) -> Vector3<usize> {
        let mut local = Self::offset_to_local_index(offset);

        for i in 0..3 {
            local[i] <<= TChild::BRANCHING_TOTAL;
        }

        return local + self.origin;
    }

    #[inline]
    fn add_child(&mut self, offset: usize) {
        debug_assert!(!self.child_mask[offset]);
        
        // Do we really need this?
        // self.value_mask.set(offset, false);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = TChild::new(child_origin);
        let child_box = Box::new(child_node);
        
        self.child_mask.set(offset, true);
        self.childs[offset] = Some(child_box);
    }

    #[inline]
    fn child_mut(&mut self, offset: usize) -> &mut TChild {
        assert!(self.child_mask[offset]);

        let child = &mut self.childs[offset];
        return child.as_mut().unwrap();
    }
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> HasChild for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
    type Child = TChild;
}

impl<TChild: TreeNode, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> TreeNode for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> {
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    #[inline]
    fn new(origin: Vector3<usize>) -> Self {
        let childs = unsafe { MaybeUninit::uninit().assume_init() };
        return Self {
            origin,
            childs,
            value_mask: Default::default(),
            child_mask: Default::default()
        };
    }

    fn at(&self, index: &Vector3<usize>) -> bool {
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            let child = &self.childs[offset];
            return child.as_ref().unwrap().at(index);
        }

        return self.value_mask[offset];
    }

    fn insert(&mut self, index: &Vector3<usize>) {
        let offset = Self::offset(index);

        if !self.child_mask[offset] {
            self.add_child(offset);
        }

        let child = self.child_mut(offset);
        child.insert(index);
    }

    fn remove(&mut self, index: &Vector3<usize>) {
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
