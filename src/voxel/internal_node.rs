use std::{alloc::Layout, mem::MaybeUninit};

use nalgebra::Vector3;

use crate::data_structures::bitset::BitSet;

use super::{Accessor, FloodFill, GridValue, ParVisitor, Signed, Tile, TreeNode};

#[derive(Debug)]
pub(super) struct InternalNode<
    TValue,
    TChild: TreeNode,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
    const PARALLEL: bool,
> {
    origin: Vector3<isize>,
    child_mask: BitSet<SIZE, BIT_SIZE>,
    value_mask: BitSet<SIZE, BIT_SIZE>,
    childs: [Option<Box<TChild>>; SIZE],
    values: [TValue; SIZE],
}

impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: TreeNode,
{
    fn offset_to_global_index(&self, offset: usize) -> Vector3<isize> {
        let mut local = Self::offset_to_local_index(offset);

        for i in 0..3 {
            local[i] <<= TChild::BRANCHING_TOTAL;
        }

        return local.cast() + self.origin;
    }

    fn add_child(&mut self, offset: usize) {
        debug_assert!(!self.child_mask.at(offset));

        self.child_mask.on(offset);
        self.value_mask.off(offset);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = TChild::empty(child_origin);
        self.childs[offset] = Some(child_node);
    }

    #[inline]
    fn remove_child(&mut self, offset: usize) {
        debug_assert!(self.child_mask.at(offset));

        self.child_mask.off(offset);
        self.childs[offset] = None;
    }

    #[inline]
    fn replace_child_with_tile(&mut self, offset: usize, value: TChild::Value) {
        debug_assert!(self.child_mask.at(offset));

        self.remove_child(offset);
        self.value_mask.on(offset);
        self.values[offset] = value;
    }

    #[inline]
    fn child(&self, offset: usize) -> &TChild {
        let child = &self.childs[offset];
        debug_assert!(self.child_mask.at(offset));
        debug_assert!(child.is_some());

        return unsafe { child.as_ref().unwrap_unchecked() };
    }

    #[inline]
    fn child_mut(&mut self, offset: usize) -> &mut TChild {
        let child = &mut self.childs[offset];
        debug_assert!(self.child_mask.at(offset));
        debug_assert!(child.is_some());

        return unsafe { child.as_mut().unwrap_unchecked() };
    }

    #[inline]
    fn offset(index: &Vector3<isize>) -> usize {
        let offset =
            (((index.x & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL) << (Self::BRANCHING + Self::BRANCHING))+
            (((index.y & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL) << Self::BRANCHING) +
             ((index.z & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as TreeNode>::Child::BRANCHING_TOTAL);

        offset as usize
    }

    #[inline]
    fn offset_to_local_index(mut offset: usize) -> Vector3<usize> {
        debug_assert!(offset < (1 << 3 * BRANCHING));

        let x = offset >> 2 * BRANCHING;
        offset &= (1 << 2 * BRANCHING) - 1;
        let y = offset >> BRANCHING;
        let z = offset & ((1 << BRANCHING) - 1);

        return Vector3::new(x, y, z);
    }

    const fn childs_per_dim() -> usize {
        1 << BRANCHING
    }

    ///
    /// Allocates memory for the node on the heap and initializes it with default values.
    ///
    unsafe fn alloc_on_heap(origin: Vector3<isize>) -> Box<Self> {
        let layout = Layout::new::<Self>();
        let ptr = std::alloc::alloc(layout) as *mut Self;

        if ptr.is_null() {
            std::alloc::handle_alloc_error(layout);
        }

        (*ptr).origin = origin;
        (*ptr).child_mask.off_all();
        (*ptr).value_mask.off_all();

        let child_ptr = (*ptr).childs.as_mut_ptr();
        for i in 0..SIZE {
            child_ptr.add(i).write(None);
        }

        // (*ptr).value_mask - no need to initialize because value mask is empty

        Box::from_raw(ptr)
    }
}

impl<
        TChild: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > Accessor
    for InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: TreeNode,
{
    type Value = TChild::Value;

    #[inline]
    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            return self.child(offset).at(index);
        }

        if !self.value_mask.at(offset) {
            return None;
        }

        Some(&self.values[offset])
    }

    #[inline]
    fn at_mut(&mut self, index: &Vector3<isize>) -> Option<&mut Self::Value> {
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            return self.child_mut(offset).at_mut(index);
        }

        if !self.value_mask.at(offset) {
            return None;
        }

        Some(&mut self.values[offset])
    }

    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        // Node is branch - insert voxel
        // Node is tile:
        //   if tile is active - convert to branch, fill with tile value and insert voxel
        //   else - add empty child and insert voxel
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            let child = self.child_mut(offset);
            child.insert(index, value);
            return;
        }

        if self.value_mask.at(offset) {
            let tile_value = self.values[offset];

            // No need to add child if tile value is equal to inserted one
            if tile_value == value {
                return;
            }

            self.add_child(offset);
            let child = self.child_mut(offset);
            child.fill(tile_value);
            child.insert(index, value);
        } else {
            self.add_child(offset);
            let child = self.child_mut(offset);
            child.insert(index, value);
        }
    }

    fn remove(&mut self, index: &Vector3<isize>) {
        // Node is branch - remove voxel from child, prune child if empty
        // Node is tile:
        //   active - add active child, fill with tile value and remove voxel
        //   inactive - do nothing
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            let child = self.child_mut(offset);
            child.remove(index);

            if child.is_empty() {
                self.remove_child(offset);
                self.value_mask.off(offset); // Remove?
            }
        } else if self.value_mask.at(offset) {
            let tile_value = self.values[offset];
            self.add_child(offset);
            let child = self.child_mut(offset);
            child.fill(tile_value);
            child.remove(index);
        }
    }
}

impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > TreeNode
    for InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: TreeNode,
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    const IS_LEAF: bool = false;

    type Child = TChild;
    type Leaf = TChild::Leaf;
    type As<TNewValue: GridValue> = InternalNode<
        TNewValue,
        TChild::As<TNewValue>,
        BRANCHING,
        BRANCHING_TOTAL,
        SIZE,
        BIT_SIZE,
        PARALLEL,
    >;

    fn empty(origin: Vector3<isize>) -> Box<Self> {
        // Allocate directly on a heap, otherwise we will overflow the stack with large grids
        unsafe { Self::alloc_on_heap(origin) }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return self.child_mask.is_empty() && self.value_mask.is_empty();
    }

    #[inline]
    fn origin(&self) -> Vector3<isize> {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.child_mask = BitSet::zeroes();
        self.value_mask = BitSet::ones();
        self.values = [value; SIZE];
    }

    fn clear(&mut self) {
        self.child_mask.off_all();
        self.value_mask.off_all();
        self.childs.fill_with(|| None);
    }

    fn is_constant(&self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Internal node should never be constant");
    }

    fn prune(&mut self, tolerance: Self::Value) -> Option<Self::Value> {
        if self.is_empty() {
            return None;
        }

        for offset in 0..SIZE {
            if !self.child_mask.at(offset) {
                continue;
            }

            let child = self.child_mut(offset);

            if child.is_empty() {
                self.remove_child(offset);
                continue;
            }

            let pruned = if TChild::IS_LEAF {
                child.is_constant(tolerance)
            } else {
                child.prune(tolerance)
            };

            if let Some(value) = pruned {
                self.replace_child_with_tile(offset, value);
            }
        }

        if !self.value_mask.is_full() {
            return None;
        }

        let first_value = self.values[0];
        let is_constant = self
            .values
            .iter()
            .skip(1)
            .all(|value| (*value - first_value) <= tolerance);

        if is_constant {
            return Some(first_value);
        }

        None
    }

    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where
        TNewValue: super::GridValue,
        TCast: Fn(Self::Value) -> TNewValue,
    {
        let mut new_node = InternalNode {
            origin: self.origin,
            childs: std::array::from_fn(|_| None),
            child_mask: self.child_mask,
            value_mask: self.value_mask,
            values: unsafe { MaybeUninit::uninit().assume_init() }, // We  will fill it later
        };

        for i in 0..SIZE {
            if self.child_mask.at(i) {
                let child = self.child(i);
                new_node.childs[i] = Some(Box::new(child.cast(cast)));
            } else if self.value_mask.at(i) {
                new_node.values[i] = cast(self.values[i]);
            }
        }

        new_node
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        use rayon::prelude::*;

        if PARALLEL {
            (0..SIZE)
                .filter_map(|i| match self.child_mask.at(i) {
                    true => Some(self.child(i)),
                    false => None,
                })
                .par_bridge()
                .into_par_iter()
                .for_each(|c| c.visit_leafs_par(visitor));

            (0..SIZE).filter(|i| self.value_mask.at(*i)).for_each(|i| {
                let tile = Tile {
                    origin: self.offset_to_global_index(i),
                    size: TChild::resolution(),
                    value: self.values[i],
                };

                visitor.tile(tile);
            });
        } else {
            for i in 0..SIZE {
                if self.child_mask.at(i) {
                    let child = self.child(i);
                    child.visit_leafs_par(visitor);
                } else if self.value_mask.at(i) {
                    let tile = Tile {
                        origin: self.offset_to_global_index(i),
                        size: TChild::resolution(),
                        value: self.values[i],
                    };

                    visitor.tile(tile);
                }
            }
        }
    }

    fn visit_leafs<T: super::Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        for i in 0..SIZE {
            if self.child_mask.at(i) {
                let child = self.child(i);
                child.visit_leafs(visitor);
            } else if self.value_mask.at(i) {
                let tile = Tile {
                    origin: self.offset_to_global_index(i),
                    size: TChild::resolution(),
                    value: self.values[i],
                };

                visitor.tile(tile);
            }
        }
    }
}

impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > FloodFill
    for InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: TreeNode + FloodFill,
    TChild::Value: Signed,
{
    fn flood_fill(&mut self) {
        self.childs
            .iter_mut()
            .filter_map(|c| c.as_mut())
            .for_each(|c| c.flood_fill());

        let first_value = self.value_mask.find_first_on();
        let first_child = self.child_mask.find_first_on();

        let mut i = match (first_value, first_child) {
            (Some(v), Some(c)) if v <= c => self.values[v].sign(),
            (Some(_), Some(c)) => self.child(c).first_value_sign(),
            (Some(i), None) => self.values[i].sign(),
            (None, Some(i)) => self.child(i).first_value_sign(),
            (None, None) => return,
        };

        for x in 0..Self::childs_per_dim() {
            let x00 = x << (BRANCHING + BRANCHING); // offset for block(x, 0, 0)

            if self.child_mask.is_on(x00) {
                i = self.child(x00).last_value_sign();
            } else if self.value_mask.is_on(x00) {
                i = self.values[x00].sign();
            }

            let mut j = i;
            for y in 0..Self::childs_per_dim() {
                let xy0 = x00 + (y << BRANCHING); // offset for block(x, y, 0)

                if self.child_mask.is_on(xy0) {
                    i = self.child(xy0).last_value_sign();
                } else if self.value_mask.is_on(xy0) {
                    j = self.values[xy0].sign();
                }

                let mut k = j;
                for z in 0..Self::childs_per_dim() {
                    let xyz = xy0 + z; // offset for block(x, y, z)

                    if self.child_mask.is_on(xyz) {
                        k = self.child(xyz).last_value_sign();
                    } else if self.value_mask.is_on(xyz) {
                        k = self.values[xyz].sign();
                    } else {
                        self.values[xyz] = Self::Value::far();
                        self.values[xyz].set_sign(k);
                    }
                }
            }
        }

        self.value_mask = !self.child_mask;
    }

    #[inline]
    fn first_value_sign(&self) -> super::ValueSign {
        self.values[0].sign()
    }

    #[inline]
    fn last_value_sign(&self) -> super::ValueSign {
        self.values[SIZE - 1].sign()
    }
}

pub const fn internal_node_size(branching: usize) -> usize {
    return 1 << branching * 3;
}

pub const fn internal_node_bit_size<T: TreeNode>(branching: usize) -> usize {
    let size = internal_node_size::<T>(branching) / usize::BITS as usize;

    if size == 0 {
        1
    } else {
        size
    }
}

pub const fn internal_node_branching<TChild: TreeNode>(branching: usize) -> usize {
    return branching + TChild::BRANCHING_TOTAL;
}

#[cfg(test)]
mod tests {
    use crate::{
        data_structures::bitset::BitSet,
        helpers::aliases::Vec3i,
        voxel::{
            internal_node_bit_size, internal_node_size, leaf_node_bit_size, leaf_node_size,
            LeafNode, TreeNode,
        },
    };

    use super::InternalNode;

    #[test]
    fn test_alloc_internal_node() {
        const LEAF_LOG2: usize = 2;
        const INTERNAL_LOG2: usize = 3;

        type Leaf = LeafNode<
            f32,
            LEAF_LOG2,
            LEAF_LOG2,
            { leaf_node_size(LEAF_LOG2) },
            { leaf_node_bit_size(LEAF_LOG2) },
        >;
        type Internal = InternalNode<
            f32,
            Leaf,
            INTERNAL_LOG2,
            { LEAF_LOG2 + INTERNAL_LOG2 },
            { internal_node_size(INTERNAL_LOG2) },
            { internal_node_bit_size(INTERNAL_LOG2) },
            false,
        >;

        let origin = Vec3i::new(1, 2, 3);
        let node = Internal::empty(origin);
        assert_eq!(node.child_mask, BitSet::zeroes());
        assert_eq!(node.value_mask, BitSet::zeroes());
        assert_eq!(node.origin, origin);
        assert!(node.childs.iter().all(|c| c.is_none()));
    }
}
