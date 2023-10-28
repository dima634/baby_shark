use std::{marker::PhantomData, mem::MaybeUninit};

use bitvec::prelude::BitArray;
use nalgebra::Vector3;

use super::{
    utils::{is_mask_empty, is_mask_full},
    Accessor, HasChild, TreeNode, Leaf, Tile, Traverse, Child,
};

pub struct InternalNode<
    TValue,
    TChild: TreeNode,
    TLeaf: TreeNode,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    childs: [Option<Box<TChild>>; SIZE],
    child_mask: BitArray<[usize; BIT_SIZE]>,
    values: [TValue; SIZE],
    value_mask: BitArray<[usize; BIT_SIZE]>,
    origin: Vector3<isize>,
    leaf_type: PhantomData<TLeaf>,
}

impl<TChild, TLeaf, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> Traverse<TLeaf> for InternalNode<TChild::Value, TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
where 
    TChild: TreeNode<LeafNode = TLeaf> + Traverse<TLeaf>,
    TLeaf: TreeNode<Value = TChild::Value>
{
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = Child<'a, TLeaf>> + 'a> {
        let it = (0..SIZE).into_iter()
            .filter_map(|offset| {
                if self.child_mask[offset] {
                    let child = self.child(offset);
            
                    return if TChild::IS_LEAF {
                        // Should I use specialization here once rust supports it?
                        let as_leaf = unsafe { std::mem::transmute(child) };
                        Some(Child::Leaf(as_leaf))
                    } else {
                        Some(Child::Branch(child))
                    };
                }

                if self.value_mask[offset] {
                    let tile = Child::Tile(Tile {
                        origin: self.offset_to_global_index(offset),
                        size: TChild::resolution()
                    });
                    return Some(tile);
                }
        
                None
            });

        Box::new(it)
    }
}

// impl<TChild, TLeaf, const BRANCHING: usize, const BRANCHING_TOTAL: usize, const SIZE: usize, const BIT_SIZE: usize> Node<TLeaf> for InternalNode<TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE> 
// where 
//     TChild: TreeNode<LeafNode = TLeaf>,
//     TLeaf: TreeNode
// {
//     #[inline]
//     fn childs_count(&self) -> usize {
//         SIZE
//     }

//     // #[inline]
//     // fn child(&self, index: usize) -> Leaf<TLeaf> {
//     //     debug_assert!(index < SIZE);

//     //     if self.child_mask[index] {
//     //         return Leaf::Node(self.child(index));
//     //     }

//     //     Leaf::Tile(Tile {
//     //         origin: self.offset_to_global_index(index),
//     //         size: TChild::resolution()
//     //     })
//     // }

//     #[inline]
//     fn is_leaf(&self) -> bool {
//         false
//     }

//     #[inline]
//     fn next(&self, index: &Vector3<isize>) -> Option<&dyn Node<TLeaf>> {
//         let offset = Self::offset(index);

//         if self.child_mask[offset] {
//             return Some(self.child(offset));
//         }

//         None
//     }

//     #[inline]
//     fn total_branching(&self) -> usize {
//         Self::BRANCHING_TOTAL
//     }

//     #[inline]
//     fn at_if_leaf(&self, _: &Vector3<isize>) -> Option<bool> {
//         None
//     }
// }

impl<
        TChild,
        TLeaf,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > InternalNode<TChild::Value, TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
where 
    TChild: TreeNode<LeafNode = TLeaf>,
    TLeaf: TreeNode<Value = TChild::Value>
{
    #[inline]
    pub fn new() -> Self {
        return Self::empty(Vector3::zeros());
    }

    // #[inline]
    // pub fn cached_accessor(&self) -> CachedAccessor<Self> {
    //     return CachedAccessor::new(self);
    // }

    fn offset_to_global_index(&self, offset: usize) -> Vector3<isize> {
        let mut local = Self::offset_to_local_index(offset);

        for i in 0..3 {
            local[i] <<= TChild::BRANCHING_TOTAL;
        }

        return local.cast() + self.origin;
    }

    fn add_child(&mut self, offset: usize) {
        debug_assert!(!self.child_mask[offset]);

        self.child_mask.set(offset, true);
        self.value_mask.set(offset, false);

        let child_origin = self.offset_to_global_index(offset);
        let child_node = TChild::empty(child_origin);
        self.childs[offset] = Some(Box::new(child_node));
    }

    #[inline]
    fn remove_child(&mut self, offset: usize) {
        debug_assert!(self.child_mask[offset]);

        // self.value_mask.set(offset, false);
        self.child_mask.set(offset, false);
        self.childs[offset] = None;
    }

    #[inline]
    fn child(&self, offset: usize) -> &TChild {
        let child = &self.childs[offset];
        debug_assert!(self.child_mask[offset]);
        debug_assert!(child.is_some());

        return unsafe { child.as_ref().unwrap_unchecked() };
    }

    #[inline]
    fn child_mut(&mut self, offset: usize) -> &mut TChild {
        let child = &mut self.childs[offset];
        debug_assert!(self.child_mask[offset]);
        debug_assert!(child.is_some());

        return unsafe { child.as_mut().unwrap_unchecked() };
    }

    #[inline]
    fn offset(index: &Vector3<isize>) -> usize {
        let offset =
            (((index.x & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << (Self::BRANCHING + Self::BRANCHING))+
            (((index.y & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL) << Self::BRANCHING) +
             ((index.z & (1 << Self::BRANCHING_TOTAL) - 1) >> <Self as HasChild>::Child::BRANCHING_TOTAL);

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
}

impl<
        TChild: TreeNode,
        TLeaf: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > HasChild for InternalNode<TChild::Value, TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    type Child = TChild;
}

impl<
        TChild: TreeNode,
        TLeaf: TreeNode,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Accessor for InternalNode<TChild::Value, TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
where 
    TChild: TreeNode<LeafNode = TLeaf>, 
    TLeaf: TreeNode<Value = TChild::Value>
{
    type Value = TChild::Value;

    #[inline]
    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            return self.child(offset).at(index);
        }

        if !self.value_mask[offset] {
            return None;
        }

        Some(&self.values[offset])
    }

    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        // Node is branch - insert voxel
        // Node is tile:
        //   if tile is active - convert to branch, fill with tile value and insert voxel
        //   else - add empty child and insert voxel
        let offset = Self::offset(index);

        if self.child_mask[offset] {
            let child = self.child_mut(offset);
            child.insert(index, value);
            return;
        }

        if self.value_mask[offset] {
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

        if self.child_mask[offset] {
            let child = self.child_mut(offset);
            child.remove(index);

            if child.is_empty() {
                self.remove_child(offset);
                self.value_mask.set(offset, false); // Remove?
            }
        } else if self.value_mask[offset] {
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
        TLeaf,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > TreeNode for InternalNode<TChild::Value, TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
where 
    TChild: TreeNode<LeafNode = TLeaf>, 
    TLeaf: TreeNode<Value = TChild::Value>
{
    const BRANCHING: usize = BRANCHING;
    const BRANCHING_TOTAL: usize = BRANCHING_TOTAL;
    const SIZE: usize = SIZE;

    const IS_LEAF: bool = false;

    type LeafNode = TLeaf;

    fn empty(origin: Vector3<isize>) -> Self {
        return Self {
            origin,
            childs: std::array::from_fn(|_| None),
            child_mask: Default::default(),
            values: unsafe { MaybeUninit::uninit().assume_init() }, // Safe because value mask is empty
            value_mask: Default::default(),
            leaf_type: PhantomData,
        };
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return is_mask_empty::<BIT_SIZE>(&self.child_mask.data)
            && is_mask_empty::<BIT_SIZE>(&self.value_mask.data);
    }

    fn traverse_leafs<F: FnMut(Leaf<Self::LeafNode>)>(&self, f: &mut F) {
        for i in 0..SIZE {
            if self.child_mask[i] {
                let child = self.child(i);
                child.traverse_leafs(f);
            } else if self.value_mask[i] {
                let tile = Leaf::Tile(Tile {
                    origin: self.offset_to_global_index(i),
                    size: TChild::resolution()
                });

                f(tile);
            }
        }
    }

    #[inline]
    fn origin(&self) -> Vector3<isize> {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.child_mask.data = [0; BIT_SIZE];
        self.value_mask.data = [usize::MAX; BIT_SIZE];
        self.values = [value; SIZE];
    }
}

// impl<
//         'tree,
//         TChild: TreeNode + TreeTraverse + 'tree,
//         const BRANCHING: usize,
//         const BRANCHING_TOTAL: usize,
//         const SIZE: usize,
//         const BIT_SIZE: usize,
//     > TreeTraverse for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
// {
//     fn visit_and_cache(&mut self, index: &Vector3<usize>, cache: &mut Vec<CacheEntry>) {
//         let offset = Self::offset(index);

//         if self.child_mask[offset] {
//             let child = &mut self.childs[offset];
//             child.as_mut().unwrap().visit_and_cache(index, cache);
//         }

//         cache.push(CacheEntry {
//             key: self.index_key(index),
//             accessor: unsafe { NonNull::new_unchecked(self as *mut Self) },
//         });
//     }
// }

// impl<
//         TChild: TreeNode,
//         const BRANCHING: usize,
//         const BRANCHING_TOTAL: usize,
//         const SIZE: usize,
//         const BIT_SIZE: usize,
//     > MarchingCubes for InternalNode<TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
// {
//     type Cubes;

//     fn cubes(&self) -> Self::Cubes {
//         todo!()
//     }
// }

// struct CubesIter<
//         TChild: TreeNode,
//         TLeaf: TreeNode,
//         const BRANCHING: usize,
//         const BRANCHING_TOTAL: usize,
//         const SIZE: usize,
//         const BIT_SIZE: usize,
//     >
// {
//     node: InternalNode<TChild, TLeaf, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>,
//     offset: usize,
// }

pub const fn internal_node_size<T: TreeNode>(branching: usize) -> usize {
    return 1 << branching * 3;
}

pub const fn internal_node_bit_size<T: TreeNode>(branching: usize) -> usize {
    return internal_node_size::<T>(branching) / usize::BITS as usize;
}

pub const fn internal_node_branching<T: TreeNode>(branching: usize) -> usize {
    return branching + T::BRANCHING_TOTAL;
}
