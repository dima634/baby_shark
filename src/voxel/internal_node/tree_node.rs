use super::*;
use std::mem::MaybeUninit;

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

    type Value = TChild::Value;
    type Child = TChild;
    type Leaf = TChild::Leaf;
    type As<TNewValue: Value> = InternalNode<
        TNewValue,
        TChild::As<TNewValue>,
        BRANCHING,
        BRANCHING_TOTAL,
        SIZE,
        BIT_SIZE,
        PARALLEL,
    >;

    #[inline]
    fn at(&self, index: &Vec3i) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            return self.child_node(offset).at(index);
        }

        if !self.value_mask.at(offset) {
            return None;
        }

        Some(&self.values[offset])
    }

    #[inline]
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value> {
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            return self.child_node_mut(offset).at_mut(index);
        }

        if !self.value_mask.at(offset) {
            return None;
        }

        Some(&mut self.values[offset])
    }

    fn insert(&mut self, index: &Vec3i, value: Self::Value) {
        // Node is branch - insert voxel
        // Node is tile:
        //   if tile is active - convert to branch, fill with tile value and insert voxel
        //   else - add empty child and insert voxel
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            let child = self.child_node_mut(offset);
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
            let child = self.child_node_mut(offset);
            child.fill(tile_value);
            child.insert(index, value);
        } else {
            self.add_child(offset);
            let child = self.child_node_mut(offset);
            child.insert(index, value);
        }
    }

    fn remove(&mut self, index: &Vec3i) {
        // Node is branch - remove voxel from child, prune child if empty
        // Node is tile:
        //   active - add active child, fill with tile value and remove voxel
        //   inactive - do nothing
        let offset = Self::offset(index);

        if self.child_mask.at(offset) {
            let child = self.child_node_mut(offset);
            child.remove(index);

            if child.is_empty() {
                self.remove_child_node(offset);
                self.value_mask.off(offset); // Remove?
            }
        } else if self.value_mask.at(offset) {
            let tile_value = self.values[offset];
            self.add_child(offset);
            let child = self.child_node_mut(offset);
            child.fill(tile_value);
            child.remove(index);
        }
    }

    #[inline]
    fn empty(origin: Vec3i) -> Box<Self> {
        // Allocate directly on a heap, otherwise we will overflow the stack with large grids
        unsafe { Self::alloc_on_heap(origin) }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.child_mask.is_empty() && self.value_mask.is_empty()
    }

    #[inline]
    fn origin(&self) -> Vec3i {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.child_mask = BitArray::zeroes();
        self.value_mask = BitArray::ones();
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

            let child = self.child_node_mut(offset);

            if child.is_empty() {
                self.remove_child_node(offset);
                continue;
            }

            let pruned = if TChild::IS_LEAF {
                child.is_constant(tolerance)
            } else {
                child.prune(tolerance)
            };

            if let Some(value) = pruned {
                self.replace_node_with_tile(offset, value);
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

    fn clone_map<TNewValue, TMap>(&self, map: &TMap) -> Self::As<TNewValue>
    where
        TNewValue: super::Value,
        TMap: Fn(Self::Value) -> TNewValue,
    {
        let mut new_node = InternalNode {
            origin: self.origin,
            childs: std::array::from_fn(|_| None),
            child_mask: self.child_mask,
            value_mask: self.value_mask,
            values: [Default::default(); SIZE],
        };

        for i in 0..SIZE {
            if self.child_mask.at(i) {
                let child = self.child_node(i);
                new_node.childs[i] = Some(Box::new(child.clone_map(map)));
            } else if self.value_mask.at(i) {
                new_node.values[i] = map(self.values[i]);
            }
        }

        new_node
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        use rayon::prelude::*;

        if PARALLEL {
            (0..SIZE)
                .filter_map(|i| match self.child_mask.at(i) {
                    true => Some(self.child_node(i)),
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
                    let child = self.child_node(i);
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
                let child = self.child_node(i);
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
