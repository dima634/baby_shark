use super::*;

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
        match self.child(offset) {
            Some(OneOf::T1(branch)) => branch.at(index),
            Some(OneOf::T2(tile)) => Some(tile),
            None => None,
        }
    }

    #[inline]
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value> {
        let offset = Self::offset(index);
        match self.child_mut(offset) {
            Some(OneOf::T1(branch)) => branch.at_mut(index),
            Some(OneOf::T2(tile)) => Some(tile),
            None => None,
        }
    }

    fn insert(&mut self, index: &Vec3i, value: Self::Value) {
        // Node is branch - insert voxel
        // Node is tile:
        //   if tile is active - convert to branch, fill with tile value and insert voxel
        //   else - add empty child and insert voxel
        let offset = Self::offset(index);
        match self.child_mut(offset) {
            Some(OneOf::T1(branch)) => branch.insert(index, value),
            Some(OneOf::T2(tile)) => {
                let tile_value = *tile;

                // No need to add child if tile value is equal to inserted one
                if tile_value == value {
                    return;
                }

                let branch = self.add_branch(offset);
                branch.fill(tile_value);
                branch.insert(index, value);
            }
            None => {
                let branch = self.add_branch(offset);
                branch.insert(index, value);
            }
        };
    }

    fn remove(&mut self, index: &Vec3i) {
        // Node is branch - remove voxel from child, prune child if empty
        // Node is tile:
        //   active - add active child, fill with tile value and remove voxel
        //   inactive - do nothing
        let offset = Self::offset(index);
        match self.child_mut(offset) {
            Some(OneOf::T1(branch)) => {
                branch.remove(index);

                if branch.is_empty() {
                    self.remove_branch(offset);
                }
            }
            Some(OneOf::T2(tile)) => {
                let tile_value = *tile;
                let branch = self.add_branch(offset);
                branch.fill(tile_value);
                branch.remove(index);
            }
            None => {}
        };
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

    fn fill(&mut self, value: Self::Value) {
        self.clear();
        self.value_mask.on_all();

        for offset in 0..SIZE {
            self.childs[offset].tile = value;
        }
    }

    fn clear(&mut self) {
        for offset in 0..SIZE {
            if self.child_mask.is_on(offset) {
                unsafe { let _ = ManuallyDrop::take(&mut self.childs[offset].branch); }
            }
        }

        self.child_mask.off_all();
        self.value_mask.off_all();
    }

    fn clone_map<TNewValue, TMap>(&self, map: &TMap) -> Box<Self::As<TNewValue>>
    where
        TNewValue: super::Value,
        TMap: Fn(Self::Value) -> TNewValue,
    {
        let mut clone = unsafe { Self::As::<TNewValue>::alloc_on_heap(self.origin) };
        clone.child_mask = self.child_mask;
        clone.value_mask = self.value_mask;

        for i in 0..SIZE {
            if let Some(child) = self.child(i) {
                match child {
                    OneOf::T1(branch) => {
                        clone.childs[i] = ChildUnion {
                            branch: ManuallyDrop::new(branch.clone_map(map)),
                        }
                    }
                    OneOf::T2(tile) => clone.childs[i] = ChildUnion { tile: map(*tile) },
                }
            }
        }

        clone
    }

    fn clone(&self) -> Box<Self> {
        let mut clone = unsafe { Self::alloc_on_heap(self.origin) };
        clone.child_mask = self.child_mask;
        clone.value_mask = self.value_mask;

        for i in 0..SIZE {
            if let Some(child) = self.child(i) {
                match child {
                    OneOf::T1(branch) => {
                        clone.childs[i] = ChildUnion {
                            branch: ManuallyDrop::new(branch.clone()),
                        }
                    }
                    OneOf::T2(tile) => clone.childs[i] = ChildUnion { tile: *tile },
                }
            }
        }

        clone
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        use rayon::prelude::*;

        if PARALLEL {
            (0..SIZE)
                .filter_map(|offset| match self.child(offset) {
                    Some(OneOf::T1(branch)) => Some(branch),
                    _ => None,
                })
                .par_bridge()
                .into_par_iter()
                .for_each(|c| c.visit_leafs_par(visitor));

            (0..SIZE)
                .filter_map(|offset| match self.child(offset) {
                    Some(OneOf::T2(tile)) => Some((offset, *tile)),
                    _ => None,
                })
                .for_each(|(offset, value)| {
                    let tile = Tile {
                        origin: self.offset_to_global_index(offset),
                        size: TChild::resolution(),
                        value,
                    };

                    visitor.tile(tile);
                });
        } else {
            for (offset, child) in self.childs() {
                match child {
                    OneOf::T1(branch) => branch.visit_leafs_par(visitor),
                    OneOf::T2(tile) => visitor.tile(Tile {
                        origin: self.offset_to_global_index(offset),
                        size: TChild::resolution(),
                        value: *tile,
                    }),
                };
            }
        }
    }

    fn visit_leafs<T: super::Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        for (offset, child) in self.childs() {
            match child {
                OneOf::T1(branch) => branch.visit_leafs(visitor),
                OneOf::T2(tile) => visitor.tile(Tile {
                    origin: self.offset_to_global_index(offset),
                    size: TChild::resolution(),
                    value: *tile,
                }),
            };
        }
    }

    fn visit_values_mut<T: ValueVisitorMut<Self::Value>>(&mut self, visitor: &mut T) {
        for offset in 0..SIZE {
            if let Some(child) = self.child_mut(offset) {
                match child {
                    OneOf::T1(branch) => branch.visit_values_mut(visitor),
                    OneOf::T2(tile) => visitor.value(tile),
                };
            }
        }
    }

    #[inline]
    fn leaf_at(&self, index: &Vec3i) -> Option<&Self::Leaf> {
        let offset = Self::offset(index);

        if let Some(OneOf::T1(branch)) = self.child(offset) {
            return branch.leaf_at(index);
        }

        None
    }

    fn take_leaf_at(&mut self, index: &Vec3i) -> Option<Box<Self::Leaf>> {
        let offset = Self::offset(index);

        if let Some(OneOf::T1(branch)) = self.child_mut(offset) {
            if Self::Child::IS_LEAF {
                let child = self.remove_branch(offset);
                unsafe {
                    return std::mem::transmute(child);
                }
            } else {
                return branch.take_leaf_at(index);
            }
        }

        None
    }

    fn insert_leaf_at(&mut self, leaf: Box<Self::Leaf>) {
        let index = leaf.origin();
        let offset = Self::offset(&index);
        self.value_mask.off(offset);

        if Self::Child::IS_LEAF {
            self.remove_branch(offset);
            self.child_mask.on(offset);
            self.value_mask.off(offset);
            self.childs[offset] = ChildUnion {
                branch: ManuallyDrop::new(unsafe { core::mem::transmute(leaf) }),
            };
        } else {
            match self.child_mut(offset) {
                Some(OneOf::T1(branch)) => branch.insert_leaf_at(leaf),
                Some(OneOf::T2(_)) | None => self.add_branch(offset).insert_leaf_at(leaf),
            };
        }
    }

    fn remove_empty_branches(&mut self) {
        for offset in 0..SIZE {
            if let Some(OneOf::T1(branch)) = self.child_mut(offset) {
                branch.remove_empty_branches();

                if branch.is_empty() {
                    self.remove_branch(offset);
                }
            }
        }
    }

    fn remove_if<TPred>(&mut self, pred: TPred)
    where
        TPred: Fn(&Self::Value) -> bool + Copy,
    {
        for offset in 0..SIZE {
            let child = match self.child_mut(offset) {
                Some(child) => child,
                None => continue,
            };

            match child {
                OneOf::T1(branch) => {
                    branch.remove_if(pred);

                    if branch.is_empty() {
                        self.remove_branch(offset);
                    }
                }
                OneOf::T2(tile) => {
                    if pred(tile) {
                        self.value_mask.off(offset);
                    }
                }
            }
        }
    }
}
