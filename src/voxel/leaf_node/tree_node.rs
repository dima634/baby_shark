use super::*;
use crate::data_structures::bitset::BitSet;

impl<
        TValue: Value,
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

    type Value = TValue;
    type Child = Self;
    type Leaf = Self;
    type As<TNewValue: Value> = LeafNode<TNewValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>;

    #[inline]
    fn at(&self, index: &Vec3i) -> Option<&Self::Value> {
        let offset = Self::offset(index);

        if self.value_mask.at(offset) {
            Some(&self.values[offset])
        } else {
            None
        }
    }

    #[inline]
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value> {
        let offset = Self::offset(index);

        if self.value_mask.at(offset) {
            Some(&mut self.values[offset])
        } else {
            None
        }
    }

    #[inline]
    fn insert(&mut self, index: &Vec3i, value: Self::Value) {
        let offset = Self::offset(index);
        self.value_mask.on(offset);
        self.values[offset] = value;
    }

    #[inline]
    fn remove(&mut self, index: &Vec3i) {
        self.value_mask.off(Self::offset(index));
    }

    #[inline]
    fn empty(origin: Vec3i) -> Box<Self> {
        Box::new(Self {
            origin,
            value_mask: BitArray::zeroes(),
            values: [Default::default(); SIZE],
        })
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.value_mask.is_empty()
    }

    #[inline]
    fn origin(&self) -> Vec3i {
        self.origin
    }

    #[inline]
    fn fill(&mut self, value: Self::Value) {
        self.value_mask = BitArray::ones();
        self.values = [value; SIZE];
    }

    fn clear(&mut self) {
        self.value_mask.off_all();
    }

    fn clone_map<TNewValue, TMap>(&self, map: &TMap) -> Box<Self::As<TNewValue>>
    where
        TNewValue: Value,
        TMap: Fn(Self::Value) -> TNewValue,
    {
        let mut new_node = LeafNode {
            origin: self.origin,
            value_mask: self.value_mask,
            values: [Default::default(); SIZE],
        };

        for i in 0..SIZE {
            if !self.value_mask.at(i) {
                continue;
            }

            new_node.values[i] = map(self.values[i]);
        }

        Box::new(new_node)
    }

    fn clone(&self) -> Box<Self> {
        Box::new(LeafNode {
            origin: self.origin,
            value_mask: self.value_mask,
            values: self.values,
        })
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        visitor.dense(self);
    }

    #[inline]
    fn visit_leafs<T: super::Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        visitor.dense(self);
    }

    fn visit_values_mut<T: ValueVisitorMut<Self::Value>>(&mut self, visitor: &mut T) {
        for i in 0..SIZE {
            if self.value_mask.is_on(i) {
                visitor.value(&mut self.values[i]);
            }
        }
    }

    #[inline]
    fn leaf_at(&self, _: &Vec3i) -> Option<&Self::Leaf> {
        Some(self)
    }

    fn take_leaf_at(&mut self, _: &Vec3i) -> Option<Box<Self::Leaf>> {
        unimplemented!("Unsupported operation. Leaf node has no childs");
    }

    fn insert_leaf_at(&mut self, _: Box<Self::Leaf>) {
        unimplemented!("Unsupported operation. Leaf node has no childs");
    }

    #[inline]
    fn prune_empty_nodes(&mut self) {
        // Do nothing for leaf nodes
    }

    fn prune_if<TPred>(&mut self, pred: TPred)
    where
        TPred: Fn(&Self::Value) -> bool + Copy,
    {
        for i in 0..SIZE {
            if self.value_mask.is_on(i) && !pred(&self.values[i]) {
                self.value_mask.off(i);
            }
        }
    }
}
