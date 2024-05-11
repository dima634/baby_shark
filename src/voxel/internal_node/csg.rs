use super::*;
use crate::data_structures::bitset::BitSet;

///
/// Helper methods for CSG operations.
/// This implementation assuming that nodes are flood filled prior to CSG operations
///
impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: Csg,
    TChild::Value: Signed + Neg<Output = TChild::Value>,
{
    #[inline]
    fn is_inside_tile(&self, offset: usize) -> bool {
        self.child_mask.is_off(offset) && unsafe { self.childs[offset].tile.sign() == Sign::Negative }
    }

    #[inline]
    fn is_outside_tile(&self, offset: usize) -> bool {
        self.child_mask.is_off(offset) && unsafe { self.childs[offset].tile.sign() == Sign::Positive }
    }

    #[inline]
    fn take_child(&mut self, other: &mut Self, offset: usize) {
        if other.child_mask.is_on(offset) {
            other.child_mask.set(offset, self.child_mask.is_on(offset));
            other.value_mask.set(offset, self.value_mask.is_on(offset));
            self.child_mask.on(offset);
            self.value_mask.off(offset);
            core::mem::swap(&mut self.childs[offset], &mut other.childs[offset]);
        }
    }

    #[inline]
    fn make_child_inside(&mut self, offset: usize) {
        self.remove_child(offset);
        self.value_mask.on(offset);
        self.childs[offset] = ChildUnion {
            tile: TChild::Value::far(),
        };
    }
}

///
/// CSG operations for internal nodes.
/// This implementation assuming that nodes are flood filled prior to CSG operations
///
impl<
        TChild,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
        const PARALLEL: bool,
    > Csg
    for InternalNode<TChild::Value, TChild, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE, PARALLEL>
where
    TChild: Csg,
    TChild::Value: Signed + Neg<Output = TChild::Value>,
{
    fn union(&mut self, mut other: Box<Self>) {
        for offset in 0..SIZE {
            if self.is_inside_tile(offset) {
                continue;
            }

            if other.is_inside_tile(offset) {
                self.make_child_inside(offset);
                continue;
            }

            if self.is_outside_tile(offset) {
                self.take_child(&mut other, offset);
                continue;
            }

            match (self.child_mut(offset), other.remove_child(offset)) {
                (Some(OneOf::T1(self_branch)), Some(OneOf::T1(other_branch))) => {
                    self_branch.union(other_branch);

                    if self_branch.is_empty() {
                        self.make_child_inside(offset);
                    }
                },
                _ => continue,
            };
        }
    }

    fn subtract(&mut self, mut other: Box<Self>) {
        for offset in 0..SIZE {
            if self.is_outside_tile(offset) || other.is_outside_tile(offset) {
                continue;
            }

            if other.is_inside_tile(offset) {
                self.remove_child(offset);
                continue;
            }

            if self.is_inside_tile(offset) {
                self.take_child(&mut other, offset);

                match self.child_mut(offset) {
                    Some(OneOf::T1(branch)) => {
                        branch.flip_signs();
                    },
                    Some(OneOf::T2(value)) => {
                        *value = -*value; // TODO: replace with `value.flip_sign()` when it's implemented
                    },
                    None => {}
                }

                continue;
            }

            match (self.child_mut(offset), other.remove_child(offset)) {
                (Some(OneOf::T1(self_branch)), Some(OneOf::T1(other_branch))) => self_branch.subtract(other_branch),
                _ => continue,
            };
        }
    }

    fn intersect(&mut self, mut other: Box<Self>) {
        for offset in 0..SIZE {
            if self.is_outside_tile(offset) || other.is_inside_tile(offset) {
                continue;
            }

            if other.is_outside_tile(offset) {
                self.remove_child(offset);
                continue;
            }

            if self.is_inside_tile(offset) {
                self.take_child(&mut other, offset);
                continue;
            }

            match (self.child_mut(offset), other.remove_child(offset)) {
                (Some(OneOf::T1(self_branch)), Some(OneOf::T1(other_branch))) => self_branch.intersect(other_branch),
                _ => continue,
            };
        }
    }

    fn flip_signs(&mut self) {
        for offset in 0..SIZE {
            match self.child_mut(offset) {
                Some(OneOf::T1(branch)) => branch.flip_signs(),
                Some(OneOf::T2(value)) => *value = -*value, // TODO: replace with `value.flip_sign()` when it's implemented
                None => {}
            };
        }
    }
}
