use super::*;

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
        self.child_mask.is_off(offset) && self.values[offset].sign() == Sign::Negative
    }

    #[inline]
    fn is_outside_tile(&self, offset: usize) -> bool {
        self.child_mask.is_off(offset) && self.values[offset].sign() == Sign::Positive
    }

    #[inline]
    fn take_child(&mut self, other: &mut Self, offset: usize) {
        if other.child_mask.is_on(offset) {
            self.child_mask.on(offset);
            core::mem::swap(&mut self.childs[offset], &mut other.childs[offset]);
        }
    }

    #[inline]
    fn make_child_inside(&mut self, offset: usize) {
        self.child_mask.off(offset);
        self.childs[offset] = None;
        self.value_mask.on(offset);
        let mut value = TChild::Value::far();
        value.set_sign(Sign::Negative);
        self.values[offset] = value;
    }

    #[inline]
    fn remove_child(&mut self, offset: usize) {
        self.value_mask.off(offset);
        self.child_mask.off(offset);
        self.childs[offset] = None;
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

            if other.child_mask.is_on(offset) {
                self.child_node_mut(offset).union(other.child_owned(offset));

                if self.child_node(offset).is_empty() {
                    self.make_child_inside(offset);
                }
            }
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
                self.child_node_mut(offset).flip_signs();
                continue;
            }

            self.child_node_mut(offset)
                .subtract(other.child_owned(offset));
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

            self.child_node_mut(offset)
                .intersect(other.child_owned(offset));
        }
    }

    fn flip_signs(&mut self) {
        for offset in 0..SIZE {
            if self.child_mask.is_on(offset) {
                self.child_node_mut(offset).flip_signs();
            }

            self.values[offset] = -self.values[offset];
        }
    }
}
