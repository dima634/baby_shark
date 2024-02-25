use super::*;
use crate::voxel::utils::{partial_max, partial_min};
use std::ops::Neg;

///
/// CSG operations for leaf nodes
/// This implementation assuming that nodes are flood filler prior to CSG operations
///
impl<
        TValue: Signed + Neg<Output = Self::Value>,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Csg for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    fn union(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_min(self.values[i], other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn subtract(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_max(self.values[i], -other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn intersect(&mut self, other: Box<Self>) {
        for i in 0..SIZE {
            self.values[i] = partial_max(self.values[i], other.values[i]);
        }

        self.value_mask |= other.value_mask;
    }

    fn flip_signs(&mut self) {
        for i in 0..SIZE {
            self.values[i] = -self.values[i];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::static_vdb;

    #[test]
    fn test_union() {
        type Leaf = static_vdb!(f32, 1);

        // Create two leaf nodes
        let mut node1 = Leaf::empty(Vec3i::zeros());
        let mut node2 = Leaf::empty(Vec3i::zeros());

        // Set values for node1
        node1.values = [10.0, 20.0, 30.0, 40.0, -10.0, -20.0, -30.0, -40.0];
        node1.value_mask = BitSet::ones();

        // Set values for node2
        node2.values = [5.0, 15.0, 25.0, 35.0, -5.0, -15.0, -25.0, -35.0];
        node2.value_mask = BitSet::ones();

        // Perform union operation
        node1.union(node2);

        // Check the result
        assert_eq!(
            node1.values,
            [5.0, 15.0, 25.0, 35.0, -10.0, -20.0, -30.0, -40.0]
        );
        assert_eq!(node1.value_mask, BitSet::ones());
    }

    #[test]
    fn test_subtract() {
        type Leaf = static_vdb!(f32, 1);

        // Create two leaf nodes
        let mut node1 = Leaf::empty(Vec3i::zeros());
        let mut node2 = Leaf::empty(Vec3i::zeros());

        // Set values for node1
        node1.values = [10.0, 20.0, 30.0, 40.0, -10.0, -20.0, -30.0, -40.0];
        node1.value_mask = BitSet::ones();

        // Set values for node2
        node2.values = [5.0, 15.0, 25.0, 35.0, -5.0, -15.0, -25.0, -35.0];
        node2.value_mask = BitSet::ones();

        // Perform subtract operation
        node1.subtract(node2);

        // Check the result
        assert_eq!(
            node1.values,
            [10.0, 20.0, 30.0, 40.0, 5.0, 15.0, 25.0, 35.0]
        );
        assert_eq!(node1.value_mask, BitSet::ones());
    }

    #[test]
    fn test_intersect() {
        type Leaf = static_vdb!(f32, 1);

        // Create two leaf nodes
        let mut node1 = Leaf::empty(Vec3i::zeros());
        let mut node2 = Leaf::empty(Vec3i::zeros());

        // Set values for node1
        node1.values = [10.0, 20.0, 30.0, 40.0, -10.0, -20.0, -30.0, -40.0];
        node1.value_mask = BitSet::ones();

        // Set values for node2
        node2.values = [5.0, 15.0, 25.0, 35.0, -5.0, -15.0, -25.0, -35.0];
        node2.value_mask = BitSet::ones();

        // Perform intersect operation
        node1.intersect(node2);

        // Check the result
        assert_eq!(
            node1.values,
            [10.0, 20.0, 30.0, 40.0, -5.0, -15.0, -25.0, -35.0]
        );
        assert_eq!(node1.value_mask, BitSet::ones());
    }
}
