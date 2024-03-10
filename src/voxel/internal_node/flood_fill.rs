use super::*;

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
        if self.value_mask.is_full() {
            return;
        }

        self.childs
            .iter_mut()
            .filter_map(|c| c.as_mut())
            .for_each(|c| c.flood_fill());

        let first_value = self.value_mask.find_first_on();
        let first_child = self.child_mask.find_first_on();

        let mut i = match (first_value, first_child) {
            (Some(v), Some(c)) if v <= c => self.values[v].sign(),
            (Some(_), Some(c)) => self.child_node(c).first_value_sign(),
            (Some(i), None) => self.values[i].sign(),
            (None, Some(i)) => self.child_node(i).first_value_sign(),
            (None, None) => return,
        };

        for x in 0..Self::childs_per_dim() {
            let x00 = x << (BRANCHING + BRANCHING); // offset for block(x, 0, 0)

            if self.child_mask.is_on(x00) {
                i = self.child_node(x00).last_value_sign();
            } else if self.value_mask.is_on(x00) {
                i = self.values[x00].sign();
            }

            let mut j = i;
            for y in 0..Self::childs_per_dim() {
                let xy0 = x00 + (y << BRANCHING); // offset for block(x, y, 0)

                if self.child_mask.is_on(xy0) {
                    j = self.child_node(xy0).last_value_sign();
                } else if self.value_mask.is_on(xy0) {
                    j = self.values[xy0].sign();
                }

                let mut k = j;
                for z in 0..Self::childs_per_dim() {
                    let xyz = xy0 + z; // offset for block(x, y, z)

                    if self.child_mask.is_on(xyz) {
                        k = self.child_node(xyz).last_value_sign();
                    } else if self.value_mask.is_on(xyz) {
                        k = self.values[xyz].sign();
                    } else {
                        self.values[xyz] = Self::Value::far();
                        self.values[xyz].set_sign(k);
                    }
                }
            }
        }
    }

    fn fill_with_sign(&mut self, sign: Sign) {
        if self.value_mask.is_full() {
            return;
        }

        for i in 0..SIZE {
            if self.child_mask.is_on(i) {
                self.child_node_mut(i).fill_with_sign(sign);
            }

            if self.value_mask.is_off(i) {
                self.values[i] = Self::Value::far();
            }

            self.values[i].set_sign(sign);
        }
    }

    #[inline]
    fn first_value_sign(&self) -> Sign {
        if self.child_mask.is_on(0) {
            self.child_node(0).first_value_sign()
        } else {
            self.values[0].sign()
        }
    }

    #[inline]
    fn last_value_sign(&self) -> Sign {
        let offset = SIZE - 1;
        if self.child_mask.is_on(offset) {
            self.child_node(offset).last_value_sign()
        } else {
            self.values[offset].sign()
        }
    }

    #[inline]
    fn sign_at(&self, index: &Vec3i) -> Sign {
        let offset = Self::offset(index);
        if self.child_mask.is_on(offset) {
            self.child_node(offset).sign_at(index)
        } else {
            self.values[offset].sign()
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        data_structures::bitset::BitArray,
        helpers::aliases::{Vec3, Vec3i},
        static_vdb,
        voxel::{utils::box_indices, *},
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
        assert_eq!(node.child_mask, BitArray::zeroes());
        assert_eq!(node.value_mask, BitArray::zeroes());
        assert_eq!(node.origin, origin);
        assert!(node.childs.iter().all(|c| c.is_none()));
    }

    #[test]
    fn test_flood_fill() {
        type Internal = static_vdb!(f32, 2, 1);
        type Leaf = <Internal as TreeNode>::Child;

        // One voxel with positive sign
        let mut node = Internal::empty(Vec3i::zeros());
        node.insert(&Vec3i::new(3, 2, 1), 1.0);
        node.flood_fill();

        box_indices(0, Internal::resolution() as isize)
            .for_each(|i| assert_eq!(node.sign_at(&i), Sign::Positive));

        // One voxel with negative sign
        let mut node = Internal::empty(Vec3i::zeros());
        node.insert(&Vec3i::new(3, 3, 3), -1.0);
        node.flood_fill();
        box_indices(0, Internal::resolution() as isize)
            .for_each(|i| assert_eq!(node.sign_at(&i), Sign::Negative));

        // Node split in half with narrow stripe of leaf nodes
        let mut node = Internal::empty(Vec3i::zeros());
        let x_neg = 4;
        let x_pos = 5;

        for y in 0..Internal::resolution() {
            for z in 0..Internal::resolution() {
                node.insert(&Vec3i::new(x_neg, y as isize, z as isize), -1.0);
                node.insert(&Vec3i::new(x_pos, y as isize, z as isize), 1.0);
            }
        }

        node.flood_fill();

        box_indices(0, x_neg + 1).for_each(|i| assert_eq!(node.sign_at(&i), Sign::Negative));
        box_indices(x_pos, Internal::resolution() as isize)
            .for_each(|i| assert_eq!(node.sign_at(&i), Sign::Positive));

        // Node split in half with tiles
        let mut tiled_node = Internal::empty(Vec3i::zeros());
        let x_neg = 2;
        let x_pos = 4;

        for x in x_neg..Leaf::resolution() + x_neg {
            for y in 0..Internal::resolution() {
                for z in 0..Internal::resolution() {
                    tiled_node.insert(&Vec3::new(x, y, z).cast(), -1.0);
                }
            }
        }

        for x in x_pos..Leaf::resolution() + x_pos {
            for y in 0..Internal::resolution() {
                for z in 0..Internal::resolution() {
                    tiled_node.insert(&Vec3::new(x, y, z).cast(), 1.0);
                }
            }
        }

        tiled_node.prune(0.1);
        tiled_node.flood_fill();

        box_indices(0, (x_neg + Leaf::resolution()) as isize)
            .for_each(|i| assert_eq!(tiled_node.sign_at(&i), Sign::Negative));
        box_indices(x_pos as isize, Internal::resolution() as isize)
            .for_each(|i| assert_eq!(tiled_node.sign_at(&i), Sign::Positive));
    }
}
