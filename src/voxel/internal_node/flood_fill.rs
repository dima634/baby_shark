use super::*;
use crate::data_structures::bitset::BitSet;

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

        for offset in 0..SIZE {
            if let Some(OneOf::T1(child)) = self.child_mut(offset) {
                child.flood_fill();
            }
        }

        let first_value = self.value_mask.find_first_on();
        let first_branch = self.child_mask.find_first_on();

        let mut i = unsafe {
            match (first_value, first_branch) {
                (Some(v), Some(b)) if v <= b => self.childs[v].tile.sign(),
                (Some(_), Some(b)) => self.childs[b].branch.first_value_sign(),
                (Some(i), None) => self.childs[i].tile.sign(),
                (None, Some(i)) => self.childs[i].branch.first_value_sign(),
                (None, None) => return,
            }
        };

        for x in 0..Self::childs_per_dim() {
            let x00 = x << (BRANCHING + BRANCHING); // offset for block(x, 0, 0)

            match self.child(x00) {
                Some(OneOf::T1(branch)) => i = branch.last_value_sign(),
                Some(OneOf::T2(value)) => i = value.sign(),
                None => {}
            };

            let mut j = i;
            for y in 0..Self::childs_per_dim() {
                let xy0 = x00 + (y << BRANCHING); // offset for block(x, y, 0)

                match self.child(xy0) {
                    Some(OneOf::T1(branch)) => j = branch.last_value_sign(),
                    Some(OneOf::T2(value)) => j = value.sign(),
                    None => {}
                };

                let mut k = j;
                for z in 0..Self::childs_per_dim() {
                    let xyz = xy0 + z; // offset for block(x, y, z)

                    match self.child(xyz) {
                        Some(OneOf::T1(branch)) => k = branch.last_value_sign(),
                        Some(OneOf::T2(value)) => k = value.sign(),
                        None => unsafe {
                            self.childs[xyz].tile = Self::Value::far();
                            self.childs[xyz].tile.set_sign(k);
                        },
                    };
                }
            }
        }
    }

    fn fill_with_sign(&mut self, sign: Sign) {
        if self.value_mask.is_full() {
            return;
        }

        for i in 0..SIZE {
            match self.child_mut(i) {
                Some(OneOf::T1(branch)) => branch.fill_with_sign(sign),
                Some(OneOf::T2(tile)) => tile.set_sign(sign),
                None => unsafe {
                    self.childs[i].tile = Self::Value::far();
                    self.childs[i].tile.set_sign(sign);
                },
            };
        }
    }

    #[inline]
    fn first_value_sign(&self) -> Sign {
        match self.child(0) {
            Some(OneOf::T1(branch)) => branch.first_value_sign(),
            Some(OneOf::T2(_)) | None => unsafe { self.childs[0].tile.sign() },
        }
    }

    #[inline]
    fn last_value_sign(&self) -> Sign {
        match self.child(SIZE - 1) {
            Some(OneOf::T1(branch)) => branch.first_value_sign(),
            Some(OneOf::T2(_)) | None => unsafe { self.childs[SIZE - 1].tile.sign() },
        }
    }

    #[inline]
    fn sign_at(&self, index: &Vec3i) -> Sign {
        let offset = Self::offset(index);
        match self.child(offset) {
            Some(OneOf::T1(branch)) => branch.sign_at(index),
            Some(OneOf::T2(_)) | None => unsafe { self.childs[offset].tile.sign() },
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        helpers::aliases::{Vec3, Vec3i},
        static_vdb,
        voxel::{utils::box_indices, *},
    };

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

        tiled_node.flood_fill();

        box_indices(0, (x_neg + Leaf::resolution()) as isize)
            .for_each(|i| assert_eq!(tiled_node.sign_at(&i), Sign::Negative));
        box_indices(x_pos as isize, Internal::resolution() as isize)
            .for_each(|i| assert_eq!(tiled_node.sign_at(&i), Sign::Positive));
    }
}
