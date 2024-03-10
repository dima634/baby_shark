use super::*;
use crate::{data_structures::bitset::BitSet, helpers::aliases::Vec3i};

impl<
        TValue: Signed,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > FloodFill for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    fn flood_fill(&mut self) {
        if self.value_mask.is_full() {
            return;
        }

        let mut i = match self.value_mask.find_first_on() {
            Some(i) => self.values[i].sign(),
            None => return,
        };

        for x in 0..Self::resolution() {
            let x00 = x << (BRANCHING + BRANCHING); // offset for element(x, 0, 0)

            if self.value_mask.is_on(x00) {
                i = self.values[x00].sign();
            }

            let mut j = i;
            for y in 0..Self::resolution() {
                let xy0 = x00 + (y << BRANCHING); // offset for element(x, y, 0)

                if self.value_mask.is_on(xy0) {
                    j = self.values[xy0].sign();
                }

                let mut k = j;
                for z in 0..Self::resolution() {
                    let xyz = xy0 + z; // offset for element(x, y, z)

                    if self.value_mask.is_on(xyz) {
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
        for i in 0..self.values.len() {
            if self.value_mask.is_off(i) {
                self.values[i] = Self::Value::far();
            }

            self.values[i].set_sign(sign);
        }
    }

    #[inline]
    fn first_value_sign(&self) -> Sign {
        self.values[0].sign()
    }

    #[inline]
    fn last_value_sign(&self) -> Sign {
        self.values[SIZE - 1].sign()
    }

    #[inline]
    fn sign_at(&self, index: &Vec3i) -> Sign {
        self.values[Self::offset(index)].sign()
    }
}

#[cfg(test)]
mod tests {
    use crate::{helpers::aliases::Vec3i, static_vdb, voxel::*};

    #[test]
    fn test_flood_fill() {
        type Leaf = static_vdb!(f32, 2);

        // One voxel with positive sign
        let mut node = Leaf::empty(Vec3i::zeros());
        node.insert(&Vec3i::new(2, 2, 2), 1.0);
        node.flood_fill();
        assert!(node.values.iter().all(|v| v.is_sign_positive()));

        // One voxel with negative sign
        let mut node = Leaf::empty(Vec3i::zeros());
        node.insert(&Vec3i::new(2, 2, 2), -1.0);
        node.flood_fill();
        assert!(node.values.iter().all(|v| v.is_sign_negative()));

        // Node split in half with positive and negative signs
        let mut node = Leaf::empty(Vec3i::zeros());
        let x_neg = 1;
        let x_pos = 2;

        for y in 0..Leaf::resolution() {
            for z in 0..Leaf::resolution() {
                node.insert(&Vec3i::new(x_neg, y as isize, z as isize), -1.0);
                node.insert(&Vec3i::new(x_pos, y as isize, z as isize), 1.0);
            }
        }

        node.flood_fill();
        assert!(node.values[0..32].iter().all(|v| v.is_sign_negative()));
        assert!(node.values[32..64].iter().all(|v| v.is_sign_positive()));
    }
}
