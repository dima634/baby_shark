use std::{mem::MaybeUninit, ops::Neg};

use crate::{data_structures::bitset::BitSet, helpers::aliases::Vec3i};

use super::{utils::{partial_max, partial_min}, Accessor, Csg, FloodFill, Value, ParVisitor, Signed, TreeNode, Sign};

#[derive(Debug)]
pub struct LeafNode<
    TValue,
    const BRANCHING: usize,
    const BRANCHING_TOTAL: usize,
    const SIZE: usize,
    const BIT_SIZE: usize,
> {
    values: [TValue; SIZE],
    value_mask: BitSet<SIZE, BIT_SIZE>,
    origin: Vec3i,
}

impl<
        TValue: Value,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    #[inline]
    fn offset(index: &Vec3i) -> usize {
        let offset = ((index.x & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING + Self::BRANCHING)
            + ((index.y & (1 << Self::BRANCHING_TOTAL) - 1) << Self::BRANCHING)
            + (index.z & (1 << Self::BRANCHING_TOTAL) - 1);

        offset as usize
    }
}

impl<
        TValue: Value,
        const BRANCHING: usize,
        const BRANCHING_TOTAL: usize,
        const SIZE: usize,
        const BIT_SIZE: usize,
    > Accessor for LeafNode<TValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>
{
    type Value = TValue;

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
}

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

    type Child = Self;
    type Leaf = Self;
    type As<TNewValue: Value> = LeafNode<TNewValue, BRANCHING, BRANCHING_TOTAL, SIZE, BIT_SIZE>;

    #[inline]
    fn empty(origin: Vec3i) -> Box<Self> {
        Box::new(Self {
            origin,
            value_mask: BitSet::zeroes(),
            values: unsafe { MaybeUninit::uninit().assume_init() }, // Safe because value mask is empty
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
        self.value_mask = BitSet::ones();
        self.values = [value; SIZE];
    }

    fn clear(&mut self) {
        self.value_mask.off_all();
    }

    fn is_constant(&self, tolerance: Self::Value) -> Option<Self::Value> {
        if self.is_empty() || !self.value_mask.is_full() {
            return None;
        }

        let first_value_offset = self.value_mask.find_first_on()?;
        let first_value = self.values[first_value_offset];

        // Check if all values are within tolerance
        for offset in (first_value_offset + 1)..SIZE {
            if !self.value_mask.at(offset) {
                continue;
            }

            let value = self.values[offset];

            if (value - first_value) > tolerance {
                return None;
            }
        }

        Some(first_value)
    }

    #[inline]
    fn prune(&mut self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Leaf node should never be pruned")
    }

    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where
        TNewValue: Value,
        TCast: Fn(Self::Value) -> TNewValue,
    {
        let mut new_node = LeafNode {
            origin: self.origin,
            value_mask: self.value_mask,
            values: unsafe { MaybeUninit::uninit().assume_init() }, // We  will f
        };

        for i in 0..SIZE {
            if !self.value_mask.at(i) {
                continue;
            }

            new_node.values[i] = cast(self.values[i]);
        }

        new_node
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        visitor.dense(self);
    }

    #[inline]
    fn visit_leafs<T: super::Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        visitor.dense(self);
    }
}

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

        // self.value_mask.on_all();
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
    fn first_value_sign(&self) -> super::Sign {
        self.values[0].sign()
    }

    #[inline]
    fn last_value_sign(&self) -> super::Sign {
        self.values[SIZE - 1].sign()
    }

    #[inline]
    fn sign_at(&self, index: &Vec3i) -> Sign {
        self.values[Self::offset(index)].sign()
    }
}

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

    fn subtract(&mut self, mut other: Box<Self>) {
        
        // println!("{}", self.value_mask.iter().filter(|i| *i).count());
        // println!("{}\n", other.value_mask.iter().filter(|i| *i).count());
        
        for i in 0..SIZE {
            // match (self.value_mask.is_on(i), other.value_mask.is_on(i)) {
            //     (true, true) => self.values[i] = partial_max(self.values[i], -other.values[i]),
            //     (true, false) =>  println!("1"),
            //     (false, true) =>  println!("2"),
            //     (false, false) => println!("3"),
            // };

            self.values[i] = partial_max(self.values[i], -other.values[i]);
            // if (self.value_mask | other.value_mask).is_on(i) {
            //     println!("{:?}", self.values[i]);
            // }
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

pub const fn leaf_node_size(branching: usize) -> usize {
    1 << branching * 3
}

pub const fn leaf_node_bit_size(branching: usize) -> usize {
    let size = leaf_node_size(branching) / usize::BITS as usize;

    if size == 0 {
        1
    } else {
        size
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
