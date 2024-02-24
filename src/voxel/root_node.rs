use std::{
    collections::{BTreeMap, BTreeSet},
    hash::{Hash, Hasher}, ops::Neg,
};

use nalgebra::Vector3;
use rayon::iter::{IntoParallelIterator, ParallelBridge, ParallelIterator};

use crate::helpers::aliases::Vec3i;

use super::{Accessor, Csg, FloodFill, Value, ParVisitor, Signed, Tile, TreeNode, Sign, Visitor};

#[derive(Debug)]
pub(super) struct RootNode<TChild: TreeNode> {
    root: BTreeMap<RootKey, Box<TChild>>,
}

impl<TChild: TreeNode> RootNode<TChild> {
    #[inline]
    pub fn new() -> Self {
        Self {
            root: Default::default(),
        }
    }

    #[inline]
    fn root_key(index: &Vec3i) -> RootKey {
        return RootKey(Vec3i::new(
            index.x & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.y & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.z & !((1 << TChild::BRANCHING_TOTAL) - 1),
        ));
    }
}

impl<TChild> TreeNode for RootNode<TChild>
where
    TChild: TreeNode,
{
    const BRANCHING: usize = usize::MAX;
    const BRANCHING_TOTAL: usize = usize::MAX;
    const SIZE: usize = usize::MAX;

    const IS_LEAF: bool = false;

    type Child = TChild;
    type Leaf = TChild::Leaf;
    type As<TValue: Value> = RootNode<TChild::As<TValue>>;

    #[inline]
    fn empty(_: Vector3<isize>) -> Box<Self> {
        Box::new(Self::new())
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.root.is_empty() || self.root.iter().all(|(_, node)| node.is_empty())
    }

    fn origin(&self) -> Vector3<isize> {
        unimplemented!("Unsupported operation. Dynamic root node has no origin");
    }

    fn fill(&mut self, _: Self::Value) {
        unimplemented!("Unsupported operation. Dynamic root node can't be filled");
    }

    fn clear(&mut self) {
        self.root.clear();
    }

    fn is_constant(&self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Dynamic root node can't be constant");
    }

    fn prune(&mut self, tolerance: Self::Value) -> Option<Self::Value> {
        for node in self.root.values_mut() {
            node.prune(tolerance);
        }

        // TODO: Remove empty nodes

        None
    }

    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where
        TNewValue: Value,
        TCast: Fn(Self::Value) -> TNewValue,
    {
        let root = self
            .root
            .iter()
            .map(|(key, child)| (*key, child.cast(cast).into()))
            .collect();

        RootNode { root }
    }

    fn visit_leafs_par<T: ParVisitor<Self::Leaf>>(&self, visitor: &T) {
        self.root
            .values()
            .par_bridge()
            .into_par_iter()
            .for_each(|node| node.visit_leafs_par(visitor));
    }

    fn visit_leafs<T: Visitor<Self::Leaf>>(&self, visitor: &mut T) {
        self.root
            .values()
            .for_each(|node| node.visit_leafs(visitor));
    }
}

impl<TChild: TreeNode> Accessor for RootNode<TChild> {
    type Value = TChild::Value;

    #[inline]
    fn at(&self, index: &Vector3<isize>) -> Option<&Self::Value> {
        let root_key = Self::root_key(index);

        if let Some(child) = self.root.get(&root_key) {
            child.at(index)
        } else {
            None
        }
    }

    #[inline]
    fn at_mut(&mut self, index: &Vector3<isize>) -> Option<&mut Self::Value> {
        let root_key = Self::root_key(index);
        self.root
            .get_mut(&root_key)
            .map(|child| child.at_mut(index))
            .flatten()
    }

    #[inline]
    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        let root_key = Self::root_key(index);
        self.root
            .entry(root_key)
            .or_insert_with(|| TChild::empty(root_key.0))
            .insert(index, value);
    }

    fn remove(&mut self, index: &Vector3<isize>) {
        let root_key = Self::root_key(index);

        if let Some(child) = self.root.get_mut(&root_key) {
            child.remove(index);

            if child.is_empty() {
                self.root.remove(&root_key);
            }
        }
    }
}

impl<TChild: TreeNode> FloodFill for RootNode<TChild>
where
    TChild: FloodFill,
    TChild::Value: Signed,
{
    fn flood_fill(&mut self) {
        if self.root.is_empty() {
            return;
        }

        self.root.values_mut().for_each(|c| c.flood_fill());
        let child_origins: Vec<_> = self.root.keys().copied().collect();
        let child_res = TChild::resolution() as isize;

        for (a, b) in child_origins.iter().zip(child_origins.iter().skip(1)) {
            // Check if tiles are on same z-line
            if a.x() != b.x() || a.y() != b.y() || b.z() == a.z() + child_res {
                continue;
            }

            let a_sign = self.root[a].last_value_sign();
            let b_sign = self.root[b].first_value_sign();

            // Is inside?
            if a_sign != Sign::Negative || b_sign != Sign::Negative {
                continue;
            }

            // Add tiles between inside childs
            let mut tile_origin = a.0 + Vec3i::new(0, 0, child_res);

            while tile_origin.z < b.z() {
                let key = Self::root_key(&tile_origin);
                let mut node = TChild::empty(key.0);
                node.fill_with_sign(Sign::Negative);
                self.root.insert(key, node);
                tile_origin.z += child_res;
            }
        }
    }
    
    fn fill_with_sign(&mut self, _: Sign) {
        unimplemented!("Unsupported operation")
    }

    fn first_value_sign(&self) -> Sign {
        unimplemented!("Unsupported operation")
    }

    fn last_value_sign(&self) -> Sign {
        unimplemented!("Unsupported operation")
    }
}

impl<TChild: TreeNode> Csg for RootNode<TChild>
where
    TChild: FloodFill + Csg,
    TChild::Value: Signed + Neg<Output = TChild::Value>,
{
    fn union(&mut self, mut other: Box<Self>) {
        let keys = self.root.keys()
            .chain(other.root.keys())
            .copied()
            .collect::<BTreeSet<_>>();

        for key in keys {
            match (self.root.get_mut(&key), other.root.remove(&key)) {
                (Some(n1), Some(n2)) => n1.union(n2),
                (None, Some(n2)) => { self.root.insert(key, n2); },
                (Some(_), None) | (None, None) => continue,
            };
        }
    }

    fn subtract(&mut self, mut other: Box<Self>) {
        let other_key = other.root.keys().copied().collect::<BTreeSet<_>>();

        for key in other_key {
            if let Some(node) = self.root.get_mut(&key) {
                let other_node = other.root.remove(&key).unwrap(); // It exists for sure because we are iterating over `other.root`` keys
                node.subtract(other_node);
            }
        }
    }

    fn intersect(&mut self, mut other: Box<Self>) {
        let self_keys = self.root.keys().copied().collect::<BTreeSet<_>>();
        let other_keys = other.root.keys().copied().collect::<BTreeSet<_>>();
        let nodes_outside_intersection = self_keys.symmetric_difference(&other_keys);

        for key in nodes_outside_intersection {
            self.root.remove(&key);
        }

        let keys = self.root.keys().copied().collect::<BTreeSet<_>>();
        for key in keys {
            if let Some(other_node) = other.root.remove(&key) {
                self.root.get_mut(&key).unwrap().intersect(other_node);
            }
        }
    }

    fn flip_signs(&mut self) {
        self.root.values_mut().for_each(|n| n.flip_signs());
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct RootKey(Vec3i);

impl RootKey {
    #[inline]
    fn x(&self) -> isize {
        self.0.x
    }

    #[inline]
    fn y(&self) -> isize {
        self.0.y
    }

    #[inline]
    fn z(&self) -> isize {
        self.0.z
    }
}

impl PartialOrd for RootKey {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

impl Ord for RootKey {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.data.0.cmp(&other.0.data.0)
    }
}

impl Hash for RootKey {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        let hash =
            ((1 << 8) - 1) & (self.0.x * 73856093 ^ self.0.y * 19349663 ^ self.0.z * 83492791);
        state.write_isize(hash);
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        dynamic_vdb,
        helpers::aliases::Vec3i,
        voxel::{utils::region, Accessor, FloodFill, Signed, TreeNode, Sign},
    };

    #[test]
    fn test_flood_fill() {
        type Tree = dynamic_vdb!(f32, 1);

        let mut tree = Tree::empty(Vec3i::zeros());

        let o1 = Vec3i::new(0, 4, 4);
        for i in region(o1, o1.add_scalar(2)) {
            tree.insert(&i, -1.0);
        }

        let o2 = Vec3i::new(0, 4, 10);
        for i in region(o2, o2.add_scalar(2)) {
            tree.insert(&i, -1.0);
        }

        tree.flood_fill();

        for i in region(o1, o2.add_scalar(2)) {
            assert!(tree.at(&i).is_some_and(|v| v.sign() == Sign::Negative));
        }
    }
}
