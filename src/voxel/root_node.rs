use std::{
    collections::BTreeMap,
    hash::{Hash, Hasher},
};

use nalgebra::Vector3;

use super::{utils::box_indices, Accessor, Leaf, Traverse, TreeNode, GridValue, Scalar};

pub struct RootNode<TChild: TreeNode> {
    root: BTreeMap<RootKey, Box<TChild>>,
}

impl<TChild> Traverse<TChild::LeafNode> for RootNode<TChild>
where
    TChild: TreeNode + Traverse<TChild::LeafNode>,
{
    #[inline]
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = super::Child<'a, TChild::LeafNode>> + 'a> {
        let it = self.root.values().map(|child| super::Child::Branch(child.as_ref()));
        Box::new(it)
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
    type LeafNode = TChild::LeafNode;
    type As<TValue: GridValue> = RootNode<TChild::As<TValue>>;

    #[inline]
    fn empty(_: Vector3<isize>) -> Box<Self> {
        Box::new(Self::new())
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.root.is_empty() || self.root.iter().all(|(_, node)| node.is_empty())
    }

    #[inline]
    fn traverse_leafs<F: FnMut(Leaf<Self::LeafNode>)>(&self, f: &mut F) {
        self.root
            .iter()
            .for_each(|(_, node)| node.traverse_leafs(f));
    }

    fn origin(&self) -> Vector3<isize> {
        unimplemented!("Unsupported operation. Dynamic root node has no origin");
    }

    fn fill(&mut self, _: Self::Value) {
        unimplemented!("Unsupported operation. Dynamic root node can't be filled");
    }

    fn is_constant(&self, _: Self::Value) -> Option<Self::Value> {
        unimplemented!("Unsupported operation. Dynamic root node can't be constant");
    }

    fn prune(&mut self, tolerance: Self::Value) -> Option<Self::Value> {
        for node in self.root.values_mut() {
            node.prune(tolerance);
        }
        
        // todo!("Remove empty nodes");

        None
    }

    fn cast<TNewValue, TCast>(&self, cast: &TCast) -> Self::As<TNewValue>
    where 
        TNewValue: GridValue,
        TCast: Fn(Self::Value) -> TNewValue 
    {
        let root = self.root.iter()
            .map(|(key, child)| (*key, child.cast(cast).into()))
            .collect();

        RootNode {
            root
        }
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
    fn insert(&mut self, index: &Vector3<isize>, value: Self::Value) {
        let root_key = Self::root_key(index);

        // let child = if let Some(child) = self.root.get_mut(&root_key) {
        //     child
        // } else {
        //     let new_child = TChild::empty(root_key.0);
        //     self.root.insert(root_key, new_child);
        //     self.root.get_mut(&root_key).unwrap()
        // };

        self.root.entry(root_key)
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

impl<TChild: TreeNode> RootNode<TChild> {
    #[inline]
    pub fn new() -> Self {
        Self {
            root: Default::default(),
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.root.iter().all(|(_, node)| node.is_empty())
    }

    #[inline]
    fn root_key(index: &Vector3<isize>) -> RootKey {
        return RootKey(Vector3::new(
            index.x & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.y & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.z & !((1 << TChild::BRANCHING_TOTAL) - 1),
        ));
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct RootKey(Vector3<isize>);

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
