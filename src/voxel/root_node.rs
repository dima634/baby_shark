use std::{collections::BTreeMap, marker::PhantomData};

use nalgebra::Vector3;

use super::{utils::box_indices, HasChild, TreeNode, Accessor, Leaf, Traverse};

pub struct RootNode<TChild: TreeNode, TLeaf: TreeNode> {
    root: BTreeMap<RootKey, TChild>,
    leaf_type: PhantomData<TLeaf>
}

impl<TChild, TLeaf> Traverse<TLeaf> for RootNode<TChild, TLeaf> 
where 
    TChild: TreeNode + Traverse<TLeaf>, 
    TLeaf: TreeNode 
{
    #[inline]
    fn childs<'a>(&'a self) -> Box<dyn Iterator<Item = super::Child<'a, TLeaf>> + 'a> {
        let it = self.root.values().map(|child| super::Child::Branch(child));
        Box::new(it)
    }
}

impl<TChild, TLeaf> TreeNode for RootNode<TChild, TLeaf> 
where 
    TChild: TreeNode<LeafNode = TLeaf>, 
    TLeaf: TreeNode
{
    const BRANCHING: usize = usize::MAX;
    const BRANCHING_TOTAL: usize = usize::MAX;
    const SIZE: usize = usize::MAX;

    const IS_LEAF: bool = false;

    type LeafNode = TLeaf;

    fn new_inactive(origin: Vector3<isize>) -> Self {
        todo!()
    }

    fn new_active(origin: Vector3<isize>) -> Self {
        todo!()
    }

    fn is_empty(&self) -> bool {
        todo!()
    }

    #[inline]
    fn traverse_leafs<F: FnMut(Leaf<Self::LeafNode>)>(&self, f: &mut F) {
        self.root.iter().for_each(|(_, node)| node.traverse_leafs(f));
    }

    fn origin(&self) -> Vector3<isize> {
        todo!()
    }
}

impl<TChild: TreeNode, TLeaf: TreeNode> Accessor for RootNode<TChild, TLeaf> {
    fn at(&self, index: &Vector3<isize>) -> bool {
        let root_key = Self::root_key(index);

        if let Some(child) = self.root.get(&root_key) {
            return child.at(index);
        }

        return false;
    }

    fn insert(&mut self, index: &Vector3<isize>) {
        let root_key = Self::root_key(index);

        let child = if let Some(child) = self.root.get_mut(&root_key) {
            child
        } else {
            let new_child = TChild::new_inactive(root_key.0);
            self.root.insert(root_key, new_child);
            self.root.get_mut(&root_key).unwrap()
        };

        child.insert(index);
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

impl<TChild: TreeNode, TLeaf: TreeNode> RootNode<TChild, TLeaf> {
    #[inline]
    pub fn new() -> Self {
        return Self {
            root: BTreeMap::new(),
            leaf_type: PhantomData
        };
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        return self.root.iter().all(|(_, node)| node.is_empty());
    }

    ///
    /// Inside is positive
    /// 
    pub fn from_singed_scalar_field<T: Fn(&Vector3<f32>) -> f32>(
        grid_size: usize,
        min: f32,
        max: f32,
        f: T,
    ) -> Self {
        let mut tree = Self::new();

        let origin = Vector3::new(min, min, min);
        let spacing = (max - min) / grid_size as f32;

        for idx in box_indices(0, grid_size as isize) {
            let p = origin + Vector3::new(
                idx.x as f32 * spacing, 
                idx.y as f32 * spacing, 
                idx.z as f32 * spacing
            );

            if f(&p) < 0.0 {
                continue;
            }

            tree.insert(&idx);
        }

        return tree;
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

impl<TChild: TreeNode, TLeaf: TreeNode> HasChild for RootNode<TChild, TLeaf> {
    type Child = TChild;
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
