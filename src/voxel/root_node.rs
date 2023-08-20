use std::collections::BTreeMap;

use nalgebra::Vector3;

use super::{TreeNode, HasChild};

pub struct RootNode<TChild: TreeNode> {
    root: BTreeMap<RootKey, TChild>
}

impl<TChild: TreeNode> RootNode<TChild> {
    #[inline]
    pub fn new() -> Self {
        return Self {
            root: BTreeMap::new()
        };
    }

    pub fn at(&self, index: &Vector3<usize>) -> bool {
        let root_key = Self::root_key(index);

        if let Some(child) = self.root.get(&root_key) {
            return child.at(index);
        }

        return false;
    }

    pub fn insert(&mut self, index: &Vector3<usize>) {
        let root_key = Self::root_key(index);

        let child = 
            if let Some(child) = self.root.get_mut(&root_key) {
                child
            } else {
                let new_child = TChild::new_inactive(root_key.0);
                self.root.insert(root_key, new_child);
                self.root.get_mut(&root_key).unwrap()
            };

        child.insert(index);
    }

    pub fn remove(&mut self, index: &Vector3<usize>) {
        let root_key = Self::root_key(index);
        
        if let Some(child) = self.root.get_mut(&root_key) {
            child.remove(index);
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        return self.root.iter()
            .all(|(_, node)| node.is_empty());
    }

    #[inline]
    fn root_key(index: &Vector3<usize>) -> RootKey {
        return RootKey(Vector3::new( 
            index.x & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.y & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.z & !((1 << TChild::BRANCHING_TOTAL) - 1)
        ));
    }
}

impl<TChild: TreeNode> HasChild for RootNode<TChild> {
    type Child = TChild;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct RootKey(Vector3<usize>);

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
