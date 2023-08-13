use std::collections::HashMap;

use nalgebra::Vector3;

use super::traits::{TreeNode, HasChild};

pub struct RootNode<TChild: TreeNode> {
    root: HashMap<Vector3<usize>, TChild>
}

impl<TChild: TreeNode> RootNode<TChild> {
    #[inline]
    pub fn empty() -> Self {
        return Self {
            root: HashMap::new()
        };
    }
}

impl<TChild: TreeNode> HasChild for RootNode<TChild> {
    type Child = TChild;
}

impl<TChild: TreeNode> TreeNode for RootNode<TChild> {
    const BRANCHING: usize = usize::MAX;
    const BRANCHING_TOTAL: usize = usize::MAX;
    const SIZE: usize = usize::MAX;

    fn at(&self, index: Vector3<usize>) -> bool {
        let root_key = Vector3::new(
            index.x & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.y & !((1 << TChild::BRANCHING_TOTAL) - 1),
            index.z & !((1 << TChild::BRANCHING_TOTAL) - 1)
        );

        println!("{}", root_key);

        return false;
    }

    fn insert(&mut self, index: Vector3<usize>) {
        todo!()
    }

    fn remove(&mut self, index: Vector3<usize>) {
        todo!()
    }

    #[inline]
    fn is_empty(&self) -> bool {
        return self.root.iter()
            .all(|(_, node)| node.is_empty());
    }
}

