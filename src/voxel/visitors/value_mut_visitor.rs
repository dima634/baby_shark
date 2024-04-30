use crate::voxel::{TreeNode, ValueVisitorMut};
use std::marker::PhantomData;

pub struct ValueMutVisitor<TTree: TreeNode, TMutate: Fn(&mut TTree::Value)> {
    mutate: TMutate,
    _tree: PhantomData<TTree>,
}

impl<TTree, TMutate> ValueMutVisitor<TTree, TMutate>
where
    TTree: TreeNode,
    TMutate: Fn(&mut TTree::Value),
{
    #[inline]
    pub fn from_fn(func: TMutate) -> Self {
        Self {
            mutate: func,
            _tree: PhantomData,
        }
    }
}

impl<TTree, TMutate> ValueVisitorMut<TTree::Value> for ValueMutVisitor<TTree, TMutate>
where
    TTree: TreeNode,
    TMutate: Fn(&mut TTree::Value),
{
    #[inline]
    fn value(&mut self, value: &mut TTree::Value) {
        (self.mutate)(value);
    }
}
