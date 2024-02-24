use super::*;
use rayon::prelude::*;

impl<TChild> TreeNode for RootNode<TChild>
where
    TChild: TreeNode,
{
    const BRANCHING: usize = usize::MAX;
    const BRANCHING_TOTAL: usize = usize::MAX;
    const SIZE: usize = usize::MAX;

    const IS_LEAF: bool = false;

    type Value = TChild::Value;
    type Child = TChild;
    type Leaf = TChild::Leaf;
    type As<TValue: Value> = RootNode<TChild::As<TValue>>;

    #[inline]
    fn at(&self, index: &Vec3i) -> Option<&Self::Value> {
        let root_key = Self::root_key(index);
        self.root
            .get(&root_key)
            .map(|child| child.at(index))
            .flatten()
    }

    #[inline]
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value> {
        let root_key = Self::root_key(index);
        self.root
            .get_mut(&root_key)
            .map(|child| child.at_mut(index))
            .flatten()
    }

    #[inline]
    fn insert(&mut self, index: &Vec3i, value: Self::Value) {
        let root_key = Self::root_key(index);
        self.root
            .entry(root_key)
            .or_insert_with(|| TChild::empty(root_key.0))
            .insert(index, value);
    }

    fn remove(&mut self, index: &Vec3i) {
        let root_key = Self::root_key(index);

        if let Some(child) = self.root.get_mut(&root_key) {
            child.remove(index);

            if child.is_empty() {
                self.root.remove(&root_key);
            }
        }
    }

    #[inline]
    fn empty(_: Vec3i) -> Box<Self> {
        Box::new(Self::new())
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.root.is_empty() || self.root.iter().all(|(_, node)| node.is_empty())
    }

    fn origin(&self) -> Vec3i {
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
