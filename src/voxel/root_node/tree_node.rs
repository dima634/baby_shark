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
        self.root.get(&root_key).and_then(|child| child.at(index))
    }

    #[inline]
    fn at_mut(&mut self, index: &Vec3i) -> Option<&mut Self::Value> {
        let root_key = Self::root_key(index);
        self.root
            .get_mut(&root_key)
            .and_then(|child| child.at_mut(index))
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

    #[inline]
    fn clear(&mut self) {
        self.root.clear();
    }

    fn clone_map<TNewValue, TMap>(&self, map: &TMap) -> Box<Self::As<TNewValue>>
    where
        TNewValue: Value,
        TMap: Fn(Self::Value) -> TNewValue,
    {
        let root = self
            .root
            .iter()
            .map(|(key, child)| (*key, child.clone_map(map).into()))
            .collect();

        Box::new(RootNode { root })
    }

    fn clone(&self) -> Box<Self> {
        let root = self
            .root
            .iter()
            .map(|(key, child)| (*key, (*child).clone()))
            .collect();

        Box::new(RootNode { root })
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

    fn touch_leaf_at(&mut self, index: &Vec3i) -> LeafMut<'_, Self::Leaf> {
        let root_key = Self::root_key(index);
        self.root
            .entry(root_key)
            .or_insert_with(|| TChild::empty(root_key.0))
            .touch_leaf_at(index)
    }

    fn values(&self) -> impl Iterator<Item = Option<Self::Value>> {
        self.root.values().flat_map(|node| node.values())
    }

    fn visit_values_mut<T: ValueVisitorMut<Self::Value>>(&mut self, visitor: &mut T) {
        self.root
            .values_mut()
            .for_each(|node| node.visit_values_mut(visitor));
    }

    #[inline]
    fn leaf_at(&self, index: &Vec3i) -> Option<&Self::Leaf> {
        let root_key = Self::root_key(index);
        let child = self.root.get(&root_key)?;
        child.leaf_at(index)
    }

    fn take_leaf_at(&mut self, index: &Vec3i) -> Option<Box<Self::Leaf>> {
        let root_key = Self::root_key(index);
        let child = self.root.get_mut(&root_key)?;
        child.take_leaf_at(index)
    }

    fn insert_leaf_at(&mut self, leaf: Box<Self::Leaf>) {
        let index = leaf.origin();
        let root_key = Self::root_key(&index);
        self.root
            .entry(root_key)
            .or_insert_with(|| TChild::empty(root_key.0))
            .insert_leaf_at(leaf);
    }

    fn prune_empty_nodes(&mut self) {
        self.root.retain(|_, node| {
            node.prune_empty_nodes();
            !node.is_empty()
        });
    }
}
