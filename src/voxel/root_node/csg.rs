use super::*;
use std::collections::BTreeSet;

impl<TChild: TreeNode> Csg for RootNode<TChild>
where
    TChild: FloodFill + Csg,
    TChild::Value: Signed + Neg<Output = TChild::Value>,
{
    fn union(&mut self, mut other: Box<Self>) {
        let keys = self
            .root
            .keys()
            .chain(other.root.keys())
            .copied()
            .collect::<BTreeSet<_>>();

        for key in keys {
            match (self.root.get_mut(&key), other.root.remove(&key)) {
                (Some(n1), Some(n2)) => n1.union(n2),
                (None, Some(n2)) => {
                    self.root.insert(key, n2);
                }
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
            self.root.remove(key);
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
