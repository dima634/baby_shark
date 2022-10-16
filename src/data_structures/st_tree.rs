use std::cmp::{Ordering};

pub type NodeIndex = usize;

pub struct STTree<TWeightType: PartialOrd> {
    nodes: Vec<Node<TWeightType>>
}

impl<TWeight: Ord + Copy> STTree<TWeight> {
    pub fn new() -> Self {
        return Self {
            nodes: Vec::new()
        };
    }

    #[inline]
    pub fn create_node(&mut self) -> NodeIndex {
        self.nodes.push(Node::new(None));
        return self.nodes.len() - 1;
    }

    #[inline]
    pub fn parent(&self, node: NodeIndex) -> Option<NodeIndex> {
        return self.nodes[node].parent.and_then(|(parent_index, _)| Some(parent_index));
    }

    #[inline]
    pub fn root(&self, node: NodeIndex) -> Option<NodeIndex> {
        return self.root_path(node).last();
    }

    #[inline]
    pub fn root_path(&self, node: NodeIndex) -> RootPath<'_, TWeight> {
        return RootPath::new(self, node);
    }

    #[inline]
    pub fn weight(&self, node: NodeIndex) -> Option<TWeight> {
        return self.nodes[node].parent.and_then(|(_, weight)| Some(weight));
    }

    #[inline]
    pub fn min_weight(&self, node: NodeIndex) -> NodeIndex {
        debug_assert!(self.nodes[node].parent.is_some());

        return self.root_path(node).min_by(|n1, n2| {
            let node1 = &self.nodes[*n1];
            let node2 = &self.nodes[*n2];

            if node1.parent.is_none() {
                return Ordering::Greater;
            }

            if node2.parent.is_none() {
                return Ordering::Less;
            }

            return node1.parent.unwrap().1.cmp(&node2.parent.unwrap().1);
        }).unwrap();
    }

    #[inline]
    pub fn link(&mut self, node1: NodeIndex, node2: NodeIndex, weight: TWeight) {
        debug_assert!(self.root(node1) != self.root(node2));

        self.evert(node1);
        let n1 = &mut self.nodes[node1];
        n1.parent = Some((node2, weight));
    }

    pub fn cut(&mut self, n1: NodeIndex, n2: NodeIndex) -> Result<(), ()> {
        let node1 = &self.nodes[n1];
        let node2 = &self.nodes[n1];

        match (node1.parent, node2.parent) {
            (Some((n1_parent, _)), _) if n1_parent == n2 => {
                self.nodes[n1].parent = None;
                return Ok(());
            },
            (_, Some((n2_parent, _))) if n2_parent == n1 => {
                self.nodes[n2].parent = None;
                return Ok(());
            },
            _ => Err(())
        }
    }

    #[inline]
    pub fn evert(&mut self, node: NodeIndex) {
        self.evert_node(node, None);
    }

    fn evert_node(&mut self, node: usize, new_parent: Option<(usize, TWeight)>) {
        if let Some((_, weight)) = self.nodes[node].parent {
            self.evert_node(node, Some((node, weight)));
            self.nodes[node].parent = new_parent;
        }
    }
}

pub struct RootPath<'a, TWeight: PartialOrd> {
    st_tree: &'a STTree<TWeight>,
    current: Option<NodeIndex>
}

impl<'a, TCostType: PartialOrd> RootPath<'a, TCostType> {
    pub fn new(st_tree: &'a STTree<TCostType>, from: NodeIndex) -> Self { 
        return Self { 
            st_tree, 
            current: Some(from) 
        }; 
    }
}

impl<'a, TCostType: PartialOrd + Copy> Iterator for RootPath<'a, TCostType> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<Self::Item> {
        return match self.current {
            Some(current_index) => {
                let parent = &self.st_tree.nodes[current_index].parent;
                self.current = parent.and_then(|(parent_index, _)| Some(parent_index));
                return Some(current_index);
            },
            None => None,
        };
    }
}

struct Node<TWeight> {
    parent: Option<(NodeIndex, TWeight)>
}

impl<TWeight> Node<TWeight> {
    pub fn new(parent: Option<(NodeIndex, TWeight)>) -> Self { 
        return Self { parent };
    }
}
