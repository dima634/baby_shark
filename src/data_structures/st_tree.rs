use std::cmp::{Ordering};

pub type NodeIndex = usize;


struct Node<TEdgeWeight, TNodeWeight> {
    parent: Option<(NodeIndex, TEdgeWeight)>,
    weight: TNodeWeight
}

impl<TWeight, TNodeWeight> Node<TWeight, TNodeWeight> {
    pub fn new(parent: Option<(NodeIndex, TWeight)>, weight: TNodeWeight) -> Self { 
        return Self { parent, weight };
    }
}

pub struct DynamicTree<TWeightType: PartialOrd, TNodeWeight> {
    nodes: Vec<Node<TWeightType, TNodeWeight>>
}

impl<TEdgeWeight: Ord + Copy, TNodeWeight> DynamicTree<TEdgeWeight, TNodeWeight> {
    pub fn new() -> Self {
        return Self {
            nodes: Vec::new()
        };
    }

    #[inline]
    pub fn create_node(&mut self, weight: TNodeWeight) -> NodeIndex {
        self.nodes.push(Node::new(None, weight));
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
    pub fn root_path(&self, node: NodeIndex) -> RootPath<'_, TEdgeWeight, TNodeWeight> {
        return RootPath::new(self, node);
    }

    #[inline]
    pub fn edge_weight(&self, node: NodeIndex) -> Option<TEdgeWeight> {
        return self.nodes[node].parent.and_then(|(_, weight)| Some(weight));
    }

    #[inline]
    pub fn node_weight(&self, node: NodeIndex) -> Option<&TNodeWeight> {
        return self.nodes.get(node).and_then(|n| Some(&n.weight));
    }

    #[inline]
    pub fn node_weight_mut(&mut self, node: NodeIndex) -> Option<&mut TNodeWeight> {
        return self.nodes.get_mut(node).and_then(|n| Some(&mut n.weight));
    }

    #[inline]
    pub fn weight_mut(&mut self, node: NodeIndex) -> Option<&mut TEdgeWeight> {
        return match &mut self.nodes[node].parent {
            Some(weight) => Some(&mut weight.1),
            None => None,
        };
    }

    #[inline]
    pub fn min_weight(&self, node: NodeIndex) -> Option<NodeIndex> {
        if self.nodes[node].parent.is_none() {
            return None;
        }

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
        });
    }

    #[inline]
    pub fn link(&mut self, node1: NodeIndex, node2: NodeIndex, weight: TEdgeWeight) {
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

    fn evert_node(&mut self, node: usize, new_parent: Option<(usize, TEdgeWeight)>) {
        if let Some((parent, weight)) = self.nodes[node].parent {
            self.evert_node(parent, Some((node, weight)));
            self.nodes[node].parent = new_parent;
        }
    }
}

pub struct RootPath<'a, TEdgeWeight: PartialOrd, TNodeWeight> {
    st_tree: &'a DynamicTree<TEdgeWeight, TNodeWeight>,
    current: Option<NodeIndex>
}

impl<'a, TEdgeWeight: PartialOrd, TNodeWeight> RootPath<'a, TEdgeWeight, TNodeWeight> {
    pub fn new(st_tree: &'a DynamicTree<TEdgeWeight, TNodeWeight>, from: NodeIndex) -> Self { 
        return Self { 
            st_tree, 
            current: Some(from) 
        }; 
    }
}

impl<'a, TEdgeWeight: PartialOrd + Copy, TNodeWeight> Iterator for RootPath<'a, TEdgeWeight, TNodeWeight> {
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
