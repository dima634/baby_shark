use std::cmp::{Ordering};

pub type NodeIndex = usize;

pub struct STTree<TCostType: PartialOrd> {
    nodes: Vec<Node<TCostType>>
}

impl<TWeightType: PartialOrd + Copy> STTree<TWeightType> {
    pub fn new() -> Self {
        return Self {
            nodes: Vec::new()
        };
    }

    #[inline]
    pub fn parent(&self, node: NodeIndex) -> Option<NodeIndex> {
        return self.nodes[node].parent;
    }

    #[inline]
    pub fn root(&self, node: NodeIndex) -> Option<NodeIndex> {
        return self.root_path(node).last();
    }

    #[inline]
    pub fn root_path(&self, node: NodeIndex) -> RootPath<'_, TWeightType> {
        return RootPath::new(self, node);
    }

    #[inline]
    pub fn weight(&self, node: NodeIndex) -> &TWeightType {
        return &self.nodes[node].weight;
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

            let order = node1.weight.partial_cmp(&node2.weight);
            return order.unwrap_or(Ordering::Greater);
        }).unwrap();
    }

    #[inline]
    pub fn link(&mut self, node1: NodeIndex, node2: NodeIndex, weight: TWeightType) {
        debug_assert!(self.root(node1) != self.root(node2));

        self.evert(node1);
        let n1 = &mut self.nodes[node1];
        n1.parent = Some(node2);
        n1.weight = weight;
    }

    pub fn cut(&mut self, n1: NodeIndex, n2: NodeIndex) {
        let node1 = &self.nodes[n1];
        let node2 = &self.nodes[n1];

        match (node1.parent, node2.parent) {
            (None, None) => debug_assert!(false),
            (None, Some(n2_parent)) => {
                debug_assert!(n2_parent == n1);
                self.nodes[n2].parent = None;
            },
            (Some(n1_parent), None) => {
                debug_assert!(n1_parent == n2);
                self.nodes[n1].parent = None;
            },
            (Some(n1_parent), Some(n2_parent)) => {
                if n1_parent == n2 {
                    self.nodes[n1].parent = None;
                } else if n2_parent == n1  {
                    self.nodes[n2].parent = None;
                } else {
                    debug_assert!(false);
                }
            },
        }
    }

    pub fn evert(&mut self, node: NodeIndex) {
        if self.root(node).is_none() {
            return;
        }

        let mut current = node;
        let current_node = &self.nodes[current];

        let mut parent = current_node.parent;
        let mut parent_weight = current_node.weight;
        let parent_node = &self.nodes[current_node.parent.unwrap()];
  
        let mut grand_parent = parent_node.parent;
        let mut grand_parent_weight = parent_node.weight;
  
        self.nodes[current].parent = None;
  
        // Reverse all nodes until the root
        loop {
            let parent_node = &mut self.nodes[parent.unwrap()];
            parent_node.parent = Some(current);
            parent_node.weight = parent_weight;
    
            current = parent.unwrap();
            parent = grand_parent;
            parent_weight = grand_parent_weight;
    
            if let Some(grand_parent_index) = grand_parent {
                let grand_parent_node = &mut self.nodes[grand_parent_index];
                grand_parent_weight = grand_parent_node.weight;
                grand_parent = grand_parent_node.parent;
            } else {
                break;
            }
        }
    }
}

pub struct RootPath<'a, TCostType: PartialOrd> {
    st_tree: &'a STTree<TCostType>,
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

impl<'a, TCostType: PartialOrd> Iterator for RootPath<'a, TCostType> {
    type Item = NodeIndex;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        return match self.current {
            Some(current_index) => {
                self.current = self.st_tree.nodes[current_index].parent;
                return Some(current_index);
            },
            None => None,
        };
    }
}

struct Node<TCostType> {
    weight: TCostType,
    parent: Option<NodeIndex>
}

impl<TCostType> Node<TCostType> {
    pub fn new(weight: TCostType, parent: Option<NodeIndex>) -> Self { 
        return Self { weight, parent };
    }

    #[inline]
    pub fn is_root(&self) -> bool {
        return self.parent.is_none();
    }
}
