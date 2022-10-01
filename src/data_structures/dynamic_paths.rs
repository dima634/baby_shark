
use bitflags::bitflags;

use crate::geometry::traits::RealNumber;

pub struct DynamicPaths<TPathCost: RealNumber> {
    nodes: Vec<Node<TPathCost>>
}

impl<TPathCost: RealNumber> DynamicPaths<TPathCost> {
    #[inline]
    pub fn path(&self, from: NodeID) -> NodeID {
        return self.path_iter(from).last().unwrap().1;
    }

    #[inline]
    pub fn path_iter(&self, from: NodeID) -> PathIter<'_, TPathCost> {
        return PathIter::new(self, from);
    }

    #[inline]
    pub fn head(&self, node: NodeID) -> NodeID {
        let node = &self.nodes[node];

        if node.is_reversed() {
            return node.tail;
        } else {
            return node.head;
        }
    }

    #[inline]
    pub fn tail(&self, node: NodeID) -> NodeID {
        let node = &self.nodes[node];

        if node.is_reversed() {
            return node.head;
        } else {
            return node.tail;
        }
    }

    pub fn before(&self, external_node: NodeID) -> NodeID {
        debug_assert!(self.nodes[external_node].is_external());

        for (current_idx, parent_idx) in self.path_iter(external_node) {
            let parent = &self.nodes[parent_idx];

            if parent.right == current_idx {
                let sibling = &self.nodes[parent.left];

                if sibling.is_external() {
                    return parent.left;
                } else {
                    return self.tail(parent.left);
                }
            }
        }

        panic!();
    }

    pub fn cost(&self, external_node: NodeID) -> TPathCost {
        debug_assert!(self.nodes[external_node].is_external());

        for (current_idx, parent_idx) in self.path_iter(external_node) {
            let parent = &self.nodes[parent_idx];

            if parent.left == current_idx {
                return self.gross_cost(parent_idx);
            }
        }

        panic!();
    }

    pub fn min_cost(&self, external_node: NodeID) -> TPathCost {
        debug_assert!(self.nodes[external_node].is_external());

        todo!();
    }

    #[inline]
    pub fn update(&mut self, internal_node: NodeID, additional_cost: TPathCost) {
        debug_assert!(self.nodes[internal_node].is_internal());

        self.nodes[internal_node].net_min += additional_cost;
    }

    #[inline]
    pub fn reverse(&mut self, internal_node: NodeID) {
        debug_assert!(self.nodes[internal_node].is_internal());

        self.nodes[internal_node].reverse();
    }

    fn gross_min(&self, internal_node: NodeID) -> TPathCost {
        debug_assert!(!self.nodes[internal_node].is_internal());

        let mut gross_min = self.nodes[internal_node].net_min;

        for (_, parent_idx) in self.path_iter(internal_node).skip(1) {
            let parent = &self.nodes[parent_idx];
            gross_min += parent.net_min;
        }

        return gross_min;
    }

    #[inline]
    fn gross_cost(&self, internal_node: NodeID) -> TPathCost {
        debug_assert!(!self.nodes[internal_node].is_internal());

        return self.nodes[internal_node].net_cost + self.gross_min(internal_node);
    }
}

pub struct PathIter<'a, TPathCost: RealNumber> {
    paths: &'a DynamicPaths<TPathCost>,
    current_index: NodeID
}

impl<'a, TPathCost: RealNumber> PathIter<'a, TPathCost> {
    pub fn new(paths: &'a DynamicPaths<TPathCost>, from: NodeID) -> Self {
        return Self { paths, current_index: from } ;
    }
}

impl<TPathCost: RealNumber> Iterator for PathIter<'_, TPathCost> {
    type Item = (NodeID, NodeID);

    fn next(&mut self) -> Option<Self::Item> {
        let current_node = &self.paths.nodes[self.current_index];

        return match current_node.parent {
            Some(parent_index) => {
                let current_index = self.current_index;
                self.current_index = parent_index;

                return Some((current_index, parent_index));
            },
            None => None,
        };
    }
}

bitflags! {
    pub struct Bits: u8 {
        const REVERSED = 0b00000001;
        const EXTERNAL = 0b00000010;
    }
}

pub type NodeID = usize;
pub struct Node<TCostType: RealNumber> {
    bits: Bits,

    parent: Option<NodeID>,
    left: NodeID,
    right: NodeID,
    head: NodeID,
    tail: NodeID,

    net_min: TCostType,
    net_cost: TCostType,
}

impl<TCostType: RealNumber> Node<TCostType> {
    #[inline]
    pub fn is_reversed(&self) -> bool {
        return self.bits.contains(Bits::REVERSED);
    }

    #[inline]
    pub fn is_external(&self) -> bool {
        return self.bits.contains(Bits::EXTERNAL);
    }

    #[inline]
    pub fn is_internal(&self) -> bool {
        return !self.is_external();
    }

    #[inline]
    pub fn reverse(&mut self) {
        self.bits.toggle(Bits::REVERSED);
    }
}
