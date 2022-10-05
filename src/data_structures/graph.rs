
pub type NodeID = usize;
pub type ArcID = usize;

pub struct Arc<TPayload> {
    node1: NodeID,
    node2: NodeID,
    payload: TPayload
}

impl<TPayload> Arc<TPayload> {
    pub fn new(node1: NodeID, node2: NodeID, payload: TPayload) -> Self { 
        return Self { node1, node2, payload };
    }

    #[inline]
    pub fn has_node(&self, node: NodeID) -> bool {
        return self.node1 == node || self.node2 == node;
    }
}

pub struct Node<TPayload> {
    arcs: Vec<ArcID>,
    payload: TPayload
}

impl<TPayload> Node<TPayload> {
    pub fn new(payload: TPayload) -> Self { 
        return Self { arcs: Vec::new(), payload };
    }
}

pub struct Graph<TVertexPayload, TEdgePayload> {
    nodes: Vec<Node<TVertexPayload>>,
    arcs: Vec<Arc<TEdgePayload>>
}

impl<TNodePayload, TArcPayload> Graph<TNodePayload, TArcPayload> {
    pub fn empty() -> Self { 
        return Self { 
            nodes: Vec::new(),
            arcs: Vec::new()
        };
    }

    #[inline]
    pub fn add_node(&mut self, node: Node<TNodePayload>) -> &mut Self {
        self.nodes.push(node);
        return self;
    }

    #[inline]
    pub fn create_node(&mut self, payload: TNodePayload) -> NodeID {
        self.nodes.push(Node::new(payload));
        return self.nodes.len() - 1;
    }

    pub fn add_arc(&mut self, n1: NodeID, n2: NodeID, payload: TArcPayload) -> ArcID {
        debug_assert!(!self.arc_exist(n1, n2));

        self.arcs.push(Arc::new(n1, n2, payload));
        let arc_id = self.arcs.len() - 1;
        self.nodes[n1].arcs.push(arc_id);
        self.nodes[n2].arcs.push(arc_id);

        return arc_id;
    }

    #[inline]
    pub fn arc_exist(&mut self, n1: NodeID, n2: NodeID) -> bool {
        debug_assert!(n1 < self.nodes.len() && n2 < self.nodes.len());
        return self.nodes[n1].arcs.iter().any(|arc| self.arcs[*arc].has_node(n2));
    }
}
