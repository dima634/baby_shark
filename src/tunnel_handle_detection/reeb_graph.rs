
use std::{collections::HashMap, cmp::min_by};

use petgraph::{prelude::{UnGraph, NodeIndex, EdgeIndex}};

use crate::mesh::traits::{TopologicalMesh, Mesh};

pub type ReebGraph<TMesh> = UnGraph<(<TMesh as Mesh>::VertexDescriptor, <TMesh as Mesh>::ScalarType), (), usize>;

pub struct ReebGraphBuilder<'a, TMesh: TopologicalMesh> {
    vertex_node_map: HashMap<TMesh::VertexDescriptor, Node>,
    mesh: &'a TMesh,
    graph: ReebGraph<TMesh>
}

impl<'a, TMesh: TopologicalMesh> ReebGraphBuilder<'a, TMesh> {
    pub fn new(mesh: &'a TMesh) -> Self { 
        let (_, max_vertices) = mesh.vertices().size_hint();
        let (_, max_edges) = mesh.edges().size_hint();

        return Self {
            vertex_node_map: HashMap::with_capacity(max_vertices.unwrap_or(0)),
            graph: ReebGraph::<TMesh>::with_capacity(
                max_vertices.unwrap_or(0),
                max_edges.unwrap_or(0)
            ),
            mesh
        }; 
    }

    pub fn build<TReebFn>(mut self, mut reeb_func: TReebFn) -> ReebGraph<TMesh> 
    where
        TReebFn: FnMut(&TMesh::VertexDescriptor) -> TMesh::ScalarType
    {
        for vertex in self.mesh.vertices() {
            let reeb_value = reeb_func(&vertex);
            let node = self.graph.add_node((vertex, reeb_value));
            self.vertex_node_map.insert(vertex, node);
        }

        for face in self.mesh.faces() {
            let (e0, e1, e2) = self.mesh.face_edges(&face);
            let a0 = self.add_arc(&e0);
            let a1 = self.add_arc(&e1);
            let a2 = self.add_arc(&e2);


        }

        return self.graph;
    }

    #[inline]
    fn add_arc(&mut self, edge: &TMesh::EdgeDescriptor) -> Arc {
        let (v1, v2) = self.mesh.edge_vertices(edge);
        let n1 = *self.vertex_node_map.get(&v1).unwrap();
        let n2 = *self.vertex_node_map.get(&v2).unwrap();

        return self.graph.update_edge(n1, n2, ());
    }

    fn glue_by_merge_sorting<TReebFn>(&mut self, a0: Arc, a1: Arc, e0: &TMesh::EdgeDescriptor, e1: &TMesh::EdgeDescriptor) {
        while true {
            let n0 = self.bottom_node(a0);
            let n1 = self.bottom_node(a1);

            let (_, n0_value) = self.graph[n0];
            let (_, n1_value) = self.graph[n1];

            if n0_value > n1_value {
                self.merge_arcs(a0, a1);
            } else {
                self.merge_arcs(a1, a0);
            }
        }
    }

    #[inline]
    fn bottom_node(&self, arc: Arc) -> Node {
        let (n0, n1) = self.graph.edge_endpoints(arc).unwrap();

        return min_by(n0, n1, |node1, node2| {
            let (_, n0_value) = self.graph[*node1];
            let (_, n1_value) = self.graph[*node2];
            return n0_value.partial_cmp(&n1_value).unwrap();
        });
    }

    fn merge_arcs(&mut self, a1: Arc, a2: Arc) {

    }
}

type Node = NodeIndex<usize>;
type Arc = EdgeIndex<usize>;
