use std::{collections::{HashMap, BTreeMap, HashSet}, mem::swap, cmp::min_by};

use nalgebra::RealField;
use petgraph::{prelude::{UnGraph, NodeIndex, EdgeIndex}, data::Build};

use crate::{mesh::traits::{TopologicalMesh, Mesh, MeshMarker, Marker}, geometry::traits::RealNumber};

pub struct ReebNodeWeight<TMesh: Mesh> {
    vertex: TMesh::VertexDescriptor,
    reeb_value: TMesh::ScalarType
}

impl<TMesh: Mesh> ReebNodeWeight<TMesh> {
    pub fn new(vertex: TMesh::VertexDescriptor, reeb_value: TMesh::ScalarType) -> Self { 
        return Self { vertex, reeb_value };
    }

    #[inline]
    pub fn vertex(&self) -> &TMesh::VertexDescriptor {
        return &self.vertex;
    }

    #[inline]
    pub fn reeb_value(&self) -> &TMesh::ScalarType {
        return &self.reeb_value;
    }
}

pub struct ReebArcWeight<TMesh: Mesh> {
    edges: HashSet<TMesh::EdgeDescriptor>
}

pub type ReebGraph<TMesh> = UnGraph<ReebNodeWeight<TMesh>, HashSet<<TMesh as Mesh>::EdgeDescriptor>, usize>;

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
            let node = self.graph.add_node(ReebNodeWeight::new(vertex, reeb_value));
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

// #[derive(PartialEq, PartialOrd, Clone, Copy)]
// struct PreimageWeight<TScalar: RealNumber>(TScalar);

// impl<TScalar: RealNumber> Eq for PreimageWeight<TScalar> {}

// impl<TScalar: RealNumber> Ord for PreimageWeight<TScalar> {
//     #[inline]
//     fn cmp(&self, other: &Self) -> std::cmp::Ordering {
//         return self.partial_cmp(&other).unwrap();
//     }
// }

// type Edge<TMesh> = (<TMesh as Mesh>::VertexDescriptor, <TMesh as Mesh>::VertexDescriptor);

// struct DynTrees<TMesh: Mesh> {
//     st_tree: STTree<PreimageWeight<TMesh::ScalarType>>,
//     edge_node_map: HashMap<(TMesh::VertexDescriptor, TMesh::VertexDescriptor), NodeIndex>
// }

// impl<TMesh: Mesh> DynTrees<TMesh> {
//     fn new() -> Self { 
//         return Self { 
//             st_tree: STTree::new(),
//             edge_node_map: HashMap::new()
//         } ;
//     }

//     #[inline]
//     pub fn find(&self, edge: &Edge<TMesh>) -> Option<NodeIndex> {
//         let n = self.edge_node_map.get(edge);
//         return n.and_then(|node| self.st_tree.root(*node));
//     }

//     pub fn insert(&mut self, edge1: &Edge<TMesh>, edge2: &Edge<TMesh>, weight: PreimageWeight<TMesh::ScalarType>) {
//         let n1 = self.get_node(edge1);
//         let n2 = self.get_node(edge2);

//         if self.st_tree.root(n1) == self.st_tree.root(n2) {
//             self.st_tree.evert(n1);
//             let x = self.st_tree.min_weight(n2);
//             let x_weight = self.st_tree.weight(x).unwrap();

//             if x_weight < weight {
//                 self.st_tree.cut(x, self.st_tree.parent(x).unwrap()).expect("Should cut");
//                 self.st_tree.link(n1, n2, weight);
//             }
//         } else {
//             self.st_tree.link(n1, n2, weight);
//         }
//     }

//     pub fn delete(&mut self, edge1: &Edge<TMesh>, edge2: &Edge<TMesh>) {
//         let n1 = self.get_node(edge1);
//         let n2 = self.get_node(edge2);
//         let _ = self.st_tree.cut(n1, n2);
//     }

//     #[inline]
//     pub fn create_node(&mut self) -> NodeIndex {
//         return self.st_tree.create_node();
//     }

//     fn get_node(&mut self, edge: &Edge<TMesh>) -> NodeIndex {
//         let existing_node = self.edge_node_map.get(&edge);

//         return match existing_node {
//             Some(node) => *node,
//             None => {
//                 let new_node = self.create_node();
//                 self.edge_node_map.insert(*edge, new_node);
//                 return new_node;
//             },
//         };
//     }
// }

// pub struct ReebGraphBuilder<'a, TMesh: TopologicalMesh + MeshMarker> {
//     dyn_trees: DynTrees<TMesh>,
//     mesh: &'a TMesh,
//     graph: ReebGraph<TMesh>,
//     vertex_reeb_node_map: HashMap<TMesh::VertexDescriptor, petgraph::prelude::NodeIndex<usize>>
// }

// impl<'a, TMesh: TopologicalMesh + MeshMarker> ReebGraphBuilder<'a, TMesh> {
//     pub fn from(mesh: &'a TMesh) -> Self { 
//         let (_, max_vertices) = mesh.vertices().size_hint();
//         let (_, max_edges) = mesh.edges().size_hint();

//         return Self {
//             dyn_trees: DynTrees::new(),
//             graph: ReebGraph::<TMesh>::with_capacity(
//                 max_vertices.unwrap_or(0),
//                 max_edges.unwrap_or(0)
//             ),
//             vertex_reeb_node_map: HashMap::new(),
//             mesh
//         }; 
//     }

//     pub fn build<TReebFn>(mut self, mut reeb_func: TReebFn) -> ReebGraph<TMesh> 
//     where
//         TReebFn: FnMut(&TMesh::VertexDescriptor) -> TMesh::ScalarType
//     {
//         let mut marker = self.mesh.marker();
//         let mut reeb_values_map = BTreeMap::new();

//         for vertex in self.mesh.vertices() {
//             let reeb_value = reeb_func(&vertex);
//             reeb_values_map.insert(vertex, reeb_value);
//         }

//         for vertex in reeb_values_map.keys() {
//             let lower = self.lower_components(&vertex, &mut marker, &reeb_values_map);
            
//         }

//         return self.graph;
//     }

//     fn lower_components(
//         &self, 
//         v: &TMesh::VertexDescriptor, 
//         marker: &mut TMesh::Marker,
//         reeb_values_map: &BTreeMap<TMesh::VertexDescriptor, TMesh::ScalarType>
//     ) -> Vec<TMesh::VertexDescriptor> {
//         let mut lower = Vec::new();

//         self.mesh.edges_around_vertex(v, |edge| {
//             let (mut v1, mut v2) = self.mesh.edge_vertices(edge);
//             let mut r_val1 = reeb_values_map.get(&v1).unwrap();
//             let mut r_val2 = reeb_values_map.get(&v2).unwrap();

//             if r_val1 > r_val2 {
//                 swap(&mut v1, &mut v2);
//                 swap(&mut r_val1, &mut r_val2);
//             }

//             if &v2 == v { // && !marker.is_edge_marked(edge) {
//                 if let Some(component) = self.dyn_trees.find(&(v1, v2)) {
//                     lower.push(v1);
//                     // marker.mark_edge(edge, true);
//                 }
//             }
//         });

//         return lower;
//     }

//     fn upper_components(
//         &self, 
//         v: &TMesh::VertexDescriptor, 
//         marker: &mut TMesh::Marker,
//         reeb_values_map: &BTreeMap<TMesh::VertexDescriptor, TMesh::ScalarType>
//     ) -> Vec<TMesh::VertexDescriptor> {
//         let mut upper = Vec::new();

//         self.mesh.edges_around_vertex(v, |edge| {
//             let (mut v1, mut v2) = self.mesh.edge_vertices(edge);
//             let mut r_val1 = reeb_values_map.get(&v1).unwrap();
//             let mut r_val2 = reeb_values_map.get(&v2).unwrap();

//             if r_val1 > r_val2 {
//                 swap(&mut v1, &mut v2);
//                 swap(&mut r_val1, &mut r_val2);
//             }

//             if &v1 == v { // && !marker.is_edge_marked(edge) {
//                 if let Some(component) = self.dyn_trees.find(&(v1, v2)) {
//                     upper.push(v2);
//                     // marker.mark_edge(edge, true);
//                 }
//             }
//         });

//         return upper;
//     }

//     fn update_preimage_graph(
//         &mut self, 
//         v: &TMesh::VertexDescriptor, 
//         reeb_values_map: &BTreeMap<TMesh::VertexDescriptor, TMesh::ScalarType>
//     ) {
//         self.mesh.faces_around_vertex(v, |f| {
//             let (mut v1, mut v2, mut v3) = self.mesh.face_vertices(f);
//             let mut v1_rv = reeb_values_map[&v1];
//             let mut v2_rv = reeb_values_map[&v2];
//             let mut v3_rv = reeb_values_map[&v3];

//             if v1_rv > v3_rv {
//                 swap(&mut v1, &mut v3);
//                 swap(&mut v1_rv, &mut v3_rv);
//             }

//             if v1_rv > v2_rv {
//                 swap(&mut v1, &mut v2);
//                 swap(&mut v1_rv, &mut v2_rv);
//             }

//             if v2_rv > v3_rv {
//                 swap(&mut v2, &mut v3);
//                 swap(&mut v2_rv, &mut v3_rv);
//             }

//             if v == &v3 {
//                 self.dyn_trees.delete(&(v1, *v), &(v2, *v));
//             }

//             if v == &v2 {
//                 self.dyn_trees.delete(&(v1, v3), &(v1, *v));
//                 self.dyn_trees.insert(&(*v, v3), &(v1, v3), PreimageWeight(reeb_values_map[&v3]));
//             }

//             if v == &v1 {
//                 let weight = PreimageWeight(reeb_values_map[&v2].min(reeb_values_map[&v3]));
//                 self.dyn_trees.insert(&(*v, v2), &(*v, v3), weight);
//             }
//         });
//     }

//     fn update_reeb_graph(
//         &mut self, 
//         v: &TMesh::VertexDescriptor,  
//         upper: Vec<TMesh::VertexDescriptor>, 
//         lower: Vec<TMesh::VertexDescriptor>,
//         reeb_values_map: &BTreeMap<TMesh::VertexDescriptor, TMesh::ScalarType>
//     ) {
//         let node = self.graph.add_node((*v, reeb_values_map[v]));

//         for lower_component in lower {
//             let lower_node = self.vertex_reeb_node_map[&lower_component];
//             self.graph.update_edge(lower_node, node, ());
//         }

//         for upper_component in upper {
//             let upper_node = self.vertex_reeb_node_map.get(&upper_component).unwrap_or(self.graph.add_node(weight));
//             self.graph.update_edge(node, upper_node, ());
//         }
//     }
// }
