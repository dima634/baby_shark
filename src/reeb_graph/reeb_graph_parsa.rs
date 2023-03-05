// use std::{collections::{HashMap, HashSet}, mem::swap};

// use petgraph::prelude::{DiGraph, NodeIndex};

// use crate::{geometry::traits::RealNumber, mesh::traits::{Mesh, TopologicalMesh, VertexProperties}, data_structures::st_tree::DynamicTree};

// use super::ordered_triangle::{OrderedTriangle, ReebValue};

// pub type ReebGraph<TMesh> = DiGraph<<TMesh as Mesh>::ScalarType, usize, usize>;

// #[derive(Clone, Copy)]
// struct PreimageWeight<TScalar: RealNumber> {
//     reeb_value: TScalar,
//     marked: bool
// }

// impl<TScalar: RealNumber> PreimageWeight<TScalar> {
//     fn new(reeb_value: TScalar) -> Self { 
//         return Self { reeb_value, marked: false };
//     }
// }

// impl<TScalar: RealNumber> PartialEq for PreimageWeight<TScalar> {
//     #[inline]
//     fn eq(&self, other: &Self) -> bool {
//         return self.reeb_value == other.reeb_value;
//     }
// }

// impl<TScalar: RealNumber> Eq for PreimageWeight<TScalar> {}

// impl<TScalar: RealNumber> PartialOrd for PreimageWeight<TScalar> {
//     #[inline]
//     fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
//         return self.reeb_value.partial_cmp(&other.reeb_value);
//     }
// }

// impl<TScalar: RealNumber> Ord for PreimageWeight<TScalar> {
//     #[inline]
//     fn cmp(&self, other: &Self) -> std::cmp::Ordering {
//         return self.partial_cmp(&other).unwrap();
//     }
// }

// struct DynTrees<TMesh: Mesh> {
//     dyn_tree: DynamicTree<PreimageWeight<TMesh::ScalarType>, ()>,
//     edge_node_map: HashMap<TMesh::EdgeDescriptor, usize>
// }

// impl<TMesh: Mesh> DynTrees<TMesh> {
//     fn new() -> Self { 
//         return Self { 
//             dyn_tree: DynamicTree::new(),
//             edge_node_map: HashMap::new()
//         };
//     }

//     #[inline]
//     pub fn find(&mut self, edge: &TMesh::EdgeDescriptor) -> Option<usize> {
//         let n = self.edge_node_map.get(edge);
//         return n.and_then(|node| self.dyn_tree.root(*node));
//     }

//     pub fn insert(&mut self, edge1: &TMesh::EdgeDescriptor, edge2: &TMesh::EdgeDescriptor, weight: PreimageWeight<TMesh::ScalarType>) -> Result<(), ()> {
//         let n1 = self.get_node(edge1);
//         let n2 = self.get_node(edge2);

//         if self.dyn_tree.root(n1) == self.dyn_tree.root(n2) {
//             self.dyn_tree.evert(n1);
//             let x = self.dyn_tree.min_weight(n2).unwrap();
//             let x_weight = self.dyn_tree.edge_weight(x).unwrap();

//             if x_weight < weight {
//                 self.dyn_tree.cut(x, self.dyn_tree.parent(x).unwrap())?;
//                 self.dyn_tree.link(n1, n2, weight);
//             }
//         } else {
//             self.dyn_tree.link(n1, n2, weight);
//         }

//         return Ok(());
//     }

//     pub fn delete(&mut self, edge1: &TMesh::EdgeDescriptor, edge2: &TMesh::EdgeDescriptor) {
//         let n1 = self.get_node(edge1);
//         let n2 = self.get_node(edge2);
//         let _ = self.dyn_tree.cut(n1, n2);
//     }

//     #[inline]
//     pub fn create_node(&mut self) -> usize {
//         return self.dyn_tree.create_node(());
//     }

//     fn get_node(&mut self, edge: &TMesh::EdgeDescriptor) -> usize {
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

// pub struct ReebGraphBuilder<'a, TMesh: TopologicalMesh + VertexProperties> {
//     dyn_trees: DynTrees<TMesh>,
//     mesh: &'a TMesh,
//     graph: ReebGraph<TMesh>,
//     reeb_values: TMesh::VertexPropertyMap<ReebValue<TMesh::ScalarType>>,
//     component_node_map: HashMap<usize, NodeIndex<usize>>
// }

// impl<'a, TMesh: TopologicalMesh + VertexProperties> ReebGraphBuilder<'a, TMesh> {
//     pub fn from(mesh: &'a TMesh) -> Self { 
//         let (_, max_vertices) = mesh.vertices().size_hint();
//         let (_, max_edges) = mesh.edges().size_hint();

//         return Self {
//             dyn_trees: DynTrees::new(),
//             graph: ReebGraph::<TMesh>::with_capacity(
//                 max_vertices.unwrap_or(0),
//                 max_edges.unwrap_or(0)
//             ),
//             reeb_values: mesh.create_vertex_properties_map(),
//             component_node_map: HashMap::new(),
//             mesh
//         }; 
//     }

//     pub fn build<TReebFn>(mut self, mut reeb_func: TReebFn) -> ReebGraph<TMesh>
//     where
//         TReebFn: FnMut(&TMesh::VertexDescriptor) -> TMesh::ScalarType
//     {
//         let mut sorted_vertices = Vec::new();

//         for vertex in self.mesh.vertices() {
//             let reeb_value = ReebValue::new(reeb_func(&vertex));
//             self.reeb_values[vertex] = reeb_value;
//             sorted_vertices.push(vertex);
//         }

//         sorted_vertices.sort_by(|a, b| self.reeb_values[*a].cmp(&self.reeb_values[*b]));
        
//         for vertex in sorted_vertices {
//             let lowers = self.lower_components(&vertex);
//             self.update_preimage_graph(&vertex);
//             let uppers = self.upper_components(&vertex);
//             self.update_reeb_graph(vertex, uppers, lowers);
//         }

//         return self.graph;
//     }

//     fn lower_components(&mut self, v: &TMesh::VertexDescriptor) -> HashSet<usize> {
//         let mut lower = HashSet::new();

//         self.mesh.edges_around_vertex(&v, |edge| {
//             let (mut v1, mut v2) = self.mesh.edge_vertices(edge);
//             let mut r1 = self.reeb_values[v1];
//             let mut r2 = self.reeb_values[v2];

//             if r1 > r2 {
//                 swap(&mut v1, &mut v2);
//                 swap(&mut r1, &mut r2);
//             }

//             if &v2 != v {
//                 return;
//             }

//             if let Some(component) = self.dyn_trees.find(edge) {
//                 lower.insert(component);
//             }
//         });

//         return lower;
//     }

//     fn upper_components(&mut self, v: &TMesh::VertexDescriptor) -> HashSet<usize> {
//         let mut upper = HashSet::new();

//         self.mesh.edges_around_vertex(&v, |edge| {
//             let (mut v1, mut v2) = self.mesh.edge_vertices(edge);
//             let mut r1 = self.reeb_values[v1];
//             let mut r2 = self.reeb_values[v2];

//             if r1 > r2 {
//                 swap(&mut v1, &mut v2);
//                 swap(&mut r1, &mut r2);
//             }

//             if &v1 != v {
//                 return;
//             }

//             if let Some(component) = self.dyn_trees.find(edge) {
//                 upper.insert(component);
//             }
//         });

//         return upper;
//     }

//     #[inline]
//     fn update_preimage_graph(&mut self, v: &TMesh::VertexDescriptor) {
//         self.mesh.faces_around_vertex(v, |f| {
//             let ordered_face = OrderedTriangle::from_face(f, self.mesh, &self.reeb_values);

//             if v == ordered_face.top_vertex().vertex() {
//                 self.dyn_trees.delete(ordered_face.e2().edge(), ordered_face.e3().edge());
//                 return;
//             }

//             if v == ordered_face.middle_vertex().vertex() {
//                 self.dyn_trees.delete(ordered_face.e3().edge(), ordered_face.e2().edge());
 
//                 let weight = PreimageWeight::new(self.reeb_values[*ordered_face.bottom_vertex().vertex()].value());
//                 self.dyn_trees.insert(ordered_face.e2().edge(), ordered_face.e3().edge(), weight).unwrap();
//                 return;
//             }

//             if v == ordered_face.bottom_vertex().vertex() {
//                 let weight = PreimageWeight::new(self.reeb_values[*ordered_face.bottom_vertex().vertex()].value());
//                 self.dyn_trees.insert(ordered_face.e1().edge(), ordered_face.e3().edge(), weight).unwrap();
//                 return;
//             }
//         });
//     }

//     fn update_reeb_graph(
//         &mut self, 
//         vertex: TMesh::VertexDescriptor,  
//         uppers: HashSet<usize>, 
//         lowers: HashSet<usize>
//     ) {
//         // if uppers.len() == 1 && lowers.len() == 1 {
//         //     return;
//         // }

//         println!("{}", vertex);
//         println!("uppers: {}", uppers.len());
//         println!("lowers: {}", lowers.len());
//         println!();
//         let node = self.graph.add_node(self.reeb_values[vertex].value());

//         for component in lowers {
//             // if let Some(lower_node) = self.component_node_map.get(&component) {
//             // self.graph.add_edge(*lower_node, node, 0);}
//             let lower_node = self.component_node_map[&component];
//             self.graph.add_edge(lower_node, node, 0);
//         }

//         for component in uppers {
//             self.component_node_map.insert(component, node);
//         }
//     }

//     // fn get_node(&mut self, comp: usize) -> NodeIndex<usize> {
//     //     let existing_node = self.component_node_map.get(&comp);

//     //     return match existing_node {
//     //         Some(node) => *node,
//     //         None => {
//     //             let new_node = self.graph.add_node(comp);
//     //             self.component_node_map.insert(comp, new_node);
//     //             return new_node;
//     //         },
//     //     };
//     // }
// }