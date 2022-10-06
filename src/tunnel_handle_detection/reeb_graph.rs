
use std::collections::HashMap;

use petgraph::prelude::UnGraph;

use crate::mesh::traits::{TopologicalMesh, Mesh, MeshMarker, Marker};

pub type ReebGraph<TMesh> = UnGraph<<TMesh as Mesh>::VertexDescriptor, (), usize>;

pub fn build<TMesh, TReebFn>(mesh: &TMesh, mut reeb_func: TReebFn) -> ReebGraph<TMesh> 
where
    TMesh: TopologicalMesh + MeshMarker,
    TReebFn: FnMut(&TMesh::VertexDescriptor) -> TMesh::ScalarType
{
    let vertices_count = mesh.vertices().count();
    let mut vertex_node_map = HashMap::with_capacity(vertices_count);
    let mut graph = ReebGraph::<TMesh>::with_capacity(
        vertices_count,
        mesh.edges().count()
    );

    for vertex in mesh.vertices() {
        let node = graph.add_node(vertex);
        vertex_node_map.insert(vertex, node);
    }

    let mut marker = mesh.marker();

    let mut add_edge = |edge: &TMesh::EdgeDescriptor| {
        if !marker.is_edge_marked(edge) {
            let (v1, v2) = mesh.edge_vertices(edge);
            let n1 = *vertex_node_map.get(&v1).unwrap();
            let n2 = *vertex_node_map.get(&v2).unwrap();

            graph.add_edge(n1, n2, ());

            marker.mark_edge(edge, true);
        }
    };

    for face in mesh.faces() {
        let (e0, e1, e2) = mesh.face_edges(&face);
        add_edge(&e0); 
        add_edge(&e1); 
        add_edge(&e2);

        
    }

    return graph;
}