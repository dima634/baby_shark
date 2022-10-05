
use crate::{data_structures::graph::{Graph, Node}, mesh::traits::TopologicalMesh};

struct ReebGraph<TMesh: TopologicalMesh> {
    graph: Graph<TMesh::VertexDescriptor, ()>
}

impl<TMesh: TopologicalMesh> ReebGraph<TMesh> {
    pub fn build<TReebFn>(mesh: &TMesh, mut reeb_func: TReebFn) -> Self 
    where
        TReebFn: FnMut(&TMesh::VertexDescriptor) -> TMesh::ScalarType
    {
        let mut graph = Graph::empty();

        for vertex in mesh.vertices() {
            graph.create_node(vertex);
        }

        for edge in mesh.edges() {
            
        }

        return Self {
            graph
        };
    }
}
