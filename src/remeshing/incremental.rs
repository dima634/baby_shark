use std::marker::PhantomData;

use num_traits::cast;
use crate::mesh::traits::EditableMesh;

pub struct IncrementalRemesher<TMesh: EditableMesh> {
    split_edges: bool,
    iterations: u16,

    mesh_type: PhantomData<TMesh>
}

impl<TMesh: EditableMesh> IncrementalRemesher<TMesh> {
    pub fn new() -> Self {
        return Self {
            split_edges: true,
            iterations: 5,
            mesh_type: PhantomData
        };
    }

    #[inline]
    pub fn with_split_edges(mut self, split_edges: bool) -> Self {
        self.split_edges = split_edges;
        return self;
    }

    #[inline]
    pub fn with_iterations_count(mut self, iterations: u16) -> Self {
        self.iterations = iterations;
        return self;
    }

    pub fn remesh(&self, mesh: &mut TMesh, target_edge_length: TMesh::ScalarType) {
        let max_edge_length = cast::<f64, TMesh::ScalarType>(4.0 / 3.0).unwrap() * target_edge_length;
        
        for _ in 0..self.iterations {
            if self.split_edges {
                self.split_edges(mesh, max_edge_length);
            }
        }
    }

    fn split_edges(&self, mesh: &mut TMesh, max_edge_length: TMesh::ScalarType) {
        // Cache all edges, in the case when split edge affects edges iterator
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();

        for edge in edges {
            let edge_length = mesh.edge_length(&edge);

            // Split long edges at the middle
            if edge_length > max_edge_length {
                let (v1, v2) = mesh.edge_positions(&edge);
                let split_at = v1 + (v2 - v1).scale(cast(0.5).unwrap());
                mesh.split_edge(edge, &split_at);
            }
        }
    }
}
