use std::marker::PhantomData;

use num_traits::cast;
use crate::{mesh::traits::{TopologicalMesh, EditableMesh}, algo::utils::tangential_relaxation};

pub struct IncrementalRemesher<TMesh: TopologicalMesh + EditableMesh> {
    split_edges: bool,
    shift_vertices: bool,
    iterations: u16,

    mesh_type: PhantomData<TMesh>
}

impl<TMesh: TopologicalMesh + EditableMesh> IncrementalRemesher<TMesh> {
    pub fn new() -> Self {
        return Self {
            split_edges: true,
            shift_vertices: true,
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
    pub fn with_shift_vertices(mut self, shift_vertices: bool) -> Self {
        self.shift_vertices = shift_vertices;
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

            if self.shift_vertices {
                self.shift_vertices(mesh);
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
                mesh.split_edge(&edge, &split_at);
            }
        }
    }

    fn shift_vertices(&self, mesh: &mut TMesh) {
        let vertices: Vec<TMesh::VertexDescriptor> = mesh.vertices().collect();

        for vertex in vertices {
            let vertex_position = mesh.vertex_position(&vertex);
            let vertex_normal = mesh.vertex_normal(&vertex);
            let one_ring = mesh.vertices_around_vertex(&vertex).map(|v| mesh.vertex_position(&v));
            let new_position = tangential_relaxation(one_ring, vertex_position, &vertex_normal);
            mesh.shift_vertex(&vertex, &new_position);
        }
    }
}
