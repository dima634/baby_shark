use std::marker::PhantomData;

use nalgebra::Point3;
use num_traits::cast;
use crate::{mesh::traits::{TopologicalMesh, EditableMesh, mesh_stats::MAX_VERTEX_VALENCE}, algo::utils::tangential_relaxation};

pub struct IncrementalRemesher<TMesh: TopologicalMesh + EditableMesh> {
    split_edges: bool,
    shift_vertices: bool,
    collapse_edges: bool,
    iterations: u16,

    mesh_type: PhantomData<TMesh>
}

impl<TMesh: TopologicalMesh + EditableMesh> IncrementalRemesher<TMesh> {
    pub fn new() -> Self {
        return Self {
            split_edges: true,
            shift_vertices: true,
            collapse_edges: true,
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
    pub fn with_collapse_edges(mut self, collapse_edges: bool) -> Self {
        self.collapse_edges = collapse_edges;
        return self;
    }

    #[inline]
    pub fn with_iterations_count(mut self, iterations: u16) -> Self {
        self.iterations = iterations;
        return self;
    }

    pub fn remesh(&self, mesh: &mut TMesh, target_edge_length: TMesh::ScalarType) {
        let max_edge_length = cast::<f64, TMesh::ScalarType>(4.0 / 3.0).unwrap() * target_edge_length;
        let min_edge_length = cast::<f64, TMesh::ScalarType>(4.0 / 5.0).unwrap() * target_edge_length;
        
        for _ in 0..self.iterations {
            if self.split_edges {
                self.split_edges(mesh, max_edge_length);
            }

            if self.collapse_edges {
                self.collapse_edges(mesh, min_edge_length);
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
        let mut one_ring: Vec<Point3<TMesh::ScalarType>> = Vec::with_capacity(MAX_VERTEX_VALENCE);

        for vertex in vertices {
            let vertex_position = mesh.vertex_position(&vertex);
            let vertex_normal = mesh.vertex_normal(&vertex);
            one_ring.clear();
            mesh.vertices_around_vertex(&vertex, |v| one_ring.push(*mesh.vertex_position(&v)));
            let new_position = tangential_relaxation(one_ring.iter(), vertex_position, &vertex_normal);
            mesh.shift_vertex(&vertex, &new_position);
        }
    }

    pub fn collapse_edges(&self, mesh: &mut TMesh, min_edge_length: TMesh::ScalarType) {
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();

        for edge in edges {
            let edge_length = mesh.edge_length(&edge);

            if edge_length < min_edge_length {
                mesh.collapse_edge(&edge);
            }
        }
    }
}
