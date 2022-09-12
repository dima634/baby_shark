use std::{marker::PhantomData, collections::BTreeSet};
use nalgebra::Point3;
use num_traits::cast;
use crate::{mesh::traits::{TopologicalMesh, EditableMesh, Position, mesh_stats::MAX_VERTEX_VALENCE}, algo::utils::tangential_relaxation, geometry::primitives::Triangle3, spatial_partitioning::{aabb_tree::{AABBTree, MedianCut}, grid::Grid}};

pub struct IncrementalRemesher<TMesh: TopologicalMesh + EditableMesh> {
    split_edges: bool,
    shift_vertices: bool,
    collapse_edges: bool,
    project_vertices: bool,
    iterations: u16,

    mesh_type: PhantomData<TMesh>
}

impl<TMesh: TopologicalMesh + EditableMesh> IncrementalRemesher<TMesh> {
    pub fn new() -> Self {
        return Self {
            split_edges: true,
            shift_vertices: true,
            collapse_edges: true,
            project_vertices: true,
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
    pub fn with_project_vertices(mut self, project_vertices: bool) -> Self {
        self.project_vertices = project_vertices;
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
        
        let mut reference_mesh = Grid::empty();
        if self.project_vertices {
            reference_mesh = Grid::from_mesh(mesh);
        }

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

            if self.project_vertices {
                self.project_vertices(mesh, &reference_mesh, target_edge_length);
            }
        }
    }

    fn split_edges(&self, mesh: &mut TMesh, max_edge_length: TMesh::ScalarType) {
        // Cache all edges, in the case when split edge affects edges iterator
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();
        let max_edge_length_squared = max_edge_length * max_edge_length;

        for edge in edges {
            let edge_length_squared = mesh.edge_length_squared(&edge);

            // Split long edges at the middle
            if edge_length_squared > max_edge_length_squared {
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

    fn collapse_edges(&self, mesh: &mut TMesh, min_edge_length: TMesh::ScalarType) {
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();
        let min_edge_length_squared = min_edge_length * min_edge_length;

        for edge in edges {
            if !self.is_collapse_safe(mesh, &edge) {
                continue;
            }

            let edge_length_squared = mesh.edge_length_squared(&edge);

            if edge_length_squared < min_edge_length_squared {
                mesh.collapse_edge(&edge);
            }
        }
    }

    fn project_vertices(&self, mesh: &mut TMesh, grid: &Grid<Triangle3<TMesh::ScalarType>>, target_edge_length: TMesh::ScalarType) {
        let vertices: Vec<TMesh::VertexDescriptor> = mesh.vertices().collect();

        for vertex in vertices {
            let vertex_position = mesh.vertex_position(&vertex);
            
            if let Some(closest_point) = grid.closest_point(vertex_position, target_edge_length) {
                mesh.shift_vertex(&vertex, &closest_point);
            }
        }
    }

    fn is_collapse_safe(&self, mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {        
        // Was collapsed?
        if mesh.edge_exist(edge) {
            return false;
        }

        // Count common vertices of edge vertices
        let (e_start, e_end) = mesh.get_edge_vertices(edge);
        let mut e_start_neighbors = BTreeSet::new();
        mesh.vertices_around_vertex(&e_start, |vertex| { e_start_neighbors.insert(*vertex); });
        let mut common_neighbors_count = 0;
        mesh.vertices_around_vertex(&e_end, |vertex|
        {
            if e_start_neighbors.contains(vertex) {
                common_neighbors_count += 1;
            }
        });

        // Is topologically safe?
        if common_neighbors_count != 2 {
            return false;
        }

        // Check new normals
        let new_position = (mesh.vertex_position(&e_start) + mesh.vertex_position(&e_end).coords) / cast(2).unwrap();
        return self.check_faces_normals_after_collapse(mesh, &e_start, &new_position) && 
               self.check_faces_normals_after_collapse(mesh, &e_end, &new_position);
    }

    fn check_faces_normals_after_collapse(&self, mesh: &TMesh, collapsed_vertex: &TMesh::VertexDescriptor, new_position: &Point3<TMesh::ScalarType>) -> bool {
        let mut normal_flipped = false;
    
        mesh.faces_around_vertex(&collapsed_vertex, |face| {
            let mut pos = TMesh::Position::from_vertex_on_face(mesh, &face, &collapsed_vertex);

            let v2 = mesh.vertex_position(&pos.next().get_vertex());
            let v3 = mesh.vertex_position(&pos.next().get_vertex());

            let old_normal = mesh.face_normal(face);
            let new_normal = Triangle3::normal(new_position, v2, v3);

            if old_normal.dot(&new_normal) < cast(0.7).unwrap() {
                normal_flipped = true;
            }
        });

        return !normal_flipped;
    }
}
