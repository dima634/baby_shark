use std::{marker::PhantomData, collections::BTreeSet};
use nalgebra::Point3;
use num_traits::{cast, Float};
use crate::{mesh::{traits::{TopologicalMesh, EditableMesh, Position, mesh_stats }}, algo::utils::tangential_relaxation, geometry::primitives::Triangle3, spatial_partitioning::grid::Grid};

pub struct IncrementalRemesher<TMesh: TopologicalMesh + EditableMesh> {
    split_edges: bool,
    shift_vertices: bool,
    collapse_edges: bool,
    flip_edges: bool,
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
            flip_edges: true,
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
    pub fn with_flip_edges(mut self, flip_edges: bool) -> Self {
        self.flip_edges = flip_edges;
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

            if self.flip_edges {
                self.flip_edges(mesh);
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
        let mut one_ring: Vec<Point3<TMesh::ScalarType>> = Vec::with_capacity(mesh_stats::MAX_VERTEX_VALENCE);

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
            let edge_length_squared = mesh.edge_length_squared(&edge);

            if edge_length_squared < min_edge_length_squared && self.is_collapse_safe(mesh, &edge) {
                mesh.collapse_edge(&edge);
            }
        }
    }

    fn flip_edges(&self, mesh: &mut TMesh) {
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();

        for edge in edges {
            if self.will_flip_improve_quality(mesh, &edge) && self.is_flip_safe(mesh, &edge) {
                mesh.flip_edge(&edge);
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
        let (e_start, e_end) = mesh.edge_vertices(edge);
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

        // Check new normals (geometrical safety)
        let new_position = (mesh.vertex_position(&e_start) + mesh.vertex_position(&e_end).coords) / cast(2).unwrap();
        return self.check_faces_normals_after_collapse(mesh, &e_start, &new_position) && 
               self.check_faces_normals_after_collapse(mesh, &e_end, &new_position);
    }

    fn check_faces_normals_after_collapse(&self, mesh: &TMesh, collapsed_vertex: &TMesh::VertexDescriptor, new_position: &Point3<TMesh::ScalarType>) -> bool {
        let mut bad_collapse = false;
    
        mesh.faces_around_vertex(&collapsed_vertex, |face| {
            let mut pos = TMesh::Position::from_vertex_on_face(mesh, &face, &collapsed_vertex);

            let v1 = mesh.vertex_position(&pos.get_vertex());
            let v2 = mesh.vertex_position(&pos.next().get_vertex());
            let v3 = mesh.vertex_position(&pos.next().get_vertex());

            let old_quality = Triangle3::quality(v1, v2, v3);
            let new_quality = Triangle3::quality(new_position, v2, v3);

            // Quality become too bad?
            if new_quality < old_quality * cast(0.5).unwrap() {
                bad_collapse = true;
            }

            let old_normal = Triangle3::normal(v1, v2, v3);
            let new_normal = Triangle3::normal(new_position, v2, v3);

            // Normal flipped?
            if old_normal.dot(&new_normal) < cast(0.7).unwrap() {
                bad_collapse = true;
            }
        });

        return !bad_collapse;
    }

    fn is_flip_safe(&self, mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
        // Is topologically safe?
        if mesh.is_edge_on_boundary(edge) {
            return false;
        }

        // Check normals after flip (geometrical safety)
        let mut pos = TMesh::Position::from_edge(mesh, edge);
        
        let v1 = mesh.vertex_position(&pos.get_vertex());
        let v2 = mesh.vertex_position(&pos.next().get_vertex());
        let v0 = mesh.vertex_position(&pos.next().get_vertex());
        let v3 = mesh.vertex_position(&pos.next().opposite().get_vertex());

        let old_normal1 = Triangle3::normal(v0, v1, v2);
        let new_normal1 = Triangle3::normal(v1, v2, v3);

        let threshold = cast::<f64, TMesh::ScalarType>(5.0).unwrap().to_radians();

        if old_normal1.angle(&new_normal1) > threshold {
            return false;
        }

        let old_normal2 = Triangle3::normal(v0, v2, v3);
        let new_normal2 = Triangle3::normal(v0, v1, v3);

        if old_normal2.angle(&new_normal2) > threshold || 
           old_normal2.angle(&new_normal1) > threshold || 
           old_normal1.angle(&new_normal2) > threshold 
        {
            return false;
        }

        return true;
    }

    fn will_flip_improve_quality(&self, mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
        let mut pos = TMesh::Position::from_edge(mesh, edge);

        let v1 = pos.get_vertex();
        let v2 = pos.next().get_vertex();
        let v0 = pos.next().get_vertex();
        let v3 = pos.next().opposite().get_vertex();

        let v0_ideal_val = self.ideal_valence(mesh, &v0);
        let v1_ideal_val = self.ideal_valence(mesh, &v1);
        let v2_ideal_val = self.ideal_valence(mesh, &v2);
        let v3_ideal_val = self.ideal_valence(mesh, &v3);

        let v0_val = self.valence(mesh, &v0);
        let v1_val = self.valence(mesh, &v1);
        let v2_val = self.valence(mesh, &v2);
        let v3_val = self.valence(mesh, &v3);

        let old_deviation =
            (v0_val - v0_ideal_val).abs() +
            (v1_val - v1_ideal_val).abs() +
            (v2_val - v2_ideal_val).abs() +
            (v3_val - v3_ideal_val).abs();

        let new_deviation = 
            (v0_val - 1 - v0_ideal_val).abs() +
            (v1_val + 1 - v1_ideal_val).abs() +
            (v2_val - 1 - v2_ideal_val).abs() +
            (v3_val + 1 - v3_ideal_val).abs();

        let v0_pos = mesh.vertex_position(&v0);
        let v1_pos = mesh.vertex_position(&v1);
        let v2_pos = mesh.vertex_position(&v2);
        let v3_pos = mesh.vertex_position(&v3);

        let old_face_quality = Triangle3::quality(v0_pos, v1_pos, v2_pos).min(Triangle3::quality(v0_pos, v2_pos, v3_pos));
        let new_face_quality = Triangle3::quality(v1_pos, v2_pos, v3_pos).min(Triangle3::quality(v0_pos, v1_pos, v3_pos));

        return (new_deviation < old_deviation && new_face_quality >= old_face_quality * cast(0.5).unwrap()) ||
               (new_deviation == old_deviation && new_face_quality > old_face_quality) || // Same valence but better quality
               (new_face_quality > old_face_quality * cast(1.5).unwrap());                // Hurt valence but improve quality by much
    }

    #[inline]
    fn valence(&self, mesh: &TMesh, vertex: &TMesh::VertexDescriptor) -> isize {
        let mut valence = 0;
        mesh.vertices_around_vertex(vertex, |_| valence += 1);

        return valence;
    }    
    
    #[inline]
    fn ideal_valence(&self, mesh: &TMesh, vertex: &TMesh::VertexDescriptor) -> isize {
        if mesh.is_vertex_on_boundary(vertex) {
            return mesh_stats::IDEAL_BOUNDARY_VERTEX_VALENCE as isize;
        } else {
            return mesh_stats::IDEAL_INTERIOR_VERTEX_VALENCE as isize;
        }
    }
}
