use crate::{
    algo::{edge_collapse, utils::tangential_relaxation, vertex_shift},
    geometry::primitives::triangle3::Triangle3,
    mesh::traits::{mesh_stats, EditableMesh, Position, TopologicalMesh},
    spatial_partitioning::grid::Grid,
};
use num_traits::{cast, Float};
use std::marker::PhantomData;

///
/// Incremental isotropic remesher.
/// This algorithm incrementally performs simple operations such as edge splits,
/// edge collapses, edge flips, and Laplacian smoothing.
/// All the vertices of the remeshed patch are reprojected to
/// the original surface to keep a good approximation of the input.
///
/// ## Example
/// ```ignore
/// let remesher = IncrementalRemesher::new()
///     .with_iterations_count(10)
///     .with_split_edges(true)
///     .with_collapse_edges(true)
///     .with_flip_edges(true)
///     .with_shift_vertices(true)
///     .with_project_vertices(true);
/// remesher.remesh(&mut mesh, 0.002f32);
/// ```
///
pub struct IncrementalRemesher<TMesh: TopologicalMesh + EditableMesh> {
    split_edges: bool,
    shift_vertices: bool,
    collapse_edges: bool,
    flip_edges: bool,
    project_vertices: bool,
    iterations: u16,
    keep_boundary: bool,

    mesh_type: PhantomData<TMesh>,
}

impl<TMesh: TopologicalMesh + EditableMesh> IncrementalRemesher<TMesh> {
    pub fn new() -> Self {
        Default::default()
    }

    /// Set flag indicating whether edge split should be performed. Default is `true`
    #[inline]
    pub fn with_split_edges(mut self, split_edges: bool) -> Self {
        self.split_edges = split_edges;
        self
    }

    /// Set flag indicating whether laplacian smoothing should be performed. Default is `true`
    #[inline]
    pub fn with_shift_vertices(mut self, shift_vertices: bool) -> Self {
        self.shift_vertices = shift_vertices;
        self
    }

    /// Set flag indicating whether edge collapse should be performed. Default is `true`
    #[inline]
    pub fn with_collapse_edges(mut self, collapse_edges: bool) -> Self {
        self.collapse_edges = collapse_edges;
        self
    }

    /// Set flag indicating whether edge flip should be performed. Default is `true`
    #[inline]
    pub fn with_flip_edges(mut self, flip_edges: bool) -> Self {
        self.flip_edges = flip_edges;
        self
    }

    /// Set flag indicating whether vertices of resulting mesh should be projected to original. Default is `true`
    #[inline]
    pub fn with_project_vertices(mut self, project_vertices: bool) -> Self {
        self.project_vertices = project_vertices;
        self
    }

    /// Set number of remeshing iterations. Default is `10`
    #[inline]
    pub fn with_iterations_count(mut self, iterations: u16) -> Self {
        self.iterations = iterations;
        self
    }

    /// Set whether keep mesh boundary unchanged
    #[inline]
    pub fn with_keep_boundary(mut self, keep: bool) -> Self {
        self.keep_boundary = keep;
        self
    }

    ///
    /// Remesh given `mesh`
    /// ## Arguments
    /// * `mesh` - triangular mesh
    /// * `target_edge_length` - desired length of edge
    ///
    pub fn remesh(&self, mesh: &mut TMesh, target_edge_length: TMesh::ScalarType) {
        let max_edge_length =
            cast::<f64, TMesh::ScalarType>(4.0 / 3.0).unwrap() * target_edge_length;
        let min_edge_length =
            cast::<f64, TMesh::ScalarType>(4.0 / 5.0).unwrap() * target_edge_length;

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
                self.shift_vertices(mesh, target_edge_length * target_edge_length);
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

    fn shift_vertices(&self, mesh: &mut TMesh, target_edge_length_squared: TMesh::ScalarType) {
        let vertices: Vec<TMesh::VertexDescriptor> = mesh.vertices().collect();
        let mut one_ring = Vec::with_capacity(mesh_stats::MAX_VERTEX_VALENCE);

        // Perform laplacian smoothing for each vertex
        for vertex in vertices {
            let Some(vertex_normal) = mesh.vertex_normal(&vertex) else {
                continue;
            };

            let vertex_position = mesh.vertex_position(&vertex);
            one_ring.clear();
            mesh.vertices_around_vertex(&vertex, |v| one_ring.push(*mesh.vertex_position(v)));
            let new_position = tangential_relaxation(one_ring.iter(), vertex_position, &vertex_normal); 

            let shift_vertex = 
                !(self.keep_boundary && mesh.is_vertex_on_boundary(&vertex)) &&
                vertex_shift::is_vertex_shift_safe(&vertex, vertex_position, &new_position, target_edge_length_squared,  mesh);

            if shift_vertex {
                mesh.shift_vertex(&vertex, &new_position);
            }
        }
    }

    fn collapse_edges(&self, mesh: &mut TMesh, min_edge_length: TMesh::ScalarType) {
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();
        let min_edge_length_squared = min_edge_length * min_edge_length;

        // Collapse long edges
        for edge in edges {
            if !mesh.edge_exist(&edge) {
                continue;
            }

            // Keep boundary
            let (v1, v2) = mesh.edge_vertices(&edge);
            if self.keep_boundary && (mesh.is_vertex_on_boundary(&v1) || mesh.is_vertex_on_boundary(&v2)) {
                continue;
            }

            // Long edge?
            if mesh.edge_length_squared(&edge) >= min_edge_length_squared {
                continue;
            }

            let v1_pos = mesh.vertex_position(&v1);
            let v2_pos = mesh.vertex_position(&v2);
            let collapse_at = (v1_pos + v2_pos) * cast::<f32, TMesh::ScalarType>(0.5).unwrap();

            if edge_collapse::is_safe(mesh, &edge, &collapse_at, cast(0.5).unwrap()) {
                mesh.collapse_edge(&edge, &collapse_at);
            }
        }
    }

    fn flip_edges(&self, mesh: &mut TMesh) {
        let edges: Vec<TMesh::EdgeDescriptor> = mesh.edges().collect();

        // Flip edges to improve valence
        for edge in edges {
            if self.is_flip_safe(mesh, &edge) && self.will_flip_improve_quality(mesh, &edge) {
                mesh.flip_edge(&edge);
            }
        }
    }

    fn project_vertices(
        &self,
        mesh: &mut TMesh,
        grid: &Grid<Triangle3<TMesh::ScalarType>>,
        target_edge_length: TMesh::ScalarType,
    ) {
        let vertices: Vec<TMesh::VertexDescriptor> = mesh.vertices().collect();

        // Project vertices back on original mesh
        for vertex in vertices {
            let vertex_position = mesh.vertex_position(&vertex);

            if let Some(closest_point) = grid.closest_point(vertex_position, target_edge_length) {
                mesh.shift_vertex(&vertex, &closest_point);
            }
        }
    }

    fn is_flip_safe(&self, mesh: &mut TMesh, edge: &TMesh::EdgeDescriptor) -> bool {
        // Is topologically safe?
        if mesh.is_edge_on_boundary(edge) {
            return false;
        }

        // Check that flipped edge doest not already exist
        let mut pos = TMesh::Position::from_edge(mesh, edge);
        let v1_idx = pos.get_vertex();
        let v2_idx = pos.opposite().get_vertex();

        let mut safe = true;
        mesh.vertices_around_vertex(&v2_idx, |v| safe &= v1_idx != *v);

        if !safe {
            return false;
        }

        // Check normals after flip (geometrical safety)
        let mut pos = TMesh::Position::from_edge(mesh, edge);

        let v1 = mesh.vertex_position(&pos.get_vertex());
        let v2 = mesh.vertex_position(&pos.next().get_vertex());
        let v0 = mesh.vertex_position(&pos.next().get_vertex());
        let v3 = mesh.vertex_position(&pos.next().opposite().get_vertex());

        if Triangle3::is_degenerate(v1, v2, v3) || Triangle3::is_degenerate(v0, v1, v3) {
            return false;
        }

        let Some(old_normal1) = Triangle3::normal(v0, v1, v2) else { return false; };
        let Some(new_normal1) = Triangle3::normal(v1, v2, v3) else { return false; };
        let threshold = cast::<f64, TMesh::ScalarType>(5.0).unwrap().to_radians();

        if old_normal1.angle(&new_normal1) > threshold {
            return false;
        }

        let Some(old_normal2) = Triangle3::normal(v0, v2, v3) else { return false; };
        let Some(new_normal2) = Triangle3::normal(v0, v1, v3) else { return false; };

        if old_normal2.angle(&new_normal2) > threshold
            || old_normal2.angle(&new_normal1) > threshold
            || old_normal1.angle(&new_normal2) > threshold
        {
            return false;
        }

        true
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

        let old_deviation = (v0_val - v0_ideal_val).abs()
            + (v1_val - v1_ideal_val).abs()
            + (v2_val - v2_ideal_val).abs()
            + (v3_val - v3_ideal_val).abs();

        let new_deviation = (v0_val - 1 - v0_ideal_val).abs()
            + (v1_val + 1 - v1_ideal_val).abs()
            + (v2_val - 1 - v2_ideal_val).abs()
            + (v3_val + 1 - v3_ideal_val).abs();

        let v0_pos = mesh.vertex_position(&v0);
        let v1_pos = mesh.vertex_position(&v1);
        let v2_pos = mesh.vertex_position(&v2);
        let v3_pos = mesh.vertex_position(&v3);

        let old_face_quality = Triangle3::quality(v0_pos, v1_pos, v2_pos)
            .min(Triangle3::quality(v0_pos, v2_pos, v3_pos));
        let new_face_quality = Triangle3::quality(v1_pos, v2_pos, v3_pos)
            .min(Triangle3::quality(v0_pos, v1_pos, v3_pos));

        (new_deviation < old_deviation && new_face_quality >= old_face_quality * cast(0.5).unwrap()) ||
               (new_deviation == old_deviation && new_face_quality > old_face_quality) || // Same valence but better quality
               (new_face_quality > old_face_quality * cast(1.5).unwrap()) // Hurt valence but improve quality by much
    }

    #[inline]
    fn valence(&self, mesh: &TMesh, vertex: &TMesh::VertexDescriptor) -> isize {
        let mut valence = 0;
        mesh.vertices_around_vertex(vertex, |_| valence += 1);

        valence
    }

    #[inline]
    fn ideal_valence(&self, mesh: &TMesh, vertex: &TMesh::VertexDescriptor) -> isize {
        if mesh.is_vertex_on_boundary(vertex) {
            mesh_stats::IDEAL_BOUNDARY_VERTEX_VALENCE as isize
        } else {
            mesh_stats::IDEAL_INTERIOR_VERTEX_VALENCE as isize
        }
    }
}

impl<TMesh: TopologicalMesh + EditableMesh> Default for IncrementalRemesher<TMesh> {
    fn default() -> Self {
        Self {
            split_edges: true,
            shift_vertices: true,
            collapse_edges: true,
            flip_edges: true,
            project_vertices: true,
            iterations: 10,
            keep_boundary: true,
            mesh_type: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use crate::{io::stl::{StlReader, StlWriter}, mesh::{corner_table::CornerTable, traits::Mesh}};
    use super::IncrementalRemesher;

    #[test]
    fn should_collapse_short_edges() {
        let mut mesh: CornerTable<f32> = StlReader::default()
            .read_stl_from_file(Path::new("./assets/tube.stl"))
            .expect("Read mesh");

        // Only collapse edges
        let target_edge_length = 0.1f32;
        IncrementalRemesher::default()
            .with_iterations_count(5)
            .remesh(&mut mesh, target_edge_length);

        let has_short_edges = mesh.edges().any(|edge| {
            let length = mesh.edge_length(&edge);
            length < target_edge_length * 0.8
        });

        StlWriter::default()
            .write_stl_to_file(&mesh, Path::new("./tube_collapse.stl"))
            .expect("Write mesh");

        assert!(!has_short_edges, "should not have short edges after remeshing");
    }
}
