use std::{collections::HashSet, path};

use crate::{
    algo::{edge_collapse, utils::tangential_relaxation, vertex_shift},
    geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
    mesh::{corner_table::*, polygon_soup::data_structure::PolygonSoup, traits::stats},
    spatial_partitioning::grid::Grid,
};

///
/// Incremental isotropic remesher.
/// This algorithm incrementally performs simple operations such as edge splits,
/// edge collapses, edge flips, and Laplacian smoothing.
/// All the vertices of the remeshed patch are reprojected to
/// the original surface to keep a good approximation of the input.
///
/// ## Example
/// ```ignore
/// let remesher = IncrementalRemesher::default()
///     .with_iterations_count(10)
///     .with_split_edges(true)
///     .with_collapse_edges(true)
///     .with_flip_edges(true)
///     .with_shift_vertices(true)
///     .with_project_vertices(true);
/// remesher.remesh(&mut mesh, 0.002f32);
/// ```
///
#[derive(Debug, Clone)]
pub struct IncrementalRemesher {
    split_edges: bool,
    shift_vertices: bool,
    collapse_edges: bool,
    flip_edges: bool,
    project_vertices: bool,
    keep_boundary: bool,
    iterations: u16,
    sharp_edge_angle: Option<f64>,
    constrained_vertices: HashSet<VertexId>,
    constrained_edges: HashSet<(VertexId, VertexId)>,
}

impl IncrementalRemesher {
    #[inline]
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

    #[inline]
    pub fn with_sharp_edge_angle(mut self, angle_degrees: Option<f64>) -> Self {
        self.sharp_edge_angle = angle_degrees;
        self
    }

    ///
    /// Remesh given `mesh`
    /// ## Arguments
    /// * `mesh` - triangular mesh
    /// * `target_edge_length` - desired length of edge
    ///
    pub fn remesh<R: RealNumber>(&mut self, mesh: &mut CornerTable<R>, target_edge_length: R) {
        let max_edge_length = R::f64(4.0 / 3.0) * target_edge_length;
        let min_edge_length = R::f64(4.0 / 5.0) * target_edge_length;

        let mut reference_mesh = Grid::empty();
        if self.project_vertices {
            reference_mesh = Grid::from_mesh(mesh);
        }

        self.find_feature_vertices(mesh);

        for i in 0..self.iterations as usize {
            //self.save_constraints(i, "begin", mesh);

            if self.split_edges {
                self.split_long_edges(mesh, max_edge_length);
            }

            //self.save_constraints(i, "split", mesh);

            // crate::io::write_to_file(mesh, &std::path::Path::new("before.ply"))
            //     .expect("should save mesh to file");

            if self.collapse_edges {
                self.collapse_short_edges(mesh, min_edge_length);
            }

            //self.save_constraints(i, "collapse", mesh);

            // crate::io::write_to_file(mesh, &std::path::Path::new("after.ply"))
            //     .expect("should save mesh to file");

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

    fn split_long_edges<R: RealNumber>(&mut self, mesh: &mut CornerTable<R>, max_edge_length: R) {
        let max_edge_length_squared = max_edge_length * max_edge_length;

        self.save_constraints(0, "begin", mesh);

        // Split longest edge of each face.
        // Iterating over edges does not work well
        // because sometimes it leads to creation of degenerate faces.
        // So we split longest edge of each face instead.
        let mut i = 0;
        for face in mesh.faces().collect::<Vec<_>>() {
            let (e1, e2, e3) = mesh.face_edges(face);
            let l1 = mesh.edge_length_squared(e1);
            let l2 = mesh.edge_length_squared(e2);
            let l3 = mesh.edge_length_squared(e3);

            if l1 < max_edge_length_squared
                && l2 < max_edge_length_squared
                && l3 < max_edge_length_squared
            {
                continue; // All edges are short enough
            }

            let longest_edge = if l1 >= l2 && l1 >= l3 {
                e1
            } else if l2 >= l1 && l2 >= l3 {
                e2
            } else {
                e3
            };

            let (v1, v2) = mesh.edge_vertices(longest_edge);
            let (v1_pos, v2_pos) = (mesh[v1].position(), mesh[v2].position());
            let split_at = (v2_pos + v1_pos) * R::half();
            let Some((e1, e2)) = mesh.split_edge(longest_edge, split_at) else { continue };

            let (new_v1, split_at) = mesh.edge_vertices(e1);
            if self.constrained_vertices.remove(&v1) {
                self.constrained_vertices.insert(new_v1);
            }

            let (_, new_v2) = mesh.edge_vertices(e2);
            if self.constrained_vertices.remove(&v2) {
                self.constrained_vertices.insert(new_v2);
            }

            // If we split constrained edge then new edges are also constrained.
            if self.remove_edge_constraint(v1, v2) {
                self.constrained_edges.insert((new_v1, split_at));
                self.constrained_edges.insert((split_at, new_v2));

                self.constrained_vertices.insert(new_v1); // TODO: check if needed
                self.constrained_vertices.insert(new_v2);
                self.constrained_vertices.insert(split_at);
            }
            i+=1;

            self.save_constraints(i, "", mesh);
        }

        println!("Split {} edges", i);
    }

    fn shift_vertices<R: RealNumber>(
        &self,
        mesh: &mut CornerTable<R>,
        target_edge_length_squared: R,
    ) {
        let mut one_ring = Vec::with_capacity(stats::MAX_VERTEX_VALENCE);

        // Perform laplacian smoothing for each vertex
        for vertex in mesh.vertices().collect::<Vec<_>>() {
            if self.keep_boundary && mesh.is_vertex_on_boundary(vertex) {
                // TODO: constrained vertex
                continue;
            }

            if self.constrained_vertices.contains(&vertex) {
                continue;
            }

            let Some(vertex_normal) = mesh.vertex_normal(vertex) else { continue };
            let vertex_position = mesh.vertex_position(vertex);

            one_ring.clear();
            mesh.vertices_around_vertex(vertex, |v| one_ring.push(*mesh.vertex_position(v)));
            let new_position =
                tangential_relaxation(one_ring.iter(), vertex_position, &vertex_normal);

            let is_shift_safe = vertex_shift::is_vertex_shift_safe(
                vertex,
                vertex_position,
                &new_position,
                target_edge_length_squared,
                mesh,
            );

            if is_shift_safe {
                mesh[vertex].set_position(new_position);
            }
        }
    }

    fn collapse_short_edges<R: RealNumber>(
        &mut self,
        mesh: &mut CornerTable<R>,
        min_edge_length: R,
    ) {
        let min_edge_length_squared = min_edge_length * min_edge_length;

        let mut i = 0;

        // Collapse long edges
        for edge in mesh.unique_edges().collect::<Vec<_>>() {
            if !mesh.edge_exists(edge) {
                continue;
            }

            if self.keep_boundary && mesh.is_edge_on_boundary(edge) {
                continue; // TODO: constrained vertex
            }

            if mesh.edge_length_squared(edge) >= min_edge_length_squared {
                continue;
            }

            // println!("{} > {}", mesh.edge_length_squared(edge), min_edge_length_squared);

            let (v1, v2) = mesh.edge_vertices(edge);
            let is_v1_constrained = self.constrained_vertices.contains(&v1);
            let is_v2_constrained = self.constrained_vertices.contains(&v2);

            let v1_pos = mesh.vertex_position(v1);
            let v2_pos = mesh.vertex_position(v2);
            let constrained_thr = R::one() - R::f32(0.001);
            let (collapse_at, normal_deviation_thr) = match (is_v1_constrained, is_v2_constrained) {
                (true, false) => (*v1_pos, constrained_thr),
                (false, true) => (*v2_pos, constrained_thr),
                (true, true) => ((v1_pos + v2_pos) * R::half(), constrained_thr),
                (false, false) => ((v1_pos + v2_pos) * R::half(), R::f32(0.7)),
            };

            if edge_collapse::is_safe(mesh, edge, &collapse_at, R::half(), normal_deviation_thr) {
                match (is_v1_constrained, is_v2_constrained) {
                    (true, false) => self.constrained_vertices.insert(v2),
                    (false, true) => self.constrained_vertices.insert(v1),
                    _ => false,
                };

                let edge_corner = &mesh[edge.corner()];
              
                mesh.vertices_around_vertex(v2, |v| {
                    if v != v1 && self.remove_edge_constraint(v2, v) {
                        self.constrained_edges.insert((v1, v));
                    }
                });

                let (removed_vertex, _, _) = mesh.collapse_edge(edge, &collapse_at).unwrap(); // We already checked that collapse is safe
                assert!(removed_vertex == v2);

                self.constrained_vertices.remove(&removed_vertex);
                self.remove_edge_constraint(v1, v2);
                
                //self.save_constraints(i, "test", mesh);
                i += 1;

            }
        }

        
    }

    fn flip_edges<R: RealNumber>(&self, mesh: &mut CornerTable<R>) {
        let edges: Vec<EdgeId> = mesh.unique_edges().collect();

        // Flip edges to improve valence
        for edge in edges {
            let (v1, v2) = mesh.edge_vertices(edge);
            //if self.constrained_vertices.contains(&v1) || self.constrained_vertices.contains(&v2) {
            if self.constrained_vertices.contains(&v1)
                || self.constrained_vertices.contains(&v2)
                || self.is_edge_constrained(v1, v2)
                // || self.constrained_edges.contains(&edge)
            {
                // TODO: check if edge is constrained
                continue;
            }

            if Self::is_flip_safe(mesh, edge) && Self::will_flip_improve_quality(mesh, edge) {
                mesh.flip_edge(edge);
            }
        }
    }

    fn project_vertices<R: RealNumber>(
        &self,
        mesh: &mut CornerTable<R>,
        grid: &Grid<Triangle3<R>>,
        target_edge_length: R,
    ) {
        let vertices: Vec<VertexId> = mesh.vertices().collect();

        // Project vertices back on original mesh
        for vertex in vertices {
            let vertex_position = mesh.vertex_position(vertex);

            if let Some(closest_point) = grid.closest_point(vertex_position, target_edge_length) {
                mesh[vertex].set_position(closest_point);
            }
        }
    }

    fn is_flip_safe<R: RealNumber>(mesh: &mut CornerTable<R>, edge: EdgeId) -> bool {
        let corner = &mesh[edge.corner()];
        let Some(opposite_corner) = corner.opposite_corner().map(|c| &mesh[c]) else {
            return false; // Not safe if edge is on boundary
        };

        // Check that flipped edge does not already exist
        let v1_idx = corner.vertex();
        let v2_idx = opposite_corner.vertex();

        let mut safe = true;
        mesh.vertices_around_vertex(v2_idx, |v| safe &= v1_idx != v); // TODO: return control flow

        if !safe {
            return false;
        }

        // Check normals after flip (geometrical safety)
        let mut walker = mesh.walker_from_corner(edge.corner());
        let v1 = *walker.vertex().position();
        let v2 = *walker.move_to_next().vertex().position();
        let v0 = *walker.move_to_next().vertex().position();
        let v3 = *walker.move_to_next().move_to_opposite().vertex().position();

        if Triangle3::is_degenerate(&v1, &v2, &v3) || Triangle3::is_degenerate(&v0, &v1, &v3) {
            return false;
        }

        let Some(old_normal1) = Triangle3::normal(&v0, &v1, &v2) else { return false };
        let Some(new_normal1) = Triangle3::normal(&v1, &v2, &v3) else { return false };
        let threshold = R::f64(5.0_f64.to_radians());

        if old_normal1.angle(&new_normal1) > threshold {
            return false;
        }

        let Some(old_normal2) = Triangle3::normal(&v0, &v2, &v3) else { return false };
        let Some(new_normal2) = Triangle3::normal(&v0, &v1, &v3) else { return false };

        if old_normal2.angle(&new_normal2) > threshold
            || old_normal2.angle(&new_normal1) > threshold
            || old_normal1.angle(&new_normal2) > threshold
        {
            return false;
        }

        true
    }

    fn will_flip_improve_quality<R: RealNumber>(mesh: &mut CornerTable<R>, edge: EdgeId) -> bool {
        let mut walker = mesh.walker_from_corner(edge.corner());
        let v1 = walker.corner().vertex();
        let v2 = walker.move_to_next().corner().vertex();
        let v0 = walker.move_to_next().corner().vertex();
        let v3 = walker.move_to_next().move_to_opposite().corner().vertex();

        let v0_ideal_val = Self::ideal_valence(mesh, v0);
        let v1_ideal_val = Self::ideal_valence(mesh, v1);
        let v2_ideal_val = Self::ideal_valence(mesh, v2);
        let v3_ideal_val = Self::ideal_valence(mesh, v3);

        let v0_val = mesh.vertex_degree(v0) as isize;
        let v1_val = mesh.vertex_degree(v1) as isize;
        let v2_val = mesh.vertex_degree(v2) as isize;
        let v3_val = mesh.vertex_degree(v3) as isize;

        let old_deviation = (v0_val - v0_ideal_val).abs()
            + (v1_val - v1_ideal_val).abs()
            + (v2_val - v2_ideal_val).abs()
            + (v3_val - v3_ideal_val).abs();

        let new_deviation = (v0_val - 1 - v0_ideal_val).abs()
            + (v1_val + 1 - v1_ideal_val).abs()
            + (v2_val - 1 - v2_ideal_val).abs()
            + (v3_val + 1 - v3_ideal_val).abs();

        let v0_pos = mesh.vertex_position(v0);
        let v1_pos = mesh.vertex_position(v1);
        let v2_pos = mesh.vertex_position(v2);
        let v3_pos = mesh.vertex_position(v3);

        let old_face_quality = R::min(
            Triangle3::quality(v0_pos, v1_pos, v2_pos),
            Triangle3::quality(v0_pos, v2_pos, v3_pos),
        );
        let new_face_quality = R::min(
            Triangle3::quality(v1_pos, v2_pos, v3_pos),
            Triangle3::quality(v0_pos, v1_pos, v3_pos),
        );

        (new_deviation < old_deviation && new_face_quality >= old_face_quality * R::half()) ||
               (new_deviation == old_deviation && new_face_quality > old_face_quality) || // Same valence but better quality
               (new_face_quality > old_face_quality * (R::one() + R::half())) // Hurt valence but improve quality by much
    }

    #[inline]
    fn ideal_valence<R: RealNumber>(mesh: &CornerTable<R>, vertex: VertexId) -> isize {
        if mesh.is_vertex_on_boundary(vertex) {
            stats::IDEAL_BOUNDARY_VERTEX_VALENCE as isize
        } else {
            stats::IDEAL_INTERIOR_VERTEX_VALENCE as isize
        }
    }

    fn find_feature_vertices<R: RealNumber>(&mut self, mesh: &CornerTable<R>) {
        let Some(sharp_angle) = self.sharp_edge_angle else { return };
        self.constrained_vertices.clear();
        let sharp_threshold_cos = R::f64(sharp_angle).cos();

        for edge in mesh.unique_edges() {
            let (f1, Some(f2)) = mesh.edge_faces(edge) else { continue };
            let Some(n1) = mesh.face_normal(f1) else { continue };
            let Some(n2) = mesh.face_normal(f2) else { continue };

            let angle_cos = n1.dot(&n2).clamp(-R::one(), R::one());
            if angle_cos < sharp_threshold_cos {
                let (v1, v2) = mesh.edge_vertices(edge);
                self.constrained_vertices.insert(v1);
                self.constrained_vertices.insert(v2);
                self.constrained_edges.insert((v1, v2));
            }
        }
    }

    #[inline]
    fn is_edge_constrained(&self, v1: VertexId, v2: VertexId) -> bool {
        self.constrained_edges.contains(&(v1, v2)) || self.constrained_edges.contains(&(v2, v1))
    }

    #[inline]
    fn remove_edge_constraint(&mut self, v1: VertexId, v2: VertexId) -> bool {
        let r1 = self.constrained_edges.remove(&(v1, v2)); 
        let r2 = self.constrained_edges.remove(&(v2, v1));
        r1 || r2
    }

    fn save_constrained_vertices<P: AsRef<path::Path>, R: RealNumber>(
        &self,
        path: P,
        mesh: &CornerTable<R>,
    ) {
        let mut ps = PolygonSoup::<R>::default();
        for vertex in self.constrained_vertices.iter() {
            let size = R::f32(0.05);
            let point = crate::mesh::builder::cube::<PolygonSoup<R>>(
                *mesh[*vertex].position()
                    - nalgebra::Vector3::new(size * R::half(), size * R::half(), size * R::half()),
                size,
                size,
                size,
            );
            ps.concat(point);
        }

        crate::io::write_to_file(&ps, path.as_ref()).expect("should save mesh to file");
    }

    fn save_constrained_edges<P: AsRef<path::Path>, R: RealNumber>(
        &self,
        path: P,
        mesh: &CornerTable<R>,
    ) {
        let mut ps = PolygonSoup::<R>::default();
        for &(v1, v2) in self.constrained_edges.iter() {
            assert!(
                mesh.vertex_exists(v1),
                "v1 deleted"
            );
            assert!(
                mesh.vertex_exists(v2),
                "v2 deleted"
            );

            let mut are_neighbors = false;
            mesh.vertices_around_vertex(v1, |v| {
                if v == v2 {
                    are_neighbors = true;
                }
            });

            assert!(are_neighbors, "constrained edge should exist in mesh");

            let size = R::f32(0.01);
            let edge_cylinder = crate::mesh::builder::cylinder_by_endpoints::<PolygonSoup<R>>(
                mesh[v1].position(),
                mesh[v2].position(),
                size,
                10,
                1,
            )
            .unwrap();
            ps.concat(edge_cylinder);
        }

        crate::io::write_to_file(&ps, path.as_ref()).expect("should save mesh to file");
    }

    fn save_constraints<R: RealNumber>(&self, iter: usize, prefix: &str, mesh: &CornerTable<R>) {
        return;
        let path = format!("{}_{}_vertices.ply", iter, prefix);
        self.save_constrained_vertices(&path, mesh);
        let path = format!("{}_{}_edges.ply", iter, prefix);
        self.save_constrained_edges(&path, mesh);
        let path = format!("{}_{}.ply", iter, prefix);
        crate::io::write_to_file(mesh, path.as_ref()).expect("should save mesh to file");
    }
}

impl Default for IncrementalRemesher {
    fn default() -> Self {
        Self {
            split_edges: true,
            shift_vertices: true,
            collapse_edges: true,
            flip_edges: true,
            project_vertices: true,
            iterations: 15,
            keep_boundary: true,
            sharp_edge_angle: Some(30.0_f64.to_radians()),
            constrained_vertices: HashSet::new(),
            constrained_edges: HashSet::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::IncrementalRemesher;
    use crate::{io::*, mesh::corner_table::*};
    use std::path::Path;

    #[test]
    fn should_collapse_short_edges() {
        let mut mesh: CornerTable<f32> = StlReader::default()
            .read_from_file(Path::new("./assets/tube.stl"))
            .expect("Read mesh");

        // Only collapse edges
        let target_edge_length = 0.1f32;
        IncrementalRemesher::default()
            .with_iterations_count(5)
            .remesh(&mut mesh, target_edge_length);

        let has_short_edges = mesh.edges().any(|edge| {
            let length = mesh.edge_length(edge);
            length < target_edge_length * 0.8
        });

        assert!(
            !has_short_edges,
            "should not have short edges after remeshing"
        );
    }
}
