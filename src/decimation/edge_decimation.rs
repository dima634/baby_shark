use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};

use nalgebra::{Matrix4, Vector4};
use num_traits::{cast, Float, FromPrimitive, One};

use crate::{
    algo::edge_collapse,
    helpers::aliases::Vec3,
    mesh::traits::{EditableMesh, Marker, Mesh, MeshMarker, TopologicalMesh},
};

/// Collapse candidate
struct Contraction<TMesh: Mesh> {
    edge: TMesh::EdgeDescriptor,
    cost: TMesh::ScalarType,
}

impl<TMesh: Mesh> Contraction<TMesh> {
    fn new(edge: TMesh::EdgeDescriptor, cost: TMesh::ScalarType) -> Self {
        Self { edge, cost }
    }
}

impl<TMesh: Mesh> Eq for Contraction<TMesh> {}

impl<TMesh: Mesh> PartialEq for Contraction<TMesh> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.edge == other.edge
    }
}

impl<TMesh: Mesh> Ord for Contraction<TMesh> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.cost.partial_cmp(&self.cost).unwrap()
    }
}

impl<TMesh: Mesh> PartialOrd for Contraction<TMesh> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Strategy of edge collapsing
pub trait CollapseStrategy<TMesh: Mesh>: Default {
    /// Set from `mesh`
    fn set(&mut self, mesh: &TMesh);

    /// Returns `edge` collapsing cost. Cost is computed at point returned by [get_placement] method. Smaller is better.
    fn get_cost(&self, mesh: &TMesh, edge: &TMesh::EdgeDescriptor) -> TMesh::ScalarType;

    /// Returns point at which `edge` will be collapsed. Ideally it should minimize cost.
    fn get_placement(&self, mesh: &TMesh, edge: &TMesh::EdgeDescriptor) -> Vec3<TMesh::ScalarType>;

    /// Called on edge collapse. Can be used to update internal state.
    fn collapse_edge(&mut self, mesh: &TMesh, edge: &TMesh::EdgeDescriptor);
}

///
/// Collapsing strategy based on quadric error.
/// Collapsing cost is approximated using quadric matrices.
/// Collapsing point is placed on middle of edge.
/// Based on article of Heckber and Garland: http://www.cs.cmu.edu/~garland/Papers/quadrics.pdf.
///
pub struct QuadricError<TMesh: Mesh> {
    vertex_quadric_map: HashMap<TMesh::VertexDescriptor, Matrix4<TMesh::ScalarType>>,
}

impl<TMesh: Mesh> Default for QuadricError<TMesh> {
    fn default() -> Self {
        Self {
            vertex_quadric_map: HashMap::new(),
        }
    }
}

impl<TMesh: Mesh + TopologicalMesh> CollapseStrategy<TMesh> for QuadricError<TMesh> {
    fn set(&mut self, mesh: &TMesh) {
        // Preallocate memory
        if let (_, Some(max_size)) = mesh.vertices().size_hint() {
            self.vertex_quadric_map.reserve(max_size);
        }

        for vertex in mesh.vertices() {
            let mut quadric = Matrix4::zeros();

            // Vertex error quadric = sum of quadrics of one ring faces
            mesh.faces_around_vertex(&vertex, |face| {
                let plane = mesh.face_positions(face).plane();
                let n = plane.get_normal();
                let d = plane.get_distance();

                let p = Vector4::new(n.x, n.y, n.z, -d);
                let p_t = p.transpose();

                quadric += p * p_t;
            });

            self.vertex_quadric_map.insert(vertex, quadric);
        }
    }

    fn get_cost(
        &self,
        mesh: &TMesh,
        edge: &<TMesh as Mesh>::EdgeDescriptor,
    ) -> <TMesh as Mesh>::ScalarType {
        let (v1, v2) = mesh.edge_vertices(edge);

        let q1 = self.vertex_quadric_map.get(&v1).unwrap();
        let q2 = self.vertex_quadric_map.get(&v2).unwrap();

        let new_position = self.get_placement(mesh, edge);
        let v = Vector4::new(new_position.x, new_position.y, new_position.z, TMesh::ScalarType::one());
        let v_t = v.transpose();

        (v_t * (q1 + q2) * v)[0].abs().sqrt()
    }

    #[inline]
    fn get_placement(
        &self,
        mesh: &TMesh,
        edge: &<TMesh as Mesh>::EdgeDescriptor,
    ) -> Vec3<<TMesh as Mesh>::ScalarType> {
        let (v1_pos, v2_pos) = mesh.edge_positions(edge);
        (v1_pos + v2_pos) * TMesh::ScalarType::from_f64(0.5).unwrap()
    }

    fn collapse_edge(&mut self, mesh: &TMesh, edge: &<TMesh as Mesh>::EdgeDescriptor) {
        let (v1, v2) = mesh.edge_vertices(edge);
        let new_quadric = self.vertex_quadric_map[&v1] + self.vertex_quadric_map[&v2];
        self.vertex_quadric_map.insert(v1, new_quadric);
        self.vertex_quadric_map.insert(v2, new_quadric);
    }
}

///
/// Incremental edge decimator.
/// This `struct` implements incremental edge collapse algorithm.
/// On each iteration edge with smallest cost is collapsed.
///
/// ## Generics
/// * `TMesh` - mesh type
/// * `TCollapseStrategy` - strategy that defines edge collapsing cost and placement
///
/// ## Example
/// ```ignore
/// let mut decimator = IncrementalDecimator::<CornerTableD, QuadricError<CornerTableD>>::new()
///     .decimation_criteria(ConstantErrorDecimationCriteria::new(0.00015))
///     .min_faces_count(None);
/// decimator.decimate(&mut mesh);
/// ```
///
pub struct IncrementalDecimator<TMesh, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TMesh: EditableMesh + TopologicalMesh + MeshMarker,
    TCollapseStrategy: CollapseStrategy<TMesh>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<TMesh>,
{
    decimation_criteria: TEdgeDecimationCriteria,
    min_faces_count: usize,
    min_face_quality: TMesh::ScalarType,
    keep_boundary: bool,
    priority_queue: BinaryHeap<Contraction<TMesh>>,
    not_safe_collapses: Vec<Contraction<TMesh>>,
    collapse_strategy: TCollapseStrategy,
}

impl<TMesh, TCollapseStrategy, TEdgeDecimationCriteria>
    IncrementalDecimator<TMesh, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TMesh: EditableMesh + TopologicalMesh + MeshMarker,
    TCollapseStrategy: CollapseStrategy<TMesh>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<TMesh>,
{
    #[inline]
    pub fn new() -> Self {
        Default::default()
    }

    ///
    /// Set the decimation_criteria trait.
    /// This defines the strategy for deciding if an edge needs to be simplified/decimated.
    /// See the `EdgeDecimationCriteria` trait.
    ///
    #[inline]
    pub fn decimation_criteria(mut self, criteria: TEdgeDecimationCriteria) -> Self {
        self.decimation_criteria = criteria;
        self
    }

    ///
    /// Set minimum number of faces in resulting mesh. Should be a non-zero number.
    /// By default this check is disabled.
    /// Pass `None` to disable this check.
    ///
    #[inline]
    pub fn min_faces_count(mut self, min_edges_count: Option<usize>) -> Self {
        match min_edges_count {
            Some(count) => {
                debug_assert!(count != 0, "Min faces count must be positive. If you was intended to disable faces count check pass None rather than zero.");
                self.min_faces_count = count;
            }
            None => self.min_faces_count = 0,
        };

        self
    }

    ///
    /// Keep boundary on decimation.
    ///
    #[inline]
    pub fn keep_boundary(mut self, keep_boundary: bool) -> Self {
        self.keep_boundary = keep_boundary;
        self
    }

    ///
    /// Decimated given `mesh`.
    ///
    /// ## Example
    /// ```ignore
    /// let mut decimator = IncrementalDecimator::<CornerTableD, QuadricError<CornerTableD>>::new()
    ///     .decimation_criteria(ConstantErrorDecimationCriteria::new(0.00015))
    ///     .min_faces_count(None);
    /// decimator.decimate(&mut mesh);
    /// ```
    ///
    pub fn decimate(&mut self, mesh: &mut TMesh) {
        // Clear internals data structures
        self.priority_queue.clear();
        self.not_safe_collapses.clear();
        self.collapse_strategy.set(mesh);

        self.fill_queue(mesh);
        self.collapse_edges(mesh);
    }

    /// Collapse edges
    fn collapse_edges(&mut self, mesh: &mut TMesh) {
        let mut marker = mesh.marker();

        let mut remaining_faces_count = mesh.faces().count();

        while !self.priority_queue.is_empty() || !self.not_safe_collapses.is_empty() {
            // Collapse edges one by one taking them from priority queue
            while let Some(mut best) = self.priority_queue.pop() {
                // Edge was collapsed?
                if !mesh.edge_exist(&best.edge) {
                    continue;
                }

                let (v1, v2) = mesh.edge_vertices(&best.edge);
                let collapse_at = self.collapse_strategy.get_placement(mesh, &best.edge);

                // Skip not safe collapses
                if !edge_collapse::is_safe(mesh, &best.edge, &collapse_at, self.min_face_quality) {
                    self.not_safe_collapses.push(best);
                    continue;
                }

                // Need to update collapse cost?
                if marker.is_edge_marked(&best.edge) {
                    marker.mark_edge(&best.edge, false);

                    best.cost = self.collapse_strategy.get_cost(mesh, &best.edge);
                    if self
                        .decimation_criteria
                        .should_decimate(best.cost, mesh, &best.edge)
                    {
                        self.priority_queue.push(best);
                    }

                    continue;
                }

                // Find edges affected by collapse
                mesh.edges_around_vertex(&v1, |edge| marker.mark_edge(edge, true));
                mesh.edges_around_vertex(&v2, |edge| marker.mark_edge(edge, true));

                // Inform collapse strategy about collapse
                self.collapse_strategy.collapse_edge(mesh, &best.edge);

                // Update number of remaining faces
                // If edge is on boundary 1 face is collapsed
                // If edge is interior then 2
                if mesh.is_edge_on_boundary(&best.edge) {
                    remaining_faces_count -= 1;
                } else {
                    remaining_faces_count -= 2;
                }

                // Collapse edge
                mesh.collapse_edge(&best.edge, &collapse_at);

                // Stop when number of remaining faces smaller than minimal
                if remaining_faces_count <= self.min_faces_count {
                    break;
                }
            }

            // Stop when number of remaining edges smaller than minimal
            if remaining_faces_count <= self.min_faces_count {
                break;
            }

            if !self.not_safe_collapses.is_empty() {
                // Reinsert unsafe collapses (mb they are safe now)
                for collapse in self.not_safe_collapses.iter() {
                    let new_cost = self.collapse_strategy.get_cost(mesh, &collapse.edge);
                    let (v1_pos, v2_pos) = mesh.edge_positions(&collapse.edge);
                    let new_position =
                        (v1_pos + v2_pos) * TMesh::ScalarType::from_f64(0.5).unwrap();

                    // Safe to collapse and have low error
                    if self
                        .decimation_criteria
                        .should_decimate(new_cost, mesh, &collapse.edge)
                        && edge_collapse::is_safe(
                            mesh,
                            &collapse.edge,
                            &new_position,
                            self.min_face_quality,
                        )
                    {
                        self.priority_queue
                            .push(Contraction::new(collapse.edge, new_cost));
                    }
                }

                self.not_safe_collapses.clear();
            }
        }
    }

    /// Fill priority queue with edges of original mesh that have low collapse cost and can be collapsed
    fn fill_queue(&mut self, mesh: &mut TMesh) {
        for edge in mesh.edges() {
            let cost = self.collapse_strategy.get_cost(mesh, &edge);
            let is_collapse_topologically_safe = edge_collapse::is_topologically_safe(mesh, &edge);

            if self.keep_boundary && edge_collapse::will_collapse_affect_boundary(mesh, &edge) {
                continue;
            }

            // Collapsable and low cost?
            if self.decimation_criteria.should_decimate(cost, mesh, &edge)
                && is_collapse_topologically_safe
            {
                self.priority_queue.push(Contraction::new(edge, cost));
            }
        }
    }
}

impl<TMesh, TCollapseStrategy, TEdgeDecimationCriteria> Default
    for IncrementalDecimator<TMesh, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TMesh: EditableMesh + TopologicalMesh + MeshMarker,
    TCollapseStrategy: CollapseStrategy<TMesh>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<TMesh>,
{
    fn default() -> Self {
        Self {
            decimation_criteria: TEdgeDecimationCriteria::default(),
            min_faces_count: 0,
            min_face_quality: cast(0.1).unwrap(),
            keep_boundary: false,
            priority_queue: BinaryHeap::new(),
            not_safe_collapses: Vec::new(),
            collapse_strategy: TCollapseStrategy::default(),
        }
    }
}

///
/// Trait used to decide whether to decimate an edge
///
pub trait EdgeDecimationCriteria<TMesh: Mesh>: Default {
    fn should_decimate(
        &self,
        error: TMesh::ScalarType,
        mesh: &TMesh,
        edge: &TMesh::EdgeDescriptor,
    ) -> bool;
}

///
/// Always decimate edges
///
#[derive(Debug, Default)]
pub struct AlwaysDecimate;

impl<TMesh: Mesh> EdgeDecimationCriteria<TMesh> for AlwaysDecimate {
    #[inline]
    fn should_decimate(
        &self,
        _error: TMesh::ScalarType,
        _mesh: &TMesh,
        _edge: &TMesh::EdgeDescriptor,
    ) -> bool {
        true
    }
}

///
/// Never decimate edges
///
#[derive(Debug, Default)]
pub struct NeverDecimate;

impl<TMesh: Mesh> EdgeDecimationCriteria<TMesh> for NeverDecimate {
    #[inline]
    fn should_decimate(
        &self,
        _error: TMesh::ScalarType,
        _mesh: &TMesh,
        _edge: &TMesh::EdgeDescriptor,
    ) -> bool {
        false
    }
}

///
/// Decimate with a constant error value.
/// This will result in a uniform decimation result.
///
#[derive(Debug)]
pub struct ConstantErrorDecimationCriteria<TMesh: Mesh> {
    max_error: TMesh::ScalarType,
}

impl<TMesh> ConstantErrorDecimationCriteria<TMesh>
where
    TMesh: Mesh,
{
    pub fn new(max_error: TMesh::ScalarType) -> Self {
        Self { max_error }
    }
}

impl<TMesh> EdgeDecimationCriteria<TMesh> for ConstantErrorDecimationCriteria<TMesh>
where
    TMesh: Mesh,
{
    #[inline]
    fn should_decimate(
        &self,
        error: <TMesh as Mesh>::ScalarType,
        _mesh: &TMesh,
        _edge: &<TMesh as Mesh>::EdgeDescriptor,
    ) -> bool {
        error < self.max_error
    }
}

impl<TMesh> Default for ConstantErrorDecimationCriteria<TMesh>
where
    TMesh: Mesh,
{
    fn default() -> Self {
        Self::new(cast(0.001).unwrap())
    }
}

///
/// Will choose the maximum error, and decimate accordingly, depending on the distance from origin.
#[derive(Debug)]
pub struct BoundingSphereDecimationCriteria<TMesh: Mesh> {
    origin: Vec3<TMesh::ScalarType>,
    radii_sq_error_map: Vec<(TMesh::ScalarType, TMesh::ScalarType)>,
}

impl<TMesh: Mesh> BoundingSphereDecimationCriteria<TMesh> {
    pub fn new(
        origin: Vec3<TMesh::ScalarType>,
        radii_error_map: Vec<(TMesh::ScalarType, TMesh::ScalarType)>,
    ) -> Self {
        let radii_sq_error_map = radii_error_map
            .into_iter()
            .map(|(r, e)| (r * r, e))
            .collect();
        Self {
            origin,
            radii_sq_error_map,
        }
    }
}

impl<TMesh: Mesh> EdgeDecimationCriteria<TMesh> for BoundingSphereDecimationCriteria<TMesh> {
    #[inline]
    fn should_decimate(
        &self,
        error: <TMesh as Mesh>::ScalarType,
        mesh: &TMesh,
        edge: &<TMesh as Mesh>::EdgeDescriptor,
    ) -> bool {
        let edge_positions = mesh.edge_positions(edge);
        let max_error = self.radii_sq_error_map.iter().find(|(radius_sq, _)| {
            (self.origin - edge_positions.0).norm_squared() < *radius_sq
                || (self.origin - edge_positions.1).norm_squared() < *radius_sq
        });

        let max_error = max_error
            .unwrap_or(self.radii_sq_error_map.last().unwrap())
            .1;

        error < max_error
    }
}

impl<TMesh> Default for BoundingSphereDecimationCriteria<TMesh>
where
    TMesh: Mesh,
{
    fn default() -> Self {
        let origin = Vec3::<TMesh::ScalarType>::zeros();
        let radius = TMesh::ScalarType::max_value();
        let radii_error = vec![(radius, cast(0.001).unwrap())];
        Self::new(origin, radii_error)
    }
}
