use crate::{
    algo::edge_collapse, geometry::traits::RealNumber, helpers::aliases::Vec3,
    mesh::corner_table::*,
};
use nalgebra::{Matrix4, Vector4};
use num_traits::{cast, Float};
use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};


// https://hal.science/hal-01468817v1/file/supplemental.pdf
#[derive(Debug, Default)]
pub struct MeanSquareDist<S: RealNumber> {
    placement: RefCell<EdgeAttribute<MSDPlacement<S>>>,
    buffers: MSDBuffers<S>,
}

impl<S: RealNumber + Default> CollapseStrategy<S> for MeanSquareDist<S> {
    fn set(&mut self, mesh: &CornerTable<S>) {
        self.placement = RefCell::new(mesh.create_edge_attribute());
    }

    fn get_cost(&self, mesh: &CornerTable<S>, edge: EdgeId) -> S {
        let mut placement = MSDPlacement::find(mesh, edge, &mut Default::default());
        let mut self_placement = self.placement.borrow_mut();
        let old_cost = self_placement[edge].distance;
        placement.distance += old_cost;

        self_placement[edge] = placement;
        if let Some(opposite) = mesh.opposite_edge(edge) {
            self_placement[opposite] = placement;
        }

        placement.distance
    }

    #[inline]
    fn get_placement(&self, _mesh: &CornerTable<S>, edge: EdgeId) -> Vec3<S> {
        self.placement.borrow()[edge].position
    }

    fn collapse_edge(&mut self, _mesh: &CornerTable<S>, _edge: EdgeId) {}
}

#[derive(Debug, Clone, Copy, Default)]
struct MSDPlacement<S: RealNumber> {
    position: Vec3<S>,
    distance: S,
}

#[derive(Debug, Default)]
struct MSDBuffers<S: RealNumber> {
    source_samples: Vec<Vec3<S>>, // samples before collapse
    target_samples: Vec<Vec3<S>>, // samples after collapse
    target_ab: Vec<(S, Vec3<S>)>, // barycentric coordinates of target samples
    neighbors: Vec<VertexId>, // vertices around edge
    target_faces: Vec<(Vec3<S>, Vec3<S>)>,
    source_faces: Vec<Triangle3<S>>,
}

impl<S: RealNumber> MSDBuffers<S> {
    fn clear(&mut self) {
        self.source_samples.clear();
        self.target_samples.clear();
        self.neighbors.clear();
        self.target_faces.clear();
        self.source_faces.clear();
        self.target_ab.clear();
    }
}

impl<S: RealNumber> MSDPlacement<S> {
    fn find(
        mesh: &CornerTable<S>, 
        edge: EdgeId, 
        buffers: &mut MSDBuffers<S>,
    ) -> Self {
        buffers.clear();

        let (v1, v2) = mesh.edge_vertices(edge);
        let (v1_pos, v2_pos) = mesh.edge_positions(edge);
        let mut target = (v1_pos + v2_pos) * S::from_f64(0.5).unwrap();

        // Find pi samples
        buffers.source_samples.push(v1_pos);
        buffers.source_samples.push(v2_pos);

        mesh.edges_around_vertex(v1, |edge| {
            let (s, e) = mesh.edge_positions(edge.id());
            let mid = (s + e) * S::from_f64(0.5).unwrap();
            buffers.source_samples.push(mid);
        });

        mesh.edges_around_vertex(v2, |edge| {
            let (s, e) = mesh.edge_vertices(edge.id());

            if s == v1 || e == v1 {
                return; // TODO: do the same above?
            }
            
            let mid = (mesh[s].position() + mesh[e].position()) * S::from_f64(0.5).unwrap();
            buffers.source_samples.push(mid);
        });

        mesh.faces_around_vertex(v1, |face| {
            let tri = mesh.face_positions(face);
            buffers.source_faces.push(tri);
        });

        mesh.faces_around_vertex(v2, |face| {
            let tri = mesh.face_positions(face);
            buffers.source_faces.push(tri); // TODO: avoid duplicates
        });

        // Find Pi samples
        // buffers.target_samples.push(target);
        // buffers.target_ab.push((S::zero(), Vec3::zeros()));

        // let half = S::from_f64(0.5).unwrap();

        // mesh.vertices_around_vertex(v1, |vert| {
        //     if vert == v2 {
        //         return;
        //     }

        //     let mid = (target + mesh[vert].position()) * half;
        //     buffers.target_samples.push(mid);
        //     buffers.target_ab.push((half, mesh[vert].position() * half));
        //     buffers.neighbors.push(vert);
        // });

        // mesh.vertices_around_vertex(v2, |vert| {
        //     if vert == v1 || buffers.neighbors.contains(&vert) {
        //         return;
        //     }

        //     let mid = (target + mesh[vert].position()) * half;
        //     buffers.target_samples.push(mid);
        //     buffers.target_ab.push((half, mesh[vert].position() * half));
        //     buffers.neighbors.push(vert);
        // });
    
        // mesh.vertex_bounding_edges(v1, |edge| {
        //     let (s, e) = mesh.edge_vertices(edge);

        //     if s == v2 || e == v2 {
        //         return;
        //     }

        //     buffers.target_faces.push((*mesh[s].position(), *mesh[e].position()));
        // });

        // mesh.vertex_bounding_edges(v2, |edge| {
        //     let (s, e) = mesh.edge_vertices(edge);

        //     if s == v1 || e == v1 {
        //         return;
        //     }

        //     buffers.target_faces.push((*mesh[s].position(), *mesh[e].position()));
        // });

        let mut update_target_samples = |target: &Vec3<S>, buffers: &mut MSDBuffers<S>| {
            buffers.target_samples.clear();
            buffers.target_ab.clear();
            buffers.neighbors.clear();
            buffers.target_faces.clear();

            buffers.target_samples.push(*target);
            buffers.target_ab.push((S::zero(), Vec3::zeros()));
    
            let half = S::from_f64(0.5).unwrap();
    
            mesh.vertices_around_vertex(v1, |vert| {
                if vert == v2 {
                    return;
                }
    
                let mid = (target + mesh[vert].position()) * half;
                buffers.target_samples.push(mid);
                buffers.target_ab.push((half, mesh[vert].position() * half));
                buffers.neighbors.push(vert);
            });
    
            mesh.vertices_around_vertex(v2, |vert| {
                if vert == v1 || buffers.neighbors.contains(&vert) {
                    return;
                }
    
                let mid = (target + mesh[vert].position()) * half;
                buffers.target_samples.push(mid);
                buffers.target_ab.push((half, mesh[vert].position() * half));
                buffers.neighbors.push(vert);
            });
        
            mesh.vertex_bounding_edges(v1, |edge| {
                let (s, e) = mesh.edge_vertices(edge);
    
                if s == v2 || e == v2 {
                    return;
                }
    
                buffers.target_faces.push((*mesh[s].position(), *mesh[e].position()));
            });
    
            mesh.vertex_bounding_edges(v2, |edge| {
                let (s, e) = mesh.edge_vertices(edge);
    
                if s == v1 || e == v1 {
                    return;
                }
    
                buffers.target_faces.push((*mesh[s].position(), *mesh[e].position()));
            });
        };


        for _ in 0..10 {
            update_target_samples(&target, buffers);
            //println!("target: {:?}", target);
            target = Self::find_next(target, buffers);
        }

        let distance = buffers
            // .source_faces
            // .iter()
            // .map(|tri| (tri.closest_point(&target) - target).norm_squared())
            // .reduce(Float::max)
            // .unwrap_or(S::zero());
            .source_faces
            .iter()
            .map(|tri| buffers.target_samples.iter()
                .map(|sample| (tri.closest_point(sample) - sample).norm())
                .reduce(Float::max)
                .unwrap_or(S::zero())
            )
            .reduce(Float::max)
            .unwrap_or(S::zero());

        // println!("distance: {:?}", distance);

        Self {
            distance,
            position: target,
        }
    }

    fn find_next(current: Vec3<S>, buffers: &mut MSDBuffers<S>) -> Vec3<S> {
        let mut source_closest = Vec::new();
        let mut target_closest = Vec::new();
        let mut source_ab = Vec::new();

        let target_faces: Vec<_> = buffers.target_faces
            .iter()
            .map(|(v1, v2)| Triangle3::new(current, *v1, *v2))
            .collect();

        for sample in &buffers.source_samples {
            let mut closest_face = 0;
            let mut closest_point = Vec3::zeros();
            let mut min_dist = S::infinity();

            for i in 0..target_faces.len() {
                let p = target_faces[i].closest_point(sample);
                let dist = (p - sample).norm_squared();

                if dist < min_dist {
                    min_dist = dist;
                    closest_face = i;
                    closest_point = p;
                }
            }

            let closest_face = &target_faces[closest_face];
            let bary = closest_face.barycentric(&closest_point);
            let ai = bary.u();
            let bi = closest_face.p2() * bary.v() + closest_face.p3() * bary.w();

            source_ab.push((ai, bi));
            source_closest.push(closest_point);
        }

        for sample in &buffers.target_samples {
            let mut closest_point = Vec3::zeros();
            let mut min_dist = S::infinity();

            for tri in &buffers.source_faces {
                let p = tri.closest_point(sample);
                let dist = (p - sample).norm_squared();

                if dist < min_dist {
                    min_dist = dist;
                    closest_point = p;
                }
            }

            target_closest.push(closest_point);
        }

        // let mut c = S::zero();

        // for (coord, _) in &source_bary {
        //     c += (coord.inner().transpose() * coord.inner())[(0, 0)];
        // }

        // for (coord, _) in &target_bary {
        //     c += (coord.inner().transpose() * coord.inner())[(0, 0)];
        // }
        
        // let mut C = Vec3::zeros();
        // let two = S::one() + S::one();

        // for i in 0..buffers.source_samples.len() {
        //     let (ai, bi) = &source_bary[i];
        //     let ai = Matrix3::from_diagonal(ai.inner());
        //     let pi = buffers.source_samples[i];
        //     C += (ai * (bi - pi)) * two;
        // }

        // for i in 0..buffers.target_samples.len() {
        //     let (ai, bi) = &target_bary[i];
        //     let ai = Matrix3::from_diagonal(ai.inner());
        //     let pi = buffers.target_samples[i];
        //     C += (ai * (bi - pi)) * two;
        // }

        
        let mut c = S::zero();

        for (ai, _) in &source_ab {
            c += *ai * *ai;
        }

        for (ai, _) in &buffers.target_ab {
            c += *ai * *ai;
        }
        
        let mut C = Vec3::zeros();
        let two = S::one() + S::one();

        for i in 0..buffers.source_samples.len() {
            let (ai, bi) = &source_ab[i];
            let pi = &buffers.source_samples[i];
            C += (bi - pi) * *ai * two;
        }

        for i in 0..buffers.target_samples.len() {
            let (ai, bi) = &buffers.target_ab[i];
            let pi = &target_closest[i];
            C += (bi - pi) * *ai * two;
        }

        - C / (c + c)
    }
}

/// Collapse candidate
struct Contraction<S: RealNumber> {
    edge: EdgeId,
    cost: S,
}

impl<S: RealNumber> Contraction<S> {
    #[inline]
    fn new(edge: EdgeId, cost: S) -> Self {
        Self { edge, cost }
    }
}

impl<S: RealNumber> Eq for Contraction<S> {}

impl<S: RealNumber> PartialEq for Contraction<S> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.edge == other.edge
    }
}

impl<S: RealNumber> Ord for Contraction<S> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.cost.partial_cmp(&self.cost).unwrap()
    }
}

impl<S: RealNumber> PartialOrd for Contraction<S> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Strategy of edge collapsing
pub trait CollapseStrategy<S: RealNumber>: Default {
    /// Set from `mesh`
    fn set(&mut self, mesh: &CornerTable<S>);

    /// Returns `edge` collapsing cost. Cost is computed at point returned by [get_placement] method. Smaller is better.
    fn get_cost(&self, mesh: &CornerTable<S>, edge: EdgeId) -> S;

    /// Returns point at which `edge` will be collapsed. Ideally it should minimize cost.
    fn get_placement(&self, mesh: &CornerTable<S>, edge: EdgeId) -> Vec3<S>;

    /// Called on edge collapse. Can be used to update internal state.
    fn collapse_edge(&mut self, mesh: &CornerTable<S>, edge: EdgeId);
}

///
/// Collapsing strategy based on quadric error.
/// Collapsing cost is approximated using quadric matrices.
/// Collapsing point is placed on middle of edge.
/// Based on article of Heckber and Garland: http://www.cs.cmu.edu/~garland/Papers/quadrics.pdf.
///
pub struct QuadricError<S: RealNumber> {
    vertex_quadric_map: HashMap<VertexId, Matrix4<S>>,
}

impl<S: RealNumber> Default for QuadricError<S> {
    #[inline]
    fn default() -> Self {
        Self {
            vertex_quadric_map: HashMap::new(),
        }
    }
}

impl<S: RealNumber> CollapseStrategy<S> for QuadricError<S> {
    fn set(&mut self, mesh: &CornerTable<S>) {
        // Preallocate memory
        if let (_, Some(max_size)) = mesh.vertices().size_hint() {
            self.vertex_quadric_map.reserve(max_size);
        }

        for vertex in mesh.vertices() {
            let mut quadric = Matrix4::zeros();

            // Vertex error quadric = sum of quadrics of one ring faces
            mesh.faces_around_vertex(vertex, |face| {
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

    fn get_cost(&self, mesh: &CornerTable<S>, edge: EdgeId) -> S {
        let (v1, v2) = mesh.edge_vertices(edge);

        let q1 = self.vertex_quadric_map.get(&v1).unwrap();
        let q2 = self.vertex_quadric_map.get(&v2).unwrap();

        let new_position = self.get_placement(mesh, edge);
        let v = Vector4::new(new_position.x, new_position.y, new_position.z, S::one());
        let v_t = v.transpose();

        Float::sqrt(Float::abs((v_t * (q1 + q2) * v)[0]))
    }

    #[inline]
    fn get_placement(&self, mesh: &CornerTable<S>, edge: EdgeId) -> Vec3<S> {
        let (v1_pos, v2_pos) = mesh.edge_positions(edge);
        (v1_pos + v2_pos) * S::from_f64(0.5).unwrap()
    }

    fn collapse_edge(&mut self, mesh: &CornerTable<S>, edge: EdgeId) {
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
pub struct IncrementalDecimator<S: RealNumber, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TCollapseStrategy: CollapseStrategy<S>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<S>,
{
    decimation_criteria: TEdgeDecimationCriteria,
    min_faces_count: usize,
    min_face_quality: S,
    keep_boundary: bool,
    priority_queue: BinaryHeap<Contraction<S>>,
    not_safe_collapses: Vec<Contraction<S>>,
    collapse_strategy: TCollapseStrategy,
}

impl<S: RealNumber, TCollapseStrategy, TEdgeDecimationCriteria>
    IncrementalDecimator<S, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TCollapseStrategy: CollapseStrategy<S>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<S>,
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
    pub fn decimate(&mut self, mesh: &mut CornerTable<S>) {
        // Clear internals data structures
        self.priority_queue.clear();
        self.not_safe_collapses.clear();
        self.collapse_strategy.set(mesh);

        self.fill_queue(mesh);
        self.collapse_edges(mesh);
    }

    /// Collapse edges
    fn collapse_edges(&mut self, mesh: &mut CornerTable<S>) {
        let mut remaining_faces_count = mesh.faces().count();
        let mut cost_needs_update = mesh.create_edge_attribute::<bool>();

        while !self.priority_queue.is_empty() || !self.not_safe_collapses.is_empty() {
            // Collapse edges one by one taking them from priority queue
            while let Some(mut best) = self.priority_queue.pop() {
                // Edge was collapsed?
                if !mesh.edge_exists(best.edge) {
                    continue;
                }

                // Need to update collapse cost?
                if cost_needs_update[best.edge] {
                    cost_needs_update[best.edge] = false;
                    best.cost = self.collapse_strategy.get_cost(mesh, best.edge);

                    if self
                        .decimation_criteria
                        .should_decimate(best.cost, mesh, best.edge)
                    {
                        self.priority_queue.push(best);
                    }

                    continue;
                }

                let (v1, v2) = mesh.edge_vertices(best.edge);
                let collapse_at = self.collapse_strategy.get_placement(mesh, best.edge);

                // Skip not safe collapses
                if !edge_collapse::is_safe(mesh, best.edge, &collapse_at, self.min_face_quality) {
                    self.not_safe_collapses.push(best);
                    continue;
                }

                // Find edges affected by collapse
                mesh.edges_around_vertex(v1, |edge| cost_needs_update[edge] = true);
                mesh.edges_around_vertex(v2, |edge| cost_needs_update[edge] = true);

                // Inform collapse strategy about collapse
                self.collapse_strategy.collapse_edge(mesh, best.edge);

                // Update number of remaining faces
                // If edge is on boundary 1 face is collapsed
                // If edge is interior then 2
                if mesh.is_edge_on_boundary(best.edge) {
                    remaining_faces_count -= 1;
                } else {
                    remaining_faces_count -= 2;
                }

                // Collapse edge
                mesh.collapse_edge(best.edge, &collapse_at);

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
                    if !mesh.edge_exists(collapse.edge) {
                        continue;
                    }

                    let new_cost = self.collapse_strategy.get_cost(mesh, collapse.edge);
                    let (v1_pos, v2_pos) = mesh.edge_positions(collapse.edge);
                    let new_position = (v1_pos + v2_pos) * S::from_f64(0.5).unwrap();

                    // Safe to collapse and have low error
                    let should_decimate =
                        self.decimation_criteria
                            .should_decimate(new_cost, mesh, collapse.edge);
                    let is_safe = edge_collapse::is_safe(
                        mesh,
                        collapse.edge,
                        &new_position,
                        self.min_face_quality,
                    );

                    if should_decimate && is_safe {
                        self.priority_queue
                            .push(Contraction::new(collapse.edge, new_cost));
                    }
                }

                self.not_safe_collapses.clear();
            }
        }
    }

    /// Fill priority queue with edges of original mesh that have low collapse cost and can be collapsed
    fn fill_queue(&mut self, mesh: &CornerTable<S>) {
        for edge in mesh.unique_edges() {
            let cost = self.collapse_strategy.get_cost(mesh, edge);
            let is_collapse_topologically_safe = edge_collapse::is_topologically_safe(mesh, edge);

            if self.keep_boundary && edge_collapse::will_collapse_affect_boundary(mesh, edge) {
                continue;
            }

            // Collapsable and low cost?
            if self.decimation_criteria.should_decimate(cost, mesh, edge)
                && is_collapse_topologically_safe
            {
                self.priority_queue.push(Contraction::new(edge, cost));
            }
        }
    }
}

impl<S: RealNumber, TCollapseStrategy, TEdgeDecimationCriteria> Default
    for IncrementalDecimator<S, TCollapseStrategy, TEdgeDecimationCriteria>
where
    TCollapseStrategy: CollapseStrategy<S>,
    TEdgeDecimationCriteria: EdgeDecimationCriteria<S>,
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
pub trait EdgeDecimationCriteria<S: RealNumber>: Default {
    fn should_decimate(&self, error: S, mesh: &CornerTable<S>, edge: EdgeId) -> bool;
}

///
/// Always decimate edges
///
#[derive(Debug, Default)]
pub struct AlwaysDecimate;

impl<S: RealNumber> EdgeDecimationCriteria<S> for AlwaysDecimate {
    #[inline]
    fn should_decimate(&self, _error: S, _mesh: &CornerTable<S>, _edge: EdgeId) -> bool {
        true
    }
}

///
/// Never decimate edges
///
#[derive(Debug, Default)]
pub struct NeverDecimate;

impl<S: RealNumber> EdgeDecimationCriteria<S> for NeverDecimate {
    #[inline]
    fn should_decimate(&self, _error: S, _mesh: &CornerTable<S>, _edge: EdgeId) -> bool {
        false
    }
}

///
/// Decimate with a constant error value.
/// This will result in a uniform decimation result.
///
#[derive(Debug)]
pub struct ConstantErrorDecimationCriteria<S: RealNumber> {
    max_error: S,
}

impl<S: RealNumber> ConstantErrorDecimationCriteria<S> {
    #[inline]
    pub fn new(max_error: S) -> Self {
        Self { max_error }
    }
}

impl<S: RealNumber> EdgeDecimationCriteria<S> for ConstantErrorDecimationCriteria<S> {
    #[inline]
    fn should_decimate(&self, error: S, _mesh: &CornerTable<S>, _edge: EdgeId) -> bool {
        error < self.max_error
    }
}

impl<S: RealNumber> Default for ConstantErrorDecimationCriteria<S> {
    #[inline]
    fn default() -> Self {
        Self::new(cast(0.001).unwrap())
    }
}

///
/// Will choose the maximum error, and decimate accordingly, depending on the distance from origin.
#[derive(Debug)]
pub struct BoundingSphereDecimationCriteria<S: RealNumber> {
    origin: Vec3<S>,
    radii_sq_error_map: Vec<(S, S)>,
}

impl<S: RealNumber> BoundingSphereDecimationCriteria<S> {
    pub fn new(origin: Vec3<S>, radii_error_map: Vec<(S, S)>) -> Self {
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

impl<S: RealNumber> EdgeDecimationCriteria<S> for BoundingSphereDecimationCriteria<S> {
    #[inline]
    fn should_decimate(&self, error: S, mesh: &CornerTable<S>, edge: EdgeId) -> bool {
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

impl<S: RealNumber> Default for BoundingSphereDecimationCriteria<S> {
    fn default() -> Self {
        let origin = Vec3::<S>::zeros();
        let radius = Float::max_value();
        let radii_error = vec![(radius, cast(0.001).unwrap())];
        Self::new(origin, radii_error)
    }
}
