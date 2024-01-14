use std::{collections::{BinaryHeap, HashMap}, cmp::Ordering};

use nalgebra::{Vector4, Matrix4};
use num_traits::{cast, Float};

use crate::{
    mesh::traits::{
        EditableMesh, 
        Mesh, 
        TopologicalMesh, 
        MeshMarker, 
        Marker
    }, 
    algo::edge_collapse, helpers::aliases::Vec3
};

/// Collapse candidate
struct Contraction<TMesh: Mesh> {
    edge: TMesh::EdgeDescriptor,
    cost: TMesh::ScalarType
}

impl<TMesh: Mesh> Contraction<TMesh> {
    fn new(edge: TMesh::EdgeDescriptor, cost: TMesh::ScalarType) -> Self {
        return Self { edge, cost } ;
    }
}

impl<TMesh: Mesh> Eq for Contraction<TMesh> {}

impl<TMesh: Mesh> PartialEq for Contraction<TMesh> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.edge == other.edge;
    }
}

impl<TMesh: Mesh> Ord for Contraction<TMesh> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.partial_cmp(other).unwrap();
    }
}

impl<TMesh: Mesh> PartialOrd for Contraction<TMesh> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        return other.cost.partial_cmp(&self.cost);
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
    vertex_quadric_map: HashMap<TMesh::VertexDescriptor, Matrix4<TMesh::ScalarType>>
}

impl<TMesh: Mesh> Default for QuadricError<TMesh> {
    fn default() -> Self {
        return Self { vertex_quadric_map: HashMap::new() };
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

    fn get_cost(&self, mesh: &TMesh, edge: &<TMesh as Mesh>::EdgeDescriptor) -> <TMesh as Mesh>::ScalarType {
        let (v1, v2) = mesh.edge_vertices(edge);

        let q1 = self.vertex_quadric_map.get(&v1).unwrap();
        let q2 = self.vertex_quadric_map.get(&v2).unwrap();

        let new_position = self.get_placement(mesh, edge);
        let v = new_position.to_homogeneous();
        let v_t = v.transpose();

        return (v_t * (q1 + q2) * v)[0].abs().sqrt();
    }

    #[inline]
    fn get_placement(&self, mesh: &TMesh, edge: &<TMesh as Mesh>::EdgeDescriptor) -> Vec3<<TMesh as Mesh>::ScalarType> {
        let (v1_pos, v2_pos) = mesh.edge_positions(edge);
        return (v1_pos + v2_pos) * cast::<f64, TMesh::ScalarType>(0.5).unwrap();
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
///     .max_error(Some(0.00015))
///     .min_faces_count(None);
/// decimator.decimate(&mut mesh);
/// ```
/// 
pub struct IncrementalDecimator<TMesh, TCollapseStrategy>
where 
    TMesh: EditableMesh + TopologicalMesh + MeshMarker, 
    TCollapseStrategy: CollapseStrategy<TMesh>
{
    max_error: TMesh::ScalarType,
    min_faces_count: usize,
    min_face_quality: TMesh::ScalarType,
    priority_queue: BinaryHeap<Contraction<TMesh>>,
    not_safe_collapses: Vec<Contraction<TMesh>>,
    collapse_strategy: TCollapseStrategy
}

impl<TMesh, TCollapseStrategy> IncrementalDecimator<TMesh, TCollapseStrategy> 
where 
    TMesh: EditableMesh + TopologicalMesh + MeshMarker, 
    TCollapseStrategy: CollapseStrategy<TMesh> 
{
    #[inline]
    pub fn new() -> Self {
        return Default::default();
    }

    ///
    /// Set maximum allowed error for decimation. Edges with higher error won't be collapsed. 
    /// Should be a positive number. Default is `0.001`.
    /// Pass `None` to disable max error check.
    /// 
    #[inline]
    pub fn max_error(mut self, max_error: Option<TMesh::ScalarType>) -> Self {
        match max_error {
            Some(err) => { 
                debug_assert!(err.is_sign_positive(), "Max error should be a positive");
                self.max_error = err;
            },
            None => self.max_error = TMesh::ScalarType::infinity(),
        };

        return self;
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
            },
            None => self.min_faces_count = 0,
        };

        return self;
    }

    ///
    /// Decimated given `mesh`.
    /// 
    /// ## Example
    /// ```ignore
    /// let mut decimator = IncrementalDecimator::<CornerTableD, QuadricError<CornerTableD>>::new()
    ///     .max_error(Some(0.00015))
    ///     .min_faces_count(None);
    /// decimator.decimate(&mut mesh);
    /// ```
    /// 
    pub fn decimate(&mut self, mesh: &mut TMesh) {
        debug_assert!(self.max_error.is_finite() || self.min_faces_count > 0, "Either max error or min faces count should be set.");

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
                    if best.cost < self.max_error {
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
                    let new_position = (v1_pos + v2_pos) * cast::<_, TMesh::ScalarType>(0.5).unwrap();

                    // Safe to collapse and have low error
                    if new_cost < self.max_error && edge_collapse::is_safe(mesh, &collapse.edge, &new_position, self.min_face_quality) {
                        self.priority_queue.push(Contraction::new(collapse.edge, new_cost));
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

            // Collapsable and low cost?
            if cost < self.max_error && is_collapse_topologically_safe {
                self.priority_queue.push(Contraction::new(edge, cost));
            }
        }
    }
}

impl<TMesh, TCollapseStrategy> Default for IncrementalDecimator<TMesh, TCollapseStrategy> 
where 
    TMesh: EditableMesh + TopologicalMesh + MeshMarker, 
    TCollapseStrategy: CollapseStrategy<TMesh> 
{
    fn default() -> Self {
        return Self {
            max_error: cast(0.001).unwrap(),
            min_faces_count: 0,
            min_face_quality: cast(0.1).unwrap(),
            priority_queue: BinaryHeap::new(),
            not_safe_collapses: Vec::new(),
            collapse_strategy: TCollapseStrategy::default()
        };
    }
}
