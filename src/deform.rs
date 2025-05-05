use crate::{helpers::aliases::Vec3d, mesh::corner_table::*};
use faer::{
    linalg::solvers::{ShapeCore, Solve},
    sparse::{self as sp},
};
use nalgebra as na;
use std::collections::{HashMap, HashSet};

// References:
// 1. As-Rigid-As-Possible Surface Modeling
//    https://igl.ethz.ch/projects/ARAP/arap_web.pdf

#[derive(Debug)]
pub enum PrepareDeformError {
    InvalidHandle,
    InvalidRegionOfInterest,
    InternalError(&'static str),
}

impl ToString for PrepareDeformError {
    fn to_string(&self) -> String {
        match self {
            PrepareDeformError::InvalidHandle => "Invalid handle".to_string(),
            PrepareDeformError::InvalidRegionOfInterest => "Invalid region of interest".to_string(),
            PrepareDeformError::InternalError(msg) => msg.to_string(),
        }
    }
}

/// Prepare mesh deformation using the ARAP (As-Rigid-As_possible) method.
/// Later, prepared deformation can be used to do interactive/real-time deformation.
///
/// # Arguments
/// * `mesh` - The input mesh to be deformed
/// * `handle_region` - Set of vertices that will be manipulated during deformation.
///                     Should be a subset of `region_of_interest`.
/// * `region_of_interest` - Set of vertices that will be affected by the deformation.
pub fn prepare_deform(
    mesh: &CornerTableD,
    handle_region: &HashSet<VertexId>,
    region_of_interest: &HashSet<VertexId>,
) -> Result<PreparedDeform, PrepareDeformError> {
    if region_of_interest.is_empty() {
        return Err(PrepareDeformError::InvalidRegionOfInterest);
    }

    let handle = Vec::from_iter(handle_region.intersection(region_of_interest).copied());

    if handle.is_empty() || handle.len() == region_of_interest.len() {
        return Err(PrepareDeformError::InvalidHandle);
    }

    let region_of_interest = Vec::from_iter(region_of_interest.difference(handle_region).copied());

    // Compute vertex to row.column map, we will need it to know where to put coefficients in the matrix
    let mut vertex_to_idx = HashMap::with_capacity(region_of_interest.len() + handle.len());

    for vert in &region_of_interest {
        vertex_to_idx.insert(*vert, vertex_to_idx.len());
    }

    for vert in &handle {
        vertex_to_idx.insert(*vert, vertex_to_idx.len());
    }

    let (coeffs, edge_weights) = compute_coeffs(mesh, &handle, &region_of_interest, &vertex_to_idx);
    let size = region_of_interest.len() + handle.len();

    let Ok(a_mat) = sp::SparseColMat::try_new_from_triplets(size, size, &coeffs) else {
        return Err(PrepareDeformError::InternalError(
            "failed to create sparse matrix",
        ));
    };

    let Ok(factorization) = a_mat.sp_lu() else {
        return Err(PrepareDeformError::InternalError(
            "failed to factorize matrix",
        ));
    };

    Ok(PreparedDeform {
        factorization,
        handle,
        region_of_interest,
        edge_weights,
        vertex_to_idx,
        max_iters: 30,
    })
}

#[derive(Debug)]
pub enum DeformError {
    InvalidTarget,
    InternalError(&'static str),
}

impl ToString for DeformError {
    fn to_string(&self) -> String {
        match self {
            DeformError::InvalidTarget => "Invalid target".to_string(),
            DeformError::InternalError(msg) => msg.to_string(),
        }
    }
}

/// Prepared mesh deformation. Can be reused to do interactive/real-time deformation.
///
/// # Example
/// ```ignore
/// let prepared_deform = prepare_deform(&mesh, &handle, &region_of_interest)
///
/// for _ in 0..5 {
///    // ... update target handle positions
///    let deformed = prepared_deform.deform(&mesh, &target_handle_positions).unwrap();
/// }
/// ```
#[derive(Debug)]
pub struct PreparedDeform {
    factorization: faer::sparse::linalg::solvers::Lu<usize, f64>,
    handle: Vec<VertexId>,
    region_of_interest: Vec<VertexId>,
    edge_weights: EdgeAttribute<f64>,
    vertex_to_idx: HashMap<VertexId, usize>,
    max_iters: usize,
}

impl PreparedDeform {
    /// Sets the maximum number of iterations for the deformation process.
    #[inline]
    pub fn with_max_iters(mut self, max_iters: usize) -> Self {
        self.max_iters = max_iters;
        self
    }

    /// Sets the maximum number of iterations for the deformation process.
    #[inline]
    pub fn set_max_iters(&mut self, max_iters: usize) -> &mut Self {
        self.max_iters = max_iters;
        self
    }

    /// Returns deformed mesh.
    /// # Arguments
    /// * `mesh` - The input mesh to be deformed. Should be the same as the one used to prepare deformation.
    /// * `target` - Map of vertex IDs to target positions. The target positions will be used to deform the mesh.
    ///              If handle vertex is not in the target, its position will stay the same (it is considered to be fixed).
    #[inline(never)]
    pub fn deform(
        &self,
        mesh: &CornerTableD,
        target: &HashMap<VertexId, Vec3d>,
    ) -> Result<CornerTableD, DeformError> {
        if target.is_empty() {
            return Err(DeformError::InvalidTarget);
        }

        let mut deformed = mesh.clone();

        // Collect handle positions
        let handle_transformed: Vec<_> = self
            .handle
            .iter()
            .map(|vert| {
                *target
                    .get(vert)
                    .unwrap_or_else(|| mesh[*vert].position()) // position stays the same if not in target
            })
            .collect();

        // Ax = b
        let size = self.factorization.nrows();
        let mut b = [
            faer::Col::<f64>::zeros(size), // x
            faer::Col::<f64>::zeros(size), // y
            faer::Col::<f64>::zeros(size), // z
        ];

        let mut rotations = vec![na::Matrix3::<f64>::zeros(); size];

        for _ in 0..self.max_iters {
            for (i, &vert) in self.vertices().enumerate() {
                let mut s = na::Matrix3::<f64>::zeros();

                mesh.edges_around_vertex(vert, |edge| {
                    let (mut v1, mut v2) = mesh.edge_vertices(edge.id());
                    if edge.is_outgoing() {
                        core::mem::swap(&mut v1, &mut v2);
                    }

                    debug_assert!(v2 == vert);

                    let vi = mesh.vertex_position(v1);
                    let vj = mesh.vertex_position(v2);

                    let vi_deformed = deformed.vertex_position(v1); // IDs after clone may not be the same
                    let vj_deformed = deformed.vertex_position(v2);

                    let e0 = vi - vj;
                    let e1 = vi_deformed - vj_deformed;
                    let weight = self.edge_weights[edge.id()];

                    s += weight * (e0 * e1.transpose());
                });

                let svd = s.svd_unordered(true, true);
                let Some(u) = svd.u else {
                    return Err(DeformError::InternalError("failed to compute SVD"));
                };
                let Some(v) = svd.v_t.map(|v_t| v_t.transpose()) else {
                    return Err(DeformError::InternalError("failed to compute SVD"));
                };
                let det = (v * u.transpose()).determinant();
                let d = na::Vector3::new(1.0, 1.0, det);

                rotations[i] = v * na::Matrix3::from_diagonal(&d) * u.transpose();
            }

            for (i, &vi) in self.vertices().enumerate() {
                let mut bi = na::Vector3::zeros();

                if i >= self.region_of_interest.len() {
                    bi = handle_transformed[i - self.region_of_interest.len()];
                } else {
                    mesh.edges_around_vertex(vi, |edge| {
                        let weight = self.edge_weights[edge.id()];
                        let (mut v0, mut v1) = mesh.edge_vertices(edge.id());
                        if edge.is_outgoing() {
                            core::mem::swap(&mut v0, &mut v1);
                        }
                        debug_assert!(vi == v1);

                        if let Some(v0_idx) = self.vertex_to_idx.get(&v0) {
                            bi += weight * 0.5 * ((rotations[i] + rotations[*v0_idx]) * (mesh[v1].position() - mesh[v0].position()));
                        }
                    });
                }

                b[0][i] = bi.x;
                b[1][i] = bi.y;
                b[2][i] = bi.z;
            }

            for comp in 0..3 {
                let p_prime = self.factorization.solve(&b[comp]);

                for (i, vert) in self.vertices().enumerate() {
                    deformed[*vert].position_mut()[comp] = p_prime[i];
                }
            }

            // TODO: compute energy and early exit if converged
        }

        Ok(deformed)
    }

    fn vertices(&self) -> impl Iterator<Item = &VertexId> + '_ {
        self.region_of_interest.iter().chain(self.handle.iter())
    }
}

type Triplet = sp::Triplet<usize, usize, f64>;

/// Computes cotangent edge weight
fn compute_edge_weights(mesh: &CornerTableD) -> EdgeAttribute<f64> {
    let mut edge_weights = mesh.create_edge_attribute::<f64>();
    edge_weights.fill(f64::INFINITY);

    for edge in mesh.edges() {
        if edge_weights[edge] != f64::INFINITY {
            // Already computed for opposite oriented edge
            continue;
        }

        let mut walker = CornerWalker::from_corner(mesh, edge.corner());
        let v0 = *walker.vertex().position();
        let v1 = *walker.move_to_next().vertex().position();
        let v2 = *walker.move_to_next().vertex().position();

        let a = v2 - v0;
        let b = v1 - v0;
        let mut weight = a.dot(&b) / a.cross(&b).norm();

        if let Some(opposite) = mesh[edge.corner()].opposite_corner() {
            let v3 = walker.set_current_corner(opposite).vertex().position();
            let c = v2 - v3;
            let d = v1 - v3;
            weight += c.dot(&d) / c.cross(&d).norm();
        }

        edge_weights[edge] = weight * 0.5; // max(weight, 0.0)?

        if let Some(opposite) = mesh.opposite_edge(edge) {
            edge_weights[opposite] = weight * 0.5; // Opposite oriented edge has the same weight
        }
    }

    debug_assert!(mesh.edges().all(|edge| edge_weights[edge].is_finite()));

    edge_weights
}

fn compute_coeffs(
    mesh: &CornerTableD,
    handle: &[VertexId],
    roi: &[VertexId],
    vertex_to_idx: &HashMap<VertexId, usize>,
) -> (Vec<Triplet>, EdgeAttribute<f64>) {
    let mut coeffs = Vec::with_capacity(handle.len() + roi.len());
    let edge_weights = compute_edge_weights(mesh);

    for (i, &vert) in roi.into_iter().enumerate() {
        let mut total_weight = 0.0;

        mesh.edges_around_vertex(vert, |edge| {
            let (mut v1, mut v2) = mesh.edge_vertices(edge.id());

            if !vertex_to_idx.contains_key(&v1) || !vertex_to_idx.contains_key(&v2) {
                return;
            }
    
            if edge.is_outgoing() {
                core::mem::swap(&mut v1, &mut v2);
            }

            let weight = edge_weights[edge.id()];
            coeffs.push(Triplet::new(
                i,
                vertex_to_idx[&v1],
                -weight,
            ));
            total_weight += weight;
        });

        if total_weight > 0.0 {
            coeffs.push(Triplet::new(i, i, total_weight));
        }
    }

    for i in 0..handle.len() {
        let row_col = i + roi.len();
        coeffs.push(Triplet::new(row_col, row_col, 1.0));
    }

    (coeffs, edge_weights)
}

#[cfg(test)]
mod tests {
    use crate::mesh::builder::cylinder;
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_deform() {
        let cylinder: CornerTableD = cylinder(10.0, 2.0, 4, 15);
        let handle = HashSet::from_iter(cylinder.vertices().filter(|&v| {
            cylinder.vertex_position(v).y == 10.0 || cylinder.vertex_position(v).y == 0.0
        }));
        let roi = HashSet::from_iter(cylinder.vertices());
    
        // Rotate part of the handle
        let transform = na::Matrix4::new_rotation(na::Vector3::new(0.0, PI / 2.0, 0.0));
        let mut target = HashMap::new();
    
        for &vert in &handle {
            let pos = cylinder.vertex_position(vert).clone();
            if pos.y != 10.0 {
                continue; // Skip the bottom vertices
            }
    
            let new_pos = transform.transform_point(&pos.into()).coords;
            target.insert(vert, new_pos);
        }
    
        prepare_deform(&cylinder, &handle, &roi)
            .expect("should prepare deformation")
            .deform(&cylinder, &target)
            .expect("should deform mesh");
    }
}
