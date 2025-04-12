use crate::{helpers::aliases::Vec3d, mesh::{corner_table::{prelude::CornerTableD, VertexId}, traits::*}};
use faer::{
    linalg::solvers::{ShapeCore, Solve},
    sparse as sp,
};
use nalgebra as na;
use std::collections::{HashMap, HashSet};

#[derive(Debug)]
pub enum DeformError {
    InvalidHandle,
    InvalidRegionOfInterest,
    InternalError(&'static str),
}

pub fn prepare_deform(
    mesh: &CornerTableD,
    handle: &HashSet<VertexId>,
    region_of_interest: &HashSet<VertexId>,
    anchor: &HashSet<VertexId>,
) -> Result<PreparedDeform, DeformError> {
    if handle.is_empty() {
        return Err(DeformError::InvalidHandle);
    }

    if region_of_interest.is_empty() {
        return Err(DeformError::InvalidRegionOfInterest);
    }

    // TODO: should we remove vertices from the handle that do not border with roi?
    if !handle_borders_with_roi(mesh, handle, region_of_interest) {
        return Err(DeformError::InvalidHandle);
    }

    let handle = Vec::from_iter(handle.iter().copied());
    let region_of_interest = Vec::from_iter(region_of_interest.iter().copied());
    let anchor = Vec::from_iter(anchor.iter().copied());

    let mut vertex_to_col = HashMap::with_capacity(region_of_interest.len() + handle.len() + anchor.len());

    for vert in &region_of_interest {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    for vert in &handle {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    for vert in &anchor {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    let num_distortion_equations = region_of_interest.len() * 3;
    let num_fitting_equations = (handle.len() + anchor.len()) * 3;
    let num_equations = num_distortion_equations + num_fitting_equations;

    let mut coeffs = Vec::with_capacity(num_equations);
    compute_distortion_term_coeffs(mesh, &region_of_interest, &vertex_to_col, &mut coeffs);
    compute_fitting_term_coeffs(handle.len() + anchor.len(), num_distortion_equations, &mut coeffs);

    // Ax = b
    let Ok(a_mat) = sp::SparseColMat::try_new_from_triplets(num_equations, num_equations, &coeffs) else {
        return Err(DeformError::InternalError("failed to create sparse matrix"));
    };

    for row in a_mat.to_dense().row_iter() {
        println!("{:+.2?}", row);
    }

    let Ok(factorization) = a_mat.sp_qr() else {
        return Err(DeformError::InternalError("failed to factorize matrix"));
    };

    Ok(PreparedDeform {
        factorization,
        handle,
        region_of_interest,
        anchor,
    })
}

#[derive(Debug)]
pub struct PreparedDeform {
    factorization: faer::sparse::linalg::solvers::Qr<usize, f64>,
    handle: Vec<VertexId>,
    region_of_interest: Vec<VertexId>,
    anchor: Vec<VertexId>,
}

impl PreparedDeform {
    pub fn deform(&self, mesh: &mut CornerTableD, transform: na::Matrix4<f64>) {
        let handle_transformed = self
            .handle
            .iter()
            .map(|vertex| {
                let position = *mesh.vertex_position(vertex);
                transform.transform_point(&position.into()).coords
            })
            .collect::<Vec<_>>();

        let anchor_positions: Vec<_> = self
            .anchor
            .iter()
            .map(|vertex| *mesh.vertex_position(vertex))
            .collect();

        // Ax = b
        let b_vec = compute_b_vec(
            self.factorization.nrows(),
            self.region_of_interest.len() * 3,
            &handle_transformed,
            &anchor_positions,
        );
        let solution = self.factorization.solve(&b_vec);

        println!("B = {:?}", b_vec);

        println!("{solution:?}");

        for i in 0..self.region_of_interest.len() {
            let point = Vec3d::new(solution[i * 3], solution[i * 3 + 1], solution[i * 3 + 2]);
            mesh[self.region_of_interest[i]].set_position(point);
        }

        for i in 0..self.handle.len() {
            mesh[self.handle[i]].set_position(handle_transformed[i]);
        }
    }
}

/// Remove vertices from the handle that are not significant.
/// If vertex does not border with a vertex in the region of interest it will not compute the distortion term.
fn handle_borders_with_roi(
    mesh: &CornerTableD,
    handle: &HashSet<VertexId>,
    roi: &HashSet<VertexId>,
) -> bool {
    for &handle_vert in handle {
        let mut is_significant = false;
        mesh.vertices_around_vertex(&handle_vert, |neighbor| {
            if roi.contains(neighbor) {
                is_significant = true;
            }
        });

        if is_significant {
            return true;
        }
    }

    false
}

// References:
// 1. Laplacian Surface Editing
//    https://people.eecs.berkeley.edu/~jrs/meshpapers/SCOLARS.pdf
// 2. CSE 554 Lecture 8: Laplacian Deformation
//    https://www.cse.wustl.edu/~taoju/cse554/lectures/lect08_Deformation.pdf

fn c_mat(vertex_and_neighbors: &Vec<Vec3d>) -> na::OMatrix<f64, na::Dyn, na::U7> {
    let mut a_mat = na::OMatrix::<f64, na::Dyn, na::U7>::zeros(vertex_and_neighbors.len() * 3);

    for i in 0..vertex_and_neighbors.len() {
        let first_row = i * 3;
        let vert = &vertex_and_neighbors[i];

        // First row
        a_mat[(first_row, 0)] = vert.x;
        a_mat[(first_row, 2)] = vert.z;
        a_mat[(first_row, 3)] = -vert.y;
        a_mat[(first_row, 4)] = 1.0;

        // Second row
        let second_row = first_row + 1;
        a_mat[(second_row, 0)] = vert.y;
        a_mat[(second_row, 1)] = -vert.z;
        a_mat[(second_row, 3)] = vert.x;
        a_mat[(second_row, 5)] = 1.0;

        // Third row
        let third_row = second_row + 1;
        a_mat[(third_row, 0)] = vert.z;
        a_mat[(third_row, 1)] = vert.y;
        a_mat[(third_row, 2)] = -vert.x;
        a_mat[(third_row, 6)] = 1.0;
    }

    a_mat
}

fn laplacian(vertex_and_neighbors: &Vec<Vec3d>) -> Vec3d {
    let mut laplacian_coord = Vec3d::zeros();
    for neighbor in &vertex_and_neighbors[1..] {
        laplacian_coord += neighbor;
    }
    laplacian_coord /= (vertex_and_neighbors.len() - 1) as f64;
    vertex_and_neighbors[0] - laplacian_coord
}

fn d_mat(laplacian: &Vec3d) -> na::OMatrix<f64, na::U3, na::U7> {
    let mut d = na::OMatrix::<f64, na::U3, na::U7>::zeros();
    d[(0, 0)] = laplacian.x;
    d[(0, 2)] = laplacian.z;
    d[(0, 3)] = -laplacian.y;
    d[(0, 4)] = 1.0;

    // Second row
    d[(1, 0)] = laplacian.y;
    d[(1, 1)] = -laplacian.z;
    d[(1, 3)] = laplacian.x;
    d[(1, 5)] = 1.0;

    // Third row
    d[(2, 0)] = laplacian.z;
    d[(2, 1)] = laplacian.y;
    d[(2, 2)] = -laplacian.x;
    d[(2, 6)] = 1.0;

    d
}

fn l_mat(num_neighbors: usize) -> na::OMatrix<f64, na::U3, na::Dyn> {
    let num_cols = (num_neighbors + 1) * 3;
    let num_neighbors_inv = 1.0 / num_neighbors as f64;
    let mut l = na::OMatrix::<f64, na::U3, na::Dyn>::zeros(num_cols);

    l[(0, 0)] = 1.0;
    l[(1, 1)] = 1.0;
    l[(2, 2)] = 1.0;

    for i in 3..num_cols {
        match i % 3 {
            0 => l[(0, i)] = -num_neighbors_inv,
            1 => l[(1, i)] = -num_neighbors_inv,
            2 => l[(2, i)] = -num_neighbors_inv,
            _ => unreachable!(),
        }
    }

    l
}

type Triplet = sp::Triplet<usize, usize, f64>;

fn compute_distortion_term_coeffs(
    mesh: &CornerTableD,
    roi: &Vec<VertexId>,
    vertex_to_col: &HashMap<VertexId, usize>,
    coeffs: &mut Vec<Triplet>,
) {
    let mut vertex_and_neighbors = Vec::new();
    let mut vertex_and_neighbors_positions = Vec::new();

    for i in 0..roi.len() {
        let vertex = roi[i];

        vertex_and_neighbors.clear();
        vertex_and_neighbors.push(vertex);
        mesh.vertices_around_vertex(&vertex, |neighbor| {
            if !vertex_to_col.contains_key(neighbor) {
                return;
            }

            vertex_and_neighbors.push(*neighbor);
        });

        vertex_and_neighbors_positions.clear();
        for vertex in &vertex_and_neighbors {
            vertex_and_neighbors_positions.push(*mesh.vertex_position(vertex));
        }

        // TODO: avoid allocations
        let c = c_mat(&vertex_and_neighbors_positions);
        let ct = c.transpose();
        let t = (&ct * c).try_inverse().unwrap() * ct;

        let lap = laplacian(&vertex_and_neighbors_positions);
        let d = d_mat(&lap);
        let t_lap = d * t;

        let l = l_mat(vertex_and_neighbors.len() - 1);
        let h = l - t_lap;

        let row = i * 3;
        for j in 0..vertex_and_neighbors.len() {
            let vert = &vertex_and_neighbors[j];
            let col = vertex_to_col[vert] * 3;
            let h_col = j * 3;

            coeffs.push(Triplet::new(row + 0, col + 0, h[(0, h_col + 0)]));
            coeffs.push(Triplet::new(row + 0, col + 1, h[(0, h_col + 1)]));
            coeffs.push(Triplet::new(row + 0, col + 2, h[(0, h_col + 2)]));

            coeffs.push(Triplet::new(row + 1, col + 0, h[(1, h_col + 0)]));
            coeffs.push(Triplet::new(row + 1, col + 1, h[(1, h_col + 1)]));
            coeffs.push(Triplet::new(row + 1, col + 2, h[(1, h_col + 2)]));

            coeffs.push(Triplet::new(row + 2, col + 0, h[(2, h_col + 0)]));
            coeffs.push(Triplet::new(row + 2, col + 1, h[(2, h_col + 1)]));
            coeffs.push(Triplet::new(row + 2, col + 2, h[(2, h_col + 2)]));
        }
    }
}

fn compute_fitting_term_coeffs(
    num_handles: usize,
    num_distortion_terms: usize,
    coeffs: &mut Vec<Triplet>,
) {
    for i in 0..num_handles {
        let row_distortion = num_distortion_terms + i * 3;
        coeffs.push(Triplet::new(row_distortion + 0, row_distortion + 0, 1.0));
        coeffs.push(Triplet::new(row_distortion + 1, row_distortion + 1, 1.0));
        coeffs.push(Triplet::new(row_distortion + 2, row_distortion + 2, 1.0));
    }
}

fn compute_b_vec(
    num_equations: usize,
    num_distortion_equations: usize,
    handle_final: &Vec<Vec3d>,
    anchor: &Vec<Vec3d>,
) -> faer::Col<f64> {
    let mut b = faer::Col::zeros(num_equations);
    let mut handle_idx = 0;

    for row in (num_distortion_equations..num_distortion_equations + handle_final.len() * 3).step_by(3) {
        let transformed = &handle_final[handle_idx];
        b[row + 0] = transformed.x;
        b[row + 1] = transformed.y;
        b[row + 2] = transformed.z;

        handle_idx += 1;
    }

    let mut anchor_idx = 0;
    for row in (num_distortion_equations + handle_final.len() * 3..num_equations).step_by(3) {
        b[row + 0] = anchor[anchor_idx].x;
        b[row + 1] = anchor[anchor_idx].y;
        b[row + 2] = anchor[anchor_idx].z;
        anchor_idx += 1;
    }

    b
}
