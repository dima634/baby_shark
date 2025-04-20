use crate::{helpers::aliases::Vec3d, mesh::{corner_table::{prelude::CornerTableD, traversal::CornerWalker, *}, traits::*}};
use faer::{
    linalg::solvers::{ShapeCore, Solve},
    sparse::{self as sp},
};
use nalgebra as na;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

#[derive(Debug)]
pub enum DeformError {
    InvalidHandle,
    InvalidRegionOfInterest,
    InternalError(&'static str),
}

pub fn prepare_deform_arap(
    mesh: &CornerTableD,
    handle: &BTreeSet<VertexId>,
    region_of_interest: &BTreeSet<VertexId>,
    anchor: &BTreeSet<VertexId>,
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

    //let mut vertex_to_col = HashMap::with_capacity(region_of_interest.len() + handle.len() + anchor.len());
    let mut vertex_to_col = HashMap::new();

    for vert in &region_of_interest {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    for vert in &handle {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    for vert in &anchor {
        vertex_to_col.insert(*vert, vertex_to_col.len());
    }

    let handle_and_anchor: Vec<_> = handle.iter().chain(anchor.iter()).copied().collect();
    let (coeffs, edge_weights) = compute_coeffs(mesh, &handle_and_anchor, &region_of_interest, &vertex_to_col);

    let size = region_of_interest.len() + handle.len() + anchor.len();
    let Ok(a_mat) = sp::SparseColMat::try_new_from_triplets(size, size, &coeffs) else {
        return Err(DeformError::InternalError("failed to create sparse matrix"));
    };

    // for row in a_mat.to_dense().row_iter() {
    //     println!("{:+.2?}", row);
    // }

    let Ok(factorization) = a_mat.sp_lu() else {
        return Err(DeformError::InternalError("failed to factorize matrix"));
    };

    Ok(PreparedDeform {
        factorization,
        handle,
        region_of_interest,
        anchor,
        edge_weights,
        vertex_to_col,
    })
}

// pub fn prepare_deform(
//     mesh: &CornerTableD,
//     handle: &BTreeSet<VertexId>,
//     region_of_interest: &BTreeSet<VertexId>,
//     anchor: &BTreeSet<VertexId>,
// ) -> Result<PreparedDeform, DeformError> {
//     if handle.is_empty() {
//         return Err(DeformError::InvalidHandle);
//     }

//     if region_of_interest.is_empty() {
//         return Err(DeformError::InvalidRegionOfInterest);
//     }

//     // TODO: should we remove vertices from the handle that do not border with roi?
//     if !handle_borders_with_roi(mesh, handle, region_of_interest) {
//         return Err(DeformError::InvalidHandle);
//     }

//     let handle = Vec::from_iter(handle.iter().copied());
//     let region_of_interest = Vec::from_iter(region_of_interest.iter().copied());
//     let anchor = Vec::from_iter(anchor.iter().copied());

//     //let mut vertex_to_col = HashMap::with_capacity(region_of_interest.len() + handle.len() + anchor.len());
//     let mut vertex_to_col = BTreeMap::new();

//     for vert in &region_of_interest {
//         vertex_to_col.insert(*vert, vertex_to_col.len());
//     }

//     for vert in &handle {
//         vertex_to_col.insert(*vert, vertex_to_col.len());
//     }

//     for vert in &anchor {
//         vertex_to_col.insert(*vert, vertex_to_col.len());
//     }

//     let num_distortion_equations = region_of_interest.len() * 3;
//     let num_fitting_equations = (handle.len() + anchor.len()) * 3;
//     let num_equations = num_distortion_equations + num_fitting_equations;

//     let mut coeffs = Vec::with_capacity(num_equations);
//     compute_distortion_term_coeffs(mesh, &region_of_interest, &vertex_to_col, &mut coeffs);
//     compute_fitting_term_coeffs(handle.len() + anchor.len(), num_distortion_equations, &mut coeffs);

//     // Ax = b
//     let Ok(a_mat) = sp::SparseColMat::try_new_from_triplets(num_equations, num_equations, &coeffs) else {
//         return Err(DeformError::InternalError("failed to create sparse matrix"));
//     };

//     for row in a_mat.to_dense().row_iter() {
//         println!("{:+.2?}", row);
//     }

//     let Ok(factorization) = a_mat.sp_qr() else {
//         return Err(DeformError::InternalError("failed to factorize matrix"));
//     };

//     Ok(PreparedDeform {
//         factorization,
//         handle,
//         region_of_interest,
//         anchor,
//     })
// }

#[derive(Debug)]
pub struct PreparedDeform {
    factorization: faer::sparse::linalg::solvers::Lu<usize, f64>,
    handle: Vec<VertexId>,
    region_of_interest: Vec<VertexId>,
    anchor: Vec<VertexId>,
    edge_weights: EdgeAttribute<f64>,
    vertex_to_col: HashMap<VertexId, usize>,
}

impl PreparedDeform {
    pub fn deform(&self, mesh: &mut CornerTableD, transform: na::Matrix4<f64>) {
        let handle_transformed: Vec<_> = self
            .handle
            .iter()
            .map(|vertex| {
                let position = *mesh.vertex_position(vertex);
                transform.transform_point(&position.into()).coords
            })
            .collect();

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

        println!("B = {:+.2?}", b_vec);

        println!("{solution:+.2?}");

        for i in 0..self.region_of_interest.len() {
            let point = Vec3d::new(solution[i * 3], solution[i * 3 + 1], solution[i * 3 + 2]);
            mesh[self.region_of_interest[i]].set_position(point);
        }

        for i in 0..self.handle.len() {
            mesh[self.handle[i]].set_position(handle_transformed[i]);
        }
    }

    pub fn deform_arap(&self, mesh: &CornerTableD, transform: na::Matrix4<f64>) -> CornerTableD {
        let mut prime = mesh.clone();

        let size = self.factorization.nrows();
        let mut b = [
            faer::Col::<f64>::zeros(size), // x
            faer::Col::<f64>::zeros(size), // y
            faer::Col::<f64>::zeros(size), // z
        ];

        let mut rs = vec![na::Matrix3::<f64>::zeros(); size];

        let handle_transformed: Vec<_> = self
            .handle
            .iter()
            .map(|vertex| {
                let position = *mesh.vertex_position(vertex);
                transform.transform_point(&position.into()).coords
            })
            .collect();

        let anchor_positions: Vec<_> = self
            .anchor
            .iter()
            .map(|vertex| *mesh.vertex_position(vertex))
            .collect();

        for _ in 0..20 {
            for (i, vert) in self.vertices().enumerate() {
                let mut s = na::Matrix3::<f64>::zeros();

                mesh.edges_around_vertex(vert, |edge| {
                    let (v1, v2) = mesh.edge_vertices(edge);
                    let other_vert = if vert == &v1 { v2 } else { v1 }; // TODO: ordered edges

                    let vi = mesh.vertex_position(vert);
                    let vj = mesh.vertex_position(&other_vert);

                    let vi_prime = prime.vertex_position(vert); // IDs after clone may not be the same
                    let vj_prime = prime.vertex_position(&other_vert);

                    let e0 = vi - vj;
                    let e1 = vi_prime - vj_prime;
                    let weight = self.edge_weights[*edge];

                    s += weight * (e0 * e1.transpose());
                });

                let svd = s.svd(true, true);
                let u = svd.u.unwrap();
                let v = svd.v_t.unwrap().transpose();
                let det = (v * u.transpose()).determinant();
                let d = na::Vector3::new(1.0, 1.0, det);

                rs[i] = v * na::Matrix3::from_diagonal(&d) * u.transpose();
            }

            for (i, &vi) in self.vertices().enumerate() {
                let mut bi = na::Vector3::zeros();

                if i >= self.region_of_interest.len() + self.handle.len() {
                    bi = anchor_positions[i - self.region_of_interest.len() - self.handle.len()];
                } else if i >= self.region_of_interest.len() {
                    bi = handle_transformed[i - self.region_of_interest.len()];
                } else {
                    mesh.edges_around_vertex(&vi, |&edge| {
                        let weight = self.edge_weights[edge];
                        let (v0, v1) = mesh.edge_vertices(&edge);
                        let vj = if vi == v0 { v1 } else { v0 }; // TODO: ordered edges
                        bi += weight * 0.5 * ((rs[i] + rs[self.vertex_to_col[&vj]]) * (mesh[vi].position() - mesh[vj].position()));
                    });
                }

                b[0][i] = bi.x;
                b[1][i] = bi.y;
                b[2][i] = bi.z;
            }

            for comp in 0..3 {
                let p_prime = self.factorization.solve(&b[comp]);

                for (i, vert) in self.vertices().enumerate() {
                    prime[*vert].position_mut()[comp] = p_prime[i];
                }
            }
        }

        prime
    }

    fn vertices(&self) -> impl Iterator<Item = &VertexId> + '_ {
        self.region_of_interest.iter().chain(self.handle.iter()).chain(self.anchor.iter())
    }
}

/// Remove vertices from the handle that are not significant.
/// If vertex does not border with a vertex in the region of interest it will not compute the distortion term.
fn handle_borders_with_roi(
    mesh: &CornerTableD,
    handle: &BTreeSet<VertexId>,
    roi: &BTreeSet<VertexId>,
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
// 3. Laplacian Mesh Editing for Interaction Learning
//    https://www.ias.informatik.tu-darmstadt.de/publications/Thai_BachThesis_2014.pdf

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

fn tilda_t(lap: Vec3d, t: &na::OMatrix<f64, na::U7, na::Dyn>) -> na::OMatrix<f64, na::U3, na::Dyn> {
    let mut tt = na::OMatrix::<f64, na::U3, na::Dyn>::zeros(t.ncols());

    for col in 0..t.ncols() {
        tt[(0, col)] =   t[(0, col)] * lap.x - t[(3, col)] * lap.y + t[(2, col)] * lap.z;
        tt[(1, col)] =   t[(3, col)] * lap.x + t[(0, col)] * lap.y - t[(1, col)] * lap.z;
        tt[(2, col)] = - t[(2, col)] * lap.x + t[(1, col)] * lap.y - t[(0, col)] * lap.z;
    }

    tt
}

fn laplacian(vertex_and_neighbors: &Vec<Vec3d>) -> Vec3d {
    let mut laplacian_coord = Vec3d::zeros();
    for neighbor in &vertex_and_neighbors[1..] {
        laplacian_coord += vertex_and_neighbors[0] - neighbor;
    }
    laplacian_coord /= (vertex_and_neighbors.len() - 1) as f64;
    laplacian_coord
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
    vertex_to_col: &BTreeMap<VertexId, usize>,
    coeffs: &mut Vec<Triplet>,
) {
    let mut vertex_and_neighbors = Vec::new();
    let mut vertex_and_neighbors_positions = Vec::new();

    // for (i, vert) in roi.into_iter().enumerate() {
    //     let idx = i * 3;
    //     coeffs.push(Triplet::new(idx + 0, idx + 0, 1.0));
    //     coeffs.push(Triplet::new(idx + 1, idx + 1, 1.0));
    //     coeffs.push(Triplet::new(idx + 2, idx + 2, 1.0));

    //     vertex_and_neighbors.clear();
    //     mesh.vertices_around_vertex(vert, |neighbor| {
    //         if !vertex_to_col.contains_key(neighbor) {
    //             return;
    //         }

    //         vertex_and_neighbors.push(*neighbor);
    //     });

    //     mesh.vertices_around_vertex(vert, |neighbor| {
    //         if !vertex_to_col.contains_key(neighbor) {
    //             return;
    //         }

    //         coeffs.push(Triplet::new(idx + 0, vertex_to_col[neighbor] * 3 + 0, -1.0 / vertex_and_neighbors.len() as f64));
    //         coeffs.push(Triplet::new(idx + 1, vertex_to_col[neighbor] * 3 + 1, -1.0 / vertex_and_neighbors.len() as f64));
    //         coeffs.push(Triplet::new(idx + 2, vertex_to_col[neighbor] * 3 + 2, -1.0 / vertex_and_neighbors.len() as f64));
    //     });
    // }

    println!("{:?}", vertex_to_col);

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
        //let t = c.pseudo_inverse(1e-8).unwrap();
        let t = (c.transpose() * &c).try_inverse().unwrap() * c.transpose();
        let lap = laplacian(&vertex_and_neighbors_positions);

        let tt = tilda_t(lap, &t).row_sum();

        let s = t.row(0);
        let hx = t.row(1);
        let hy = t.row(2);
        let hz = t.row(3);

        let d = d_mat(&lap);
        let t_lap = d * t;

        let l = l_mat(vertex_and_neighbors.len() - 1);
        let h = l - t_lap;

        let row = i * 3;
        for j in 0..vertex_and_neighbors.len() {
            let vert = &vertex_and_neighbors[j];
            let j_col = j * 3;
            let col = vertex_to_col[vert] * 3;
            
            // -------------------------------------------

            // coeffs.push(Triplet::new(row, col, -tt[j * 3]));
            // coeffs.push(Triplet::new(row + 1, col + 1, -tt[j * 3 + 1]));
            // coeffs.push(Triplet::new(row + 2, col + 2, -tt[j * 3 + 2]));

            // -------------------------------------------

            // coeffs.push(Triplet::new(row, col, -(s[j_col] * lap.x)));
            // coeffs.push(Triplet::new(row, col, -(-hz[j_col] * lap.y)));
            // coeffs.push(Triplet::new(row, col, -(hy[j_col] * lap.z)));

            
            // coeffs.push(Triplet::new(row + 1, col + 1, -(hz[j_col + 1] * lap.x)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(s[j_col + 1] * lap.y)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(-hx[j_col + 1] * lap.z)));
            

            // coeffs.push(Triplet::new(row + 2, col + 2, -(-hy[j_col + 2] * lap.x)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(hx[j_col + 2] * lap.y)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(s[j_col + 2] * lap.z)));

            // -------------------------------------------

            // coeffs.push(Triplet::new(row, col, -(s[j_col] * lap.x)));
            // coeffs.push(Triplet::new(row, col, -(-hz[j_col] * lap.y)));
            // coeffs.push(Triplet::new(row, col, -(hy[j_col] * lap.z)));
            // coeffs.push(Triplet::new(row, col, -(s[j_col+ 1] * lap.x)));
            // coeffs.push(Triplet::new(row, col, -(-hz[j_col+ 1] * lap.y)));
            // coeffs.push(Triplet::new(row, col, -(hy[j_col+ 1] * lap.z)));
            // coeffs.push(Triplet::new(row, col, -(s[j_col + 2] * lap.x)));
            // coeffs.push(Triplet::new(row, col, -(-hz[j_col + 2] * lap.y)));
            // coeffs.push(Triplet::new(row, col, -(hy[j_col + 2] * lap.z)));

            
            // coeffs.push(Triplet::new(row + 1, col + 1, -(hz[j_col + 0] * lap.x)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(s[j_col + 0] * lap.y)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(-hx[j_col + 0] * lap.z)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(hz[j_col + 1] * lap.x)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(s[j_col + 1] * lap.y)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(-hx[j_col + 1] * lap.z)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(hz[j_col + 2] * lap.x)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(s[j_col + 2] * lap.y)));
            // coeffs.push(Triplet::new(row + 1, col + 1, -(-hx[j_col + 2] * lap.z)));
            

            // coeffs.push(Triplet::new(row + 2, col + 2, -(-hy[j_col + 0] * lap.x)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(hx[j_col + 0] * lap.y)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(s[j_col + 0] * lap.z)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(-hy[j_col + 1] * lap.x)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(hx[j_col + 1] * lap.y)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(s[j_col + 1] * lap.z)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(-hy[j_col + 2] * lap.x)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(hx[j_col + 2] * lap.y)));
            // coeffs.push(Triplet::new(row + 2, col + 2, -(s[j_col + 2] * lap.z)));

            // -------------------------------------------

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

fn compute_edge_weights(mesh: &CornerTableD) -> EdgeAttribute<f64> {
    let mut edge_weights = mesh.create_edge_attribute::<f64>();
    
    for edge in mesh.edges() {
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

        // println!("edge: {:?}, weight: {}", edge, edge_weights[edge]);
    }

    edge_weights
}

fn compute_coeffs(mesh: &CornerTableD, handle: &Vec<VertexId>, roi: &Vec<VertexId>, vertex_to_col: &HashMap<VertexId, usize>) -> (Vec<Triplet>, EdgeAttribute<f64>) {
    let mut coeffs = Vec::with_capacity(handle.len() + roi.len());
    let edge_weights = compute_edge_weights(mesh);

    for (i, vert) in roi.into_iter().enumerate() {
        let mut total_weight = 0.0;
        
        mesh.edges_around_vertex(vert, |edge| {
            let (v1, v2) = mesh.edge_vertices(edge);
            let weight = edge_weights[*edge];
            coeffs.push(Triplet::new(i, vertex_to_col[if vert == &v1 {&v2} else {&v1}], -weight)); // TODO: add ordered edges
            total_weight += weight;
        });

        if total_weight > 0.0 {
            coeffs.push(Triplet::new(i, i, total_weight));
        }
    }

    let mut row = roi.len();
    for vert in handle {
        coeffs.push(Triplet::new(row, vertex_to_col[vert], 1.0));
        row += 1;
    }

    (coeffs, edge_weights)
}
