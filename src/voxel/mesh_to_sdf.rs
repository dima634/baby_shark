use std::sync::Mutex;

use rayon::prelude::*;

use crate::{
    geometry::{
        primitives::{box3::Box3, triangle3::Triangle3},
        traits::{ClosestPoint3, HasBBox3},
    },
    helpers::aliases::{Vec3f, Vec3i, Vec3u},
    mesh::traits::Mesh,
    spatial_partitioning::aabb_tree::winding_numbers::WindingNumbers,
    voxel::{ParVisitor, Tile, TreeNode, Visitor},
};

use super::{sdf::{Sdf, SdfGrid}, Grid};

pub struct MeshToSdf {
    band_width: isize,
    voxel_size: f32,
    inverse_voxel_size: f32,
    distance_field: Box<SdfGrid>,
    subdivided_mesh: Vec<Triangle3<f32>>,
    winding_numbers: WindingNumbers,
}

impl MeshToSdf {
    #[inline]
    pub fn with_narrow_band_width(mut self, width: isize) -> Self {
        self.set_narrow_band_width(width);
        self
    }

    #[inline]
    pub fn set_narrow_band_width(&mut self, width: isize) -> &mut Self {
        self.band_width = width;
        self
    }

    #[inline]
    pub fn with_voxel_size(mut self, size: f32) -> Self {
        self.set_voxel_size(size);
        self
    }

    #[inline]
    pub fn set_voxel_size(&mut self, size: f32) -> *mut Self {
        self.voxel_size = size;
        self.inverse_voxel_size = 1.0 / size;
        self
    }

    pub fn convert<T: Mesh<ScalarType = f32>>(&mut self, mesh: &T) -> Option<Sdf> {
        if mesh.faces().count() == 0 {
            return None;
        }

        self.clear();
        for tri in mesh.faces().map(|f| mesh.face_positions(&f)) {
            self.subdivide_triangle(&tri);
        }

        self.winding_numbers = WindingNumbers::from_mesh(mesh);
        self.compute_unsigned_distance_field();

        if !self.compute_sings() {
            return None;
        }

        let mut sdf = SdfGrid::empty(Vec3i::zeros());
        std::mem::swap(&mut sdf, &mut self.distance_field);

        Some(sdf.into())
    }

    fn subdivide_triangle(&mut self, tri: &Triangle3<f32>) {
        let num_subs = (tri.max_side() / self.voxel_size).floor();
        let num_subs_inv = 1.0 / num_subs;

        //    p1
        //    |
        //    | s1
        //    |     s2
        //    p2--------p3

        if num_subs < 2.0 {
            self.subdivided_mesh.push(tri.clone());
            return;
        }

        let s1 = (tri.p2() - tri.p1()) * num_subs_inv;
        let s2 = (tri.p3() - tri.p2()) * num_subs_inv;

        let mut a = *tri.p1();

        for i in 0..num_subs as usize {
            let b = a + s1;
            let c = b + s2;

            let mut a_s = a + s2;
            let mut b_s = b + s2;
            let mut c_s = c + s2;
            let mut a_prev = a;

            for _ in 0..i {
                self.subdivided_mesh.push(Triangle3::new(a_prev, b_s, a_s));
                self.subdivided_mesh.push(Triangle3::new(a_s, b_s, c_s));
                a_prev = a_s;
                a_s += s2;
                b_s += s2;
                c_s += s2;
            }

            self.subdivided_mesh.push(Triangle3::new(a, b, c));
            a += s1;
        }
    }

    fn compute_unsigned_distance_field(&mut self) {
        let neighbors: Vec<_> = self
            .subdivided_mesh
            .par_iter()
            .map(|tri| {
                // Compute distance for voxels intersecting triangle and its `band_width` neighborhood
                let bbox = tri.bbox();
                let mut min = Vec3i::new(
                    (bbox.get_min().x * self.inverse_voxel_size).floor() as isize - self.band_width,
                    (bbox.get_min().y * self.inverse_voxel_size).floor() as isize - self.band_width,
                    (bbox.get_min().z * self.inverse_voxel_size).floor() as isize - self.band_width,
                );
                let mut max = Vec3i::new(
                    (bbox.get_max().x * self.inverse_voxel_size).ceil() as isize + self.band_width,
                    (bbox.get_max().y * self.inverse_voxel_size).ceil() as isize + self.band_width,
                    (bbox.get_max().z * self.inverse_voxel_size).ceil() as isize + self.band_width,
                );

                // Triangle intersecting voxel along the voxel side?
                if max.x == min.x || max.y == min.y || max.z == min.z {
                    // Extend box so it is not 0-volume
                    min.add_scalar_mut(-1);
                    max.add_scalar_mut(1);
                }

                let neighbors_box = Box3::new(min, max);

                let mut distances = Vec::with_capacity(neighbors_box.volume() as usize);

                for x in min.x..=max.x {
                    let x_world = x as f32 * self.voxel_size;
                    for y in min.y..=max.y {
                        let y_world = y as f32 * self.voxel_size;
                        for z in min.z..=max.z {
                            let z_world = z as f32 * self.voxel_size;
                            let grid_point = Vec3f::new(x_world, y_world, z_world);
                            let closest = tri.closest_point(&grid_point.into());
                            let dist = (closest - grid_point).norm();
                            distances.push(dist);

                            debug_assert!(
                                dist.is_finite(),
                                "Mesh to SDF: distance from grid point to mesh is not finite"
                            );
                        }
                    }
                }

                (neighbors_box, distances)
            })
            .collect();

        for (bbox, dist) in neighbors {
            let min = bbox.get_min();
            let max = bbox.get_max();

            let mut i = 0;

            for x in min.x..=max.x {
                for y in min.y..=max.y {
                    for z in min.z..=max.z {
                        let idx = Vec3i::new(x, y, z);

                        let cur_dist = self
                            .distance_field
                            .at(&idx)
                            .map(|v| *v)
                            .unwrap_or(f32::INFINITY);

                        if dist[i] < cur_dist {
                            self.distance_field.insert(&idx, dist[i].into());
                        }

                        i += 1;
                    }
                }
            }
        }
    }

    fn compute_sings(&mut self) -> bool {
        let signs = Mutex::new(SdfGrid::empty(Vec3i::zeros()));
        let mut visitor = ComputeSignsVisitor {
            distance_field: signs,
            winding_numbers: &self.winding_numbers,
            voxel_size: self.voxel_size,
        };

        self.distance_field.visit_leafs_par(&mut visitor);

        match visitor.distance_field.into_inner() {
            Ok(df) => {
                self.distance_field = df;
                true
            }
            Err(_) => false,
        }
    }

    fn clear(&mut self) {
        self.subdivided_mesh.clear();
        self.distance_field.clear();
    }
}

impl Default for MeshToSdf {
    #[inline]
    fn default() -> Self {
        let voxel_size = 1.0;
        Self {
            voxel_size,
            band_width: 1,
            distance_field: SdfGrid::empty(Vec3i::zeros()),
            subdivided_mesh: Vec::new(),
            inverse_voxel_size: 1.0 / voxel_size,
            winding_numbers: WindingNumbers::from_triangles(vec![]),
        }
    }
}

struct ComputeSignsVisitor<'a, TGrid: Grid<Value = f32>> {
    distance_field: Mutex<Box<TGrid>>,
    winding_numbers: &'a WindingNumbers,
    voxel_size: f32,
}

impl<'a, TGrid: Grid<Value = f32>> ComputeSignsVisitor<'a, TGrid> {
    fn compute_sings_in_node(&self, node: &TGrid::Leaf) {
        if self.distance_field.is_poisoned() {
            return;
        }

        let origin = node.origin();
        let size = TGrid::Leaf::resolution();
        let max = origin + Vec3u::new(size, size, size).cast();

        for x in origin.x..max.x {
            for y in origin.y..max.y {
                for z in origin.z..max.z {
                    let idx = Vec3i::new(x, y, z);
                    let grid_point = idx.cast() * self.voxel_size;

                    let mut dist = match node.at(&idx) {
                        Some(v) => *v,
                        None => continue,
                    };
                    let wn = self.winding_numbers.approximate(&grid_point, 2.0);

                    if wn < 0.2 {
                        // Outside, threshold value picked experimentally
                        dist = dist.copysign(1.0);
                    } else {
                        dist = dist.copysign(-1.0);
                    }

                    match self.distance_field.lock() {
                        Ok(mut df) => df.insert(&idx, dist.into()),
                        Err(_) => return,
                    }
                }
            }
        }
    }
}

impl<'a, TGrid: Grid<Value = f32>> ParVisitor<TGrid::Leaf> for ComputeSignsVisitor<'a, TGrid> {
    fn tile(&self, _tile: Tile<TGrid::Value>) {
        debug_assert!(false, "Mesh to SDF: tile encountered. This is not possible because we are not pruning the tree.");
    }

    #[inline]
    fn dense(&self, n: &TGrid::Leaf) {
        self.compute_sings_in_node(n);
    }
}

impl<'a, TGrid: Grid<Value = f32>> Visitor<TGrid::Leaf> for ComputeSignsVisitor<'a, TGrid> {
    fn tile(&mut self, _tile: Tile<TGrid::Value>) {
        debug_assert!(false, "Mesh to SDF: tile encountered. This is not possible because we are not pruning the tree.");
    }

    fn dense(&mut self, n: &TGrid::Leaf) {
        self.compute_sings_in_node(n);
    }
}
