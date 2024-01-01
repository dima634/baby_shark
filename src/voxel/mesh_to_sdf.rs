use std::path::Path;

use svg::node::Value;

use crate::{geometry::{primitives::triangle3::Triangle3, traits::{HasBBox3, ClosestPoint3}}, helpers::aliases::{Vec3f, Vec3i}, voxel::{Leaf, TreeNode}, mesh::polygon_soup::data_structure::PolygonSoup, io::stl::StlWriter};

use super::{Grid, Scalar, Accessor, Sdf, sdf, meshing::CubesMesher};

const MAX_SUBDIVISIONS: f32 = 1000.0; 

pub struct MeshToSdf<TGrid: Grid<Value = Scalar>> {
    points: Vec<(Triangle3<f32>, Vec3f)>,
    band_width: isize,
    sdf: Box<TGrid>,
    pub voxel_size: f32,
    inverse_voxel_size: f32,
}

impl<TGrid> MeshToSdf<TGrid> where TGrid: Grid<Value = Scalar> {
    pub fn new() -> Self {
        let voxel_size = 1.0;
        Self {
            band_width: 4,
            sdf: TGrid::empty(Vec3i::zeros()),
            points: Vec::new(),
            inverse_voxel_size: 1.0 / voxel_size,
            voxel_size,
        }
    }

    pub fn voxel_size(mut self, size: f32) -> Self {
        self.voxel_size = size;
        self.inverse_voxel_size = 1.0 / size;
        self
    }

    pub fn approximate(&mut self, mesh: impl Iterator<Item = Triangle3<f32>>) -> Box<Sdf<TGrid>> {
        for tri in mesh {
            self.subdivide_triangle(&tri);
        }
        self.save_sub();

        println!("Points: {}", self.points.len());
        self.compute_udfs();
        self.construct_sdf();


        let mut sdf = TGrid::empty(Vec3i::zeros());
        std::mem::swap(&mut sdf, &mut self.sdf);

        Box::new(Sdf { grid: sdf })
    }

    fn save_sub(&self) {
        let writer = StlWriter::new();
        let mesh = PolygonSoup::from_vertices(self.points.iter().map(|(t, _)| [*t.p1(), *t.p2(), *t.p3()]).flatten().collect());
        writer.write_stl_to_file(&mesh, Path::new("subdiv.stl")).expect("Write mesh");
    }

    fn subdivide_triangle(&mut self, tri: &Triangle3<f32>) {
        let n = match tri.try_get_normal() {
            Some(n) => n,
            None => return,
        };

        let num_subs = (tri.max_side() / self.voxel_size).floor(); // TODO !!!!!!!!!!
        // println!("Num subs: {}", num_subs);
        let num_subs = num_subs.min(MAX_SUBDIVISIONS);
        let num_subs_inv = 1.0 / num_subs;

        //    p1
        //    |
        //    | s1
        //    |     s2
        //    p2--------p3

        if num_subs < 2.0 {
            self.points.push((tri.clone(), n));
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
                self.points.push((Triangle3::new(a_prev, b_s, a_s), n));
                self.points.push((Triangle3::new(a_s, b_s, c_s), n));
                a_prev = a_s;
                a_s += s2;
                b_s += s2;
                c_s += s2;
            }

            self.points.push((Triangle3::new(a, b, c), n));
            a += s1;
        }
    
    }

    fn compute_udfs(&mut self) {
        for i in 0..self.points.len() {
            // Compute grid point index based on voxel size
            // 0       1       2
            // 0 --1-- 2 --3-- 4
            // voxel_size = 2
            // 1.1 / 2 => 0.55 => 1
            // 2.1 / 2 => 1.05 => 1
            // 3.0 / 2 => 1.5 => 2
            // 0.1 / 2 => 0.05 => 0
            let (tri, normal) = &self.points[i];
            let point = tri.center();
            let nbh_min = Vec3i::new(
                (point.x * self.inverse_voxel_size).round() as isize - self.band_width, 
                (point.y * self.inverse_voxel_size).round() as isize - self.band_width, 
                (point.z * self.inverse_voxel_size).round() as isize - self.band_width,
            );
            let nbh_max = nbh_min.add_scalar(self.band_width * 2);

            // rintln!("new point");

            for x in nbh_min.x..nbh_max.x {
                for y in nbh_min.y..nbh_max.y {
                    for z in nbh_min.z..nbh_max.z {
                        let idx = Vec3i::new(x, y, z);
                        let grid_point = idx.cast() * self.voxel_size;
                        let closest = tri.closest_point(&grid_point.into());
                        let diff = (closest - grid_point).coords;
                        let sign = -diff.dot(normal);
                        let dist = (closest - grid_point).coords.norm().copysign(sign);

                        debug_assert!(dist.is_finite(), "Distance from grid point to mesh is not finite");

                        let cur_dist = self.sdf.at(&idx).map(|v| v.value).unwrap_or(f32::INFINITY);
                        
                        if dist.abs() < cur_dist.abs() {
                            self.sdf.insert(&idx, dist.into());
                        }
                    }
                }
            }
        }
    }

    fn construct_sdf(&mut self) {
        // Now we will treat internal grid as SDF to avoid constructing new grid
        // self.edf.traverse_leafs(&mut |leaf| match leaf {
        //     Leaf::Tile(tile) => {
        //         let o = tile.origin;
        //         let size = tile.size as isize;
        //         let external = tile.value;

        //         for x in o.x..o.x + size {
        //             for y in o.y..o.y + size {
        //                 for z in o.z..o.z + size {
        //                     let idx = Vec3i::new(x, y, z);
        //                     let internal = self.sdff.at(&idx).cloned().unwrap();
        //                     let sdf = sdf_value(internal.into(), external.into());
        //                     self.sdff.insert(&idx, sdf.into());
        //                 }
        //             }
        //         }
        //     },
        //     Leaf::Dense(node) => {
        //         let o = node.origin();
        //         let size = node.size_t() as isize;

        //         for x in o.x..o.x + size {
        //             for y in o.y..o.y + size {
        //                 for z in o.z..o.z + size {
        //                     let idx = Vec3i::new(x, y, z);
        //                     let external = node.at(&idx).cloned();

        //                     if external.is_none() {
        //                         continue;
        //                     }

        //                     let external = external.unwrap();
        //                     let internal = self.sdff.at(&idx).cloned().unwrap();

        //                     let sdf = sdf_value(internal.into(), external.into());
        //                     self.sdff.insert(&idx, sdf.into());
        //                 }
        //             }
        //         }
        //     }
        // });
    }
}




