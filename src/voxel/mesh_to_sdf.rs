use std::path::Path;

use svg::node::Value;

use crate::{geometry::{primitives::triangle3::Triangle3, traits::HasBBox3}, helpers::aliases::{Vec3f, Vec3i}, voxel::{Leaf, TreeNode}, mesh::polygon_soup::data_structure::PolygonSoup, io::stl::StlWriter};

use super::{Grid, Scalar, Accessor, Sdf, sdf, meshing::CubesMesher};

const SMALL_NUMBER: f32 = 1e-6;
const PHI: f32 = 0.05;
const MAX_SUBDIVISIONS: usize = 1000; 

pub struct MeshToSdf<TGrid: Grid<Value = Scalar>> {
    points: Vec<Vec3f>,
    internal: Vec<Vec3f>,
    external: Vec<Vec3f>,
    band_width: isize,
    idf: Box<TGrid>,
    edf: Box<TGrid>,
    voxel_size: f32,
    inverse_voxel_size: f32,
}

impl<TGrid> MeshToSdf<TGrid> where TGrid: Grid<Value = Scalar> {
    pub fn new() -> Self {
        let voxel_size = 0.005;
        Self {
            band_width: 3,
            edf: Box::new(TGrid::empty(Vec3i::zeros())),
            idf: Box::new(TGrid::empty(Vec3i::zeros())),
            points: Vec::new(),
            internal: Vec::new(),
            external: Vec::new(),
            inverse_voxel_size: 1.0 / voxel_size,
            voxel_size,
        }
    }

    pub fn approximate(&mut self, mesh: impl Iterator<Item = Triangle3<f32>>) -> Box<Sdf<TGrid>> {
        for tri in mesh {
            self.subdivide_triangle(&tri);
        }

        println!("Internal: {}", self.internal.len());
        self.compute_udfs();
        self.construct_sdf();


        let mut sdf = Box::new(TGrid::empty(Vec3i::zeros()));
        std::mem::swap(&mut sdf, &mut self.idf);

        Box::new(Sdf { grid: *sdf })
    }

    fn subdivide_triangle(&mut self, tri: &Triangle3<f32>) {
        let num_subs = 40; // TODO !!!!!!!!!!
        let num_subs = num_subs.min(MAX_SUBDIVISIONS);
        let num_subs_inv = 1.0 / num_subs as f32;

        let s1 = (tri.p2() - tri.p1()) * num_subs_inv;
        let s2 = (tri.p3() - tri.p2()) * num_subs_inv;
        let s3 = (tri.p3() - tri.p1()) * num_subs_inv;

        let a = tri.p1().coords;
        let mut ps = (1.0 / 3.0) * (a + a + s1 + a + s3);
        self.choose_internal_external(tri, ps);

        for i in 2..=num_subs {
            ps = ps + s1;
            self.choose_internal_external(tri, ps);
            
            let mut p = ps;
            for _ in 2..=i {
                p = p + s2;
                self.choose_internal_external(tri, p);
            }
        }
    }

    fn choose_internal_external(&mut self, tri: &Triangle3<f32>, p: Vec3f) {
        let n = tri.get_normal();
        
        let internal = p - n * PHI;
        self.internal.push(internal);

        let external = p + n * PHI;
        self.external.push(external);

        self.points.push(p);
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
            let point = &self.points[i];
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
                        let internal_distance = (grid_point - self.internal[i]).magnitude();
                        let external_distance = (grid_point - self.external[i]).magnitude();

                        // println!("{} {}", internal_distance, external_distance);
                        // std::io::stdin().read_line(&mut String::new()).unwrap();

                        let idf_distance = self.idf.at(&idx).map(|v| v.value).unwrap_or(f32::INFINITY);
                        if internal_distance < idf_distance {
                            self.idf.insert(&idx, internal_distance.into());
                        }
                        
                        let edf_distance = self.edf.at(&idx).map(|v| v.value).unwrap_or(f32::INFINITY);
                        if external_distance < edf_distance {
                            self.edf.insert(&idx, external_distance.into());
                        }
                    }
                }
            }
        }
    }

    fn construct_sdf(&mut self) {
        // Now we will treat internal grid as SDF to avoid constructing new grid
        self.edf.traverse_leafs(&mut |leaf| match leaf {
            Leaf::Tile(tile) => {
                let o = tile.origin;
                let size = tile.size as isize;
                let external = tile.value;

                for x in o.x..o.x + size {
                    for y in o.y..o.y + size {
                        for z in o.z..o.z + size {
                            let idx = Vec3i::new(x, y, z);
                            let internal = self.idf.at(&idx).cloned().unwrap();
                            let sdf = sdf_value(internal.into(), external.into());
                            self.idf.insert(&idx, sdf.into());
                        }
                    }
                }
            },
            Leaf::Dense(node) => {
                let o = node.origin();
                let size = node.size_t() as isize;

                for x in o.x..o.x + size {
                    for y in o.y..o.y + size {
                        for z in o.z..o.z + size {
                            let idx = Vec3i::new(x, y, z);
                            let external = node.at(&idx).cloned();

                            if external.is_none() {
                                continue;
                            }

                            let external = external.unwrap();
                            let internal = self.idf.at(&idx).cloned().unwrap();

                            let sdf = sdf_value(internal.into(), external.into());
                            self.idf.insert(&idx, sdf.into());
                        }
                    }
                }
            }
        });
    }
}

fn sdf_value(internal: f32, external: f32) -> f32 {
    if internal <= external {
        0f32.min(-(external - PHI))
    } else {
        0f32.max(internal - PHI)
    }
}



