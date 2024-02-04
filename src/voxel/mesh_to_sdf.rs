use std::{path::Path, sync::Mutex};

use crate::{geometry::{primitives::triangle3::Triangle3, traits::{HasBBox3, ClosestPoint3}}, helpers::aliases::{Vec3f, Vec3i, Vec3u}, io::stl::StlWriter, mesh::{polygon_soup::data_structure::PolygonSoup, traits::Mesh}, spatial_partitioning::aabb_tree::winding_numbers::WindingNumbers, voxel::{Leaf, ParVisitor, Tile, TreeNode, Visitor}};

use super::{Accessor, Grid, Scalar, Sdf, SdfGrid};

pub struct MeshToSdf {
    points: Vec<(Triangle3<f32>, Vec3f)>,
    band_width: isize,
    sdf: Box<SdfGrid>,
    pub voxel_size: f32,
    inverse_voxel_size: f32,
    winding_numbers: WindingNumbers,
}

impl MeshToSdf {
    pub fn new() -> Self {
        let voxel_size = 1.0;
        Self {
            band_width: 1,
            sdf: SdfGrid::empty(Vec3i::zeros()),
            points: Vec::new(),
            inverse_voxel_size: 1.0 / voxel_size,
            voxel_size,
            winding_numbers: WindingNumbers::from_triangles(vec![]),
        }
    }

    pub fn voxel_size(mut self, size: f32) -> Self {
        self.voxel_size = size;
        self.inverse_voxel_size = 1.0 / size;
        self
    }

    pub fn approximate<T: Mesh<ScalarType = f32>>(&mut self, mesh: &T) -> Sdf {
        for tri in mesh.faces().map(|f| mesh.face_positions(&f)){
            self.subdivide_triangle(&tri);
        }

        // self.save_sub();
        
        self.winding_numbers = WindingNumbers::from_mesh(mesh);

        self.compute_udfs();
        self.compute_sings();


        let mut sdf = SdfGrid::empty(Vec3i::zeros());
        std::mem::swap(&mut sdf, &mut self.sdf);

        sdf.into()
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
        // let num_subs = num_subs.min(MAX_SUBDIVISIONS);
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
            // println!("{} / {}", i, self.points.len());

            // Compute distance for voxels intersecting triangle and its `band_width` neighborhood
            let (tri, _) = &self.points[i];

            let bbox = tri.bbox();
            let nbh_min = Vec3i::new(
                (bbox.get_min().x * self.inverse_voxel_size).floor() as isize - self.band_width, 
                (bbox.get_min().y * self.inverse_voxel_size).floor() as isize - self.band_width, 
                (bbox.get_min().z * self.inverse_voxel_size).floor() as isize - self.band_width,
            );
            let nbh_max = Vec3i::new(
                (bbox.get_max().x * self.inverse_voxel_size).ceil() as isize + self.band_width, 
                (bbox.get_max().y * self.inverse_voxel_size).ceil() as isize + self.band_width, 
                (bbox.get_max().z * self.inverse_voxel_size).ceil() as isize + self.band_width,
            );

            for x in nbh_min.x..=nbh_max.x {
                for y in nbh_min.y..=nbh_max.y {
                    for z in nbh_min.z..=nbh_max.z {
                        let idx = Vec3i::new(x, y, z);
                        let grid_point = idx.cast() * self.voxel_size;
                        let closest = tri.closest_point(&grid_point.into());
                        let cur_dist = self.sdf.at(&idx).map(|v| v.value).unwrap_or(f32::INFINITY);

                        let dist = (closest - grid_point).norm();//.copysign(sign);

                        if dist.abs() < cur_dist.abs() {
                            self.sdf.insert(&idx, dist.into());
                        }

                        debug_assert!(dist.is_finite(), "Mesh to SDF: distance from grid point to mesh is not finite");
                    }
                }
            }
        }
    }

    fn compute_sings(&mut self) {
        println!("Computing signs...");

        let now = std::time::Instant::now();

        let signs = Mutex::new(SdfGrid::empty(Vec3i::zeros()));

        let mut visitor = ComputeSigns {
            signs: signs,
            winding_numbers: &self.winding_numbers,
            voxel_size: self.voxel_size,
            count: 0,
        };

        self.sdf.visit_leafs_par(&mut visitor);

        println!("Voxels {}", visitor.count);
        println!("Signs computed in {} ms", now.elapsed().as_millis());

        self.sdf = visitor.signs.into_inner().unwrap();
    }
}

struct ComputeSigns<'a, TGrid: Grid<Value = Scalar>> {
    signs: Mutex<Box<TGrid>>,
    winding_numbers: &'a WindingNumbers,
    voxel_size: f32,
    count: usize,
}

impl<'a, TGrid: Grid<Value = Scalar>> ParVisitor<TGrid::Leaf> for ComputeSigns<'a, TGrid> {
    fn tile(&self, _tile: Tile<TGrid::Value>) {
        debug_assert!(false, "Mesh to SDF: tile encountered. This is not possible because we are not pruning the tree.");
    }

    fn dense(&self, n: &TGrid::Leaf) {
        let origin = n.origin();
        let size = n.size_t();
        let max = origin + Vec3u::new(size, size, size).cast();
        for x in origin.x..max.x {
            for y in origin.y..max.y {
                for z in origin.z..max.z {
                    let idx = Vec3i::new(x, y, z);
                    let grid_point = idx.cast() * self.voxel_size;

                    let mut dist = match n.at(&idx) {
                        Some(v) => v.value,
                        None => continue,
                    };
                    let wn = self.winding_numbers.approximate(&grid_point, 2.0);

                    if wn < 0.05 {
                        dist = dist.copysign(1.0);
                    } else {
                        dist = dist.copysign(-1.0);
                    }

                    self.signs.lock().unwrap().insert(&idx, dist.into());
                }
            }
        }
    }
}

impl<'a, TGrid: Grid<Value = Scalar>> Visitor<TGrid::Leaf> for ComputeSigns<'a, TGrid> {
    fn tile(&mut self, _tile: Tile<TGrid::Value>) {
        debug_assert!(false, "Mesh to SDF: tile encountered. This is not possible because we are not pruning the tree.");
    }

    fn dense(&mut self, n: &TGrid::Leaf) {
        let origin = n.origin();
        let size = n.size_t();
        let max = origin + Vec3u::new(size, size, size).cast();
        for x in origin.x..max.x {
            for y in origin.y..max.y {
                for z in origin.z..max.z {
                    let idx = Vec3i::new(x, y, z);
                    let grid_point = idx.cast() * self.voxel_size;

                    let mut dist = match n.at(&idx) {
                        Some(v) => v.value,
                        None => continue,
                    };
                    self.count+=1;
                
                    let wn = self.winding_numbers.approximate(&grid_point, 2.0);

                    if wn < 0.05 {
                        dist = dist.copysign(1.0);
                    } else {
                        dist = dist.copysign(-1.0);
                    }

                    self.signs.lock().unwrap().insert(&idx, dist.into());
                }
            }
        }
    }
}
