use self::{
    utils::CUBE_OFFSETS,
    volume::{Volume, VolumeGrid},
};
use super::lookup_table::EdgeDir;
use crate::{geometry::primitives::triangle3::Triangle3, helpers::aliases::Vec3f, voxel::*};
use std::sync::Mutex;

///
/// https://www.cs.rice.edu/~jwarren/papers/dualcontour.pdf
///
pub struct DualContouringMesher {
    voxel_size: f32,
}

impl DualContouringMesher {
    #[inline]
    pub fn with_voxel_size(mut self, voxel_size: f32) -> Self {
        self.voxel_size = voxel_size;
        self
    }

    pub fn mesh(&mut self, volume: &Volume) -> Option<Vec<Vec3f>> {
        let grid = volume.grid();

        let compute_intersections = ComputeEdgeIntersectionsVisitor {
            grid,
            x_int: Mutex::new(<VolumeGrid as TreeNode>::As::<IntPoint>::empty(
                Vec3i::zeros(),
            )),
            y_int: Mutex::new(<VolumeGrid as TreeNode>::As::<IntPoint>::empty(
                Vec3i::zeros(),
            )),
            z_int: Mutex::new(<VolumeGrid as TreeNode>::As::<IntPoint>::empty(
                Vec3i::zeros(),
            )),
        };
        grid.visit_leafs_par(&compute_intersections);

        let x_int = compute_intersections.x_int.into_inner().ok()?;
        let y_int = compute_intersections.y_int.into_inner().ok()?;
        let z_int = compute_intersections.z_int.into_inner().ok()?;

        let compute_cell_points = ComputeCellPointsVisitor {
            cells: Mutex::new(<VolumeGrid as TreeNode>::As::<Vec3f>::empty(Vec3i::zeros())),
            x_int: x_int.as_ref(),
            y_int: y_int.as_ref(),
            z_int: z_int.as_ref(),
            grid,
        };
        grid.visit_leafs_par(&compute_cell_points);

        let cells = compute_cell_points.cells.into_inner().ok()?;

        let triangulate = TriangulateVisitor {
            grid,
            cells,
            faces: Mutex::new(Vec::new()),
        };
        grid.visit_leafs_par(&triangulate);

        if let Ok(vertices) = triangulate.faces.into_inner() {
            let mut triangles = Vec::with_capacity(vertices.len());

            for i in (0..vertices.len()).step_by(3) {
                let v0 = vertices[i] * self.voxel_size;
                let v1 = vertices[i + 1] * self.voxel_size;
                let v2 = vertices[i + 2] * self.voxel_size;

                if Triangle3::is_degenerate(&v0, &v1, &v2) {
                    continue;
                }

                triangles.push(v0);
                triangles.push(v1);
                triangles.push(v2);
            }

            return Some(triangles);
        }

        None
    }
}

impl Default for DualContouringMesher {
    #[inline]
    fn default() -> Self {
        Self { voxel_size: 1.0 }
    }
}

struct TriangulateVisitor<'a, T: TreeNode<Value = f32>> {
    faces: Mutex<Vec<Vec3f>>,
    grid: &'a T,
    cells: Box<T::As<Vec3f>>,
}

impl<'a, T: TreeNode<Value = f32>> TriangulateVisitor<'a, T> {
    fn handle_edge(&self, v1_val: f32, v1: &Vec3i, dir: EdgeDir, faces: &mut Vec<Vec3f>) {
        let v2 = match dir {
            EdgeDir::X => Vec3i::new(v1.x + 1, v1.y, v1.z),
            EdgeDir::Y => Vec3i::new(v1.x, v1.y + 1, v1.z),
            EdgeDir::Z => Vec3i::new(v1.x, v1.y, v1.z + 1),
        };

        let v2_val = match self.grid.at(&v2) {
            Some(v) => *v,
            None => return,
        };

        if v1_val.sign() == v2_val.sign() {
            return;
        }

        let offsets = match dir {
            EdgeDir::X => CELL_OFFSETS[0],
            EdgeDir::Y => CELL_OFFSETS[1],
            EdgeDir::Z => CELL_OFFSETS[2],
        };

        let cells = offsets.map(|offset| self.cells.at(&(v1 + offset)).copied());
        let mut new_faces = match cells {
            [Some(p0), Some(p1), Some(p2), Some(p3)] => [p0, p1, p2, p2, p3, p0],
            _ => return,
        };

        if v1_val.sign() == Sign::Negative {
            new_faces.swap(1, 2);
            new_faces.swap(4, 5);
        }

        faces.extend(new_faces);
    }
}

impl<'a, T: TreeNode<Value = f32>> ParVisitor<T::Leaf> for TriangulateVisitor<'a, T> {
    fn tile(&self, _tile: Tile<<T as TreeNode>::Value>) {
        todo!("Dual contouring, tile support")
    }

    fn dense(&self, dense: &T::Leaf) {
        if self.faces.is_poisoned() {
            return;
        }

        let min = dense.origin();
        let size = T::Leaf::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);

        let mut faces = Vec::with_capacity(T::Leaf::resolution());

        for x in min.x..max.x {
            for y in min.y..max.y {
                for z in min.z..max.z {
                    let v = Vec3i::new(x, y, z);

                    let v_val = match dense.at(&v) {
                        Some(v) => *v,
                        None => continue,
                    };

                    self.handle_edge(v_val, &v, EdgeDir::X, &mut faces);
                    self.handle_edge(v_val, &v, EdgeDir::Y, &mut faces);
                    self.handle_edge(v_val, &v, EdgeDir::Z, &mut faces);
                }
            }
        }

        if let Ok(mut f) = self.faces.lock() {
            f.extend(faces);
        };
    }
}

struct ComputeCellPointsVisitor<'a, T: TreeNode<Value = f32>> {
    cells: Mutex<Box<T::As<Vec3f>>>,
    x_int: &'a T::As<IntPoint>,
    y_int: &'a T::As<IntPoint>,
    z_int: &'a T::As<IntPoint>,
    grid: &'a T,
}

impl<'a, T: TreeNode<Value = f32>> ParVisitor<T::Leaf> for ComputeCellPointsVisitor<'a, T> {
    fn tile(&self, _tile: Tile<T::Value>) {
        todo!("Dual contouring, tile support")
    }

    fn dense(&self, dense: &T::Leaf) {
        if self.cells.is_poisoned() {
            return;
        }

        let min = dense.origin();
        let size = T::Leaf::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);
        let mut values = [0.0; 8];
        let mut intersections = Vec::new();

        for x in min.x..max.x {
            for y in min.y..max.y {
                'next_z: for z in min.z..max.z {
                    let o = Vec3i::new(x, y, z);

                    for (i, offset) in CUBE_OFFSETS.iter().enumerate() {
                        let p = o + offset;
                        values[i] = match self.grid.at(&p) {
                            // This can be dense
                            Some(v) => *v,
                            None => continue 'next_z,
                        };
                    }

                    let first_sign = values[0].sign();
                    let all_has_same_sign = values.iter().skip(1).all(|v| v.sign() == first_sign);

                    if all_has_same_sign {
                        continue;
                    }

                    intersections.clear();

                    for (offset, dir) in &EDGE_OFFSETS {
                        let p = o + offset;
                        let point = match dir {
                            EdgeDir::X => self.x_int.at(&p),
                            EdgeDir::Y => self.y_int.at(&p),
                            EdgeDir::Z => self.z_int.at(&p),
                        };

                        if let Some(point) = point {
                            intersections.push(*point);
                        }
                    }

                    match self.cells.lock() {
                        Ok(mut cells) => {
                            let feature_point = find_feature_point(&intersections);
                            cells.insert(&o, feature_point);
                        }
                        Err(_) => return,
                    };
                }
            }
        }
    }
}

struct ComputeEdgeIntersectionsVisitor<'a, T: TreeNode<Value = f32>> {
    grid: &'a T,
    x_int: Mutex<Box<T::As<IntPoint>>>,
    y_int: Mutex<Box<T::As<IntPoint>>>,
    z_int: Mutex<Box<T::As<IntPoint>>>,
}

impl<'a, T: TreeNode<Value = f32>> ComputeEdgeIntersectionsVisitor<'a, T> {
    fn intersection(&self, v1: Vec3i, dir: EdgeDir, intersections: &mut Vec<(Vec3i, IntPoint)>) {
        let v2 = match dir {
            EdgeDir::X => Vec3i::new(v1.x + 1, v1.y, v1.z),
            EdgeDir::Y => Vec3i::new(v1.x, v1.y + 1, v1.z),
            EdgeDir::Z => Vec3i::new(v1.x, v1.y, v1.z + 1),
        };

        let (v1_val, v2_val) = match (self.grid.at(&v1), self.grid.at(&v2)) {
            (Some(v1), Some(v2)) => (*v1, *v2),
            _ => return,
        };

        if v1_val.sign() == v2_val.sign() {
            return;
        }

        let t = if v1_val == v2_val {
            0.5
        } else {
            v1_val / (v1_val - v2_val)
        };

        debug_assert!(
            t.is_finite(),
            "Dual contouring, invalid interpolation: v1 = {}, v2 = {}",
            v1_val,
            v2_val
        );

        let point = match dir {
            EdgeDir::X => Vec3f::new(v1.x as f32 + t, v1.y as f32, v1.z as f32),
            EdgeDir::Y => Vec3f::new(v1.x as f32, v1.y as f32 + t, v1.z as f32),
            EdgeDir::Z => Vec3f::new(v1.x as f32, v1.y as f32, v1.z as f32 + t),
        };
        let normal = self.normal(&v1, &v2, t);

        let intersection = IntPoint { point, normal };

        intersections.push((v1, intersection));
    }

    fn normal(&self, v1: &Vec3i, v2: &Vec3i, t: f32) -> Vec3f {
        let x = (1.0 - t) * self.x_grad(v1) + t * self.x_grad(v2);
        let y = (1.0 - t) * self.y_grad(v1) + t * self.y_grad(v2);
        let z = (1.0 - t) * self.z_grad(v1) + t * self.z_grad(v2);

        let normal = Vec3f::new(x, y, z);
        let unit = normal.normalize();

        debug_assert!(unit.iter().all(|v| v.is_finite()));

        unit
    }

    #[inline]
    fn x_grad(&self, p: &Vec3i) -> f32 {
        self.grad(p, 0)
    }

    #[inline]
    fn y_grad(&self, p: &Vec3i) -> f32 {
        self.grad(p, 1)
    }

    #[inline]
    fn z_grad(&self, p: &Vec3i) -> f32 {
        self.grad(p, 2)
    }

    fn grad(&self, p: &Vec3i, axis: usize) -> f32 {
        let mut pl = *p;
        pl[axis] -= 1;
        let mut pr = *p;
        pr[axis] += 1;

        let v_p = self.grid.at(p);
        let v_pl = self.grid.at(&pl);
        let v_pr = self.grid.at(&pr);

        match (v_p, v_pl, v_pr) {
            (Some(_), Some(v_pl), Some(v_pr)) => (v_pr - v_pl) * 0.5,
            (Some(v_p), _, Some(v_pr)) => v_pr - v_p,
            (Some(v_p), Some(v_pl), _) => v_p - v_pl,
            _ => unreachable!(),
        }
    }
}

impl<'a, T: TreeNode<Value = f32>> ParVisitor<T::Leaf> for ComputeEdgeIntersectionsVisitor<'a, T> {
    fn tile(&self, _tile: Tile<T::Value>) {
        todo!("Dual contouring, tile support")
    }

    fn dense(&self, dense: &T::Leaf) {
        let min = dense.origin();
        let size = T::Leaf::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);

        let mut x_inters = Vec::new();
        let mut y_inters = Vec::new();
        let mut z_inters = Vec::new();

        for x in min.x..max.x {
            for y in min.y..max.y {
                for z in min.z..max.z {
                    let v = Vec3i::new(x, y, z);

                    self.intersection(v, EdgeDir::X, &mut x_inters);
                    self.intersection(v, EdgeDir::Y, &mut y_inters);
                    self.intersection(v, EdgeDir::Z, &mut z_inters);
                }
            }
        }

        let intersection_grids = (self.x_int.lock(), self.y_int.lock(), self.z_int.lock());

        if let (Ok(mut x), Ok(mut y), Ok(mut z)) = intersection_grids {
            for (idx, point) in x_inters {
                x.insert(&idx, point);
            }

            for (idx, point) in y_inters {
                y.insert(&idx, point);
            }

            for (idx, point) in z_inters {
                z.insert(&idx, point);
            }
        };
    }
}

const CELL_OFFSETS: [[Vec3i; 4]; 3] = [
    [
        Vec3i::new(0, 0, 0),
        Vec3i::new(0, 0, -1),
        Vec3i::new(0, -1, -1),
        Vec3i::new(0, -1, 0),
    ], // X
    [
        Vec3i::new(0, 0, 0),
        Vec3i::new(-1, 0, 0),
        Vec3i::new(-1, 0, -1),
        Vec3i::new(0, 0, -1),
    ], // Y
    [
        Vec3i::new(0, -1, 0),
        Vec3i::new(-1, -1, 0),
        Vec3i::new(-1, 0, 0),
        Vec3i::new(0, 0, 0),
    ], // Z
];

const EDGE_OFFSETS: [(Vec3i, EdgeDir); 12] = [
    (Vec3i::new(0, 0, 0), EdgeDir::X),
    (Vec3i::new(0, 0, 0), EdgeDir::Y),
    (Vec3i::new(0, 0, 0), EdgeDir::Z),
    (Vec3i::new(1, 0, 0), EdgeDir::Y),
    (Vec3i::new(1, 0, 0), EdgeDir::Z),
    (Vec3i::new(0, 0, 1), EdgeDir::X),
    (Vec3i::new(0, 0, 1), EdgeDir::Y),
    (Vec3i::new(0, 1, 0), EdgeDir::X),
    (Vec3i::new(0, 1, 0), EdgeDir::Z),
    (Vec3i::new(1, 0, 1), EdgeDir::Y),
    (Vec3i::new(0, 1, 1), EdgeDir::X),
    (Vec3i::new(1, 1, 0), EdgeDir::Z),
];

///
/// "Analysis and Acceleration of High Quality Isosurface Contouring"
/// https://lume.ufrgs.br/bitstream/handle/10183/151064/001010389.pdf?sequence=1
///
fn find_feature_point(points: &Vec<IntPoint>) -> Vec3f {
    let threshold = 1e-6;
    let max_particle_iterations = 50;

    // start mass point
    // calculated by mean of intersection points
    let mut c = points.iter().fold(Vec3f::zeros(), |acc, p| acc + p.point) / points.len() as f32;

    for i in 0..max_particle_iterations {
        // force that acts on mass
        let mut force = Vec3f::zeros();

        for intersection_point in points {
            let p = intersection_point.point;
            let n = intersection_point.normal;

            force += n * -1.0 * n.dot(&(c - p));
        }

        // dampen force
        let damping = 1.0 - i as f32 / max_particle_iterations as f32;
        c += force * damping / points.len() as f32;

        if force.norm_squared() < threshold {
            break;
        }
    }

    c
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
struct IntPoint {
    point: Vec3f,
    normal: Vec3f,
}

impl Sub for IntPoint {
    type Output = IntPoint;

    fn sub(self, _rhs: Self) -> Self::Output {
        unimplemented!()
    }
}
impl Value for IntPoint {}
