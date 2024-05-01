use std::{
    collections::{BTreeSet, BinaryHeap},
    fmt::Debug,
    marker::PhantomData,
    ops::{Index, IndexMut, Range},
};

use fast_sweep::utils::{option_min, partial_min};
use nalgebra::coordinates::X;

use self::{value::empty::Empty, visitors::select_sign_change_cubes::SelectSignChangeCubes};
use super::*;
use crate::helpers::{aliases::Vec3i, utils::sort3};

pub struct FastSweeping;

impl FastSweeping {
    pub fn fast_sweeping<T: TreeNode<Value = f32> + FloodFill + Csg>(
        &self,
        grid: &mut T,
        h: f32,
        limit: f32,
    ) {
        grid.flood_fill(); // TODO: remove?
        let fixed = grid.clone_map(&|_| Empty);

        sweep::<T, Origin_X_Y_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_Y_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_NY_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_NY_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_Y_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_Y_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_NY_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_NY_NZ>(grid, h, limit, &fixed);
    }
}

fn sweep<TTree: TreeNode<Value = f32> + FloodFill + Csg, TLeafOrigin: LeafOrigin>(
    grid: &mut TTree,
    h: f32,
    limit: f32,
    fixed: &TTree::As<Empty>,
) {
    let sweep_direction = TLeafOrigin::sweep_dir();
    println!("Direction: {:?}", sweep_direction);

    let size = TTree::Leaf::resolution() as isize;
    let mut collect_nodes = CollectLeafIndicesVisitor::<TTree, TLeafOrigin> {
        indices: BinaryHeap::new(),
        _tree: PhantomData,
    };
    grid.visit_leafs(&mut collect_nodes);
    let mut nodes = collect_nodes.indices;

    while let Some(origin) = nodes.pop() {
        // println!("{:?}", origin);
        // println!("Nodes: {:?}", nodes.len());

        grid.touch_leaf_at(origin.point());

        let (sweep_x, sweep_y, sweep_z) = sweep_direction.sweeps(origin.point(), size);
        for x in sweep_x {
            for y in sweep_y {
                for z in sweep_z {
                    let idx = Vec3i::new(x, y, z);
                    sweep_at(&idx, grid, fixed, h, limit);
                }
            }
        }

        // let node = match grid.leaf_at(origin.point()) {
        //     Some(node) => node,
        //     None => continue,
        // };

        insert_neighboring_nodes::<TTree, TLeafOrigin>(&mut nodes, &origin, limit, grid);
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy)]
enum SweepDirection {
    X_Y_Z,   // (1, 1, 1)
    NX_Y_Z,  // (-1, 1, 1)
    X_NY_Z,  // (1, -1, 1)
    NX_NY_Z, // (-1, -1, 1)

    X_Y_NZ,   // (1, 1, -1)
    NX_Y_NZ,  // (-1, 1, -1)
    X_NY_NZ,  // (1, -1, -1)
    NX_NY_NZ, // (-1, -1, -1)
}

impl SweepDirection {
    fn sweeps(&self, origin: &Vec3i, size: isize) -> (RangeIter, RangeIter, RangeIter) {
        match self {
            SweepDirection::X_Y_Z => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDirection::NX_Y_Z => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDirection::X_NY_Z => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDirection::NX_NY_Z => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDirection::X_Y_NZ => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDirection::NX_Y_NZ => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDirection::X_NY_NZ => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDirection::NX_NY_NZ => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
        }
    }

    #[inline]
    fn is_positive_x_dir(&self) -> bool {
        match self {
            SweepDirection::X_Y_Z
            | SweepDirection::X_NY_Z
            | SweepDirection::X_Y_NZ
            | SweepDirection::X_NY_NZ => true,
            SweepDirection::NX_Y_Z
            | SweepDirection::NX_NY_Z
            | SweepDirection::NX_Y_NZ
            | SweepDirection::NX_NY_NZ => false,
        }
    }

    #[inline]
    fn is_positive_y_dir(&self) -> bool {
        match self {
            SweepDirection::X_Y_Z
            | SweepDirection::NX_Y_Z
            | SweepDirection::X_Y_NZ
            | SweepDirection::NX_Y_NZ => true,
            SweepDirection::X_NY_Z
            | SweepDirection::NX_NY_Z
            | SweepDirection::X_NY_NZ
            | SweepDirection::NX_NY_NZ => false,
        }
    }

    #[inline]
    fn is_positive_z_dir(&self) -> bool {
        match self {
            SweepDirection::X_Y_Z
            | SweepDirection::NX_Y_Z
            | SweepDirection::X_NY_Z
            | SweepDirection::NX_NY_Z => true,
            SweepDirection::X_Y_NZ
            | SweepDirection::NX_Y_NZ
            | SweepDirection::X_NY_NZ
            | SweepDirection::NX_NY_NZ => false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct RangeIter {
    start: isize,
    end: isize,
    step: isize,
}

impl RangeIter {
    fn new(start: isize, end: isize, step: isize) -> Self {
        Self { start, end, step }
    }
}

impl Iterator for RangeIter {
    type Item = isize;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.start == self.end {
            return None;
        }

        let current = self.start;
        self.start += self.step;
        Some(current)
    }
}

fn sweep_at(
    idx: &Vec3i,
    sdf: &mut impl TreeNode<Value = f32>,
    fixed: &impl TreeNode<Value = Empty>,
    h: f32,
    limit: f32,
) {
    if let Some(_) = fixed.at(&idx) {
        return;
    }

    let x_p = Vec3i::new(idx.x + 1, idx.y, idx.z);
    let x_n = Vec3i::new(idx.x - 1, idx.y, idx.z);
    let y_p = Vec3i::new(idx.x, idx.y + 1, idx.z);
    let y_n = Vec3i::new(idx.x, idx.y - 1, idx.z);
    let z_p = Vec3i::new(idx.x, idx.y, idx.z + 1);
    let z_n = Vec3i::new(idx.x, idx.y, idx.z - 1);

    let (d1, d2, d3) = (
        option_min(sdf.at(&x_p), sdf.at(&x_n))
            .copied()
            .unwrap_or(f32::far()),
        option_min(sdf.at(&y_p), sdf.at(&y_n))
            .copied()
            .unwrap_or(f32::far()),
        option_min(sdf.at(&z_p), sdf.at(&z_n))
            .copied()
            .unwrap_or(f32::far()),
    );

    if d1 == f32::far() && d2 == f32::far() && d3 == f32::far() {
        return;
    }

    // Assuming we are offsetting outwards
    if d1.sign() == Sign::Negative || d2.sign() == Sign::Negative || d3.sign() == Sign::Negative {
        return;
    }

    let d_new = compute_distance(d1, d2, d3, h);

    if d_new > limit {
        return;
    }

    let d_old = sdf.at(&idx).copied().unwrap_or(f32::far());

    if d_new < d_old {
        sdf.insert(&idx, d_new);
    }
}

// Fast_Occlusion_Sweeping
fn compute_distance(mut a1: f32, mut a2: f32, mut a3: f32, h: f32) -> f32 {
    sort3(&mut a1, &mut a2, &mut a3);

    let s1 = a1 + h;

    if s1.abs() <= a2 {
        return s1;
    }

    let a1a2_sum = a1 + a2;
    let h_sq = h * h;
    let two_h_sq = h_sq + h_sq;
    let a1a2_diff = a1 - a2;
    let a1a2_diff_sq = a1a2_diff * a1a2_diff;
    let s2 = (a1a2_sum + (two_h_sq - a1a2_diff_sq).sqrt()) * 0.5;

    if s2.abs() <= a3 {
        return s2;
    }

    let a1a2a3_sum = a1a2_sum + a3;
    let three_h_sq = two_h_sq + h_sq;
    let a1a3_diff = a1 - a3;
    let a1a3_diff_sq = a1a3_diff * a1a3_diff;
    let a2a3_diff = a2 - a3;
    let a2a3_diff_sq = a2a3_diff * a2a3_diff;
    let s3 = (a1a2a3_sum + (three_h_sq - a1a2_diff_sq - a1a3_diff_sq - a2a3_diff_sq).sqrt())
        * (1.0 / 3.0);

    s3
}

fn insert_neighboring_nodes<TTree, TLeafOrigin>(
    nodes: &mut BinaryHeap<TLeafOrigin>,
    origin: &TLeafOrigin,
    limit: f32,
    sdf: &mut TTree,
) where
    TTree: TreeNode<Value = f32>,
    TLeafOrigin: LeafOrigin,
{
    let size = TTree::Leaf::resolution() as isize;
    let node = match sdf.leaf_at(origin.point()) {
        Some(node) => node,
        None => return,
    };
    let origin = node.origin();
    let dir = TLeafOrigin::sweep_dir();

    let (z, z_next) = if dir.is_positive_z_dir() {
        (size - 1, origin.z + size)
    } else {
        (origin.z, origin.z - size)
    };

    let mut insert_z = false;

    for x in 0..size {
        for y in 0..size {
            let idx = Vec3i::new(x, y, z);

            if let Some(value) = node.at(&idx).copied() {
                insert_z = value < limit;
                break;
            }
        }
    }

    ///////////////////////

    let (y, y_next) = if dir.is_positive_y_dir() {
        (size - 1, origin.y + size)
    } else {
        (origin.y, origin.y - size)
    };

    let mut insert_y = false;

    for x in 0..size {
        for z in 0..size {
            let idx = Vec3i::new(x, y, z);

            if let Some(value) = node.at(&idx).copied() {
                insert_y = value < limit;
                break;
            }
        }
    }

    /////////////////

    let (x, x_next) = if dir.is_positive_x_dir() {
        (size - 1, origin.x + size)
    } else {
        (origin.x, origin.x - size)
    };

    let mut insert_x = false;

    for y in 0..size {
        for z in 0..size {
            let idx = Vec3i::new(x, y, z);

            if let Some(value) = node.at(&idx).copied() {
                insert_x = value < limit;
                break;
            }
        }
    }

    /////////////////

    let next_x = Vec3i::new(x_next, origin.y, origin.z);
    if insert_x && sdf.leaf_at(&next_x).is_none() {
        // println!("insert y {}", next_y);
        sdf.touch_leaf_at(&next_x);
        nodes.push(next_x.into());
        //assert!(visited.insert(next_x.into()));
    }

    let next_y = Vec3i::new(origin.x, y_next, origin.z);
    if insert_y && sdf.leaf_at(&next_y).is_none() {
        // println!("insert y {}", next_y);
        sdf.touch_leaf_at(&next_y);
        nodes.push(next_y.into());
        //assert!(visited.insert(next_y.into()));
    }

    let next_z = Vec3i::new(origin.x, origin.y, z_next);
    if insert_z && sdf.leaf_at(&next_z).is_none() {
        // println!("insert z {}", next_z);
        sdf.touch_leaf_at(&next_z);
        nodes.push(next_z.into());
        //assert!(visited.insert(next_z.into()));
    }
}

struct SweepStencil<TTree: TreeNode> {
    top: TTree::Leaf,
    bottom: TTree::Leaf,
    left: TTree::Leaf,
    right: TTree::Leaf,
    front: TTree::Leaf,
    back: TTree::Leaf,
}

struct CollectLeafIndicesVisitor<TTree: TreeNode, TLeafOrigin: LeafOrigin> {
    indices: BinaryHeap<TLeafOrigin>,
    _tree: PhantomData<TTree>,
}

impl<TTree: TreeNode, TLeafOrigin: LeafOrigin> Visitor<TTree::Leaf>
    for CollectLeafIndicesVisitor<TTree, TLeafOrigin>
{
    #[inline]
    fn tile(&mut self, tile: Tile<TTree::Value>) {
        let origin = tile.origin;
        self.indices.push(origin.into());
    }

    #[inline]
    fn dense(&mut self, dense: &TTree::Leaf) {
        let origin = dense.origin();
        self.indices.push(origin.into());
    }
}

trait LeafOrigin: Ord + From<Vec3i> + Clone + Copy + Debug {
    fn point(&self) -> &Vec3i;
    fn sweep_dir() -> SweepDirection;
}

macro_rules! LeafOrigin {
    ($name: tt, $cmp: tt, $sweep_dir: expr) => {
        #[allow(non_camel_case_types)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        struct $name(Vec3i);

        impl PartialOrd for $name {
            #[inline]
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        impl Ord for $name {
            #[inline]
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                $cmp(&self.0, &other.0).reverse() // Should be reversed to pop the smallest element because we use max heap??
            }
        }

        impl From<Vec3i> for $name {
            #[inline]
            fn from(point: Vec3i) -> Self {
                Self(point)
            }
        }

        impl LeafOrigin for $name {
            #[inline]
            fn point(&self) -> &Vec3i {
                &self.0
            }

            #[inline]
            fn sweep_dir() -> SweepDirection {
                $sweep_dir
            }
        }
    };
}

#[inline]
fn sort_x_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).then(a.y.cmp(&b.y)).then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_X_Y_Z, sort_x_y_z, SweepDirection::X_Y_Z);

#[inline]
fn sort_nx_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_NX_Y_Z, sort_nx_y_z, SweepDirection::NX_Y_Z);

#[inline]
fn sort_x_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_X_NY_Z, sort_x_ny_z, SweepDirection::X_NY_Z);

#[inline]
fn sort_nx_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_NX_NY_Z, sort_nx_ny_z, SweepDirection::NX_NY_Z);

#[inline]
fn sort_x_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_X_Y_NZ, sort_x_y_nz, SweepDirection::X_Y_NZ);

#[inline]
fn sort_nx_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_NX_Y_NZ, sort_nx_y_nz, SweepDirection::NX_Y_NZ);

#[inline]
fn sort_x_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_X_NY_NZ, sort_x_ny_nz, SweepDirection::X_NY_NZ);

#[inline]
fn sort_nx_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_NX_NY_NZ, sort_nx_ny_nz, SweepDirection::NX_NY_NZ);
