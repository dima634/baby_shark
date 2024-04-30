use std::{
    collections::{BTreeSet, BinaryHeap}, fmt::Debug, marker::PhantomData, ops::{Index, IndexMut, Range}
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
        grid.flood_fill();
        let fixed = grid.clone_map(&|_| Empty);

        sweep::<T, Origin_X_Y_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_Y_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_NY_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_NY_Z>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_Y_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_Y_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_X_NY_NZ>(grid, h, limit, &fixed);
        sweep::<T, Origin_NX_NY_NZ>(grid, h, limit, &fixed);

        // sweep::<T, Origin_X_Y_Z>(grid, h, limit, &fixed);
        // sweep::<T, Origin_NX_Y_Z>(grid, h, limit, &fixed);
        // sweep::<T, Origin_X_NY_Z>(grid, h, limit, &fixed);
        // sweep::<T, Origin_NX_NY_Z>(grid, h, limit, &fixed);
        // sweep::<T, Origin_X_Y_NZ>(grid, h, limit, &fixed);
        // sweep::<T, Origin_NX_Y_NZ>(grid, h, limit, &fixed);
        // sweep::<T, Origin_X_NY_NZ>(grid, h, limit, &fixed);
        // sweep::<T, Origin_NX_NY_NZ>(grid, h, limit, &fixed);
    }

    
}

fn sweep<TTree: TreeNode<Value = f32> + FloodFill + Csg, TLeafOrigin: LeafOrigin>(grid: &mut TTree, h: f32, limit: f32, fixed: &TTree::As<Empty>) {
    let dir = TLeafOrigin::sweep_dir();
    println!("Direction: {:?}", dir);

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

        let (sweep_x, sweep_y, sweep_z) = dir.sweeps(origin.point(), size);
        for x in sweep_x {
            for y in sweep_y {
                for z in sweep_z {
                    let idx = Vec3i::new(x, y, z);

                    sweep_at::<true, TTree, TTree, TTree::As<Empty>>(
                        &idx,
                        grid,
                        &fixed,
                        h,
                        limit,
                        dir,
                    );
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
enum SweepDir {
    X_Y_Z,   // (1, 1, 1)
    NX_Y_Z,  // (-1, 1, 1)
    X_NY_Z,  // (1, -1, 1)
    NX_NY_Z, // (-1, -1, 1)

    X_Y_NZ,   // (1, 1, -1)
    NX_Y_NZ,  // (-1, 1, -1)
    X_NY_NZ,  // (1, -1, -1)
    NX_NY_NZ, // (-1, -1, -1)
}

impl SweepDir {
    fn neighbors(&self) -> (Vec3i, Vec3i, Vec3i) {
        match self {
            SweepDir::X_Y_Z => (Vec3i::new(-1, 0, 0), Vec3i::new(0, -1, 0), Vec3i::new(0, 0, -1)),
            SweepDir::NX_Y_Z => (Vec3i::new(1, 0, 0), Vec3i::new(0, -1, 0), Vec3i::new(0, 0, -1)),
            SweepDir::X_NY_Z => (Vec3i::new(-1, 0, 0), Vec3i::new(0, 1, 0), Vec3i::new(0, 0, -1)),
            SweepDir::NX_NY_Z => (Vec3i::new(1, 0, 0), Vec3i::new(0, 1, 0), Vec3i::new(0, 0, -1)),
            SweepDir::X_Y_NZ => (Vec3i::new(-1, 0, 0), Vec3i::new(0, -1, 0), Vec3i::new(0, 0, 1)),
            SweepDir::NX_Y_NZ => (Vec3i::new(1, 0, 0), Vec3i::new(0, -1, 0), Vec3i::new(0, 0, 1)),
            SweepDir::X_NY_NZ => (Vec3i::new(-1, 0, 0), Vec3i::new(0, 1, 0), Vec3i::new(0, 0, 1)),
            SweepDir::NX_NY_NZ => (Vec3i::new(1, 0, 0), Vec3i::new(0, 1, 0), Vec3i::new(0, 0, 1)),
        }
    }

    fn sweeps(&self, origin: &Vec3i, size: isize) -> (SweepIter, SweepIter, SweepIter) {
        match self {
            SweepDir::X_Y_Z => (
                SweepIter::new(origin.x, origin.x + size, 1),
                SweepIter::new(origin.y, origin.y + size, 1),
                SweepIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDir::NX_Y_Z => (
                SweepIter::new(origin.x + size - 1, origin.x - 1, -1),
                SweepIter::new(origin.y, origin.y + size, 1),
                SweepIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDir::X_NY_Z => (
                SweepIter::new(origin.x, origin.x + size, 1),
                SweepIter::new(origin.y + size - 1, origin.y - 1, -1),
                SweepIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDir::NX_NY_Z => (
                SweepIter::new(origin.x + size - 1, origin.x - 1, -1),
                SweepIter::new(origin.y + size - 1, origin.y - 1, -1),
                SweepIter::new(origin.z, origin.z + size, 1),
            ),
            SweepDir::X_Y_NZ => (
                SweepIter::new(origin.x, origin.x + size, 1),
                SweepIter::new(origin.y, origin.y + size, 1),
                SweepIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDir::NX_Y_NZ => (
                SweepIter::new(origin.x + size - 1, origin.x - 1, -1),
                SweepIter::new(origin.y, origin.y + size, 1),
                SweepIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDir::X_NY_NZ => (
                SweepIter::new(origin.x, origin.x + size, 1),
                SweepIter::new(origin.y + size - 1, origin.y - 1, -1),
                SweepIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            SweepDir::NX_NY_NZ => (
                SweepIter::new(origin.x + size - 1, origin.x - 1, -1),
                SweepIter::new(origin.y + size - 1, origin.y - 1, -1),
                SweepIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
        }
    }

    #[inline]
    fn is_positive_x_dir(&self) -> bool {
        match self {
            SweepDir::X_Y_Z | SweepDir::X_NY_Z | SweepDir::X_Y_NZ | SweepDir::X_NY_NZ => true,
            SweepDir::NX_Y_Z | SweepDir::NX_NY_Z | SweepDir::NX_Y_NZ | SweepDir::NX_NY_NZ => false,
        }
    }

    #[inline]
    fn is_positive_y_dir(&self) -> bool {
        match self {
            SweepDir::X_Y_Z | SweepDir::NX_Y_Z | SweepDir::X_Y_NZ | SweepDir::NX_Y_NZ => true,
            SweepDir::X_NY_Z | SweepDir::NX_NY_Z | SweepDir::X_NY_NZ | SweepDir::NX_NY_NZ => false,
        }
    }

    #[inline]
    fn is_positive_z_dir(&self) -> bool {
        match self {
            SweepDir::X_Y_Z | SweepDir::NX_Y_Z | SweepDir::X_NY_Z | SweepDir::NX_NY_Z => true,
            SweepDir::X_Y_NZ | SweepDir::NX_Y_NZ | SweepDir::X_NY_NZ | SweepDir::NX_NY_NZ => false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SweepIter {
    start: isize,
    end: isize,
    step: isize,
}

impl SweepIter {
    fn new(start: isize, end: isize, step: isize) -> Self {
        Self { start, end, step }
    }
}

impl Iterator for SweepIter {
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

fn sweep_at<const NN_SWEEP: bool, TExt, TSdf, TFixed>(
    idx: &Vec3i,
    sdf: &mut TSdf,
    fixed: &TFixed,
    h: f32,
    limit: f32,
    dir: SweepDir,
) where
    TExt: TreeNode<Value = f32>,
    TSdf: TreeNode<Value = f32>,
    TFixed: TreeNode<Value = Empty>,
{

    if let Some(_) = fixed.at(&idx) {
        // println!("Fixed");
        return;
    }

    // let (n1, n2, n3) = dir.neighbors(idx);

    let x_p = Vec3i::new(idx.x + 1, idx.y, idx.z);
    let x_n = Vec3i::new(idx.x - 1, idx.y, idx.z);
    let y_p = Vec3i::new(idx.x, idx.y + 1, idx.z);
    let y_n = Vec3i::new(idx.x, idx.y - 1, idx.z);
    let z_p = Vec3i::new(idx.x, idx.y, idx.z + 1);
    let z_n = Vec3i::new(idx.x, idx.y, idx.z - 1);

    // let nn = (
    //     option_min(ext.at(&x_p).or_else(|| sdf.at(&x_p)), (ext.at(&x_n).or_else(|| sdf.at(&x_n)))),
    //     option_min(ext.at(&y_p).or_else(|| sdf.at(&y_p)), (ext.at(&y_n).or_else(|| sdf.at(&y_n)))),
    //     option_min(ext.at(&z_p).or_else(|| sdf.at(&z_p)), (ext.at(&z_n).or_else(|| sdf.at(&z_n)))),
    // );

    let (d1, d2, d3) = (
        option_min(
            sdf.at(&x_p),
            sdf.at(&x_n),
        )
        .copied()
        .unwrap_or(f32::far()),
        option_min(
            sdf.at(&y_p),
            sdf.at(&y_n),
        )
        .copied()
        .unwrap_or(f32::far()),
        option_min(
            sdf.at(&z_p),
            sdf.at(&z_n),
        )
        .copied()
        .unwrap_or(f32::far()),
    );

    // let (d1, d2, d3) = (
    //     sdf.at(&(idx + dir.neighbors().0)).copied().unwrap_or(f32::far()),
    //     sdf.at(&(idx + dir.neighbors().1)).copied().unwrap_or(f32::far()),
    //     sdf.at(&(idx + dir.neighbors().2)).copied().unwrap_or(f32::far()),
    // );

    // assert!(d1.sign() == d2.sign() && d1.sign() == d3.sign());

    if d1 == f32::far() && d2 == f32::far() && d3 == f32::far() {
        if idx == &(Vec3i::new(51, 88, 216) + Vec3i::new(-152, -120, -16)) {
            println!("Broken pixel far");
        }
        return;
    }

    assert!(d1 != f32::far() || d2 != f32::far() || d3 != f32::far());

    // Assuming we are offsetting outwards
    if d1.sign() == Sign::Negative || d2.sign() == Sign::Negative || d3.sign() == Sign::Negative {
        // println!("Negative sign");
        // println!("{} {} {}", d1, d2, d3);
        
        if idx == &(Vec3i::new(51, 88, 216) + Vec3i::new(-152, -120, -16)) {
            println!("Broken pixel neg");
    }
        return;
    }

    // if sdf.at(&x_p).is_some_and(|v| v.sign() == Sign::Negative) ||
    //     sdf.at(&x_n).is_some_and(|v| v.sign() == Sign::Negative) ||
    //     sdf.at(&y_p).is_some_and(|v| v.sign() == Sign::Negative) ||
    //     sdf.at(&y_n).is_some_and(|v| v.sign() == Sign::Negative) ||
    //     sdf.at(&z_p).is_some_and(|v| v.sign() == Sign::Negative) ||
    //     sdf.at(&z_n).is_some_and(|v| v.sign() == Sign::Negative) {
    //     return;
    // }

    let debug = Vec3i::new(171, 20, 82);
    let d_new = compute_distance(idx, d1, d2, d3, h);
    assert!(d_new.sign() == Sign::Positive);
    // println!("Distance: {}, d1: {}, d2: {}, d3: {}", d_new, d1, d2, d3);


    if d_new > limit {
        // println!("Limit");
        
        if idx == &(debug + Vec3i::new(-152, -120, -16)) {
            println!("Broken pixel limit");
        }
        return;
    }

    let d_old = sdf.at(&idx).copied().unwrap_or(f32::INFINITY);
    assert!(d_old >= 0.0);

    // println!("Old: {}, New: {}", d_old, d_new);

    if d_new < d_old {
        
        if idx == &(debug + Vec3i::new(-152, -120, -16)) {
            println!("insert new = {}, old = {}, ({}, {}, {})", d_new, d_old, d1, d2, d3);
        }
        assert!(d_new.is_finite());
        assert!(d_new >= 0.0);
        // assert!(d_new <= 5.0, "{} {} {} {} {}", d_new, d_old, d1, d2, d3);
        // println!("{d_new}");
        sdf.insert(&idx, d_new);
    } else {

    
        if idx == &(debug + Vec3i::new(-152, -120, -16)) {
            println!("keep new = {}, old = {}, ({}, {}, {})", d_new, d_old, d1, d2, d3);
        }
    }
}

fn compute_distance(idx: &Vec3i, mut a1: f32, mut a2: f32, mut a3: f32, h: f32) -> f32 {

    sort3(&mut a1, &mut a2, &mut a3);

    // let min = a1.min(a2).min(a3);
    // return min + h;

    // VDB
    if false {
        let sqrt2h = 2.0_f32.sqrt() * h;
        let d1 = a1;
        let d2 = a2; 
        let d3 = a3;
        let mut update = a1 + h;

        if update <= d2 {
            return update;
        }

        if d2 <= sqrt2h + d1 {
            let D = 2.0 * h * h - (d1 - d2) * (d1 - d2);
            update = 0.5 * (d1 + d2 + D.sqrt());

            if update > d2 && update <= d3 {
                return update;
            }
        }

        let d123 = d1 + d2 + d3;
        let D = d123 * d123 - 3.0 * (d1 * d1 + d2 * d2 + d3 * d3 - h * h);

        if D >= 0.0 {
            update = (d123 + D.sqrt()) / 3.0;
            return update;
        }

        return f32::far();
    }


    // Fast_Occlusion_Sweeping
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

    // let a = 2.0; 
    // let b = -2.0 * (a1 + a2);
    // let c = a1 * a1 + a2 * a2 - h * h;
    // let D = (b * b - 4.0 * a * c).sqrt();
    // // choose the minimal root
    // let s2 = if (-b + D) > (-b - D) { (-b + D) } else { (-b - D)}  / (2.0 * a);


///
/// 
/// WEIGHTS IN OPENVDB
/// 
/// 
/// 

    if s2.abs() <= a3 {
        return s2;
    }

    // println!("Third");

    

    let a1a2a3_sum = a1a2_sum + a3;
    let three_h_sq = two_h_sq + h_sq;
    let a1a3_diff = a1 - a3;
    let a1a3_diff_sq = a1a3_diff * a1a3_diff;
    let a2a3_diff = a2 - a3;
    let a2a3_diff_sq = a2a3_diff * a2a3_diff;
    let s3 = (a1a2a3_sum + (three_h_sq - a1a2_diff_sq - a1a3_diff_sq - a2a3_diff_sq).sqrt())
        * (1.0 / 3.0);

    // let a = 3.0;
    // let b = -2.0 * (a1 + a2 + a3);
    // let c = a1 * a1 + a2 * a2 + a3 * a3 - h * h;
    // let D = (b * b - 4.0 * a * c).sqrt();
    // // choose the minimal root
    // let s3 = if (-b + D) > (-b - D) { (-b + D) } else { (-b - D) } / (2.0 * a);

    
    // if idx == &(Vec3i::new(51, 88, 216) + Vec3i::new(-152, -120, -16)) {
    //     println!("s3");
    // }

    // let dd1 = a1 - s3;
    // let dd2 = a2 - s3;
    // let dd3 = a3 - s3;
    // let w = 1.0 / (dd1 + dd2 + dd3); 

    // let ext = w * (dd1 * a1 + dd2 * a2 + dd3 * a3);

    // ext
    
    s3
    //s3.min(s2).min(s1)
}

fn insert_neighboring_nodes<TTree: TreeNode<Value = f32>, TLeafOrigin: LeafOrigin>(nodes: &mut BinaryHeap<TLeafOrigin>, origin: &TLeafOrigin, limit: f32, extension: &mut TTree) {
    let size = TTree::Leaf::resolution() as isize;
    let node = match extension.leaf_at(origin.point()) {
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
    if insert_x && extension.leaf_at(&next_x).is_none() {
        // println!("insert y {}", next_y);
        extension.touch_leaf_at(&next_x);
        nodes.push(next_x.into());
        //assert!(visited.insert(next_x.into()));
    }

    let next_y = Vec3i::new(origin.x, y_next, origin.z);
    if insert_y && extension.leaf_at(&next_y).is_none() {
        // println!("insert y {}", next_y);
        extension.touch_leaf_at(&next_y);
        nodes.push(next_y.into());
        //assert!(visited.insert(next_y.into()));
    }

    let next_z = Vec3i::new(origin.x, origin.y, z_next);
    if insert_z && extension.leaf_at(&next_z).is_none() {
        // println!("insert z {}", next_z);
        extension.touch_leaf_at(&next_z);
        nodes.push(next_z.into());
        //assert!(visited.insert(next_z.into()));
    }
}

struct CollectLeafIndicesVisitor<TTree: TreeNode, TLeafOrigin: LeafOrigin> {
    indices: BinaryHeap<TLeafOrigin>,
    _tree: PhantomData<TTree>,
}

impl<TTree: TreeNode, TLeafOrigin: LeafOrigin> Visitor<TTree::Leaf> for CollectLeafIndicesVisitor<TTree, TLeafOrigin> {
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
    fn sweep_dir() -> SweepDir;
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
            fn sweep_dir() -> SweepDir {
                $sweep_dir
            }
        }
    };
}

#[inline]
fn sort_x_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(
    Origin_X_Y_Z,                 
    sort_x_y_z,
    SweepDir::X_Y_Z
);

#[inline]
fn sort_nx_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(
    Origin_NX_Y_Z,                 
    sort_nx_y_z,
    SweepDir::NX_Y_Z
);

#[inline]
fn sort_x_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(
    Origin_X_NY_Z,                 
    sort_x_ny_z,
    SweepDir::X_NY_Z
);

#[inline]
fn sort_nx_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(
    Origin_NX_NY_Z,                 
    sort_nx_ny_z,
    SweepDir::NX_NY_Z
);

#[inline]
fn sort_x_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(
    Origin_X_Y_NZ,                 
    sort_x_y_nz,
    SweepDir::X_Y_NZ
);

#[inline]
fn sort_nx_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(
    Origin_NX_Y_NZ,                 
    sort_nx_y_nz,
    SweepDir::NX_Y_NZ
);

#[inline]
fn sort_x_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(
    Origin_X_NY_NZ,                 
    sort_x_ny_nz,
    SweepDir::X_NY_NZ
);

#[inline]
fn sort_nx_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(
    Origin_NX_NY_NZ,                 
    sort_nx_ny_nz,
    SweepDir::NX_NY_NZ
);


