use self::{utils::option_min_by, value::empty::Empty};
use super::*;
use crate::helpers::{aliases::Vec3i, utils::sort3};
use std::{
    collections::{BinaryHeap, HashSet},
    fmt::Debug,
    marker::PhantomData,
};

pub struct FastSweeping<TTree: TreeNode<Value = f32>> {
    limit_abs: f32,
    sweep_sign: Sign,
    grid_spacing: f32,
    frozen: Box<TTree::As<Empty>>,
    _tree: PhantomData<TTree>,
}

impl<TTree: TreeNode<Value = f32>> FastSweeping<TTree> {
    pub fn new(grid_spacing: f32, limit: f32) -> Self {
        let frozen = TreeNode::empty(Vec3i::zeros());
        Self {
            grid_spacing,
            frozen,
            limit_abs: limit.abs(),
            sweep_sign: limit.sign(),
            _tree: PhantomData,
        }
    }

    pub fn fast_sweep(&mut self, sdf: &mut TTree) {
        self.frozen = sdf.clone_map(&|_| Empty);
        let mut collect_nodes = CollectLeafIndicesVisitor::<TTree, Origin_X_Y_Z> {
            indices: BinaryHeap::new(),
            _tree: PhantomData,
        };
        sdf.visit_leafs(&mut collect_nodes);

        let mut queue = Queue::new(collect_nodes.indices);
        self.sweep::<Origin_X_Y_Z>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_NX_Y_Z>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_X_NY_Z>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_NX_NY_Z>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_X_Y_NZ>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_NX_Y_NZ>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_X_NY_NZ>(sdf, &mut queue);

        let mut queue = queue.into();
        self.sweep::<Origin_NX_NY_NZ>(sdf, &mut queue);

        // Algorithm can leave empty nodes in the tree
        // Somehow it is causing problems with flood filling
        // So we remove them, it is hacky but it works
        sdf.remove_empty_branches();
    }

    fn sweep<TLeafOrigin: LeafOrigin>(&self, sdf: &mut TTree, queue: &mut Queue<TLeafOrigin>) {
        let sweep_direction = TLeafOrigin::sweep_dir();
        let size = TTree::Leaf::resolution() as isize;

        while let Some(origin) = queue.pop() {
            let o = origin.point();
            let left_o = Vec3i::new(o.x - size, o.y, o.z);
            let right_o = Vec3i::new(o.x + size, o.y, o.z);
            let front_o = Vec3i::new(o.x, o.y - size, o.z);
            let back_o = Vec3i::new(o.x, o.y + size, o.z);
            let top_o = Vec3i::new(o.x, o.y, o.z + size);
            let bottom_o = Vec3i::new(o.x, o.y, o.z - size);

            let mut stencil = Stencil {
                left: sdf
                    .take_leaf_at(&left_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(left_o)),
                right: sdf
                    .take_leaf_at(&right_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(right_o)),
                front: sdf
                    .take_leaf_at(&front_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(front_o)),
                back: sdf
                    .take_leaf_at(&back_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(back_o)),
                top: sdf
                    .take_leaf_at(&top_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(top_o)),
                bottom: sdf
                    .take_leaf_at(&bottom_o)
                    .unwrap_or_else(|| TTree::Leaf::empty(bottom_o)),
                center: sdf.take_leaf_at(o).unwrap(),
                frozen: self.frozen.leaf_at(origin.point()),
                min: *o,
                max: o + Vec3i::new(size, size, size),
            };

            self.sweep_stencil(&mut stencil, sweep_direction);
            self.insert_neighboring_nodes(sdf, queue, stencil);
        }
    }

    fn sweep_stencil(&self, stencil: &mut Stencil<TTree::Leaf>, sweep_direction: Direction) {
        let size = TTree::Leaf::resolution() as isize;
        let (sweep_x, sweep_y, sweep_z) = sweep_direction.sweeps(&stencil.center.origin(), size);

        for x in sweep_x {
            for y in sweep_y {
                for z in sweep_z {
                    let idx = Vec3i::new(x, y, z);
                    self.sweep_voxel(&idx, stencil);
                }
            }
        }
    }

    fn sweep_voxel(&self, idx: &Vec3i, stencil: &mut Stencil<TTree::Leaf>) {
        if stencil.frozen.is_some_and(|node| node.at(idx).is_some()) {
            return;
        }

        let x_p = Vec3i::new(idx.x + 1, idx.y, idx.z);
        let x_n = Vec3i::new(idx.x - 1, idx.y, idx.z);
        let y_p = Vec3i::new(idx.x, idx.y + 1, idx.z);
        let y_n = Vec3i::new(idx.x, idx.y - 1, idx.z);
        let z_p = Vec3i::new(idx.x, idx.y, idx.z + 1);
        let z_n = Vec3i::new(idx.x, idx.y, idx.z - 1);

        let cmp_abs = |a: &&f32, b: &&f32| match a.abs().partial_cmp(&b.abs()) {
            Some(ord) => ord,
            None => core::cmp::Ordering::Less,
        };

        let distances = (
            option_min_by(stencil.at(&x_p), stencil.at(&x_n), cmp_abs),
            option_min_by(stencil.at(&y_p), stencil.at(&y_n), cmp_abs),
            option_min_by(stencil.at(&z_p), stencil.at(&z_n), cmp_abs),
        );

        match distances {
            (Some(v), _, _) | (_, Some(v), _) | (_, _, Some(v)) => {
                // Outward/Inward
                if v.sign() != self.sweep_sign {
                    return;
                }
            }
            (None, None, None) => return,
        };

        let (d1, d2, d3) = (
            distances.0.copied().unwrap_or(f32::far()),
            distances.1.copied().unwrap_or(f32::far()),
            distances.2.copied().unwrap_or(f32::far()),
        );

        let d_new_abs = compute_distance(d1.abs(), d2.abs(), d3.abs(), self.grid_spacing);
        debug_assert!(
            d_new_abs >= 0.0,
            "Should be positive: d_new_abs = {}",
            d_new_abs
        );

        let mut d_new = d_new_abs;
        d_new.set_sign(self.sweep_sign);

        if d_new_abs > self.limit_abs {
            return;
        }

        let d_old = stencil.center.at(idx).copied().unwrap_or(f32::far());

        if d_new_abs < d_old.abs() {
            stencil.center.insert(idx, d_new);
        }
    }

    fn insert_neighboring_nodes<TLeafOrigin>(
        &self,
        sdf: &mut TTree,
        queue: &mut Queue<TLeafOrigin>,
        stencil: Stencil<TTree::Leaf>,
    ) where
        TTree: TreeNode<Value = f32>,
        TLeafOrigin: LeafOrigin,
    {
        let size = TTree::Leaf::resolution() as isize;
        let origin = stencil.center.origin();
        let dir = TLeafOrigin::sweep_dir();

        let (z, z_next) = if dir.is_positive_z_dir() {
            (size - 1, origin.z + size)
        } else {
            (origin.z, origin.z - size)
        };

        let mut insert_z = false;

        'outer: for x in 0..size {
            for y in 0..size {
                let idx = Vec3i::new(x, y, z) + origin;
                insert_z = stencil
                    .center
                    .at(&idx)
                    .is_some_and(|v| v.sign() == self.sweep_sign && v.abs() < self.limit_abs);

                if insert_z {
                    break 'outer;
                }
            }
        }

        let (y, y_next) = if dir.is_positive_y_dir() {
            (size - 1, origin.y + size)
        } else {
            (origin.y, origin.y - size)
        };

        let mut insert_y = false;

        'outer: for x in 0..size {
            for z in 0..size {
                let idx = Vec3i::new(x, y, z) + origin;
                insert_y = stencil
                    .center
                    .at(&idx)
                    .is_some_and(|v| v.sign() == self.sweep_sign && v.abs() < self.limit_abs);

                if insert_y {
                    break 'outer;
                }
            }
        }

        let (x, x_next) = if dir.is_positive_x_dir() {
            (size - 1, origin.x + size)
        } else {
            (origin.x, origin.x - size)
        };

        let mut insert_x = false;

        'outer: for y in 0..size {
            for z in 0..size {
                let idx = Vec3i::new(x, y, z) + origin;
                insert_x = stencil
                    .center
                    .at(&idx)
                    .is_some_and(|v| v.sign() == self.sweep_sign && v.abs() < self.limit_abs);

                if insert_x {
                    break 'outer;
                }
            }
        }

        if insert_x {
            let node_origin = Vec3i::new(x_next, origin.y, origin.z);
            queue.push(node_origin.into());
        }

        if insert_y {
            let node_origin = Vec3i::new(origin.x, y_next, origin.z);
            queue.push(node_origin.into());
        }

        if insert_z {
            let node_origin = Vec3i::new(origin.x, origin.y, z_next);
            queue.push(node_origin.into());
        }

        sdf.insert_leaf_at(stencil.top);
        sdf.insert_leaf_at(stencil.bottom);
        sdf.insert_leaf_at(stencil.left);
        sdf.insert_leaf_at(stencil.right);
        sdf.insert_leaf_at(stencil.front);
        sdf.insert_leaf_at(stencil.back);
        sdf.insert_leaf_at(stencil.center);
    }
}

struct Queue<TLeafOrigin: LeafOrigin> {
    nodes: BinaryHeap<TLeafOrigin>,
    removed: Vec<TLeafOrigin>,
    existing: HashSet<TLeafOrigin>,
}

impl<TLeafOrigin: LeafOrigin> Queue<TLeafOrigin> {
    fn new(nodes: BinaryHeap<TLeafOrigin>) -> Self {
        let removed = Vec::with_capacity(nodes.len());
        let existing = HashSet::from_iter(nodes.iter().cloned());
        Self {
            nodes,
            removed,
            existing,
        }
    }

    #[inline]
    fn pop(&mut self) -> Option<TLeafOrigin> {
        if let Some(origin) = self.nodes.pop() {
            self.removed.push(origin);
            Some(origin)
        } else {
            None
        }
    }

    #[inline]
    fn push(&mut self, origin: TLeafOrigin) {
        if self.existing.insert(origin) {
            self.nodes.push(origin);
        }
    }

    fn into<TDest>(self) -> Queue<TDest>
    where
        TDest: LeafOrigin,
    {
        debug_assert!(self.nodes.is_empty());

        let nodes: BinaryHeap<_> = BinaryHeap::from_iter(
            self.removed
                .into_iter()
                .map(|origin| origin.point_owned().into()),
        );
        let existing = HashSet::from_iter(nodes.iter().cloned());
        let removed = Vec::with_capacity(nodes.len());

        Queue {
            nodes,
            existing,
            removed,
        }
    }
}

struct Stencil<'tree, TLeaf: TreeNode> {
    top: Box<TLeaf>,
    bottom: Box<TLeaf>,
    left: Box<TLeaf>,
    right: Box<TLeaf>,
    front: Box<TLeaf>,
    back: Box<TLeaf>,
    center: Box<TLeaf>,
    frozen: Option<&'tree TLeaf::As<Empty>>,
    min: Vec3i,
    max: Vec3i,
}

impl<'tree, TLeaf: TreeNode<Value = f32>> Stencil<'tree, TLeaf> {
    #[inline]
    fn at(&self, idx: &Vec3i) -> Option<&f32> {
        if idx.z < self.min.z {
            return self.bottom.at(idx);
        }

        if idx.z >= self.max.z {
            return self.top.at(idx);
        }

        if idx.y < self.min.y {
            return self.front.at(idx);
        }

        if idx.y >= self.max.y {
            return self.back.at(idx);
        }

        if idx.x < self.min.x {
            return self.left.at(idx);
        }

        if idx.x >= self.max.x {
            return self.right.at(idx);
        }

        self.center.at(idx)
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy)]
enum Direction {
    X_Y_Z,   // (1, 1, 1)
    NX_Y_Z,  // (-1, 1, 1)
    X_NY_Z,  // (1, -1, 1)
    NX_NY_Z, // (-1, -1, 1)

    X_Y_NZ,   // (1, 1, -1)
    NX_Y_NZ,  // (-1, 1, -1)
    X_NY_NZ,  // (1, -1, -1)
    NX_NY_NZ, // (-1, -1, -1)
}

impl Direction {
    fn sweeps(&self, origin: &Vec3i, size: isize) -> (RangeIter, RangeIter, RangeIter) {
        match self {
            Direction::X_Y_Z => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            Direction::NX_Y_Z => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            Direction::X_NY_Z => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            Direction::NX_NY_Z => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z, origin.z + size, 1),
            ),
            Direction::X_Y_NZ => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            Direction::NX_Y_NZ => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y, origin.y + size, 1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            Direction::X_NY_NZ => (
                RangeIter::new(origin.x, origin.x + size, 1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
            Direction::NX_NY_NZ => (
                RangeIter::new(origin.x + size - 1, origin.x - 1, -1),
                RangeIter::new(origin.y + size - 1, origin.y - 1, -1),
                RangeIter::new(origin.z + size - 1, origin.z - 1, -1),
            ),
        }
    }

    #[inline]
    const fn is_positive_x_dir(&self) -> bool {
        match self {
            Direction::X_Y_Z | Direction::X_NY_Z | Direction::X_Y_NZ | Direction::X_NY_NZ => true,
            Direction::NX_Y_Z | Direction::NX_NY_Z | Direction::NX_Y_NZ | Direction::NX_NY_NZ => {
                false
            }
        }
    }

    #[inline]
    const fn is_positive_y_dir(&self) -> bool {
        match self {
            Direction::X_Y_Z | Direction::NX_Y_Z | Direction::X_Y_NZ | Direction::NX_Y_NZ => true,
            Direction::X_NY_Z | Direction::NX_NY_Z | Direction::X_NY_NZ | Direction::NX_NY_NZ => {
                false
            }
        }
    }

    #[inline]
    const fn is_positive_z_dir(&self) -> bool {
        match self {
            Direction::X_Y_Z | Direction::NX_Y_Z | Direction::X_NY_Z | Direction::NX_NY_Z => true,
            Direction::X_Y_NZ | Direction::NX_Y_NZ | Direction::X_NY_NZ | Direction::NX_NY_NZ => {
                false
            }
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
    const fn new(start: isize, end: isize, step: isize) -> Self {
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

    (a1a2a3_sum + (three_h_sq - a1a2_diff_sq - a1a3_diff_sq - a2a3_diff_sq).sqrt()) * (1.0 / 3.0)
}

struct CollectLeafIndicesVisitor<TTree: TreeNode, TLeafOrigin: LeafOrigin> {
    indices: BinaryHeap<TLeafOrigin>,
    _tree: PhantomData<TTree>,
}

impl<TTree: TreeNode<Value = f32>, TLeafOrigin: LeafOrigin> Visitor<TTree::Leaf>
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

trait LeafOrigin: Ord + From<Vec3i> + Clone + Copy + core::hash::Hash + Debug {
    fn point(&self) -> &Vec3i;
    fn point_owned(self) -> Vec3i;
    fn sweep_dir() -> Direction;
}

macro_rules! LeafOrigin {
    ($name: tt, $cmp: tt, $sweep_dir: expr) => {
        #[allow(non_camel_case_types)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
            fn point_owned(self) -> Vec3i {
                self.0
            }

            #[inline]
            fn sweep_dir() -> Direction {
                $sweep_dir
            }
        }
    };
}

#[inline]
fn sort_x_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x).then(a.y.cmp(&b.y)).then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_X_Y_Z, sort_x_y_z, Direction::X_Y_Z);

#[inline]
fn sort_nx_y_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_NX_Y_Z, sort_nx_y_z, Direction::NX_Y_Z);

#[inline]
fn sort_x_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_X_NY_Z, sort_x_ny_z, Direction::X_NY_Z);

#[inline]
fn sort_nx_ny_z(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z))
}

LeafOrigin!(Origin_NX_NY_Z, sort_nx_ny_z, Direction::NX_NY_Z);

#[inline]
fn sort_x_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_X_Y_NZ, sort_x_y_nz, Direction::X_Y_NZ);

#[inline]
fn sort_nx_y_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y))
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_NX_Y_NZ, sort_nx_y_nz, Direction::NX_Y_NZ);

#[inline]
fn sort_x_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_X_NY_NZ, sort_x_ny_nz, Direction::X_NY_NZ);

#[inline]
fn sort_nx_ny_nz(a: &Vec3i, b: &Vec3i) -> std::cmp::Ordering {
    a.x.cmp(&b.x)
        .reverse()
        .then(a.y.cmp(&b.y).reverse())
        .then(a.z.cmp(&b.z).reverse())
}

LeafOrigin!(Origin_NX_NY_NZ, sort_nx_ny_nz, Direction::NX_NY_NZ);
