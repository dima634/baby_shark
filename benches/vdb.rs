use std::iter::repeat_with;

use baby_shark::{triangulation::delaunay::Triangulation2, static_vdb, voxel::{utils::box_indices, Accessor}};
use criterion::{criterion_group, criterion_main, Criterion};
use delaunator::{triangulate, Point};
use nalgebra::Point2;
use rand::{rngs::StdRng, SeedableRng, Rng};

fn criterion_benchmark(c: &mut Criterion) {
    type Tree = static_vdb!(4, 3, 2);

    let mut tree = Tree::new();

    c.bench_function("VDB", |b| b.iter(|| {
        let size = 100;

        for idx in box_indices(0, size) {
            tree.insert(&idx);
        }

        for idx in box_indices(0, size) {
            tree.remove(&idx);
        }
    }));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
