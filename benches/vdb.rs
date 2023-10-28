use std::iter::repeat_with;

use baby_shark::{triangulation::delaunay::Triangulation2, static_vdb, voxel::{utils::box_indices, Accessor}};
use criterion::{criterion_group, criterion_main, Criterion, BenchmarkId};
use delaunator::{triangulate, Point};
use nalgebra::Point2;
use rand::{rngs::StdRng, SeedableRng, Rng};

///
/// Size 200, insert then remove
/// Static VDB 4+3+2, vec + option, direct accessor: 300ms
/// 

fn criterion_benchmark(c: &mut Criterion) {
    type Tree = static_vdb!((), 5, 4, 3);


    c.bench_function("Static VDB 4+3+2, vec + option<box>, direct accessor", |b| b.iter(|| {
        let size = 200;
        let mut tree = Tree::new();

        for idx in box_indices(0, size) {
            tree.insert(&idx, ());
        }

        for idx in box_indices(0, size) {
            tree.remove(&idx);
        }
    }));


    let size = 200;
    let mut tree = Tree::new();

    for idx in box_indices(0, size) {
        tree.insert(&idx, ());
    }

    // c.benchmark_group("Random access")
    //     .bench_function("direct accessor", |b| b.iter(|| {
    //         for idx in box_indices(0, size) {
    //             tree.at(&idx);
    //         }
    //     }))
    //     .bench_function("cached accessor", |b| b.iter(|| {
    //         let cached_accessor = tree.cached_accessor();

    //         for idx in box_indices(0, size) {
    //             cached_accessor.at(&idx);
    //         }
    //     }));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
