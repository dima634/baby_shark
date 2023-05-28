use std::iter::repeat_with;

use baby_shark::triangulation::delaunay::Triangulation2;
use criterion::{criterion_group, criterion_main, Criterion};
use delaunator::{triangulate, Point};
use nalgebra::Point2;
use rand::{rngs::StdRng, SeedableRng, Rng};

fn criterion_benchmark(c: &mut Criterion) {

    let groups = [10_000, 50_000, 100_000];
    
    for points_count in groups {
        let mut rng = StdRng::seed_from_u64(rand::random());
        let points2d: Vec<Point2<f64>> = repeat_with(|| rng.gen())
            .map(|(x, y)| Point2::new(x, y))
            .take(points_count)
            .collect();

        let points_delaunator: Vec<_> = points2d.iter().map(|p| Point {x: p.x, y: p.y}).collect();

        let mut group = c.benchmark_group(format!("{} points", points_count));
        
        group.bench_function("delaunator-rs", |b| b.iter(|| {
            triangulate(&points_delaunator);
        }));

        group.bench_function("baby_shark", |b| b.iter(|| {
            let mut triangulation = Triangulation2::new().with_points(&points2d);
            triangulation.triangulate();
        }));
    }
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
