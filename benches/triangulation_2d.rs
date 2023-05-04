use std::{iter::repeat_with, io::{BufReader, Read}, fs::File};

use baby_shark::triangulation::delaunay::Triangulation2;
use criterion::{criterion_group, criterion_main, Criterion};
use delaunator::{triangulate, Point};
use nalgebra::Point2;
use rand::{rngs::StdRng, SeedableRng, Rng};

fn criterion_benchmark(c: &mut Criterion) {

    let groups = [500_000];
    
    for points_count in groups {
        // let mut reader = BufReader::new(File::open("benches/points.xy").expect("To open point dataset"));
        // let mut points_buf = Vec::new();
        // reader.read_to_end(&mut points_buf).expect("To read file");

        // let mut points2d: Vec<Point2<f64>> = points_buf.chunks(8 * 2)
        //     .map(|w| Point2::new(
        //         f64::from_le_bytes([w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]]), 
        //         f64::from_le_bytes([w[8], w[9], w[10], w[11], w[12], w[13], w[14], w[15]])
        //     ))
        //     .collect();

        let mut rng = StdRng::seed_from_u64(rand::random());
        let mut points2d: Vec<Point2<f64>> = repeat_with(|| rng.gen())
            .map(|(x, y)| Point2::new(x, y))
            .take(points_count)
            .collect();

        let points_delaunator: Vec<_> = points2d.iter().map(|p| Point {x: p.x, y: p.y}).collect();

        let mut group = c.benchmark_group(format!("{} points", points_count));
        
        group.bench_function("delaunator-rs", |b| b.iter(|| {
            triangulate(&points_delaunator);
        }));

        group.bench_function("baby_shark", |b| b.iter(|| {
            let mut triangulation = Triangulation2::new();
            triangulation.triangulate(&mut points2d);
        }));
    }
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
