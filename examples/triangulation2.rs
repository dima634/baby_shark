use std::{path::Path, iter::repeat_with};

use nalgebra::Point2;
use rand::{rngs::StdRng, SeedableRng, Rng};

use baby_shark::{
    triangulation::delaunay::Triangulation2, 
    io::stl::StlWriter, 
    mesh::polygon_soup::data_structure::PolygonSoup
};

fn main() {
    let mut rng = StdRng::seed_from_u64(rand::random());
    let mut points2d: Vec<Point2<f64>> = repeat_with(|| rng.gen())
        .map(|(x, y)| Point2::new(x, y))
        .take(100_000)
        .collect();

    let mut triangulation = Triangulation2::new();
    triangulation.triangulate(&mut points2d);
    let tri = triangulation.triangles().clone();


    let mut mesh = PolygonSoup::<f64>::new();
    for idx in (0..tri.len()).step_by(3) {
        mesh.add_face(
            triangulation.vertex_position(tri[idx]).to_homogeneous().into(), 
            triangulation.vertex_position(tri[idx + 1]).to_homogeneous().into(), 
            triangulation.vertex_position(tri[idx + 2]).to_homogeneous().into()
        );
    }
    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("triangulation.stl")).expect("Must save to file");
}
