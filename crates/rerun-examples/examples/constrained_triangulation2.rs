fn main() {}
//     use std::{iter::repeat_with, path::Path};

// use baby_shark::e::Point2;
// use rand::{rngs::StdRng, Rng, RngCore, SeedableRng};

// use baby_shark::{
//     io::stl::StlWriter, mesh::polygon_soup::data_structure::PolygonSoup,
//     triangulation::constrained_delaunay::ConstrainedTriangulation2,
// };

// fn main() {
//     let mut rng = StdRng::seed_from_u64(rand::random());
//     let points2d: Vec<Point2<f32>> = repeat_with(|| rng.gen())
//         .take(100)
//         .map(|(x, y)| Point2::new(x, y))
//         .collect();

//     let mut triangulation = ConstrainedTriangulation2::new();
//     triangulation.set_points(&points2d);

//     for _ in 0..10 {
//         let v1 = rng.next_u64() as usize % points2d.len();
//         let v2 = rng.next_u64() as usize % points2d.len();

//         if v1 == v2 {
//             continue;
//         }

//         triangulation.insert_constrained_edge(v1, v2);
//     }

//     let tri = triangulation.triangles();
//     let mut mesh = PolygonSoup::<f32>::new();
//     for idx in (0..tri.len()).step_by(3) {
//         mesh.add_face(
//             triangulation.points()[tri[idx]].to_homogeneous().into(),
//             triangulation.points()[tri[idx + 1]].to_homogeneous().into(),
//             triangulation.points()[tri[idx + 2]].to_homogeneous().into(),
//         );
//     }
//     let writer = StlWriter::new();
//     writer
//         .write_stl_to_file(&mesh, Path::new("constrained_triangulation.stl"))
//         .expect("Must save to file");
// }
