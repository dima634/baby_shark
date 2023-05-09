use std::{path::Path, iter::repeat_with, io::{BufWriter, Write, BufReader, Read}, fs::File};

use baby_shark::{triangulation::delaunay::Triangulation2, io::stl::StlWriter, mesh::polygon_soup::data_structure::PolygonSoup};
use nalgebra::{Point2, Point3};
use rand::{rngs::StdRng, SeedableRng, Rng};

fn main() {
    let mut triangulation = Triangulation2::new();
    // let points2d = vec![
    //     Point2::new(1.0, 2.0),
    //     Point2::new(5.0, 1.0),
    //     Point2::new(8.0, 6.0),
    //     Point2::new(2.0, 8.0)
    // ];

    // let mut points2d = vec![
    //     Point2::new(4.0, 0.0),
    //     Point2::new(4.0, 1.0),
    //     Point2::new(4.0, 2.0),
    //     Point2::new(4.0, 3.0),
    //     Point2::new(3.0, 0.0),
    //     Point2::new(3.0, 1.0),
    //     Point2::new(3.0, 2.0),
    //     Point2::new(3.0, 3.0),
    //     Point2::new(2.0, 0.0),
    //     Point2::new(2.0, 1.0),
    //     Point2::new(2.0, 2.0),
    //     Point2::new(2.0, 3.0),
    //     Point2::new(1.0, 0.0),
    //     Point2::new(1.0, 1.0),
    //     Point2::new(1.0, 2.0),
    //     Point2::new(1.0, 3.0),
    //     Point2::new(-1.0, 0.0),
    // ];

    // let mut rng = StdRng::seed_from_u64(rand::random());
    // let mut points2d: Vec<Point2<f64>> = repeat_with(|| rng.gen())
    //     .map(|(x, y)| Point2::new(x, y))
    //     .take(100_000)
    //     .collect();

    // let mut writer = BufWriter::new(File::create("points.xy").expect("Open"));
    // writer.write_all(points2d.as_slice().iter().map(|p| [p.x.to_le_bytes(), p.y.to_le_bytes()].concat()).flatten().collect::<Vec<u8>>().as_slice()).expect("write");

    // let buf = points2d.as_slice().iter().map(|p| [p.x.to_le_bytes(), p.y.to_le_bytes()].concat()).flatten().collect::<Vec<u8>>().as_slice();

    let points = [[0.48984146, 0.4899361], [0.4463194, 0.45261556], [0.42847013, 0.42460257], [0.41823488, 0.57288224], [0.5913105, 0.45535183], [0.53855276, 0.5922733], [0.37710214, 0.5732515], [0.5043943, 0.6273088], [0.34420383, 0.51125544], [0.62980384, 0.44524848], [0.34035563, 0.43844408], [0.61331505, 0.35406935], [0.61050564, 0.34783804], [0.66835, 0.5447868], [0.32081836, 0.3943385], [0.4718566, 0.2961123], [0.58064073, 0.66067904], [0.6851884, 0.48713744], [0.29649615, 0.57779795], [0.63608783, 0.6429018], [0.46383494, 0.27027756], [0.70931464, 0.46491158], [0.7052269, 0.55955833], [0.54671764, 0.25920832], [0.6284604, 0.68279934], [0.3177119, 0.3153975], [0.42712665, 0.7265839], [0.56969875, 0.7230318], [0.49226338, 0.7405513], [0.4741112, 0.74244386], [0.2804165, 0.33925468], [0.29501998, 0.66089964], [0.6637637, 0.6773343], [0.46313453, 0.74667466], [0.71958226, 0.37372464], [0.2911582, 0.31229526], [0.43222797, 0.77797765], [0.71959144, 0.3079893], [0.76890755, 0.59576356], [0.24977851, 0.28614485], [0.3248073, 0.20827317], [0.16804123, 0.57080585], [0.15872717, 0.5225644], [0.21868831, 0.6842079], [0.35850996, 0.17893916], [0.26844138, 0.23370874], [0.18464863, 0.64404595], [0.18881321, 0.30391115], [0.13282919, 0.49237096], [0.5348515, 0.84225017], [0.7661881, 0.7145568], [0.82922363, 0.6159804], [0.8447025, 0.41357028], [0.80576605, 0.66857046], [0.65735865, 0.1653955], [0.4404143, 0.8562609], [0.12434751, 0.56717086], [0.8447379, 0.38467562], [0.4579938, 0.11322808], [0.10814023, 0.48565876], [0.66940445, 0.15512234], [0.18147635, 0.71670175], [0.17786211, 0.72111464], [0.12957686, 0.65061164], [0.5351382, 0.088799596], [0.6344292, 0.8699391], [0.8590333, 0.6721916], [0.39739162, 0.06835717], [0.32444948, 0.8887432], [0.114165425, 0.2647937], [0.16959798, 0.18581927], [0.039387226, 0.498999], [0.70789284, 0.87855], [0.06639743, 0.31261855], [0.33921427, 0.053124905], [0.3961032, 0.02758801], [0.11840737, 0.7837957], [0.014104009, 0.51041526], [0.6770156, 0.92879564], [0.1100536, 0.78631395], [0.73594517, 0.072675765], [0.9592681, 0.6101808], [0.9563696, 0.63878834], [0.9551344, 0.31495094], [0.6514476, 0.96371853], [0.860139, 0.14633244], [0.7776793, 0.91099083], [0.86620057, 0.14327657], [0.06995958, 0.18052047], [0.79034364, 0.059315145], [0.023816466, 0.22163707], [0.056708217, 0.16669017], [0.7203423, 0.9783333], [0.23453873, 0.9738017], [0.78757405, 0.022196889], [0.833493, 0.92673594], [0.1371184, 0.036289155], [0.021484733, 0.14100307], [0.9737798, 0.10746962], [0.95703167, 0.9444567]];
    let mut points2d: Vec<_> = points.iter().map(|p| Point2::new(p[0], p[1])).collect();

    // let mut reader = BufReader::new(File::open("benches/points.xy").expect("To open point dataset"));
    // let mut points_buf = Vec::new();
    // reader.read_to_end(&mut points_buf).expect("To read file");

    // let mut points2d: Vec<Point2<f64>> = points_buf.chunks(8 * 2)
    //     .map(|w| {
    //         return Point2::new(
    //             f64::from_le_bytes([w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]]), 
    //             f64::from_le_bytes([w[8], w[9], w[10], w[11], w[12], w[13], w[14], w[15]])
    //         );
    //     })
    //     .collect();

    triangulation.triangulate(&mut points2d);
    // assert!(triangulation.is_delaunay());
    let tri = triangulation.triangles().clone();
    let pts: Vec<_> = points2d.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect();


    let mut mesh = PolygonSoup::<f64>::new();
    for idx in (0..tri.len()).step_by(3) {
        mesh.add_face(
            triangulation.v_pos(tri[idx]).to_homogeneous().into(), 
            triangulation.v_pos(tri[idx + 1]).to_homogeneous().into(), 
            triangulation.v_pos(tri[idx + 2]).to_homogeneous().into()
        );
    }
    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("test.stl")).expect("Must");

    // Delaunator
    // let points2d: Vec<_> = points.iter().map(|p| delaunator::Point{ x: p[0], y: p[1]}).collect();
    // let tri = delaunator::triangulate(&points2d);

    // let pts: Vec<_> = points2d.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect();
    // let mut mesh = PolygonSoup::<f64>::new();
    // for idx in (0..tri.triangles.len()).step_by(3) {
    //     mesh.add_face(
    //         pts[tri.triangles[idx + 2]], 
    //         pts[tri.triangles[idx + 1]], 
    //         pts[tri.triangles[idx]]
    //     );
    // }
    // let writer = StlWriter::new();
    // writer.write_stl_to_file(&mesh, Path::new("delaunator.stl")).expect("Must");
    
}
