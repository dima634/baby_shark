use std::path::Path;

use baby_shark::{
    decimation::{
        prelude::EdgeDecimator, edge_decimation::BoundingSphereDecimationCriteria,
    },
    io::stl::{StlReader, StlWriter},
    mesh::corner_table::prelude::CornerTableF,
};

use nalgebra::Point3;

fn main() {
    let mut reader = StlReader::new();
    let mut args = std::env::args();
    args.next();
    let path = args.next().expect("Enter an input file");
    let output = args.next().expect("Enter an output file");

    let mut mesh: CornerTableF = reader
        .read_stl_from_file(Path::new(&path))
        .expect("Read mesh from STL");

    let origin = Point3::<f32>::new(5.0, 0.0, 0.0);
    let radii_errors = vec![
        (10.0f32, 0.0001f32),
        (15.0f32, 0.8f32),
        (40.0f32, 0.8f32),
    ];

    let criterion = BoundingSphereDecimationCriteria::new(origin, radii_errors);

    let mut decimator = EdgeDecimator::new().edge_decimation_criterion(Some(criterion));
    decimator.decimate(&mut mesh);

    let writer = StlWriter::new();
    writer
        .write_stl_to_file(&mesh, Path::new(&output))
        .expect("Save mesh to STL");
}
