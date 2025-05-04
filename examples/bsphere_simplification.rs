use std::path::Path;

use baby_shark::{
    decimation::{BoundingSphereDecimationCriteria, EdgeDecimator},
    io::stl::{StlReader, StlWriter},
    mesh::corner_table::CornerTableF,
};
use nalgebra::Vector3;

fn main() {
    let mut reader = StlReader::new();
    let mut args = std::env::args();
    args.next();
    let path = args.next().expect("Enter an input file");
    let output = args.next().expect("Enter an output file");

    let mut mesh: CornerTableF = reader
        .read_stl_from_file(Path::new(&path))
        .expect("Read mesh from STL");

    let origin = Vector3::<f32>::zeros();
    let radii_error_map = vec![(5.0f32, 0.0001f32), (10.0f32, 0.001f32), (15.0f32, 0.8f32)];

    let criteria = BoundingSphereDecimationCriteria::new(origin, radii_error_map);

    let mut decimator = EdgeDecimator::new().decimation_criteria(criteria);
    decimator.decimate(&mut mesh);

    let writer = StlWriter::new();
    writer
        .write_stl_to_file(&mesh, Path::new(&output))
        .expect("Save mesh to STL");
}
