use std::path::Path;

use baby_shark::{
    decimation::{
        edge_decimation::{BoundingSphere, BoundingSphereMaxError},
        prelude::EdgeDecimator,
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

    let origin = Point3::<f32>::new(0.0, 0.0, 0.0);
    let origin2 = origin.clone();
    let bounding_spheres = vec![
        (
            BoundingSphere {
                origin,
                radius: 5.0,
            },
            0.0001,
        ),
        (
            BoundingSphere {
                origin: origin2,
                radius: 15.0,
            },
            0.001,
        ),
        (
            BoundingSphere {
                origin: origin2,
                radius: 40.0,
            },
            0.8,
        ),
    ];

    let max_error = BoundingSphereMaxError::new(bounding_spheres);

    let mut decimator = EdgeDecimator::new().max_error(Some(max_error));
    decimator.decimate(&mut mesh);

    let writer = StlWriter::new();
    writer
        .write_stl_to_file(&mesh, Path::new(&output))
        .expect("Save mesh to STL");
}
