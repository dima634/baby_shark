use std::path::Path;

use baby_shark::{io::stl::{StlReader, StlWriter}, mesh::{corner_table::prelude::CornerTableF, traits::{Mesh, SplitFaceAtPoint}}};

fn main() {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader.read_stl_from_file(Path::new("./test_files/box.stl")).expect("Read mesh from file");

    let faces: Vec<_> = mesh.faces().collect();

    for face in faces {
        let t = mesh.face_positions(&face);
        mesh.split_face(&face, t.center());
    }

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("./test.stl")).expect("Write to file");
}
