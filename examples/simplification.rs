use std::path::Path;

use baby_shark::{
    io::stl::{StlReader, StlWriter}, 
    decimation::{prelude::EdgeDecimator, edge_decimation::ConstantErrorDecimationCriteria}, 
    mesh::corner_table::prelude::CornerTableF
};

fn main() {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader.read_stl_from_file(Path::new("./test_files/violin.stl")).expect("Read mesh from STL");

    let decimation_criteria = ConstantErrorDecimationCriteria::new(0.01f32);
    
    let mut decimator = EdgeDecimator::new().decimation_criteria(decimation_criteria);
    decimator.decimate(&mut mesh);

    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new("decimated.stl")).expect("Save mesh to STL");
}
