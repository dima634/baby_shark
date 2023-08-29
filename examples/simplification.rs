use std::path::{Path, PathBuf};

use baby_shark::{
    decimation::{edge_decimation::ConstantErrorDecimationCriteria, prelude::EdgeDecimator},
    io::stl::{StlReader, StlWriter}, mesh::{corner_table::prelude::CornerTableF, traits::Mesh}, rerun::log_mesh,
};
use rerun::{external::re_log, RecordingStream};


#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short, required = true)]
    input_file: PathBuf,

    #[clap(long, short, required = true)]
    output_file: PathBuf,

}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    re_log::setup_native_logging();

    use clap::Parser as _;
    let args = Args::parse();

    let default_enabled = true;
    args.rerun
        .clone()
        .run("replay", default_enabled, move |session| {
            run(&session, &args).unwrap();
        })?;

    Ok(())
}

fn run(rec_stream: &RecordingStream, args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut reader = StlReader::new();
    let mut mesh: CornerTableF = reader
        .read_stl_from_file(&args.input_file)
        .expect("Read mesh from STL");

    let _ = log_mesh("original", None, &mesh, rec_stream);

    let decimation_criteria = ConstantErrorDecimationCriteria::new(0.01f32);

    let mut decimator = EdgeDecimator::new().decimation_criteria(decimation_criteria);
    decimator.decimate(&mut mesh);

    for face_idx in mesh.faces() {
        let face = mesh.face_vertices(&face_idx);
        if face.0 > mesh.vertices().count() || face.1 > mesh.vertices().count() || face.2 > mesh.vertices().count() {
            println!("faces index out of bounds: pl {} {} {} {}", mesh.vertices().count(), face.0, face.1, face.2);
        }
    }

    let _ = log_mesh("simplified", None, &mesh, rec_stream);


    let writer = StlWriter::new();
    writer.write_stl_to_file(&mesh, Path::new(&args.output_file))?;

    Ok(())
}


