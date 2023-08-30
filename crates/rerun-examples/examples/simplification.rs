use std::path::PathBuf;

use baby_shark::{
    decimation::{edge_decimation::ConstantErrorDecimationCriteria, prelude::EdgeDecimator},
    io::stl::{StlReader, StlWriter}, mesh::{corner_table::prelude::CornerTableF, traits::Mesh}, rerun::{log_mesh, log_mesh_as_line_strips},
};
use rerun::{external::re_log, RecordingStream, components::Transform3D, transform::TranslationRotationScale3D};


#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short, required = true)]
    input_file: PathBuf,

    #[clap(long, short, required = true)]
    errors: Vec<f32>,

    #[clap(long, default_value = "5.0")]
    ydiff: f32,

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

    let _ = log_mesh_as_line_strips("original", None, &mesh, None, rec_stream);

    println!("{:?}", args.errors);

    let mut cy = 0.0;

    let mut errors = args.errors.clone();
    errors.sort_by((|a, b| a.partial_cmp(b).unwrap()));

    for error in errors {
        cy += args.ydiff;
        let decimation_criteria = ConstantErrorDecimationCriteria::new(error);

        let mut decimator = EdgeDecimator::new().decimation_criteria(decimation_criteria).preserve_edges_on_boundary(true);
        let mut cloned = mesh.clone_remap();
        decimator.decimate(&mut cloned);

        let decimated = cloned.clone_remap();


        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some([0., cy, 0.].into()),
            rotation: None,
            scale: None,
        });


        let _ = log_mesh_as_line_strips(&format!("simplified-{error}"), None, &decimated, Some(transform), rec_stream);
    }

    // let writer = StlWriter::new();
    // writer
    //     .write_stl_to_file(&mesh, &args.output_file)
    //     .expect("Save mesh to STL");    
    Ok(())
}


