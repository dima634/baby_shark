use std::path::PathBuf;

use baby_shark::{
    decimation::{edge_decimation::BoundingSphereDecimationCriteria, prelude::EdgeDecimator},
    io::stl::{StlReader, StlWriter},
    mesh::{corner_table::prelude::CornerTableF, traits::Mesh},
    rerun::{log_mesh, log_mesh_as_line_strips},
};
use rerun::{
    components::{Transform3D, Vec3D},
    external::re_log,
    transform::TranslationRotationScale3D,
    RecordingStream,
};
use rerun_examples::helpers::*;

#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short, required = true)]
    input_file: PathBuf,

    #[clap(long, short)]
    output_file: Option<PathBuf>,

    #[clap(long, default_value = "10.0")]
    result_offset: f32,

    #[clap(long, default_value = "5.0")]
    wireframe_offset: f32,

    #[clap(long, short, default_value = "false")]
    wireframe: bool,

    #[clap(long, short, default_value = "0.01")]
    wireframe_width: f32,

    #[clap(long, short, default_value = "10,10,10")]
    color: String,

    #[clap(long, short, default_value = "0,0,0")]
    origin: String,

    #[clap(long, short, default_value = "5:0.001,10:0.1,15:1.0")]
    radii_error: String,
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
    let mesh: CornerTableF = reader
        .read_stl_from_file(&args.input_file)
        .expect("Read mesh from STL");

    let color = &parse_colors(&args.color);

    let _ = log_mesh("original", None, &mesh, None, Some(color), rec_stream);

    let origin = parse_point(&args.origin);
    let radii_error_map = parse_radii_errors(&args.radii_error);

    let criteria = BoundingSphereDecimationCriteria::new(origin, radii_error_map);

    let mut decimator = EdgeDecimator::new()
        .decimation_criteria(criteria)
        .preserve_edges_on_boundary(true);
    let mut cloned = mesh.clone_remap();
    decimator.decimate(&mut cloned);

    let decimated = cloned.clone_remap();

    let transform = Transform3D::new(TranslationRotationScale3D {
        translation: Some(Vec3D::new(0., args.result_offset, 0.)),
        rotation: None,
        scale: None,
    });

    let _ = log_mesh(
        &format!("simplified"),
        None,
        &decimated,
        Some(transform),
        Some(color),
        rec_stream,
    );
    if args.wireframe {
        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some(Vec3D::new(args.wireframe_offset, args.result_offset, 0.)),
            rotation: None,
            scale: None,
        });
        let _ = log_mesh_as_line_strips(
            &format!("simplified-wireframe"),
            None,
            &decimated,
            Some(transform),
            Some(args.wireframe_width),
            rec_stream,
        );
    }

    if let Some(output_file) = &args.output_file {
        let writer = StlWriter::new();
        writer
            .write_stl_to_file(&decimated, output_file)
            .expect("Save mesh to STL");
    }

    Ok(())
}
