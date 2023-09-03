use std::path::PathBuf;

use baby_shark::{
    decimation::{edge_decimation::ConstantErrorDecimationCriteria, prelude::EdgeDecimator},
    io::stl::StlReader,
    mesh::{corner_table::prelude::CornerTableF, traits::Mesh},
    rerun::{log_mesh, log_mesh_as_line_strips},
};
use rerun::{
    components::{Transform3D, Vec3D},
    external::re_log,
    transform::TranslationRotationScale3D,
    RecordingStream,
};

use baby_shark_rerun_examples::helpers::*;

#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short, required = true)]
    input_file: PathBuf,

    #[clap(long, short, required = true)]
    errors: Vec<f32>,

    #[clap(long, default_value = "1.0")]
    mesh_offset: f32,

    #[clap(long, short, default_value = "false")]
    wireframe: bool,

    #[clap(long, default_value = "1.0")]
    wireframe_offset: f32,

    #[clap(long, short, default_value = "10,133,199")]
    color: String,

    #[clap(long, default_value = "false")]
    log_original_wireframe: bool,
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

    let color = &parse_color(&args.color);

    let _ = log_mesh("original", None, &mesh, None, Some(color), rec_stream);

    if args.log_original_wireframe {
        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some(Vec3D::new(args.wireframe_offset.clone(), 0., 0.)),
            rotation: None,
            scale: None,
        });

        let _ = log_mesh_as_line_strips(
            &format!("simplified-wireframe"),
            None,
            &mesh,
            Some(transform),
            None,
            rec_stream,
        );
    }

    println!("{:?}", args.errors);

    let mut cy = 0.0;

    let mut errors = args.errors.clone();
    errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

    for error in errors {
        cy += args.mesh_offset;
        let decimation_criteria = ConstantErrorDecimationCriteria::new(error);

        let mut decimator = EdgeDecimator::new()
            .decimation_criteria(decimation_criteria)
            .keep_boundary(true);
        let mut cloned = mesh.clone_remap();
        decimator.decimate(&mut cloned);

        let decimated = cloned.clone_remap();

        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some(Vec3D::new(0., cy, 0.)),
            rotation: None,
            scale: None,
        });

        let _ = log_mesh(
            &format!("simplified-{error}"),
            None,
            &decimated,
            Some(transform),
            Some(color),
            rec_stream,
        );
        if args.wireframe {
            let transform = Transform3D::new(TranslationRotationScale3D {
                translation: Some(Vec3D::new(args.wireframe_offset.clone(), cy, 0.)),
                rotation: None,
                scale: None,
            });
            let _ = log_mesh_as_line_strips(
                &format!("simplified-wireframe-{error}"),
                None,
                &decimated,
                Some(transform),
                None,
                rec_stream,
            );
        }
    }

    Ok(())
}
