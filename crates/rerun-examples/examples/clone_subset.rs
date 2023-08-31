use std::path::PathBuf;

use baby_shark::{
    geometry::primitives::box3::Box3,
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

use rerun_examples::helpers::*;

#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short, required = true)]
    input_file: PathBuf,

    #[clap(long, short, default_value = "0,0,0")]
    origin: String,

    #[clap(long, short, default_value = "10,10,10")]
    extent: String,

    #[clap(long, default_value = "10.0")]
    result_offset: f32,

    #[clap(long, short, default_value = "false")]
    wireframe: bool,

    #[clap(long, default_value = "10.0")]
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

    let color = &parse_colors(&args.color);

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

    let cy = 0.0;

    let origin = parse_point(&args.origin);
    let extent = parse_vector(&args.extent);
    let half_extent = extent / 2.;

    let mins = origin - half_extent;
    let maxs = origin + half_extent;

    let aabb = Box3::new(mins, maxs);

    let subset = mesh.clone_subset(|mesh, face| {
        let points = mesh.face_positions(face);
        return aabb.contains_point(points.p1())
            || aabb.contains_point(points.p2())
            || aabb.contains_point(points.p3());
    });

    let transform = Transform3D::new(TranslationRotationScale3D {
        translation: Some(Vec3D::new(0., args.result_offset, 0.)),
        rotation: None,
        scale: None,
    });

    let _ = log_mesh(
        &format!("subset"),
        None,
        &subset,
        Some(transform),
        Some(color),
        rec_stream,
    );

    if args.wireframe {
        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some(Vec3D::new(
                args.wireframe_offset.clone(),
                args.result_offset,
                0.,
            )),
            rotation: None,
            scale: None,
        });

        let _ = log_mesh_as_line_strips(
            &format!("subset-wireframe"),
            None,
            &subset,
            Some(transform),
            None,
            rec_stream,
        );
    }

    Ok(())
}
