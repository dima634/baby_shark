use std::{iter::repeat_with, path::PathBuf};

use baby_shark::{
    algo::merge_points::merge_points,
    exports::nalgebra::Point2,
    mesh::{corner_table::prelude::CornerTableF, traits::Mesh},
    rerun::{log_mesh, log_mesh_as_line_strips},
};
use baby_shark_rerun_examples::helpers::parse_color;
use rand::{rngs::StdRng, Rng, RngCore, SeedableRng};

use baby_shark::{
    io::stl::StlWriter, mesh::polygon_soup::data_structure::PolygonSoup,
    triangulation::constrained_delaunay::ConstrainedTriangulation2,
};
use rerun::{
    components::{Transform3D, Vec3D},
    external::re_log,
    transform::TranslationRotationScale3D,
    RecordingStream,
};

#[derive(Debug, clap::Parser)]
#[clap(author, version, about)]
struct Args {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,

    #[clap(long, short)]
    output_file: Option<PathBuf>,

    #[clap(long, short, default_value = "false")]
    wireframe: bool,

    #[clap(long, default_value = "2.0")]
    wireframe_offset: f32,

    #[clap(long, short, default_value = "0.001")]
    wireframe_width: f32,

    #[clap(long, short, default_value = "10,133,199")]
    color: String,
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
    let mut rng = StdRng::seed_from_u64(rand::random());
    let points2d: Vec<Point2<f32>> = repeat_with(|| rng.gen())
        .take(100)
        .map(|(x, y)| Point2::new(x, y))
        .collect();

    let mut triangulation = ConstrainedTriangulation2::new();
    triangulation.set_points(&points2d);

    for _ in 0..10 {
        let v1 = rng.next_u64() as usize % points2d.len();
        let v2 = rng.next_u64() as usize % points2d.len();

        if v1 == v2 {
            continue;
        }

        triangulation.insert_constrained_edge(v1, v2);
    }

    let tri = triangulation.triangles();
    let mut mesh = PolygonSoup::<f32>::new();
    for idx in (0..tri.len()).step_by(3) {
        mesh.add_face(
            triangulation.points()[tri[idx]].to_homogeneous().into(),
            triangulation.points()[tri[idx + 1]].to_homogeneous().into(),
            triangulation.points()[tri[idx + 2]].to_homogeneous().into(),
        );
    }

    // TODO: move this into From<PolygonSoup>
    let vertices = mesh
        .faces()
        .map(|face| {
            let face = mesh.face_positions(&face);
            [face.p1().clone(), face.p2().clone(), face.p3().clone()]
        })
        .flatten()
        .collect();
    let indexed_vertices = merge_points(&vertices);
    let mesh = CornerTableF::from_vertices_and_indices(
        indexed_vertices.points.as_slice(),
        indexed_vertices.indices.as_slice(),
    );

    let _ = log_mesh(
        "constrained_triangulation",
        None,
        &mesh,
        None,
        Some(&parse_color(&args.color)),
        rec_stream,
    );

    if args.wireframe {
        let transform = Transform3D::new(TranslationRotationScale3D {
            translation: Some(Vec3D::new(args.wireframe_offset, 0., 0.)),
            rotation: None,
            scale: None,
        });
        let _ = log_mesh_as_line_strips(
            "constrained_triangulation2_wireframe",
            None,
            &mesh,
            Some(transform),
            Some(args.wireframe_width),
            rec_stream,
        );
    }

    if let Some(output_file) = &args.output_file {
        let writer = StlWriter::new();
        writer
            .write_stl_to_file(&mesh, output_file.as_path())
            .expect("Must save to file");
    }

    Ok(())
}
