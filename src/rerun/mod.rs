use std::error::Error;

use nalgebra::Vector3;
use rerun::{
    components::{Mesh3D, RawMesh3D, MeshId, LineStrip3D, Vec3D, Radius, ColorRGBA, Transform3D},
    RecordingStream, MsgSender,
};
use crate::mesh::{corner_table::prelude::CornerTableF, traits::{Mesh, TopologicalMesh}};

pub fn log_mesh(
    name: &str,
    _timestep: Option<i64>,
    mesh: &CornerTableF,
    rec_stream: &RecordingStream,
) -> Result<(), Box<dyn Error>> {
    let mesh: Mesh3D = mesh.into();
    
    let msg = MsgSender::new(name)
        .with_component(&[mesh])?;
    msg.send(rec_stream)?;

    Ok(())
}

pub fn log_mesh_as_line_strips (
    name: &str,
    _timestep: Option<i64>,
    mesh: &CornerTableF,
    transform: Option<Transform3D>,
    rec_stream: &RecordingStream,

) -> Result<(), Box<dyn Error>> {
    let mut faces: Vec<Vec<Vec3D>> = Vec::new();

    let mut lines: Vec<LineStrip3D> = Vec::new();
    let mut colors: Vec<ColorRGBA> = Vec::new();
    let color_internal = ColorRGBA::from_rgb(100, 100, 100);
    let color_edge = ColorRGBA::from_rgb(255, 50, 50);
    for edge in mesh.edges() {
        let is_on_boundary = mesh.is_edge_on_boundary(&edge);
        let edge = mesh.edge_positions(&edge);
        let edge = [edge.0, edge.1];
        let positions = edge.iter().map(|p| Vec3D::new(p.x, p.y, p.z)).collect();
        let edge = LineStrip3D(positions);
        let color = if is_on_boundary { color_edge} else {color_internal};
        colors.push(color);

        lines.push(edge);
    }
    let radius = Radius(0.003);
    let mut msg = MsgSender::new(name)
        .with_component(&lines)?
        .with_splat(radius)?
        .with_component(&colors)?;

    if let Some(transform) = transform {
        let transforms: Vec<Transform3D> = lines.iter().map(|_| transform).collect();
        msg = msg.with_component(&transforms)?;
    }
    
    
    msg.send(rec_stream)?;

    Ok(())
}

impl From<&CornerTableF> for Mesh3D {
    fn from(value: &CornerTableF) -> Self {
        let vertices: Vec<f32> = value
            .vertices()
            .map(|vertex| value.vertex_position(&vertex))
            .map(|v| [v.x, v.y, v.z])
            .flatten()
            .collect();

        let indices: Vec<u32> = value
            .faces()
            .map(|face| {
                let (i0, i1, i2) = value.face_vertices(&face);
                [i0 as u32, i1 as u32, i2 as u32]
            })
            .flatten()
            .collect();

        let normals: Vec<f32> = value
            .vertices()
            .map(|vertex| value.vertex_normal(&vertex))
            .map(|vertex| vertex.unwrap_or(Vector3::<f32>::new(0., 1., 0.)))
            .map(|vertex| [vertex.x, vertex.y, vertex.z])
            .flatten()
            .collect();

        let raw_mesh = 
                RawMesh3D {
                    mesh_id: MeshId::random(),
                    vertex_positions: vertices.into(),
                    vertex_colors: None,
                    vertex_normals: Some(normals.into()),
                    indices: Some(indices.into()),
                    albedo_factor: None,
                };
        Mesh3D::Raw(raw_mesh)
    }
}