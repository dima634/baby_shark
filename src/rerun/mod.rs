use std::error::Error;

use nalgebra::Vector3;
use rerun::{
    components::{Mesh3D, RawMesh3D, MeshId},
    RecordingStream, MsgSender,
};
use crate::mesh::{corner_table::prelude::CornerTableF, traits::Mesh};

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

            println!("{}, {}, {}", vertices.len(), indices.len(), normals.len());
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
