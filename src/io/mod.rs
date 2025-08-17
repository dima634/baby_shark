use crate::{geometry::traits::*, mesh::traits::*};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Read, Write},
    path::Path, usize,
};

pub mod obj;
pub mod stl;

pub use stl::{StlReader, StlWriter};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildMode {
    Indexed,
    Soup,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildError {
    TooBig,
    InvalidVertex,
    WrongMode,
}

pub trait MeshBuilder<R: RealNumber, M> {
    fn add_face<T: Into<[R; 3]>>(&mut self, v1: T, v2: T, v3: T) -> Result<(), BuildError>;

    fn add_vertex<T: Into<[R; 3]>>(&mut self, vertex: T) -> Result<usize, BuildError>;
    fn add_face_indexed(&mut self, v1: usize, v2: usize, v3: usize) -> Result<(), BuildError>;

    /// Sets the number of vertices in the mesh, this is just a hint for builder 
    /// and doest not limit max number of vertices
    fn set_num_vertices(&mut self, count: usize);
    /// Sets the number of faces in the mesh, this is just a hint for builder
    /// and doest not limit max number of faces
    fn set_num_faces(&mut self, count: usize);

    fn finish(self) -> Result<M, BuildError>;
}

pub trait MeshReader {
    /// Reads mesh from buffer
    fn read_from_buffer<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> std::io::Result<TMesh>
    where
        TBuffer: Read,
        TMesh: FromSoup;

    /// Reads mesh from file
    fn read_from_file<TMesh: FromSoup>(&mut self, filepath: &Path) -> std::io::Result<TMesh> {
        let file = OpenOptions::new().read(true).open(filepath)?;
        let mut reader = BufReader::new(file);

        self.read_from_buffer::<File, TMesh>(&mut reader)
    }
}

pub trait CreateBuilder {
    type Scalar: RealNumber;
    type Mesh;

    fn builder(mode: BuildMode) -> impl MeshBuilder<Self::Scalar, Self::Mesh>;
}

pub trait MeshWriter {
    fn write_to_buffer<TBuffer, TMesh>(
        &self,
        mesh: &TMesh,
        writer: &mut BufWriter<TBuffer>,
    ) -> std::io::Result<()>
    where
        TBuffer: Write,
        TMesh: Triangles;

    fn write_to_file<TMesh: Triangles>(&self, mesh: &TMesh, path: &Path) -> std::io::Result<()> {
        let file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)?;
        let mut writer = BufWriter::new(file);

        self.write_to_buffer(mesh, &mut writer)
    }
}

pub enum ReadMeshError {
    IO(std::io::Error),
    UnsupportedFormat(String),
}

pub fn read_from_file<TMesh: FromSoup>(filepath: &Path) -> Result<TMesh, ReadMeshError> {
    let file = OpenOptions::new()
        .read(true)
        .open(filepath)
        .map_err(ReadMeshError::IO)?;
    let mut reader = BufReader::new(file);

    todo!()
}
