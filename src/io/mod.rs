use crate::{geometry::traits::*, mesh::traits::*};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Read, Write},
    path::Path,
    usize,
};

pub mod obj;
pub mod ply;
pub mod stl;

pub use obj::{ObjReader, ObjWriter};
pub use ply::{PlyReader, PlyWriter};
pub use stl::{StlReader, StlWriter};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildError {
    TooBig,
    InvalidVertex,
    WrongMode,
}

pub trait IndexedBuilder<R: RealNumber, M> {
    fn add_vertex<T: Into<[R; 3]>>(&mut self, vertex: T) -> Result<usize, BuildError>;
    fn add_face(&mut self, v1: usize, v2: usize, v3: usize) -> Result<(), BuildError>;
    fn finish(self) -> Result<M, BuildError>;

    /// Sets the number of vertices in the mesh, this is just a hint for builder
    /// and doest not limit max number of vertices
    fn set_num_vertices(&mut self, count: usize);
    /// Sets the number of faces in the mesh, this is just a hint for builder
    /// and doest not limit max number of faces
    fn set_num_faces(&mut self, count: usize);

    /// Shorthand for adding multiple vertices at once.
    fn add_vertices(
        &mut self,
        vertices: impl Iterator<Item: Into<[R; 3]>>,
    ) -> Result<(), BuildError> {
        if let Some(num_vertices) = vertices.size_hint().1 {
            self.set_num_vertices(num_vertices);
        }

        for vertex in vertices {
            self.add_vertex(vertex)?;
        }

        Ok(())
    }

    /// Shorthand for adding multiple faces at once.
    fn add_faces(&mut self, faces: impl Iterator<Item = [usize; 3]>) -> Result<(), BuildError> {
        if let Some(num_faces) = faces.size_hint().1 {
            self.set_num_faces(num_faces);
        }

        for face in faces {
            self.add_face(face[0], face[1], face[2])?;
        }

        Ok(())
    }
}

pub trait SoupBuilder<R: RealNumber, M> {
    /// Sets the number of faces in the mesh, this is just a hint for builder
    /// and doest not limit max number of faces
    fn set_num_faces(&mut self, count: usize);
    fn add_face<T: Into<[R; 3]>>(&mut self, v1: T, v2: T, v3: T) -> Result<(), BuildError>;
    fn finish(self) -> Result<M, BuildError>;

    /// Shorthand for writing multiple faces at once.
    fn add_faces(
        &mut self,
        mut faces: impl Iterator<Item: Into<[R; 3]>>,
    ) -> Result<(), BuildError> {
        if let Some(num_faces) = faces.size_hint().1.map(|v| v / 3) {
            self.set_num_faces(num_faces);
        }

        loop {
            let Some(v1) = faces.next() else { break };
            let Some(v2) = faces.next() else { break };
            let Some(v3) = faces.next() else { break };

            self.add_face(v1, v2, v3)?;
        }

        Ok(())
    }
}

pub trait Builder {
    type Scalar: RealNumber;
    type Mesh;

    /// Returns a builder for an indexed triangle mesh.
    ///
    /// Use this when you already (or will) maintain a **single deduplicated vertex array** and
    /// faces are specified as triplets of vertex indices. The builder lets you:
    /// - Pre‑hint counts with `set_num_vertices` / `set_num_faces` for capacity reservation
    /// - Push vertices via `add_vertex`, receiving their index
    /// - Add faces via `add_face(index0, index1, index2)` referencing previously added vertices
    ///
    /// Example:
    /// ```ignore
    /// let mut b = MyMesh::builder_indexed();
    /// b.set_num_vertices(n);
    /// for v in vertices {
    ///     b.add_vertex(v)?;
    /// }
    /// for [i0,i1,i2] in faces {
    ///     b.add_face(i0,i1,i2)?;
    /// }
    /// let mesh = b.finish()?;
    /// ```
    fn builder_indexed() -> impl IndexedBuilder<Self::Scalar, Self::Mesh>;

    /// Returns a builder for a triangle soup (non‑indexed) mesh.
    ///
    /// Use this when triangles arrive as **independent triplets of positions**.
    ///
    /// Example:
    /// ```ignore
    /// let mut b = MyMesh::builder_soup();
    /// b.set_num_faces(tris_len);
    /// for (p0,p1,p2) in triangles {
    ///     b.add_face(p0,p1,p2)?;
    /// }
    /// let mesh = b.finish()?;
    /// ```
    fn builder_soup() -> impl SoupBuilder<Self::Scalar, Self::Mesh>;
}

pub trait MeshReader {
    /// Reads mesh from buffer
    fn read_from_buffer<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> Result<TMesh, ReadError>
    where
        TBuffer: Read,
        TMesh: Builder<Mesh = TMesh>;

    /// Reads mesh from file
    fn read_from_file<TMesh: Builder<Mesh = TMesh>>(
        &mut self,
        filepath: &Path,
    ) -> Result<TMesh, ReadError> {
        let file = OpenOptions::new().read(true).open(filepath)?;
        let mut reader = BufReader::new(file);

        self.read_from_buffer::<File, TMesh>(&mut reader)
    }
}

pub trait MeshWriter {
    fn write_to_buffer<TBuffer, TMesh>(
        &self,
        mesh: &TMesh,
        writer: &mut BufWriter<TBuffer>,
    ) -> std::io::Result<()>
    where
        TBuffer: Write,
        TMesh: TriangleMesh;

    fn write_to_file<TMesh: TriangleMesh>(&self, mesh: &TMesh, path: &Path) -> std::io::Result<()> {
        let file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)?;
        let mut writer = BufWriter::new(file);

        self.write_to_buffer(mesh, &mut writer)
    }
}

#[derive(Debug)]
pub enum ReadError {
    IO(std::io::Error),
    FilepathHasNoExtension,
    UnsupportedFormat(String),
    Malformed,
    FailedToBuildMesh(BuildError),
}

impl From<BuildError> for ReadError {
    #[inline]
    fn from(err: BuildError) -> Self {
        ReadError::FailedToBuildMesh(err)
    }
}

impl From<std::io::Error> for ReadError {
    #[inline]
    fn from(err: std::io::Error) -> Self {
        ReadError::IO(err)
    }
}

pub fn read_from_file<TMesh: Builder<Mesh = TMesh>>(filepath: &Path) -> Result<TMesh, ReadError> {
    let ext = filepath
        .extension()
        .and_then(|ext| if ext == "" { None } else { Some(ext) })
        .ok_or(ReadError::FilepathHasNoExtension)?;
    let ext_uppercase = ext.to_string_lossy().to_uppercase();

    match ext_uppercase.as_str() {
        "STL" => StlReader::default().read_from_file(filepath),
        "OBJ" => ObjReader::default().read_from_file(filepath),
        "PLY" => PlyReader::default().read_from_file(filepath),
        _ => Err(ReadError::UnsupportedFormat(ext_uppercase)),
    }
}

#[derive(Debug)]
pub enum WriteError {
    IO(std::io::Error),
    FilepathHasNoExtension,
    UnsupportedFormat(String),
}

pub fn write_to_file<TMesh: TriangleMesh>(mesh: &TMesh, path: &Path) -> Result<(), WriteError> {
    let ext = path
        .extension()
        .and_then(|ext| if ext == "" { None } else { Some(ext) })
        .ok_or(WriteError::FilepathHasNoExtension)?;
    let ext_uppercase = ext.to_string_lossy().to_uppercase();

    match ext_uppercase.as_str() {
        "STL" => StlWriter::default()
            .write_to_file(mesh, path)
            .map_err(WriteError::IO),
        "OBJ" => ObjWriter::default()
            .write_to_file(mesh, path)
            .map_err(WriteError::IO),
        "PLY" => PlyWriter::default()
            .write_to_file(mesh, path)
            .map_err(WriteError::IO),
        _ => Err(WriteError::UnsupportedFormat(ext_uppercase)),
    }
}

#[cfg(test)]
mod tests {
    use crate::{io::read_from_file, mesh::polygon_soup::data_structure::PolygonSoup};
    use std::path::Path;

    #[test]
    fn test_read_from_file() {
        type Mesh = PolygonSoup<f64>;

        let mesh = read_from_file::<Mesh>(Path::new("assets/box.stl")).expect("should read mesh");
        assert_eq!(mesh.vertices().count(), 36); // Polygon soup duplicates vertices
        assert_eq!(mesh.faces().count(), 12);

        let mesh = read_from_file::<Mesh>(Path::new("assets/box.obj")).expect("should read mesh");
        assert_eq!(mesh.vertices().count(), 36);
        assert_eq!(mesh.faces().count(), 12);

        let mesh = read_from_file::<Mesh>(Path::new("assets/box.ply")).expect("should read mesh");
        assert_eq!(mesh.vertices().count(), 36);
        assert_eq!(mesh.faces().count(), 12);
    }
}
