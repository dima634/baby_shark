use crate::{
    algo::utils::cast,
    geometry::primitives::triangle3::Triangle3,
    helpers::aliases::Vec3f,
    mesh::traits::{FromSoup, Triangles},
};
use nalgebra::{Point3, Vector3};
use simba::scalar::SupersetOf;
use std::{
    fs::{File, OpenOptions},
    io::{self, BufReader, BufWriter, Error, ErrorKind, Read, Write},
    mem::size_of,
    ops::Index,
    path::Path,
};

const STL_HEADER_SIZE: usize = 80;

pub struct StlReader {
    // Buffers for reading
    buf32: [u8; size_of::<u32>()],
    buf16: [u8; size_of::<u16>()],
}

///
/// Binary STL reader
///
impl StlReader {
    pub fn new() -> Self {
        Self {
            buf16: [0; size_of::<u16>()],
            buf32: [0; size_of::<u32>()],
        }
    }

    /// Reads mesh from file
    pub fn read_stl_from_file<TMesh>(&mut self, filepath: &Path) -> std::io::Result<TMesh>
    where
        TMesh: FromSoup,
        TMesh::Scalar: SupersetOf<f32>,
    {
        let file = OpenOptions::new().read(true).open(filepath)?;
        let mut reader = BufReader::new(file);

        self.read_stl::<File, TMesh>(&mut reader)
    }

    /// Reads mesh from buffer
    pub fn read_stl<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> std::io::Result<TMesh>
    where
        TBuffer: Read,
        TMesh: FromSoup,
        TMesh::Scalar: SupersetOf<f32>,
    {
        // Read header
        let mut header = [0u8; STL_HEADER_SIZE];
        reader.read_exact(&mut header)?;

        // Read number of triangle
        reader.read_exact(&mut self.buf32)?;
        let number_of_triangles: u32 = u32::from_le_bytes(self.buf32);

        // Faces
        let mut triangles = Vec::with_capacity(number_of_triangles as usize);
        for _ in 0..number_of_triangles {
            self.read_face(reader, &mut triangles)?;
        }

        let casted_triangles = triangles
            .into_iter()
            .map(|triangle| {
                let p1 = triangle.p1();
                let p2 = triangle.p2();
                let p3 = triangle.p3();

                [p1.cast(), p2.cast(), p3.cast()]
            })
            .flatten();

        // Create mesh
        Ok(TMesh::from_triangles_soup(casted_triangles))
    }

    fn read_face<TBuffer: Read>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
        triangles: &mut Vec<Triangle3<f32>>,
    ) -> io::Result<()> {
        // Normal
        self.read_vec3(reader)?;

        // Vertices
        let v1 = self.read_vec3(reader)?;
        let v2 = self.read_vec3(reader)?;
        let v3 = self.read_vec3(reader)?;

        triangles.push(Triangle3::new(v1, v2, v3));

        // Attribute
        reader.read_exact(&mut self.buf16)?;

        Ok(())
    }

    fn read_vec3<TBuffer: Read>(&mut self, reader: &mut BufReader<TBuffer>) -> io::Result<Vec3f> {
        reader.read_exact(&mut self.buf32)?;
        let x = f32::from_le_bytes(self.buf32);

        reader.read_exact(&mut self.buf32)?;
        let y = f32::from_le_bytes(self.buf32);

        reader.read_exact(&mut self.buf32)?;
        let z = f32::from_le_bytes(self.buf32);

        Ok(Vec3f::new(x, y, z))
    }
}

impl Default for StlReader {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

pub struct StlWriter;

impl StlWriter {
    pub fn new() -> Self {
        StlWriter {}
    }

    pub fn write_stl_to_file<TMesh: Triangles>(&self, mesh: &TMesh, path: &Path) -> io::Result<()> {
        let file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)
            .unwrap();
        let mut writer = BufWriter::new(file);

        self.write_stl(mesh, &mut writer)
    }

    pub fn write_stl<TBuffer, TMesh>(
        &self,
        mesh: &TMesh,
        writer: &mut BufWriter<TBuffer>,
    ) -> io::Result<()>
    where
        TBuffer: Write,
        TMesh: Triangles,
    {
        let header = [0u8; STL_HEADER_SIZE];
        writer.write_all(&header)?;

        let faces_count = mesh.triangles().count();
        if faces_count > u32::max_value() as usize {
            return Err(Error::new(ErrorKind::Other, "Mesh is too big for STL"));
        }

        writer.write_all(&(faces_count as u32).to_le_bytes())?;

        for triangle in mesh.triangles() {
            let normal = triangle.get_normal().unwrap_or(Vector3::zeros()); // Write zeros for degenerate faces

            let p1 = cast(triangle.p1()).into();
            let p2 = cast(triangle.p2()).into();
            let p3 = cast(triangle.p3()).into();
            let n = cast(&normal);

            self.write_face(writer, &p1, &p2, &p3, &n)?;
        }

        Ok(())
    }

    fn write_face<TBuffer: Write>(
        &self,
        writer: &mut BufWriter<TBuffer>,
        v1: &Point3<f32>,
        v2: &Point3<f32>,
        v3: &Point3<f32>,
        normal: &Vector3<f32>,
    ) -> io::Result<()> {
        self.write_point(writer, normal)?;
        self.write_point(writer, v1)?;
        self.write_point(writer, v2)?;
        self.write_point(writer, v3)?;
        writer.write_all(&[0; 2])?;

        Ok(())
    }

    fn write_point<TBuffer: Write, TPoint: Index<usize, Output = f32>>(
        &self,
        writer: &mut BufWriter<TBuffer>,
        point: &TPoint,
    ) -> io::Result<()> {
        writer.write_all(&point[0].to_le_bytes())?;
        writer.write_all(&point[1].to_le_bytes())?;
        writer.write_all(&point[2].to_le_bytes())?;

        Ok(())
    }
}

impl Default for StlWriter {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}
