use crate::{
    geometry::{primitives::triangle3::Triangle3, traits::*},
    helpers::aliases::Vec3f,
    io::*,
    mesh::traits::Triangles,
};
use nalgebra::{Point3, Vector3};
use std::{
    io::{self, BufReader, BufWriter, Error, ErrorKind, Read, Write},
    mem::size_of,
    ops::Index,
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

    fn read_face<TBuffer: Read>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> io::Result<Triangle3<f32>> {
        // Normal
        self.read_vec3(reader)?;

        // Vertices
        let v1 = self.read_vec3(reader)?;
        let v2 = self.read_vec3(reader)?;
        let v3 = self.read_vec3(reader)?;

        // Attribute
        reader.read_exact(&mut self.buf16)?;

        Ok(Triangle3::new(v1, v2, v3))
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

impl MeshReader for StlReader {
    fn read_from_buffer<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> std::io::Result<TMesh>
    where
        TBuffer: Read,
        TMesh: Builder<Mesh = TMesh>,
    {
        // Read header
        let mut header = [0u8; STL_HEADER_SIZE];
        reader.read_exact(&mut header)?;

        // Read number of triangle
        reader.read_exact(&mut self.buf32)?;
        let number_of_triangles: u32 = u32::from_le_bytes(self.buf32);

        // Faces
        let mut builder = TMesh::builder_soup();
        builder.set_num_faces(number_of_triangles as usize);

        for _ in 0..number_of_triangles {
            let triangle = self.read_face(reader)?;
            let result = builder.add_face(
                triangle.p1().cast(),
                triangle.p2().cast(),
                triangle.p3().cast()
            );

            if let Err(_) = result {
                break;
            }
        }

        builder.finish().map_err(|e| {
            Error::new(
                ErrorKind::Other,
                format!("Failed to build mesh from STL data: {:?}", e),
            )
        })
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
        writer.write(&[0; 2])?;

        Ok(())
    }

    fn write_point<TBuffer: Write, TPoint: Index<usize, Output = f32>>(
        &self,
        writer: &mut BufWriter<TBuffer>,
        point: &TPoint,
    ) -> io::Result<()> {
        writer.write(&point[0].to_le_bytes())?;
        writer.write(&point[1].to_le_bytes())?;
        writer.write(&point[2].to_le_bytes())?;

        Ok(())
    }
}

impl MeshWriter for StlWriter {
    fn write_to_buffer<TBuffer, TMesh>(
        &self,
        mesh: &TMesh,
        writer: &mut BufWriter<TBuffer>,
    ) -> std::io::Result<()>
    where
        TBuffer: Write,
        TMesh: Triangles,
    {
        let header = [0u8; STL_HEADER_SIZE];
        writer.write(&header)?;

        let faces_count = mesh.triangles().count();
        if faces_count > u32::max_value() as usize {
            return Err(Error::new(ErrorKind::Other, "Mesh is too big for STL"));
        }

        writer.write(&(faces_count as u32).to_le_bytes())?;

        for triangle in mesh.triangles() {
            let normal = triangle.get_normal().unwrap_or(Vector3::zeros()); // Write zeros for degenerate faces

            let p1 = Point3::new(
                triangle.p1().x.as_(),
                triangle.p1().y.as_(),
                triangle.p1().z.as_(),
            );
            let p2 = Point3::new(
                triangle.p2().x.as_(),
                triangle.p2().y.as_(),
                triangle.p2().z.as_(),
            );
            let p3 = Point3::new(
                triangle.p3().x.as_(),
                triangle.p3().y.as_(),
                triangle.p3().z.as_(),
            );
            let n = Vec3f::new(normal.x.as_(), normal.y.as_(), normal.z.as_());

            self.write_face(writer, &p1, &p2, &p3, &n)?;
        }

        writer.flush()
    }
}

impl Default for StlWriter {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}
