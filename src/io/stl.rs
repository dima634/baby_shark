use std::{
    mem::size_of, 
    io::{ErrorKind, Read, Error, BufReader, self, Write, BufWriter}, 
    fs::{OpenOptions, File}, path::Path, ops::Index
};
use nalgebra::{Point3, Vector3};
use simba::scalar::SupersetOf;

use crate::{algo::{merge_points::merge_points, utils::cast}, mesh::traits::Mesh, helpers::aliases::Vec3f};

const STL_HEADER_SIZE: usize = 80;

pub struct StlReader {
    vertices: Vec<Vec3f>,

    // Buffers for reading
    buf32: [u8; size_of::<u32>()],
    buf16: [u8; size_of::<u16>()]
}

///
/// Binary STL reader
/// 
impl StlReader {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            buf16: [0; size_of::<u16>()],
            buf32: [0; size_of::<u32>()]
        }
    }

    /// Reads mesh from file
    pub fn read_stl_from_file<TMesh>(&mut self, filepath: &Path) -> std::io::Result<TMesh> 
    where 
        TMesh: Mesh,
        TMesh::ScalarType: SupersetOf<f32>
    {
        let file = OpenOptions::new().read(true).open(filepath)?;
        let mut reader = BufReader::new(file);

        self.read_stl::<File, TMesh>(&mut reader)
    }

    /// Reads mesh from buffer
    pub fn read_stl<TBuffer, TMesh>(&mut self, reader: &mut BufReader<TBuffer>) -> std::io::Result<TMesh> 
    where 
        TBuffer: Read, 
        TMesh: Mesh,
        TMesh::ScalarType: SupersetOf<f32>
    {
        self.vertices.clear();

        // Read header
        let mut header = [0u8; STL_HEADER_SIZE];
        reader.read_exact(&mut header)?;

        // Read number of triangle
        reader.read_exact(&mut self.buf32)?;
        let number_of_triangles: u32 = u32::from_le_bytes(self.buf32);

        // Faces
        for _ in 0..number_of_triangles {
            self.read_face(reader)?;
        }

        // Merge face vertices
        let merged_vertices = merge_points(&self.vertices);
        
        // Case points to scalar type used by mesh
        let vertices: Vec<_> = merged_vertices.points
                .iter()
                .map(|point| point.cast::<TMesh::ScalarType>())
                .collect();
        
        // Create mesh
        Ok(TMesh::from_vertex_and_face_slices(&vertices, &merged_vertices.indices))
    }

    fn read_face<TBuffer: Read>(&mut self, reader: &mut BufReader<TBuffer>) -> io::Result<()> {
        // Normal
        self.read_vec3(reader)?;

        // Vertices
        let v1 = self.read_vec3(reader)?;
        let v2 = self.read_vec3(reader)?;
        let v3 = self.read_vec3(reader)?;

        self.vertices.push(v1);
        self.vertices.push(v2);
        self.vertices.push(v3);

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

    pub fn write_stl_to_file<TMesh: Mesh>(&self, mesh: &TMesh, path: &Path) -> io::Result<()> {
        let file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)
            .unwrap();
        let mut writer = BufWriter::new(file);

        self.write_stl(mesh, &mut writer)
    }
    
    pub fn write_stl<TBuffer, TMesh>(&self, mesh: &TMesh, writer: &mut BufWriter<TBuffer>) -> io::Result<()> 
    where 
        TBuffer: Write, 
        TMesh: Mesh
    {
        let header = [0u8; STL_HEADER_SIZE];
        writer.write_all(&header)?;

        let faces_count = mesh.faces().count();
        if faces_count > u32::max_value() as usize {
            return Err(Error::new(ErrorKind::Other, "Mesh is too big for STL"));
        } 

        writer.write_all(&(faces_count as u32).to_le_bytes())?;
    
        for face in mesh.faces() {
            let triangle = mesh.face_positions(&face);
            let normal = triangle.get_normal();
            
            let p1 = cast(triangle.p1()).into();
            let p2 = cast(triangle.p2()).into();
            let p3 = cast(triangle.p3()).into();
            let n = cast(&normal);

            self.write_face(writer, &p1, &p2, &p3, &n)?;
        }

        Ok(())
    }

    fn write_face<TBuffer: Write>(&self, writer: &mut BufWriter<TBuffer>, v1: &Point3<f32>, v2: &Point3<f32>, v3: &Point3<f32>, normal: &Vector3<f32>) -> io::Result<()> {
        self.write_point(writer, normal)?;
        self.write_point(writer, v1)?;
        self.write_point(writer, v2)?;
        self.write_point(writer, v3)?;
        writer.write_all(&[0; 2])?;

        Ok(())
    }

    fn write_point<TBuffer: Write, TPoint: Index<usize, Output = f32>>(&self, writer: &mut BufWriter<TBuffer>, point: &TPoint) -> io::Result<()> {
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
