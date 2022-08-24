use std::{
    mem::size_of, 
    io::{ErrorKind, Read, Error, BufReader, self, Write, BufWriter}, 
    fs::{OpenOptions, File}, path::Path, ops::Index
};
use nalgebra::{Point3, Vector3};
use simba::scalar::{SupersetOf, SubsetOf};

use crate::{algo::merge_points::merge_points, mesh::traits::Mesh};

const STL_HEADER_SIZE: usize = 80;

pub struct StlReader {
    vertices: Vec<Point3<f32>>,

    // Buffers for reading
    buf32: [u8; size_of::<u32>()],
    buf16: [u8; size_of::<u16>()]
}

///
/// Binary STL reader
/// 
impl StlReader {
    pub fn new() -> Self {
        return Self {
            vertices: Vec::new(),
            buf16: [0; size_of::<u16>()],
            buf32: [0; size_of::<u32>()]
        };
    }

    /// Reads mesh from file
    pub fn read_stl_from_file<TMesh>(&mut self, filepath: &Path) -> std::io::Result<TMesh> 
    where 
        TMesh: Mesh,
        TMesh::ScalarType: SupersetOf<f32>
    {
        let file = OpenOptions::new().read(true).open(filepath)?;
        let mut reader = BufReader::new(file);

        return self.read_stl::<File, TMesh>(&mut reader);
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
        let vertices = &merged_vertices.points
                .iter()
                .map(|point| point.cast::<TMesh::ScalarType>())
                .collect();
        
        // Create mesh
        return Ok(TMesh::from_vertices_and_indices(&vertices, &merged_vertices.indices));
    }

    fn read_face<TBuffer: Read>(&mut self, reader: &mut BufReader<TBuffer>) -> io::Result<()> {
        // Normal
        self.read_vec3(reader)?;

        // Vertices
        let v1: Point3<f32> = self.read_vec3(reader)?;
        let v2: Point3<f32> = self.read_vec3(reader)?;
        let v3: Point3<f32> = self.read_vec3(reader)?;

        self.vertices.push(v1);
        self.vertices.push(v2);
        self.vertices.push(v3);

        // Attribute
        reader.read_exact(&mut self.buf16)?;

        return Ok(());
    }

    fn read_vec3<TBuffer: Read>(&mut self, reader: &mut BufReader<TBuffer>) -> io::Result<Point3<f32>> {
        reader.read_exact(&mut self.buf32)?;
        let x = f32::from_le_bytes(self.buf32);

        reader.read_exact(&mut self.buf32)?;
        let y = f32::from_le_bytes(self.buf32);

        reader.read_exact(&mut self.buf32)?;
        let z = f32::from_le_bytes(self.buf32);

        return Ok(Point3::new(x, y, z));
    }
}

pub struct StlWriter {}

impl StlWriter {
    pub fn new() -> Self {
        return StlWriter {};
    }

    pub fn write_stl_to_file<TMesh>(&self, mesh: &TMesh, path: &Path) -> io::Result<()> 
    where 
        TMesh: Mesh,
        TMesh::ScalarType: SubsetOf<f32>
    {
        let file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)
            .unwrap();
        let mut writer = BufWriter::new(file);

        return self.write_stl(mesh, &mut writer);
    }
    
    pub fn write_stl<TBuffer, TMesh>(&self, mesh: &TMesh, writer: &mut BufWriter<TBuffer>) -> io::Result<()> 
    where 
        TBuffer: Write, 
        TMesh: Mesh,
        TMesh::ScalarType: SubsetOf<f32>
    {
        let header = [0u8; STL_HEADER_SIZE];
        writer.write(&header)?;

        let faces_count = mesh.faces().count();
        if faces_count > u32::max_value() as usize {
            return Err(Error::new(ErrorKind::Other, "Mesh is too big for STL"));
        } 

        writer.write(&(faces_count as u32).to_le_bytes())?;
    
        for face in mesh.faces() {
            let (v1, v2, v3) = mesh.face_positions(&face);
            let normal = mesh.face_normal(&face);
            self.write_face(writer, &v1.cast(), &v2.cast(), &v3.cast(), &normal.cast())?;
        }

        return Ok(());
    }

    fn write_face<TBuffer: Write>(&self, writer: &mut BufWriter<TBuffer>, v1: &Point3<f32>, v2: &Point3<f32>, v3: &Point3<f32>, normal: &Vector3<f32>) -> io::Result<()> {
        self.write_point(writer, normal)?;
        self.write_point(writer, v1)?;
        self.write_point(writer, v2)?;
        self.write_point(writer, v3)?;
        writer.write(&[0; 2])?;

        return Ok(());
    }

    fn write_point<TBuffer: Write, TPoint: Index<usize, Output = f32>>(&self, writer: &mut BufWriter<TBuffer>, point: &TPoint) -> io::Result<()> {
        writer.write(&point[0].to_le_bytes())?;
        writer.write(&point[1].to_le_bytes())?;
        writer.write(&point[2].to_le_bytes())?;

        return Ok(());
    }
}
