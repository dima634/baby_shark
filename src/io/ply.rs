use super::*;
use crate::helpers::aliases::Vec3;
use num_traits::{FromBytes, FromPrimitive};
use std::{
    collections::HashMap,
    io::{BufRead, Seek},
};

#[derive(Debug, Default)]
pub struct PlyReader;

#[derive(Debug)]
struct PlyHeader {
    format: PlyFormat,
    vertex_count: usize,
    face_count: usize,
    vertex_properties: Vec<PropertyInfo>,
    face_properties: Vec<PropertyInfo>,
}

#[derive(Debug, Clone)]
enum PlyFormat {
    Ascii,
    BinaryLittleEndian,
    BinaryBigEndian,
}

#[derive(Debug, Clone)]
struct ElementInfo {
    name: String,
    count: usize,
    properties: Vec<PropertyInfo>,
}

#[derive(Debug, Clone)]
struct PropertyInfo {
    name: String,
    data_type: PlyDataType,
    is_list: bool,
    list_count_type: Option<PlyDataType>,
}

#[derive(Debug, Clone, Copy)]
enum PlyDataType {
    Char,
    UChar,
    Short,
    UShort,
    Int,
    UInt,
    Float,
    Double,
}

impl PlyDataType {
    fn size(&self) -> usize {
        match self {
            PlyDataType::Char | PlyDataType::UChar => 1,
            PlyDataType::Short | PlyDataType::UShort => 2,
            PlyDataType::Int | PlyDataType::UInt | PlyDataType::Float => 4,
            PlyDataType::Double => 8,
        }
    }

    fn from_str(s: &str) -> Option<Self> {
        match s {
            "char" | "int8" => Some(PlyDataType::Char),
            "uchar" | "uint8" => Some(PlyDataType::UChar),
            "short" | "int16" => Some(PlyDataType::Short),
            "ushort" | "uint16" => Some(PlyDataType::UShort),
            "int" | "int32" => Some(PlyDataType::Int),
            "uint" | "uint32" => Some(PlyDataType::UInt),
            "float" | "float32" => Some(PlyDataType::Float),
            "double" | "float64" => Some(PlyDataType::Double),
            _ => None,
        }
    }
}

impl PlyReader {
    fn parse_header<TBuffer: BufRead>(&self, reader: &mut TBuffer) -> Result<PlyHeader, ReadError> {
        let mut line = String::new();

        // Read magic number
        reader.read_line(&mut line)?;
        if line.trim() != "ply" {
            return Err(ReadError::Malformed);
        }

        let mut format = None;
        let mut elements = Vec::new();
        let mut current_element: Option<ElementInfo> = None;

        loop {
            line.clear();
            let bytes_read = reader.read_line(&mut line)?;
            if bytes_read == 0 {
                break;
            }

            let mut parts = line.split_whitespace();
            let keyword = parts.next().ok_or(ReadError::Malformed)?;

            match keyword {
                "format" => {
                    let format_str = parts.next().ok_or(ReadError::Malformed)?;
                    format = Some(match format_str {
                        "ascii" => PlyFormat::Ascii,
                        "binary_little_endian" => PlyFormat::BinaryLittleEndian,
                        "binary_big_endian" => PlyFormat::BinaryBigEndian,
                        _ => return Err(ReadError::Malformed),
                    });
                }
                "element" => {
                    if let Some(element) = current_element.take() {
                        elements.push(element);
                    }

                    let name = parts
                        .next()
                        .map(|s| s.to_string())
                        .ok_or(ReadError::Malformed)?;
                    let count = parts
                        .next()
                        .map(|s| s.parse().ok())
                        .flatten()
                        .ok_or(ReadError::Malformed)?;
                    current_element = Some(ElementInfo {
                        name,
                        count,
                        properties: Vec::new(),
                    });
                }
                "property" => {
                    if let Some(ref mut element) = current_element {
                        let property_type = parts.next().ok_or(ReadError::Malformed)?;

                        if property_type == "list" {
                            // List property: property list <count_type> <data_type> <name>
                            let count_type_str = parts.next().ok_or(ReadError::Malformed)?;
                            let data_type_str = parts.next().ok_or(ReadError::Malformed)?;
                            let name = parts.next().ok_or(ReadError::Malformed)?.to_string();

                            let count_type = PlyDataType::from_str(count_type_str)
                                .ok_or_else(|| ReadError::Malformed)?;
                            let data_type = PlyDataType::from_str(data_type_str)
                                .ok_or_else(|| ReadError::Malformed)?;

                            element.properties.push(PropertyInfo {
                                name,
                                data_type,
                                is_list: true,
                                list_count_type: Some(count_type),
                            });
                        } else {
                            // Scalar property: property <data_type> <name>
                            let name = parts.next().ok_or(ReadError::Malformed)?.to_string();
                            let data_type = PlyDataType::from_str(property_type)
                                .ok_or_else(|| ReadError::Malformed)?;

                            element.properties.push(PropertyInfo {
                                name,
                                data_type,
                                is_list: false,
                                list_count_type: None,
                            });
                        }
                    }
                }
                "end_header" => {
                    if let Some(element) = current_element.take() {
                        elements.push(element);
                    }
                    break;
                }
                _ => {
                    // Ignore other lines (comments, etc.)
                }
            }
        }

        let format = format.ok_or_else(|| ReadError::Malformed)?;

        // Extract vertex and face information
        let mut vertex_count = 0;
        let mut face_count = 0;
        let mut vertex_properties = Vec::new();
        let mut face_properties = Vec::new();

        for element in elements {
            match element.name.as_str() {
                "vertex" => {
                    vertex_count = element.count;
                    vertex_properties = element.properties;
                }
                "face" => {
                    face_count = element.count;
                    face_properties = element.properties;
                }
                _ => {
                    // Ignore other elements for now
                }
            }
        }

        Ok(PlyHeader {
            format,
            vertex_count,
            face_count,
            vertex_properties,
            face_properties,
        })
    }

    fn read_ascii_data<TBuffer: BufRead>(
        &self,
        reader: &mut TBuffer,
        header: &PlyHeader,
    ) -> Result<(Vec<Vec3<f64>>, Vec<[usize; 3]>), ReadError> {
        let mut vertices = Vec::with_capacity(header.vertex_count);
        let mut faces = Vec::with_capacity(header.face_count);
        let mut line = String::new();

        // Check if required properties exist:
        // 1. "x", "y", "z" in vertex_properties
        // 2. vertex_indices/vertex_index in face_properties
        let has_xyz = ["x", "y", "z"]
            .iter()
            .all(|&prop| header.vertex_properties.iter().any(|p| p.name == prop));

        if !has_xyz {
            return Err(ReadError::UnsupportedFormat(
                "Vertex properties must include x, y, z".to_string(),
            ));
        }

        let has_vertex_indices = header
            .face_properties
            .iter()
            .any(|p| p.is_list && (p.name == "vertex_indices" || p.name == "vertex_index"));

        if !has_vertex_indices {
            return Err(ReadError::UnsupportedFormat(
                "Face properties must include vertex_indices or vertex_index".to_string(),
            ));
        }

        // Read vertices
        for _ in 0..header.vertex_count {
            line.clear();
            reader.read_line(&mut line)?;
            let mut parts = line.split_whitespace();

            let mut x = 0.0;
            let mut y = 0.0;
            let mut z = 0.0;

            for prop in &header.vertex_properties {
                if let Some(part) = parts.next() {
                    match prop.name.as_str() {
                        "x" => x = part.parse().map_err(|_| ReadError::Malformed)?,
                        "y" => y = part.parse().map_err(|_| ReadError::Malformed)?,
                        "z" => z = part.parse().map_err(|_| ReadError::Malformed)?,
                        _ => {} // Ignore other properties
                    }
                }
            }

            vertices.push(Vec3::new(x, y, z));
        }

        // Read faces
        for _ in 0..header.face_count {
            line.clear();
            reader.read_line(&mut line)?;
            let mut parts = line.split_whitespace();

            if let Some(vertex_count) = parts.next().map(|s| s.parse::<usize>().ok()).flatten() {
                if vertex_count < 3 {
                    return Err(ReadError::Malformed);
                }

                // Read the first three vertices for triangle
                let v1: usize = parts
                    .next()
                    .map(|s| s.parse().ok())
                    .flatten()
                    .ok_or(ReadError::Malformed)?;
                let v2: usize = parts
                    .next()
                    .map(|s| s.parse().ok())
                    .flatten()
                    .ok_or(ReadError::Malformed)?;
                let v3: usize = parts
                    .next()
                    .map(|s| s.parse().ok())
                    .flatten()
                    .ok_or(ReadError::Malformed)?;

                faces.push([v1, v2, v3]);

                // If it's a quad or polygon, triangulate by adding more triangles
                let mut prev_vertex = v3;
                for _ in 3..vertex_count {
                    if let Some(vi_str) = parts.next() {
                        if let Ok(vi) = vi_str.parse::<usize>() {
                            faces.push([v1, prev_vertex, vi]);
                            prev_vertex = vi;
                        }
                    }
                }
            }
        }

        Ok((vertices, faces))
    }

    fn read_binary_data<TBuffer: Read + Seek, R: RealNumber>(
        &self,
        reader: &mut BufReader<TBuffer>,
        header: &PlyHeader,
    ) -> Result<(Vec<Vec3<R>>, Vec<[usize; 3]>), ReadError> {
        let mut vertices = Vec::with_capacity(header.vertex_count);
        let mut faces = Vec::with_capacity(header.face_count);

        let is_little_endian = matches!(header.format, PlyFormat::BinaryLittleEndian);

        // Read vertices
        for _ in 0..header.vertex_count {
            let mut x = R::zero();
            let mut y = R::zero();
            let mut z = R::zero();

            for prop in &header.vertex_properties {
                match prop.name.as_str() {
                    "x" => x = read_binary_value(reader, prop.data_type, is_little_endian)?,
                    "y" => y = read_binary_value(reader, prop.data_type, is_little_endian)?,
                    "z" => z = read_binary_value(reader, prop.data_type, is_little_endian)?,
                    _ => skip_property(reader, prop, is_little_endian)?,
                }
            }

            vertices.push(Vec3::new(x, y, z));
        }

        // Read faces
        for _ in 0..header.face_count {
            for prop in &header.face_properties {
                let is_vertex_indices =
                    prop.is_list && (prop.name == "vertex_indices" || prop.name == "vertex_index");

                if is_vertex_indices {
                    let count: usize =
                        read_binary_value(reader, prop.list_count_type.unwrap(), is_little_endian)?;

                    if count < 3 {
                        return Err(ReadError::Malformed);
                    }

                    // Read the first three vertices for triangle
                    let v1 = read_binary_value(reader, prop.data_type, is_little_endian)?;
                    let v2 = read_binary_value(reader, prop.data_type, is_little_endian)?;
                    let v3 = read_binary_value(reader, prop.data_type, is_little_endian)?;

                    faces.push([v1, v2, v3]);

                    // If it's a quad or polygon, triangulate by adding more triangles
                    let mut prev_vertex = v3;
                    for _ in 3..count {
                        let vi = read_binary_value(reader, prop.data_type, is_little_endian)?;
                        faces.push([v1, prev_vertex, vi]);
                        prev_vertex = vi;
                    }
                } else {
                    skip_property(reader, prop, is_little_endian)?;
                }
            }
        }

        Ok((vertices, faces))
    }
}

fn skip_property<TBuffer: Read + Seek>(
    reader: &mut BufReader<TBuffer>,
    prop: &PropertyInfo,
    is_little_endian: bool,
) -> Result<(), ReadError> {
    if prop.is_list {
        // Skip list property
        // First read the count, then skip the list items
        let count = read_binary_value::<TBuffer, usize>(
            reader,
            prop.list_count_type.unwrap(),
            is_little_endian,
        )?;
        let item_size = prop.data_type.size();
        reader.seek_relative((count * item_size) as i64)?;
    } else {
        reader.seek_relative(prop.data_type.size() as i64)?;
    }

    Ok(())
}

fn read_binary_value<TBuffer: Read, TTy: FromPrimitive>(
    reader: &mut BufReader<TBuffer>,
    data_type: PlyDataType,
    is_little_endian: bool,
) -> Result<TTy, ReadError> {
    let mut buffer = [0u8; 8];
    let size = data_type.size();
    reader.read_exact(&mut buffer[..size])?;

    match data_type {
        PlyDataType::Char => {
            TTy::from_i8(i8::from_ne_bytes([buffer[0]])).ok_or(ReadError::Malformed)
        }
        PlyDataType::UChar => TTy::from_u8(buffer[0]).ok_or(ReadError::Malformed),
        PlyDataType::Short => {
            let val = from_bytes::<i16>(&[buffer[0], buffer[1]], is_little_endian);
            TTy::from_i16(val).ok_or(ReadError::Malformed)
        }
        PlyDataType::UShort => {
            let val = from_bytes::<u16>(&[buffer[0], buffer[1]], is_little_endian);
            TTy::from_u16(val).ok_or(ReadError::Malformed)
        }
        PlyDataType::Int => {
            let val = from_bytes::<i32>(
                &[buffer[0], buffer[1], buffer[2], buffer[3]],
                is_little_endian,
            );
            TTy::from_i32(val).ok_or(ReadError::Malformed)
        }
        PlyDataType::UInt => {
            let val = from_bytes::<u32>(
                &[buffer[0], buffer[1], buffer[2], buffer[3]],
                is_little_endian,
            );
            TTy::from_u32(val).ok_or(ReadError::Malformed)
        }
        PlyDataType::Float => {
            let val = from_bytes::<f32>(
                &[buffer[0], buffer[1], buffer[2], buffer[3]],
                is_little_endian,
            );
            TTy::from_f32(val).ok_or(ReadError::Malformed)
        }
        PlyDataType::Double => {
            let val = from_bytes::<f64>(&buffer, is_little_endian);
            TTy::from_f64(val).ok_or(ReadError::Malformed)
        }
    }
}

fn from_bytes<T: FromBytes>(bytes: &T::Bytes, is_little_endian: bool) -> T {
    if is_little_endian {
        T::from_le_bytes(bytes)
    } else {
        T::from_be_bytes(bytes)
    }
}

impl MeshReader for PlyReader {
    fn read_from_buffer<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> Result<TMesh, ReadError>
    where
        TBuffer: Read + Seek,
        TMesh: Builder<Mesh = TMesh>,
    {
        // Parse header first
        let header = self.parse_header(reader)?;

        // Read data based on format
        let (vertices, faces) = match header.format {
            PlyFormat::Ascii => self.read_ascii_data(reader, &header)?,
            PlyFormat::BinaryLittleEndian | PlyFormat::BinaryBigEndian => {
                self.read_binary_data(reader, &header)?
            }
        };

        // Build mesh
        let mut builder = TMesh::builder_indexed();
        builder.set_num_vertices(vertices.len());
        builder.set_num_faces(faces.len());

        builder.add_vertices(vertices.into_iter().map(|v| v.cast()))?;
        builder.add_faces(faces.into_iter())?;

        Ok(builder.finish()?)
    }
}

#[derive(Debug, Default)]
pub struct PlyWriter;

impl PlyWriter {
    fn write_header<TBuffer: Write>(
        &self,
        writer: &mut BufWriter<TBuffer>,
        vertex_count: usize,
        face_count: usize,
    ) -> std::io::Result<()> {
        writeln!(writer, "ply")?;
        writeln!(writer, "format binary_little_endian 1.0")?;
        writeln!(writer, "element vertex {}", vertex_count)?;
        writeln!(writer, "property double x")?;
        writeln!(writer, "property double y")?;
        writeln!(writer, "property double z")?;
        writeln!(writer, "element face {}", face_count)?;
        writeln!(writer, "property list uchar uint vertex_indices")?;
        writeln!(writer, "end_header")?;
        Ok(())
    }
}

impl MeshWriter for PlyWriter {
    fn write_to_buffer<TBuffer, TMesh>(
        &self,
        mesh: &TMesh,
        writer: &mut BufWriter<TBuffer>,
    ) -> std::io::Result<()>
    where
        TBuffer: Write,
        TMesh: TriangleMesh,
    {
        let vertex_count = mesh.vertices().count();
        let face_count = mesh.faces().count();

        self.write_header(writer, vertex_count, face_count)?;

        for vertex in mesh.vertices() {
            let position = mesh.position(vertex);
            let x: f64 = position[0].as_();
            let y: f64 = position[1].as_();
            let z: f64 = position[2].as_();

            writer.write(&x.to_le_bytes())?;
            writer.write(&y.to_le_bytes())?;
            writer.write(&z.to_le_bytes())?;
        }

        let vertex_to_index: HashMap<TMesh::VertexId, usize> = mesh
            .vertices()
            .enumerate()
            .map(|(index, id)| (id, index))
            .collect();

        for [v1_id, v2_id, v3_id] in mesh.faces() {
            let Ok(v1): Result<u32, _> = vertex_to_index[&v1_id].try_into() else { break };
            let Ok(v2): Result<u32, _> = vertex_to_index[&v2_id].try_into() else { break };
            let Ok(v3): Result<u32, _> = vertex_to_index[&v3_id].try_into() else { break };

            writer.write(&3_u8.to_le_bytes())?;
            writer.write(&v1.to_le_bytes())?;
            writer.write(&v2.to_le_bytes())?;
            writer.write(&v3.to_le_bytes())?;
        }

        writer.flush()
    }
}
