use super::*;
use crate::{helpers::aliases::Vec3, io::MeshReader};
use std::io::BufRead;

// OBJ format spec:
// https://www.martinreddy.net/gfx/3d/OBJ.spec

#[derive(Debug, Default)]
pub struct ObjReader;

impl MeshReader for ObjReader {
    fn read_from_buffer<TBuffer, TMesh>(
        &mut self,
        reader: &mut BufReader<TBuffer>,
    ) -> std::io::Result<TMesh>
    where
        TBuffer: Read,
        TMesh: Builder<Mesh = TMesh>,
    {
        let mut vertices = Vec::new();
        let mut faces = Vec::new();
        let mut line = String::new();

        loop {
            line.clear();
            if reader.read_line(&mut line)? == 0 {
                break; // EOF
            }

            match parse_stmt(&line) {
                Ok(Statement::Vertex(x, y, z)) => vertices.push(Vec3::new(x, y, z)),
                Ok(Statement::Face(v1, v2, v3)) => faces.push([v1, v2, v3]),
                Err(ParseError::StmtNotSupported) => {} // Ignore unsupported statements
                Err(_) => todo!("{}", line),
            }
        }

        let mut builder = TMesh::builder_indexed();
        builder
            .add_vertices(vertices.into_iter().map(|v| v.cast()))
            .map_err(|_| std::io::ErrorKind::Other)?;
        builder
            .add_faces(faces.into_iter())
            .map_err(|_| std::io::ErrorKind::Other)?;
        builder
            .finish()
            .map_err(|_| std::io::ErrorKind::Other.into())
    }
}

#[derive(Debug)]
enum Statement {
    Vertex(f64, f64, f64),
    Face(usize, usize, usize),
}

impl ToString for Statement {
    fn to_string(&self) -> String {
        match self {
            Statement::Vertex(x, y, z) => format!("v {} {} {}", x, y, z),
            Statement::Face(v1, v2, v3) => format!("f {} {} {}", v1 + 1, v2 + 1, v3 + 1), // Convert to 1-based index
        }
    }
}

#[derive(Debug)]
enum ParseError {
    InvalidStmt,
    StmtNotSupported,
}

fn parse_stmt(line: &str) -> Result<Statement, ParseError> {
    let mut parts = line.split_whitespace();

    match parts.next() {
        Some("v") => parse_vertex(parts).ok_or(ParseError::InvalidStmt),
        Some("f") => parse_face(parts).ok_or(ParseError::InvalidStmt),
        _ => Err(ParseError::StmtNotSupported),
    }
}

fn parse_vertex<'a>(mut parts: impl Iterator<Item = &'a str>) -> Option<Statement> {
    let coords = [parts.next()?, parts.next()?, parts.next()?];

    Some(Statement::Vertex(
        coords[0].parse().ok()?,
        coords[1].parse().ok()?,
        coords[2].parse().ok()?,
    ))
}

fn parse_face<'a>(mut parts: impl Iterator<Item = &'a str>) -> Option<Statement> {
    let indices = [parts.next()?, parts.next()?, parts.next()?];

    Some(Statement::Face(
        parse_vertex_idx_from_face_stmt(indices[0])?,
        parse_vertex_idx_from_face_stmt(indices[1])?,
        parse_vertex_idx_from_face_stmt(indices[2])?,
    ))
}

fn parse_vertex_idx_from_face_stmt(stmt: &str) -> Option<usize> {
    let (vertex_idx_str, _) = stmt.split_once('/')?;
    vertex_idx_str.parse::<usize>().map(|idx| idx - 1).ok() // obj indices are 1-based
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::polygon_soup::data_structure::PolygonSoup;
    use std::path::Path;

    #[test]
    fn test_obj_reader() {
        let path = Path::new("C:\\Users\\Dmytro Volovyk\\Downloads\\mini.obj");
        let mut reader = ObjReader::default();
        let mesh: PolygonSoup<f32> = reader
            .read_from_file(&path)
            .expect("Failed to read OBJ file");

        let slt_writer = super::StlWriter::default();
        slt_writer
            .write_to_file(
                &mesh,
                &Path::new("C:\\Users\\Dmytro Volovyk\\Downloads\\mini.stl"),
            )
            .expect("Failed to write SLT file");
    }
}
