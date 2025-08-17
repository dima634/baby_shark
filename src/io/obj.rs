use std::io::BufRead;

use crate::{helpers::aliases::Vec3, io::MeshReader};
use super::*;

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
        TMesh: CreateBuilder 
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
                Ok(Statement::Face(v1, v2, v3)) => {
                    faces.push(v1);
                    faces.push(v2);
                    faces.push(v3);
                },
                Err(ParseError::StmtNotSupported) => {},
                Err(_) => todo!("{}", line),
            }
        }

        todo!()
    }
}

#[derive(Debug)]
enum Statement {
    Vertex(f64, f64, f64),
    Face(u64, u64, u64),
}

enum ParseError {
    InvalidStmt,
    StmtNotSupported,
}

fn parse_stmt(line: &str) -> Result<Statement, ParseError> {
    if line.starts_with("v") {
        parse_vertex(line).ok_or(ParseError::InvalidStmt)
    } else if line.starts_with("f") {
        parse_face(line).ok_or(ParseError::InvalidStmt)
    } else {
        Err(ParseError::StmtNotSupported)
    }
}

fn parse_vertex(str: &str) -> Option<Statement> {
    let mut stmt_parts_iter = str.split_whitespace().skip(1);
    let stmt_parts = [
        stmt_parts_iter.next()?,
        stmt_parts_iter.next()?,
        stmt_parts_iter.next()?,
    ];

    if let Some(_) = stmt_parts_iter.next() {
        return None; // More than 3 parts
    }

    Some(Statement::Vertex(
        stmt_parts[0].parse().ok()?,
        stmt_parts[1].parse().ok()?,
        stmt_parts[2].parse().ok()?,
    ))
}

fn parse_face(str: &str) -> Option<Statement> {
    let mut stmt_parts_iter = str.split_whitespace().skip(1);
    let stmt_parts = [
        stmt_parts_iter.next()?,
        stmt_parts_iter.next()?,
        stmt_parts_iter.next()?,
    ];

    if let Some(_) = stmt_parts_iter.next() {
        return None; // More than 3 parts
    }

    Some(Statement::Face(
        parse_vertex_idx_from_face_stmt(stmt_parts[0])?,
        parse_vertex_idx_from_face_stmt(stmt_parts[1])?,
        parse_vertex_idx_from_face_stmt(stmt_parts[2])?,
    ))
}

fn parse_vertex_idx_from_face_stmt(stmt: &str) -> Option<u64> {
    let (vertex_idx_str, _) = stmt.split_once('/')?;
    vertex_idx_str.parse().ok()
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use crate::mesh::polygon_soup::data_structure::PolygonSoup;
    use super::*;

    #[test]
    fn test_obj_reader() {
        let path = Path::new("C:\\Users\\Dmytro Volovyk\\Downloads\\mini.obj");
        let mut reader = ObjReader::default();
        let mesh: PolygonSoup<f32> = reader.read_from_file(&path).expect("Failed to read OBJ file");

        let slt_writer = super::StlWriter::default();
        slt_writer.write_to_file(&mesh, &Path::new("C:\\Users\\Dmytro Volovyk\\Downloads\\mini.stl"))
            .expect("Failed to write SLT file");
    }
}
