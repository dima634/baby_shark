use std::{collections::HashMap, fmt::Display};
use nalgebra::{Point3, Vector3};
use tabled::Table;
use crate::{mesh::traits::{Mesh, TopologicalMesh}, geometry::traits::RealNumber};
use self::helpers::Edge;
use super::{traversal::{CornerTableFacesIter, CornerTableVerticesIter, CornerTableEdgesIter, CornerWalker, faces_around_vertex, vertices_around_vertex}, connectivity::{corner::Corner, vertex::Vertex}};


pub struct CornerTable<TScalar: RealNumber> {
    pub(super) vertices: Vec<Vertex<TScalar>>,
    pub(super) corners: Vec<Corner>
}

impl<TScalar: RealNumber> CornerTable<TScalar> {
    pub fn new<>() -> Self {
        return Self {
            corners: Vec::new(),
            vertices: Vec::new()
        };
    }

    #[inline]
    pub fn get_vertex(&self, vertex_index:  usize) -> Option<&Vertex<TScalar>> {
        return self.vertices.get(vertex_index);
    }

    #[inline]
    pub fn get_vertex_mut(&mut self, vertex_index:  usize) -> Option<&mut Vertex<TScalar>> {
        return self.vertices.get_mut(vertex_index);
    }

    #[inline]
    pub fn get_corner(&self, corner_index:  usize) -> Option<&Corner> {
        return self.corners.get(corner_index);
    }

    #[inline]
    pub fn get_corner_mut(&mut self, corner_index:  usize) -> Option<&mut Corner> {
        return self.corners.get_mut(corner_index);
    }

    /// Create new isolated corner
    #[inline]
    pub fn create_corner(&mut self) -> &mut Corner {
        let idx = self.corners.len();
        self.corners.push(Corner::default());
        return self.corners.get_mut(idx).unwrap();
    }

    /// Create new isolated vertex
    #[inline]
    pub fn create_vertex(&mut self) -> &mut Vertex<TScalar> {
        let idx = self.vertices.len();
        self.vertices.push(Default::default());
        return self.vertices.get_mut(idx).unwrap();
    }

    /// Creates isolated face from existing vertices vertices
    pub fn create_face_from_vertices(&mut self, v1: usize, v2: usize, v3: usize) {
        let c1 = self.create_corner();
        c1.set_vertex_index(v1);

        let c2 = self.create_corner();
        c2.set_vertex_index(v2);

        let c3 = self.create_corner();
        c3.set_vertex_index(v3);
    }

    /// Makes give corners opposite to each other
    #[inline]
    pub fn set_opposite_relationship(&mut self, corner1_index: usize, corner2_index: usize) {
        self.get_corner_mut(corner1_index).unwrap().set_opposite_corner_index(corner2_index);
        self.get_corner_mut(corner2_index).unwrap().set_opposite_corner_index(corner1_index);
    }

    fn corner_from(
        &mut self,
        edge_opposite_corner_map: &mut HashMap<Edge, usize>,
        opposite_edge: &mut Edge,
        vertex_index: usize
    ) {
        let corner_index = self.corners.len();
        let corner = self.create_corner();
        corner.set_vertex_index(vertex_index);

        // Find opposite corner
        let mut opposite_corner_index: Option<&usize> = edge_opposite_corner_map.get(&opposite_edge);

        // Flip directed edge
        if opposite_corner_index.is_none() {
            opposite_edge.flip();
            opposite_corner_index = edge_opposite_corner_map.get(&opposite_edge);
        }

        if opposite_corner_index.is_some() {
            // Set opposite for corners
            corner.set_opposite_corner_index(*opposite_corner_index.unwrap());
            let opposite_corner = self.corners.get_mut(*opposite_corner_index.unwrap()).unwrap();
            opposite_corner.set_opposite_corner_index(corner_index);
        } else {
            // Save edge and it`s opposite corner
            edge_opposite_corner_map.insert(*opposite_edge, corner_index);
        }

        // Set corner index for vertex
        let vertex = self.get_vertex_mut(vertex_index).unwrap();
        vertex.set_corner_index(corner_index);
    }
}

impl<TScalar: RealNumber> Mesh for CornerTable<TScalar> {
    type ScalarType = TScalar;

    type EdgeDescriptor = usize;
    type VertexDescriptor = usize;
    type FaceDescriptor = usize;

    type FacesIter<'iter> = CornerTableFacesIter<'iter, TScalar>;
    type VerticesIter<'iter> = CornerTableVerticesIter<'iter, TScalar>;
    type EdgesIter<'iter> = CornerTableEdgesIter<'iter, TScalar>;

    fn from_vertices_and_indices(vertices: &Vec<Point3<Self::ScalarType>>, faces: &Vec<usize>) -> Self {
        assert!(faces.len() % 3 == 0, "Invalid number of face indices: {}", faces.len());

        let mut edge_opposite_corner_map = HashMap::<helpers::Edge, usize>::new();
        let mut corner_table = Self::new();

        for vertex_index in 0..vertices.len() {
            let v_position = vertices.get(vertex_index).unwrap();
            let vertex = corner_table.create_vertex();
            vertex.set_position(*v_position);
        }

        for face_idx in (0..faces.len() - 1).step_by(3) {
            let v1_index = faces[face_idx];
            let v2_index = faces[face_idx + 1];
            let v3_index = faces[face_idx + 2];

            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v2_index, v3_index), v1_index);
            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v3_index, v1_index), v2_index);
            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v1_index, v2_index), v3_index);
        }

        return corner_table;
    }

    #[inline]
    fn faces<'a>(&'a self) -> Self::FacesIter<'a> {
        return Self::FacesIter::new(self);
    }

    #[inline]
    fn vertices<'a>(&'a self) -> Self::VerticesIter<'a> {
        return Self::VerticesIter::new(self);
    }

    #[inline]
    fn edges<'a>(&'a self) -> Self::EdgesIter<'a> {
        return Self::EdgesIter::new(self);
    }

    #[inline]
    fn face_positions(&self, face: &Self::FaceDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, *face);

        return (
            *walker.get_vertex().get_position(),
            *walker.next().get_vertex().get_position(),
            *walker.next().get_vertex().get_position()
        );
    }

    #[inline]
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, *edge);
        return (
            *walker.next().get_vertex().get_position(),
            *walker.next().get_vertex().get_position()
        );
    }

    #[inline]
    fn get_edge_vertices(&self, edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor) {
        let mut walker = CornerWalker::from_corner(self, *edge);
        return (
            walker.next().get_corner().get_vertex_index(),
            walker.next().get_corner().get_vertex_index()
        );
    }

    #[inline]
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Point3<Self::ScalarType> {
        return self.vertices[*vertex].get_position();
    }

    fn vertex_normal(&self, vertex: &Self::VertexDescriptor) -> Vector3<Self::ScalarType> {
        let mut sum = Vector3::zeros();
        faces_around_vertex(&self, *vertex, |face_index| sum += self.face_normal(face_index));

        return sum.normalize();
    }
}

impl<TScalar: RealNumber> TopologicalMesh for CornerTable<TScalar> {
    type Position<'a> = CornerWalker<'a, TScalar>;
    
    #[inline]
    fn vertices_around_vertex<TVisit: FnMut(&Self::VertexDescriptor) -> ()>(&self, vertex: &Self::VertexDescriptor, visit: TVisit) {
        vertices_around_vertex(&self, *vertex, visit);
    }

    #[inline]
    fn faces_around_vertex<TVisit: FnMut(&Self::FaceDescriptor) -> ()>(&self, vertex: &Self::VertexDescriptor, visit: TVisit) {
        faces_around_vertex(self, *vertex, visit);
    }

    fn is_vertex_on_boundary(&self, vertex: &Self::VertexDescriptor) -> bool {
        let mut walker = CornerWalker::from_vertex(self, *vertex);
        walker.next();
        let started_at = walker.get_corner_index();
    
        loop {
            if walker.get_corner().get_opposite_corner_index().is_none() {
                return true;
            }

            walker.opposite().previous();

            if started_at == walker.get_corner_index() {
                break;
            }
        }

        return false;
    }
}

impl<TScalar: RealNumber> Display for CornerTable<TScalar> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let vertices = Table::new(self.vertices.iter());
        let corners = Table::new(self.corners.iter());

        writeln!(f, "### VERTICES ###")?;
        writeln!(f, "{}", vertices)?;
        writeln!(f)?;
        writeln!(f, "### CORNERS ###")?;
        writeln!(f,"{}", corners)?;

        return Ok(());
    }
}

pub(super) mod helpers {
    use std::mem::swap;

    #[derive(Hash, PartialEq, Eq, Clone, Copy)]
    pub struct Edge {
        start_vertex: usize,
        end_vertex: usize
    }

    impl Edge {
        pub fn new(start: usize, end: usize) -> Self {
            return Self {
                start_vertex: start,
                end_vertex: end
            };
        }

        #[inline]
        pub fn flip(&mut self) {
            swap(&mut self.start_vertex, &mut self.end_vertex);
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::mesh::corner_table::{test_helpers::{create_unit_square_mesh, assert_mesh_equals}, connectivity::{vertex::VertexF, corner::Corner}};

    #[test]
    fn from_vertices_and_indices() {
        let mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default()),
            VertexF::new(1, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default()),
            VertexF::new(3, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default()),
            VertexF::new(4, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default())
        ];

        let expected_corners = vec![
            Corner::new(None,    0, Default::default()),
            Corner::new(Some(4), 1, Default::default()),
            Corner::new(None,    2, Default::default()),

            Corner::new(None,    2, Default::default()),
            Corner::new(Some(1), 3, Default::default()),
            Corner::new(None,    0, Default::default())
        ];

        assert_mesh_equals(&mesh, &expected_corners, &expected_vertices);
    }
}
