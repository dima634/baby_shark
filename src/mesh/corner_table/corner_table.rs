use std::{collections::HashMap};
use nalgebra::{Point3, UnitVector3};
use crate::mesh::traits::Mesh;
use self::helpers::Edge;
use super::{connectivity::traits::{Corner, Vertex}, traversal::{CornerTableFacesIter, CornerTableVerticesIter, CornerTableEdgesIter, CornerWalker}};


pub struct CornerTable<TCorner: Corner, TVertex: Vertex> {
    pub(super) vertices: Vec<TVertex>,
    pub(super) corners: Vec<TCorner>
}

impl<TCorner, TVertex> CornerTable<TCorner, TVertex> 
where 
    TCorner: Corner,
    TVertex: Vertex 
{
    pub fn new<>() -> Self {
        return Self {
            corners: Vec::new(),
            vertices: Vec::new()
        };
    }

    #[inline]
    pub fn get_vertex(&self, vertex_index:  usize) -> Option<&TVertex> {
        return self.vertices.get(vertex_index);
    }

    #[inline]
    pub fn get_vertex_mut(&mut self, vertex_index:  usize) -> Option<&mut TVertex> {
        return self.vertices.get_mut(vertex_index);
    }

    #[inline]
    pub fn get_corner(&self, corner_index:  usize) -> Option<&TCorner> {
        return self.corners.get(corner_index);
    }

    #[inline]
    pub fn get_corner_mut(&mut self, corner_index:  usize) -> Option<&mut TCorner> {
        return self.corners.get_mut(corner_index);
    }

    fn create_corner(&mut self) -> &mut TCorner {
        let idx = self.corners.len();
        let mut corner = TCorner::default();
        corner.set_index(idx);
        self.corners.push(corner);
        return self.corners.get_mut(idx).unwrap();
    }

    fn create_vertex(&mut self) -> &mut TVertex {
        let idx = self.vertices.len();
        let mut vertex = TVertex::default();
        vertex.set_index(idx);
        self.vertices.push(vertex);
        return self.vertices.get_mut(idx).unwrap();
    }

    fn corner_from(
        &mut self,
        edge_opposite_corner_map: &mut HashMap<Edge, usize>,
        opposite_edge: &mut Edge,
        vertex_index: usize,
        next_index: usize
    ) {
        let corner = self.create_corner();
        let corner_index = corner.get_index();
        corner.set_vertex_index(vertex_index);
        corner.set_next_corner_index(next_index);

        // Find opposite corner
        let mut opposite_corner_index: Option<&usize> = edge_opposite_corner_map.get(&opposite_edge);

        // Flip directed edge
        if opposite_corner_index.is_none() {
            opposite_edge.flip();
            opposite_corner_index = edge_opposite_corner_map.get(&opposite_edge);
        }

        if opposite_corner_index.is_some() {
            // Set opposite for corners
            let corner_index = corner.get_index();
            corner.set_opposite_corner_index(*opposite_corner_index.unwrap());
            let opposite_corner = self.corners.get_mut(*opposite_corner_index.unwrap()).unwrap();
            opposite_corner.set_opposite_corner_index(corner_index);
        } else {
            // Save edge and it`s opposite corner
            edge_opposite_corner_map.insert(*opposite_edge, corner.get_index());
        }

        // Set corner index for vertex
        let vertex = self.get_vertex_mut(vertex_index).unwrap();
        vertex.set_corner_index(corner_index);
    }
}

impl<'a, TCorner, TVertex> Mesh<'a> for CornerTable<TCorner, TVertex> 
where 
    TCorner: Corner + 'a,
    TVertex: Vertex + 'a
{
    type ScalarType = TVertex::ScalarType;

    type EdgeDescriptor = TCorner;
    type VertexDescriptor = TVertex;
    type FaceDescriptor = TCorner;

    type FacesIter = CornerTableFacesIter<'a, TCorner, TVertex>;
    type VerticesIter = CornerTableVerticesIter<'a, TCorner, TVertex>;
    type EdgesIter = CornerTableEdgesIter<'a, TCorner, TVertex>;

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

            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v2_index, v3_index), v1_index, face_idx + 1);
            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v3_index, v1_index), v2_index, face_idx + 2);
            corner_table.corner_from(&mut edge_opposite_corner_map, &mut Edge::new(v1_index, v2_index), v3_index, face_idx);
        }

        return corner_table;
    }

    #[inline]
    fn faces(&'a self) -> Self::FacesIter {
        return Self::FacesIter::new(self);
    }

    fn vertices(&'a self) -> Self::VerticesIter {
        todo!()
    }

    fn edges(&'a self) -> Self::EdgesIter {
        todo!()
    }

    #[inline]
    fn face_positions(&self, face: &Self::FaceDescriptor) -> (Point3<Self::ScalarType>, Point3<Self::ScalarType>, Point3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, face);

        return (
            *walker.get_vertex().get_position(),
            *walker.next().get_vertex().get_position(),
            *walker.next().get_vertex().get_position()
        );
    }

    #[inline]
    fn face_normal(&self, face: &Self::FaceDescriptor) -> UnitVector3<Self::ScalarType> {
        let (p1, p2, p3) = self.face_positions(face);
        return UnitVector3::new_normalize((p2 - p1).cross(&(p3 - p1)));
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

    use crate::mesh::corner_table::{test_helpers::create_unit_square_mesh, connectivity::{vertex::VertexF, corner::DefaultCorner}};

    #[test]
    fn from_vertices_and_indices() {
        let mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Point3::<f32>::new(0.0, 1.0, 0.0), Default::default(), 0),
            VertexF::new(1, Point3::<f32>::new(0.0, 0.0, 0.0), Default::default(), 1),
            VertexF::new(3, Point3::<f32>::new(1.0, 0.0, 0.0), Default::default(), 2),
            VertexF::new(4, Point3::<f32>::new(1.0, 1.0, 0.0), Default::default(), 3)
        ];

        let expected_corners = vec![
            DefaultCorner::new(1, None,    0, 0, Default::default()),
            DefaultCorner::new(2, Some(4), 1, 1, Default::default()),
            DefaultCorner::new(0, None,    2, 2, Default::default()),

            DefaultCorner::new(4, None,    2, 3, Default::default()),
            DefaultCorner::new(5, Some(1), 3, 4, Default::default()),
            DefaultCorner::new(3, None,    0, 5, Default::default())
        ];

        assert_eq!(expected_corners.len(), mesh.corners.len());
        assert_eq!(expected_vertices.len(), mesh.vertices.len());

        for i in 0..expected_corners.len() {
            assert_eq!(expected_corners[i], mesh.corners[i]);
        }

        for i in 0..expected_vertices.len() {
            assert_eq!(expected_vertices[i], mesh.vertices[i]);
        }
    }
}
