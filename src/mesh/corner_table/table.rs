use std::{collections::HashMap, fmt::Display};
use tabled::Table;
use crate::{mesh::traits::{Mesh, TopologicalMesh, MeshMarker}, geometry::traits::RealNumber, helpers::aliases::Vec3};
use self::helpers::Edge;
use super::{
    traversal::{
        CornerTableFacesIter, 
        CornerTableVerticesIter, 
        CornerTableEdgesIter, 
        CornerWalker, 
        faces_around_vertex, 
        vertices_around_vertex, 
        edges_around_vertex
    }, 
    connectivity::{
        corner::{Corner, first_corner_from_corner}, 
        vertex::Vertex
    }, 
    marker::CornerTableMarker, descriptors::EdgeRef
};

pub struct CornerTable<TScalar: RealNumber> {
    pub(super) vertices: Vec<Vertex<TScalar>>,
    pub(super) corners: Vec<Corner>
}

impl<TScalar: RealNumber> Default for CornerTable<TScalar> {
    #[inline]
    fn default() -> Self {
        Self { 
            vertices: Vec::new(), 
            corners: Vec::new() 
        }
    }
}

impl<TScalar: RealNumber> CornerTable<TScalar> {
    #[inline]
    pub fn new() -> Self {
        Default::default()
    }

    #[inline]
    pub fn with_capacity(num_vertices: usize, num_faces: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(num_vertices),
            corners: Vec::with_capacity(num_faces * 3)
        }
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
    /// Returns first corner of face
    pub fn create_face_from_vertices(&mut self, v1: usize, v2: usize, v3: usize) -> usize {
        let c1 = self.create_corner();
        c1.set_vertex_index(v1);

        let c2 = self.create_corner();
        c2.set_vertex_index(v2);

        let c3 = self.create_corner();
        c3.set_vertex_index(v3);

        self.corners.len() - 3
    }

    /// Makes give corners opposite to each other
    #[inline]
    pub fn set_opposite_relationship(&mut self, corner1_index: usize, corner2_index: usize) {
        self.get_corner_mut(corner1_index).unwrap().set_opposite_corner_index(Some(corner2_index));
        self.get_corner_mut(corner2_index).unwrap().set_opposite_corner_index(Some(corner1_index));
    }

    fn corner_from(
        &mut self,
        edge_opposite_corner_map: &mut HashMap<Edge, usize>,
        mut edge: Edge,
        vertex_index: usize
    ) {
        let corner_index = self.corners.len();
        let corner = self.create_corner();
        corner.set_vertex_index(vertex_index);

        // Find opposite corner
        edge.flip();
        let opposite_corner_index = edge_opposite_corner_map.get(&edge);
        edge.flip();

        if let Some(opposite_corner_index) = opposite_corner_index {
            // Set opposite for corners
            corner.set_opposite_corner_index(Some(*opposite_corner_index));
            let opposite_corner = &mut self.corners[*opposite_corner_index];

            opposite_corner.set_opposite_corner_index(Some(corner_index));
    
            edge_opposite_corner_map.insert(edge, corner_index);
        } else {
            // Save directed edge and it`s opposite corner
            edge_opposite_corner_map.insert(edge, corner_index);
        }

        // Set corner index for vertex
        let vertex = &mut self.vertices[vertex_index];
        vertex.set_corner_index(corner_index);
    }
}

///
/// Implementation of mesh trait for corner table.
/// 
/// Edge is represented by index of opposite corner.
/// Vertex is represented by it`s index in vertices vector.
/// Face is represented by index of one of it`s.
/// 
impl<TScalar: RealNumber> Mesh for CornerTable<TScalar> {
    type ScalarType = TScalar;

    /// Index of corner opposite to edge
    type EdgeDescriptor = EdgeRef;
    /// Vertex index
    type VertexDescriptor = usize;
    /// Index of one of face corners
    type FaceDescriptor = usize;

    type FacesIter<'iter> = CornerTableFacesIter<'iter, TScalar>;
    type VerticesIter<'iter> = CornerTableVerticesIter<'iter, TScalar>;
    type EdgesIter<'iter> = CornerTableEdgesIter<'iter, TScalar>;

    fn from_iters(vertices: impl Iterator<Item = Vec3<Self::ScalarType>>, mut faces: impl Iterator<Item = usize>) -> Self {
        let num_faces = faces.size_hint().1.unwrap_or(0);
        let num_vertices = vertices.size_hint().1.unwrap_or(0);
        let mut corner_table = Self::with_capacity(num_vertices, num_faces);

        for vert in vertices {
            let vertex = corner_table.create_vertex();
            vertex.set_position(vert);
        }

        let mut edge_opposite_corner_map = HashMap::<helpers::Edge, usize>::new();

        loop {
            let Some(v1_index) = faces.next() else { break; };
            let Some(v2_index) = faces.next() else { break; };
            let Some(v3_index) = faces.next() else { break; };

            let edge1 = Edge::new(v2_index, v3_index);
            let edge2 = Edge::new(v3_index, v1_index);
            let edge3 = Edge::new(v1_index, v2_index);

            // If edge already exist in map then it is non manifold. For now we will skip faces that introduce non-manifoldness.
            if edge_opposite_corner_map.contains_key(&edge1)
               || edge_opposite_corner_map.contains_key(&edge2) 
               || edge_opposite_corner_map.contains_key(&edge3) 
            {
                continue;        
            }

            corner_table.corner_from(&mut edge_opposite_corner_map, edge1, v1_index);
            corner_table.corner_from(&mut edge_opposite_corner_map, edge2, v2_index);
            corner_table.corner_from(&mut edge_opposite_corner_map, edge3, v3_index);
        }

        corner_table
    }

    #[inline]
    fn faces(&self) -> Self::FacesIter<'_> {
        return Self::FacesIter::new(self);
    }

    #[inline]
    fn vertices(&self) -> Self::VerticesIter<'_> {
        return Self::VerticesIter::new(self);
    }

    #[inline]
    fn edges(&self) -> Self::EdgesIter<'_> {
        return Self::EdgesIter::new(self);
    }

    #[inline]
    fn face_vertices(&self, face: &Self::FaceDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor, Self::VertexDescriptor) {
        let mut walker = CornerWalker::from_corner(self, *face);

        return (
            walker.get_corner().get_vertex_index(),
            walker.next().get_corner().get_vertex_index(),
            walker.next().get_corner().get_vertex_index()
        );
    }

    #[inline]
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Vec3<Self::ScalarType>, Vec3<Self::ScalarType>) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());
        return (
            *walker.next().get_vertex().get_position(),
            *walker.next().get_vertex().get_position()
        );
    }

    #[inline]
    fn edge_vertices(&self, edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor) {
        let mut walker = CornerWalker::from_corner(self, edge.get_corner_index());
        return (
            walker.next().get_corner().get_vertex_index(),
            walker.next().get_corner().get_vertex_index()
        );
    }

    #[inline]
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Vec3<Self::ScalarType> {
        return self.vertices[*vertex].get_position();
    }

    fn vertex_normal(&self, vertex: &Self::VertexDescriptor) -> Option<Vec3<Self::ScalarType>> {
        let mut sum = Vec3::zeros();
        
        faces_around_vertex(self, *vertex, |face_index| {
            sum += self.face_normal(face_index);
        });

        if sum.iter().all(|i| i.is_zero()) {
            return None;
        }

        Some(sum.normalize())
    }
}

impl<TScalar: RealNumber> TopologicalMesh for CornerTable<TScalar> {
    type Position<'a> = CornerWalker<'a, TScalar>;
    
    #[inline]
    fn vertices_around_vertex<TVisit: FnMut(&Self::VertexDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit) {
        vertices_around_vertex(self, *vertex, visit);
    }

    #[inline]
    fn faces_around_vertex<TVisit: FnMut(&Self::FaceDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit) {
        faces_around_vertex(self, *vertex, visit);
    }

    #[inline]
    fn edges_around_vertex<TVisit: FnMut(&Self::EdgeDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit) {
        edges_around_vertex(self, *vertex, visit)
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

        false
    }

    #[inline]
    fn is_edge_on_boundary(&self, edge: &Self::EdgeDescriptor) -> bool {
        self.corners[edge.get_corner_index()].get_opposite_corner_index().is_none()
    }

    #[inline]
    fn edge_faces(&self, edge: &Self::EdgeDescriptor) -> (Self::FaceDescriptor, Option<Self::FaceDescriptor>) {
        let f1 = edge.get_corner_index();
        (
            f1,
            self.corners[f1].get_opposite_corner_index()
        )
    }

    #[inline]
    fn face_edges(&self, face: &Self::FaceDescriptor) -> (Self::EdgeDescriptor, Self::EdgeDescriptor, Self::EdgeDescriptor) {
        let first_corner = first_corner_from_corner(*face);
        (
            Self::EdgeDescriptor::new(first_corner, self),
            Self::EdgeDescriptor::new(first_corner + 1, self),
            Self::EdgeDescriptor::new(first_corner + 2, self)
        )
    }
}

impl<TScalar: RealNumber> MeshMarker for CornerTable<TScalar> {
    type Marker = CornerTableMarker<TScalar>;

    #[inline]
    fn marker(&self) -> Self::Marker {
        CornerTableMarker::new(self)
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

        Ok(())
    }
}

pub mod helpers {
    use std::mem::swap;

    #[derive(Hash, PartialEq, Eq, Clone, Copy)]
    pub struct Edge {
        start_vertex: usize,
        end_vertex: usize
    }

    impl Edge {
        pub fn new(start: usize, end: usize) -> Self {
            Self {
                start_vertex: start,
                end_vertex: end
            }
        }

        #[inline]
        pub fn flip(&mut self) {
            swap(&mut self.start_vertex, &mut self.end_vertex);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{mesh::{
        corner_table::{
            test_helpers::{create_unit_square_mesh, assert_mesh_eq}, 
            connectivity::{vertex::VertexF, corner::Corner}, 
            prelude::CornerTableF
        }, 
        traits::Mesh
    }, helpers::aliases::{Vec3f, Vec3}};

    #[test]
    fn from_vertices_and_indices() {
        let mesh = create_unit_square_mesh();

        let expected_vertices = vec![
            VertexF::new(5, Vec3f::new(0.0, 1.0, 0.0), Default::default()),
            VertexF::new(1, Vec3f::new(0.0, 0.0, 0.0), Default::default()),
            VertexF::new(3, Vec3f::new(1.0, 0.0, 0.0), Default::default()),
            VertexF::new(4, Vec3f::new(1.0, 1.0, 0.0), Default::default())
        ];

        let expected_corners = vec![
            Corner::new(None,    0, Default::default()),
            Corner::new(Some(4), 1, Default::default()),
            Corner::new(None,    2, Default::default()),

            Corner::new(None,    2, Default::default()),
            Corner::new(Some(1), 3, Default::default()),
            Corner::new(None,    0, Default::default())
        ];

        assert_mesh_eq(&mesh, &expected_corners, &expected_vertices);
    }

    #[test]
    fn should_remove_face_that_introduces_non_manifold_edge() {
        let mesh = CornerTableF::from_slices(&[
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, -1.0),
        ], &[
            0, 1, 2,
            0, 1, 4,
            0, 3, 1,
            3, 5, 1,
            1, 5, 2,
        ]);

        assert!(mesh.faces().count() == 4);
    }
}
