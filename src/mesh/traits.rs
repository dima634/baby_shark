use std::{hash::Hash, fmt::Display, ops::{Index, IndexMut}};

use nalgebra::{Point3, Vector3};

use crate::{geometry::{traits::RealNumber, primitives::triangle3::Triangle3}, helpers::aliases::Vec3};

pub trait Edge {
    type VertexDescriptor;

    fn start(&self) -> Self::VertexDescriptor;
    fn end(&self) -> Self::VertexDescriptor;
}

pub trait Vertex {
    type ScalarType: RealNumber;

    fn get_position(&self) -> &Point3<Self::ScalarType>;
    fn set_position(&mut self, point: Point3<Self::ScalarType>) -> &mut Self;
}

///
/// Triangular mesh
/// 
pub trait Mesh {
    type ScalarType: RealNumber;

    type EdgeDescriptor: PartialEq + Eq + Ord + Clone + Copy + Hash + Display;
    type VertexDescriptor: PartialEq + Eq + Ord + Clone + Copy + Hash + Display;
    type FaceDescriptor: PartialEq + Eq + Ord + Clone + Copy + Hash + Display;

    type FacesIter<'iter>: Iterator<Item = Self::FaceDescriptor> where Self: 'iter;
    type VerticesIter<'iter>: Iterator<Item = Self::VertexDescriptor> where Self: 'iter;
    type EdgesIter<'iter>: Iterator<Item = Self::EdgeDescriptor> where Self: 'iter;

    /// Creates mesh from vertices and face indices
    fn from_vertex_and_face_iters(vertices: impl Iterator<Item = Vec3<Self::ScalarType>>, faces: impl Iterator<Item = usize>) -> Self;

    /// Iterator over mesh faces
    fn faces(&self) -> Self::FacesIter<'_>;
    /// Iterator over mesh vertices
    fn vertices(&self) -> Self::VerticesIter<'_>;
    /// Iterator over mesh edges
    fn edges(&self) -> Self::EdgesIter<'_>;

    /// Return vertices of given face
    fn face_vertices(&self, face: &Self::FaceDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor, Self::VertexDescriptor);
    /// Returns edge length
    fn edge_positions(&self, edge: &Self::EdgeDescriptor) -> (Vec3<Self::ScalarType>, Vec3<Self::ScalarType>);
    /// Return vertices of given edge
    fn edge_vertices(&self, edge: &Self::EdgeDescriptor) -> (Self::VertexDescriptor, Self::VertexDescriptor);

    /// Returns vertex position
    fn vertex_position(&self, vertex: &Self::VertexDescriptor) -> &Vec3<Self::ScalarType>;
    /// Returns vertex normal (average of one-ring face normals)
    fn vertex_normal(&self, vertex: &Self::VertexDescriptor) -> Option<Vec3<Self::ScalarType>>;

    /// Creates mesh from vertices and face indices saved in slices
    #[inline]
    fn from_vertex_and_face_slices(vertices: &[Vec3<Self::ScalarType>], faces: &[usize]) -> Self where Self: Sized {
        Self::from_vertex_and_face_iters(vertices.iter().cloned(), faces.iter().cloned())
    }

    /// Returns positions of face vertices in ccw order
    #[allow(clippy::type_complexity)]
    #[inline]
    fn face_positions(&self, face: &Self::FaceDescriptor) -> Triangle3<Self::ScalarType> {
        let (v1, v2, v3) = self.face_vertices(face);
        return Triangle3::new(
            *self.vertex_position(&v1),
            *self.vertex_position(&v2),
            *self.vertex_position(&v3)
        );
    }

    /// Returns face normal
    #[inline]
    fn face_normal(&self, face: &Self::FaceDescriptor) -> Vector3<Self::ScalarType> {
        let triangle = self.face_positions(face);
        triangle.get_normal()
    }

    /// Returns edge length
    #[inline]
    fn edge_length(&self, edge: &Self::EdgeDescriptor) -> Self::ScalarType {
        let (v1, v2) = self.edge_positions(edge);
        (v1 - v2).norm()
    }

    /// Returns edge length
    #[inline]
    fn edge_length_squared(&self, edge: &Self::EdgeDescriptor) -> Self::ScalarType {
        let (v1, v2) = self.edge_positions(edge);
        (v1 - v2).norm_squared()
    }
}

///
/// Position on face corner
/// 
pub trait Position<'a, TMesh: Mesh> {
    /// Create new position on face corner
    fn from_vertex_on_face(mesh: &'a TMesh, face: &TMesh::FaceDescriptor, vertex: &TMesh::VertexDescriptor) -> Self;

    /// Create new position on `face` corner opposite to `edge`
    fn from_edge_on_face(mesh: &'a TMesh, face: &TMesh::FaceDescriptor, edge: &TMesh::EdgeDescriptor) -> Self;

    /// Create new position on any corner opposite to `edge`
    fn from_edge(mesh: &'a TMesh, edge: &TMesh::EdgeDescriptor) -> Self;

    /// Set position to given corner at vertex on `face`
    fn set_from_vertex_on_face(&mut self, face: &TMesh::FaceDescriptor, vertex: &TMesh::VertexDescriptor) -> &mut Self;

    /// Set position to corner opposite to given `edge` on `face`
    fn set_from_edge_on_face(&mut self, face: &TMesh::FaceDescriptor, edge: &TMesh::EdgeDescriptor) -> &mut Self;

    /// Move to next vertex on face
    fn next(&mut self) -> &mut Self;

    /// Move to opposite corner
    fn opposite(&mut self) -> &mut Self;

    /// Returns current vertex
    fn get_vertex(&self) -> TMesh::VertexDescriptor;
}

///
/// Triangular mesh that supports topological queries
/// 
pub trait TopologicalMesh: Mesh + Sized{
    type Position<'a>: Position<'a, Self>;

    /// Iterates over one-ring vertices of vertex
    fn vertices_around_vertex<TVisit: FnMut(&Self::VertexDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit);
    /// Iterates over one-ring faces of vertex
    fn faces_around_vertex<TVisit: FnMut(&Self::FaceDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit);
    /// Iterates over edges incident to vertex
    fn edges_around_vertex<TVisit: FnMut(&Self::EdgeDescriptor)>(&self, vertex: &Self::VertexDescriptor, visit: TVisit);

    /// Return `true` if vertex is on boundary, `false` otherwise
    fn is_vertex_on_boundary(&self, vertex: &Self::VertexDescriptor) -> bool;
    /// Return `true` if edge is on boundary, `false` otherwise
    fn is_edge_on_boundary(&self, edge: &Self::EdgeDescriptor) -> bool;

    /// Returns incident faces of edge
    fn edge_faces(&self, edge: &Self::EdgeDescriptor) -> (Self::FaceDescriptor, Option<Self::FaceDescriptor>);

    /// Returns edges of face
    fn face_edges(&self, face: &Self::FaceDescriptor) -> (Self::EdgeDescriptor, Self::EdgeDescriptor, Self::EdgeDescriptor);

    fn validate_topology(&self) -> bool;
}

///
/// Triangular mesh that supports editing operations
/// 
pub trait EditableMesh: Mesh {
    /// Collapse `edge` at given point. This method do not perform checks if operation is safe.
    fn collapse_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Vec3<Self::ScalarType>);
    // Flip `edge`. This method do not perform checks if operation is safe.
    fn flip_edge(&mut self, edge: &Self::EdgeDescriptor);
    // Split `edge` at given point.
    fn split_edge(&mut self, edge: &Self::EdgeDescriptor, at: &Vec3<Self::ScalarType>);
    /// Shift vertex to new position.
    fn shift_vertex(&mut self, vertex: &Self::VertexDescriptor, to: &Vec3<Self::ScalarType>);

    /// Returns `true` when edge exist in mesh, otherwise - `false`.
    /// Can be used to check if edge was deleted by [collapse_edge] method.
    fn edge_exist(&self, edge: &Self::EdgeDescriptor) -> bool;
}

///
/// Can be used to set flags for mesh primitives.
/// Is used by some algorithms to mark processed faces/edges/vertices.
/// 
pub trait Marker<TMesh: Mesh> {
    fn mark_face(&mut self, face: &TMesh::FaceDescriptor, marked: bool);
    fn is_face_marked(&self, face: &TMesh::FaceDescriptor) -> bool;
    
    fn mark_vertex(&mut self, vertex: &TMesh::VertexDescriptor, marked: bool);
    fn is_vertex_marked(&self, vertex: &TMesh::VertexDescriptor) -> bool;
    
    fn mark_edge(&mut self, edge: &TMesh::EdgeDescriptor, marked: bool);
    fn is_edge_marked(&self, edge: &TMesh::EdgeDescriptor) -> bool;
}

/// Mesh that support [Marker] API
pub trait MeshMarker: Mesh + Sized {
    type Marker: Marker<Self>;

    /// Returns marker
    fn marker(&self) -> Self::Marker;
}

/// Property map. Can be used for fast access to properties of given entity.
pub trait PropertyMap<TKey, TProperty>: 
    Index<TKey, Output = TProperty> +
    IndexMut<TKey, Output = TProperty>
{
    fn get(&self, key: &TKey) -> Option<&TProperty>;
    fn get_mut(&mut self, key: &TKey) -> Option<&mut TProperty>;
}

///
/// Mesh that supports property maps for vertices.
/// Vertex-property map can be used to associate arbitrary data with vertices of mesh with fast access to it by vertex reference.
/// Property map is guaranteed to be valid as far as mesh is not modified.
/// 
pub trait VertexProperties: Mesh {
    type VertexPropertyMap<TProperty: Default>: PropertyMap<Self::VertexDescriptor, TProperty>;

    fn create_vertex_properties_map<TProperty: Default>(&self) -> Self::VertexPropertyMap<TProperty>;
}

pub trait SplitFaceAtPoint: Mesh {
    fn split_face(&mut self, face: & Self::FaceDescriptor, point: Vec3<Self::ScalarType>);
}

/// Contains constants which defines what is good mesh
pub mod mesh_stats {
    pub const IDEAL_INTERIOR_VERTEX_VALENCE: usize = 6;
    pub const IDEAL_BOUNDARY_VERTEX_VALENCE: usize = 4;

    /// Constant used to preallocate memory for some algorithms, it is not limiting anything
    pub const MAX_VERTEX_VALENCE: usize = 10;
}
