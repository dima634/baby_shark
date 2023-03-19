use std::mem::swap;

use crate::{mesh::traits::{Mesh, VertexProperties, TopologicalMesh}, geometry::traits::RealNumber, helpers::utils::sort3};

///
/// Wrapper for value of reeb function
/// 
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct ReebValue<TScalar: RealNumber>(TScalar);

pub type ReebFunction<TMesh> = fn(&TMesh, &<TMesh as Mesh>::VertexDescriptor) -> <TMesh as Mesh>::ScalarType;

impl<TScalar: RealNumber> ReebValue<TScalar> {
    pub fn new(value: TScalar) -> Self {
        return Self(value);
    }

    #[inline]
    pub fn value(&self) -> TScalar {
        return self.0;
    }
}

impl<TScalar: RealNumber> Eq for ReebValue<TScalar> {}

impl<TScalar: RealNumber> Ord for ReebValue<TScalar> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.partial_cmp(other).unwrap();
    }
}

impl<TScalar: RealNumber> Default for ReebValue<TScalar> {
    #[inline]
    fn default() -> Self {
        return Self(TScalar::neg_infinity())
    }
}

pub type ReebValues<TMesh> = <TMesh as VertexProperties>::VertexPropertyMap<ReebValue<<TMesh as Mesh>::ScalarType>>;

///
/// Vertex ordered by reeb-function value
/// 
pub struct OrderedVertex<TMesh: Mesh> {
    vertex: TMesh::VertexDescriptor,
    reeb_value: ReebValue<TMesh::ScalarType>,
    order: usize
}

impl<TMesh: Mesh> Copy for OrderedVertex<TMesh> {}

impl<TMesh: Mesh> Clone for OrderedVertex<TMesh> {
    fn clone(&self) -> Self {
        return Self { 
            vertex: self.vertex.clone(), 
            reeb_value: self.reeb_value.clone(), 
            order: self.order.clone() 
        };
    }
}

impl<TMesh: Mesh> OrderedVertex<TMesh> {
    pub fn new(vertex: TMesh::VertexDescriptor, reeb_value: ReebValue<TMesh::ScalarType>, order: usize) -> Self { 
        return Self { vertex, reeb_value, order };
    }

    #[inline]
    pub fn vertex(&self) -> &TMesh::VertexDescriptor {
        return &self.vertex;
    }

    #[inline]
    pub fn is_same_vertex(&self, other: &Self) -> bool {
        return self.vertex == other.vertex;
    }
}

impl<TMesh: Mesh + VertexProperties> OrderedVertex<TMesh> {
    pub fn order_mesh_vertices(mesh: &TMesh, reeb_values: &ReebValues<TMesh>) -> TMesh::VertexPropertyMap<usize> {
        let mut ordered_vertices = mesh.vertices()
            .map(|v| Self::new(v, reeb_values[v], usize::MAX))
            .collect::<Vec<_>>();

        ordered_vertices.sort_unstable_by_key(|v| v.reeb_value);

        let mut vertex_order_map = mesh.create_vertex_properties_map();
        for index in 0..ordered_vertices.len() {
            let ordered_vertex = ordered_vertices[index];
            vertex_order_map[*ordered_vertex.vertex()] = index;
        }

        return vertex_order_map;
    }
}

impl<TMesh: Mesh> PartialEq for OrderedVertex<TMesh> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return self.order == other.order;
    }
}

impl<TMesh: Mesh> Eq for OrderedVertex<TMesh> {}

impl<TMesh: Mesh> PartialOrd for OrderedVertex<TMesh> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

impl<TMesh: Mesh> Ord for OrderedVertex<TMesh> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.order.cmp(&other.order);
    }
}


///
/// Ordered edge
/// 
pub struct OrderedEdge<TMesh: Mesh> {
    start: OrderedVertex<TMesh>,
    end: OrderedVertex<TMesh>,
    edge: TMesh::EdgeDescriptor,
    order: u8
}

impl<TMesh: Mesh + VertexProperties> OrderedEdge<TMesh> {
    pub fn from_edge(
        edge: TMesh::EdgeDescriptor, 
        mesh: &TMesh, 
        bottom_vertex: &OrderedVertex<TMesh>,
        top_vertex: &OrderedVertex<TMesh>,
        reeb_values: &ReebValues<TMesh>,
        vertex_order: &TMesh::VertexPropertyMap<usize>
    ) -> Self {
        let (v1, v2) = mesh.edge_vertices(&edge);

        let mut start = OrderedVertex::new(v1, reeb_values[v1], vertex_order[v2]);
        let mut end =   OrderedVertex::new(v2, reeb_values[v2], vertex_order[v2]);

        // if end < start {
        //     swap(&mut end, &mut start);
        // }

        if end.is_same_vertex(bottom_vertex) || start.is_same_vertex(top_vertex) {
            swap(&mut end, &mut start);
        }

        let mut order = 0;
        
        if start.is_same_vertex(bottom_vertex) && end.is_same_vertex(top_vertex) {
            order = 2;
        } else if end.is_same_vertex(top_vertex) {
            order = 1;
        }

        let ordered_edge = Self {
            edge,
            start,
            end,
            order
        };

        return ordered_edge;
    }

    #[inline]
    pub fn edge(&self) -> &TMesh::EdgeDescriptor {
        return &self.edge;
    }

    #[inline]
    pub fn start(&self) -> &OrderedVertex<TMesh> {
        return &self.start;
    }
    
    #[inline]
    pub fn end(&self) -> &OrderedVertex<TMesh> {
        return &self.end;
    }
}

impl<TMesh: Mesh + VertexProperties> Eq for OrderedEdge<TMesh> {}

impl<TMesh: Mesh + VertexProperties> PartialEq for OrderedEdge<TMesh> {
    #[inline]
    fn eq(&self, _other: &Self) -> bool {
        return false;
    }
}

impl<TMesh: Mesh + VertexProperties> Ord for OrderedEdge<TMesh> {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.order.cmp(&other.order);
    }
}

impl<TMesh: Mesh + VertexProperties> PartialOrd for OrderedEdge<TMesh> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}


///
/// Ordered triangle
/// 
pub struct OrderedTriangle<TMesh: Mesh> {
    e1: OrderedEdge<TMesh>,
    e2: OrderedEdge<TMesh>,
    e3: OrderedEdge<TMesh>,

    bottom_vertex: OrderedVertex<TMesh>,
    middle_vertex: OrderedVertex<TMesh>,
    top_vertex: OrderedVertex<TMesh>
}

impl<TMesh: Mesh + TopologicalMesh + VertexProperties> OrderedTriangle<TMesh> {
    pub fn from_face(
        face: &TMesh::FaceDescriptor, 
        mesh: &TMesh, 
        reeb_values: &ReebValues<TMesh>,
        vertex_order: &TMesh::VertexPropertyMap<usize>
    ) -> Self {
        let (v1, v2, v3) = mesh.face_vertices(face);
        let mut ordered_v1 = OrderedVertex::new(v1, reeb_values[v1], vertex_order[v1]);
        let mut ordered_v2 = OrderedVertex::new(v2, reeb_values[v2], vertex_order[v2]);
        let mut ordered_v3 = OrderedVertex::new(v3, reeb_values[v3], vertex_order[v3]);
        sort3(&mut ordered_v1, &mut ordered_v2, &mut ordered_v3);

        let (e1, e2, e3) = mesh.face_edges(face);
        let mut ordered_e1 = OrderedEdge::from_edge(e1, mesh, &ordered_v1, &ordered_v3, reeb_values, vertex_order);
        let mut ordered_e2 = OrderedEdge::from_edge(e2, mesh, &ordered_v1, &ordered_v3, reeb_values, vertex_order);
        let mut ordered_e3 = OrderedEdge::from_edge(e3, mesh, &ordered_v1, &ordered_v3, reeb_values, vertex_order);
        sort3(&mut ordered_e1, &mut ordered_e2, &mut ordered_e3);
    
        return  Self {
            bottom_vertex: ordered_v1,
            middle_vertex: ordered_v2,
            top_vertex: ordered_v3,

            e1: ordered_e1,
            e2: ordered_e2,
            e3: ordered_e3
        };
    }

    /// Returns edge connecting bottom and middle vertices
    #[inline]
    pub fn e1(&self) -> &OrderedEdge<TMesh> {
        return &self.e1;
    }

    /// Returns edge connecting middle and top vertices
    #[inline]
    pub fn e2(&self) -> &OrderedEdge<TMesh> {
        return &self.e2;
    }

    /// Returns edge connecting top and bottom vertices
    #[inline]
    pub fn e3(&self) -> &OrderedEdge<TMesh> {
        return &self.e3;
    }

    #[inline]
    pub fn bottom_vertex(&self) -> &OrderedVertex<TMesh> {
        return &self.bottom_vertex;
    }

    #[inline]
    pub fn middle_vertex(&self) -> &OrderedVertex<TMesh> {
        return &self.middle_vertex;
    }

    #[inline]
    pub fn top_vertex(&self) -> &OrderedVertex<TMesh> {
        return &self.top_vertex;
    }
}
