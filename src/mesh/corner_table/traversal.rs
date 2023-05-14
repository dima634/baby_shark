use std::path::Path;

use crate::{mesh::traits::{mesh_stats::MAX_VERTEX_VALENCE, Position}, geometry::traits::RealNumber, io::stl::StlWriter};

use super::{table::CornerTable, connectivity::{flags::clear_visited, vertex::Vertex, corner::{Corner, first_corner, face, next, previous, face_contains_corner}, traits::Flags}, descriptors::EdgeRef};

///
/// Can be used to traverse corner table topology
/// 
pub struct CornerWalker<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    corner_index: usize
}

impl<'a, TScalar: RealNumber> CornerWalker<'a, TScalar> {
    /// Creates walker starting at given corner
    pub fn from_corner(table: &'a CornerTable<TScalar>, corner_index: usize) -> Self { 
        return Self {
            table, 
            corner_index
        }; 
    }

    /// Creates walker starting at random corner of given vertex
    pub fn from_vertex(table: &'a CornerTable<TScalar>, vertex_index: usize) -> Self { 
        return Self {
            table, 
            corner_index: table.get_vertex(vertex_index).unwrap().get_corner_index()
        }; 
    }

    /// Jumps to given corner
    #[inline]
    pub fn set_current_corner(&mut self, corner_index: usize) -> &mut Self {
        self.corner_index = corner_index;
        return self;
    }

    /// Moves to next corner
    #[allow(clippy::should_implement_trait)]
    #[inline]
    pub fn next(&mut self) -> &mut Self {
        self.corner_index = next(self.get_corner_index());
        return self;
    }
    
    /// Moves to opposite corner if exist, otherwise corner stays still
    #[inline]
    pub fn opposite(&mut self) -> &mut Self {
        if let Some(opposite) = self.get_corner().get_opposite_corner_index() {
            self.corner_index = opposite;
        }
        else {
            debug_assert!(false, "Moving to not existing corner");
        }

        return self;
    }

    /// Moves to previous corner. Shorthand for next().next()
    #[inline]
    pub fn previous(&mut self) -> &mut Self {
        self.corner_index = previous(self.get_corner_index());
        return self;
    }

    /// Moves to right corner
    #[inline]
    pub fn right(&mut self) -> &mut Self {
        return self.next().opposite();
    }

    /// Moves to left corner
    #[inline]
    pub fn left(&mut self) -> &mut Self {
        return self.previous().opposite();
    }

    /// Swings to right around corner vertex
    #[inline]
    pub fn swing_right(&mut self) -> &mut Self {
        return self.previous().opposite().previous();
    }

    /// Swings to left around corner vertex
    #[inline]
    pub fn swing_left(&mut self) -> &mut Self {
        return self.next().opposite().next();
    }

    /// Returns `true` if it is possible to [`Self::swing_right()`] (corner is not on the border), `false` otherwise
    #[inline]
    pub fn can_swing_right(&self) -> bool {
        return self.get_previous_corner().get_opposite_corner_index().is_some();
    }

    /// Returns `true` if it is possible to [`Self::swing_left()`] (corner is not on the border), `false` otherwise
    #[inline]
    pub fn can_swing_left(&self) -> bool {
        return self.get_next_corner().get_opposite_corner_index().is_some();
    }

    /// 
    /// Trying to swing left and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing left walker stays at starting position.
    /// 
    #[inline]
    pub fn swing_left_or_stay(&mut self) -> bool {
        self.next();

        if let Some(opposite) = self.get_corner().get_opposite_corner_index() {
            self.set_current_corner(opposite);
            self.next();
            return true;
        } else {
            self.previous();
            return false;
        }
    }

    /// 
    /// Trying to swing right and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing right walker stays at starting position.
    /// 
    #[inline]
    pub fn swing_right_or_stay(&mut self) -> bool {
        self.previous();

        if let Some(opposite) = self.get_corner().get_opposite_corner_index() {
            self.set_current_corner(opposite);
            self.previous();
            return true;
        } else {
            self.next();
            return false;
        }
    }

    /// Returns next corner
    #[inline]
    pub fn get_next_corner(&self) -> &Corner {
        return self.table.get_corner(next(self.get_corner_index())).unwrap(); 
    }

    /// Returns next corner index
    #[inline]
    pub fn get_next_corner_index(&self) -> usize {
        return next(self.get_corner_index()); 
    }

    /// Returns previous corner index
    #[inline]
    pub fn get_previous_corner_index(&self) -> usize {
        return previous(self.get_corner_index()); 
    }

    /// Returns previous corner
    #[inline]
    pub fn get_previous_corner(&self) -> &Corner {
        return self.table.get_corner(self.get_previous_corner_index()).unwrap(); 
    }

    /// Returns opposite corner
    #[inline]
    pub fn get_opposite_corner(&self) -> Option<&Corner> {
        if let Some(opposite) = self.get_corner().get_opposite_corner_index() {
            return Some(self.table.get_corner(opposite).unwrap());
        }
        else {
            return None;
        }
    }

    /// Returns current corner
    #[inline]
    pub fn get_corner(&self) -> &Corner {
        return self.table.get_corner(self.corner_index).unwrap();
    }

    /// Returns current corner index
    #[inline]
    pub fn get_corner_index(&self) -> usize {
        return self.corner_index;
    }

    /// Returns vertex of current corner
    #[inline]
    pub fn get_vertex(&self) -> &Vertex<TScalar> {
        return self.table.get_vertex(self.get_corner().get_vertex_index()).unwrap();
    }
}

impl<'a, TScalar: RealNumber> Position<'a, CornerTable<TScalar>> for CornerWalker<'a, TScalar> {
    fn from_vertex_on_face(
        mesh: &'a CornerTable<TScalar>, 
        corner: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor, 
        vertex: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor
    ) -> Self { 
        let mut walker = CornerWalker::from_corner(mesh, *corner);

        if walker.get_corner().get_vertex_index() == *vertex {
            return walker;
        }

        walker.next();
        
        if walker.get_corner().get_vertex_index() == *vertex {
            return walker;
        }

        walker.next();

        if walker.get_corner().get_vertex_index() == *vertex {
            return walker;
        }

        unreachable!("Input must be invalid or non-manifold");
    }

    #[inline]
    fn from_edge_on_face(
        mesh: &'a CornerTable<TScalar>, 
        f: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor, 
        edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor
    ) -> Self {
        let corner = if face_contains_corner(face(*f), edge.get_corner_index()) { 
            edge.get_corner_index()
        } else { 
            mesh.corners[edge.get_corner_index()].get_opposite_corner_index().unwrap() 
        };

        return CornerWalker::from_corner(mesh, corner);
    }

    #[inline]
    fn set_from_vertex_on_face(
        &mut self, 
        corner: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor, 
        vertex: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor
    ) -> &mut Self {
        self.set_current_corner(first_corner(face(*corner)));

        while self.get_corner().get_vertex_index() != *vertex {
            self.next();
        }

        return self;
    }
    
    #[inline]
    fn set_from_edge_on_face(
        &mut self, 
        f: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::FaceDescriptor, 
        edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor
    ) -> &mut Self {
        if face_contains_corner(face(*f), edge.get_corner_index()) { 
            self.set_current_corner(edge.get_corner_index());
        } else { 
            let corner = self.table.corners[edge.get_corner_index()].get_opposite_corner_index().unwrap();
            self.set_current_corner(corner);
        };

        return self;
    }

    #[inline]
    fn next(&mut self) -> &mut Self {
        return self.next();
    }

    #[inline]
    fn get_vertex(&self) -> <CornerTable<TScalar> as crate::mesh::traits::Mesh>::VertexDescriptor {
        return self.get_corner().get_vertex_index();
    }

    #[inline]
    fn from_edge(mesh: &'a CornerTable<TScalar>, edge: &<CornerTable<TScalar> as crate::mesh::traits::Mesh>::EdgeDescriptor) -> Self {
        return CornerWalker::from_corner(mesh, edge.get_corner_index());
    }

    #[inline]
    fn opposite(&mut self) -> &mut Self {
        return self.opposite();
    }
}

///
/// Iterator over faces of corner table. Face is returned as one of its corners.
///
pub struct CornerTableFacesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    corner_index: usize
}

impl<'a, TScalar: RealNumber> CornerTableFacesIter<'a, TScalar> {
    pub fn new(corner_table: &'a CornerTable<TScalar>) -> Self {
        return Self {
            table: corner_table,
            corner_index: 0
        };
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableFacesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(next) = self.table.get_corner(self.corner_index) {
            if !next.is_deleted() {
                break;
            }

            self.corner_index += 3;
        }

        match self.table.get_corner(self.corner_index) {
            Some(_) => {
                let current = self.corner_index;
                self.corner_index += 3;
    
                return Some(current);
            },
            None => return None,
        }
    }
}

///
/// Iterator over vertices of mesh
/// 
pub struct CornerTableVerticesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    vertex_index: usize
}

impl<'a, TScalar: RealNumber> CornerTableVerticesIter<'a, TScalar> {
    pub fn new(table: &'a CornerTable<TScalar>) -> Self {
        return Self { 
            table,
            vertex_index: 0
        };
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableVerticesIter<'a, TScalar> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let next_index = self.vertex_index;
        self.vertex_index += 1;

        match self.table.get_vertex(next_index) {
            Some(next) => {
                // Skip deleted
                if next.is_deleted() {
                    return self.next();
                }

                return Some(next_index);
            },
            None => return None,
        }
    }
}

///
/// Iterator over edges of mesh. Edge is returned as corner opposite to it. Uses `is_visited` flag
/// 
pub struct CornerTableEdgesIter<'a, TScalar: RealNumber> {
    table: &'a CornerTable<TScalar>,
    corner_index: usize
}

impl<'a, TScalar: RealNumber> CornerTableEdgesIter<'a, TScalar> {
    pub fn new(table: &'a CornerTable<TScalar>) -> Self {
        clear_visited(table.corners.iter());
        return Self {
            table,
            corner_index: 0
        };
    }
}

impl<'a, TScalar: RealNumber> Iterator for CornerTableEdgesIter<'a, TScalar> {
    type Item = EdgeRef;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(n) = self.table.get_corner(self.corner_index) {
            if !(n.is_visited() || n.is_deleted()) {
                break;
            }

            self.corner_index += 1;
        }

        match self.table.get_corner(self.corner_index) {
            Some(next) => {
                // Visit current
                next.set_visited(true);

                // Visit opposite, it is referencing same edge as current
                if let Some(opposite_index) = next.get_opposite_corner_index() {
                    self.table.get_corner(opposite_index).unwrap().set_visited(true);
                }

                // Move to next
                let current = self.corner_index;
                self.corner_index += 1;

                let edge = EdgeRef::new(current, self.table);
                return Some(edge);
            },
            None => return None,
        }
    }
}

/// Iterates over corners that are adjacent to given vertex
pub fn corners_around_vertex<TScalar: RealNumber, TFunc: FnMut(&usize)>(corner_table: &CornerTable<TScalar>, vertex_index: usize, mut visit: TFunc) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_index);
    walker.previous();
    let started_at = walker.get_corner_index();
    let mut border_reached = false;

    loop {
        visit(&walker.get_next_corner_index());

        walker.previous();
        
        if walker.get_corner().get_opposite_corner_index().is_none() {
            border_reached = true;
            break;
        }

        walker.opposite();

        if started_at == walker.get_corner_index() {
            break;
        }
    }

    walker.set_current_corner(started_at);

    if border_reached && walker.get_corner().get_opposite_corner_index().is_some() {
        walker.opposite();

        loop {
            visit(&walker.get_previous_corner_index());

            walker.next();

            if walker.get_corner().get_opposite_corner_index().is_none() {
                break;
            }
        
            walker.opposite();
        }
    }
}

pub fn collect_corners_around_vertex<TScalar: RealNumber>(corner_table: &CornerTable<TScalar>, vertex_index: usize) -> Vec<usize> {
    let mut corners = Vec::with_capacity(MAX_VERTEX_VALENCE);
    corners_around_vertex(corner_table, vertex_index, |corner_index| {
        corners.push(*corner_index)
    });

    return corners;
}

/// Iterates over one-ring vertices of vertex
pub fn vertices_around_vertex<TScalar: RealNumber, TFunc: FnMut(&usize)>(corner_table: &CornerTable<TScalar>, vertex_index: usize, mut visit: TFunc) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_index);
    walker.previous();
    let started_at = walker.get_corner_index();
    let mut border_reached = false;

    loop {
        visit(&walker.get_corner().get_vertex_index());

        walker.previous();

        if walker.get_corner().get_opposite_corner_index().is_none() {
            border_reached = true;
            break;
        }

        walker.opposite();

        if started_at == walker.get_corner_index() {
            break;
        }
    }

    if border_reached {
        walker.set_current_corner(started_at).previous();
        loop {
            visit(&walker.get_corner().get_vertex_index());

            walker.next();

            if walker.get_corner().get_opposite_corner_index().is_none() {
                break;
            }
        
            walker.opposite();
        }
    }
}

/// Iterates over one-ring faces of vertex. Face is returned as one of it`s corners.
pub fn faces_around_vertex<TScalar: RealNumber, TFunc: FnMut(&usize)>(corner_table: &CornerTable<TScalar>, vertex_index: usize, mut visit: TFunc) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_index);
    walker.previous();
    let started_at = walker.get_corner_index();
    let mut border_reached = false;

    loop {
        visit(&walker.get_corner_index());

        walker.previous();
        
        if walker.get_corner().get_opposite_corner_index().is_none() {
            border_reached = true;
            break;
        }

        walker.opposite();

        if started_at == walker.get_corner_index() {
            break;
        }
    }

    walker.set_current_corner(started_at);

    if border_reached && walker.get_corner().get_opposite_corner_index().is_some() {
        walker.opposite();

        loop {    
            visit(&walker.get_corner_index());

            walker.next();

            if walker.get_corner().get_opposite_corner_index().is_none() {
                break;
            }
        
            walker.opposite();
        }
    }
}

/// Iterates over edges incident to vertex. Edge is represented by opposite corner index.
pub fn edges_around_vertex<TScalar: RealNumber, TFunc: FnMut(&EdgeRef)>(corner_table: &CornerTable<TScalar>, vertex_index: usize, mut visit: TFunc) {
    let mut walker = CornerWalker::from_vertex(corner_table, vertex_index);
    walker.next();
    let started_at = walker.get_corner_index();
    let mut border_reached = false;

    loop {
        visit(&EdgeRef::new(walker.get_corner_index(), corner_table));

        if walker.get_corner().get_opposite_corner_index().is_none() {
            border_reached = true;
            break;
        }

        walker.opposite().previous();
        
        if started_at == walker.get_corner_index() {
            break;
        }
    }

    if border_reached  {
        walker.set_current_corner(started_at);
        walker.next();

        loop {
            visit(&EdgeRef::new(walker.get_corner_index(), corner_table));

            if walker.get_corner().get_opposite_corner_index().is_none() {
                break;
            }
    
            walker.opposite().next();
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::{
        corner_table::{
            test_helpers::{create_unit_square_mesh, create_unit_cross_square_mesh}, 
            traversal::{vertices_around_vertex, faces_around_vertex, corners_around_vertex}, descriptors::EdgeRef
        }, 
        traits::Mesh
    };
    
    #[test]
    fn edges_iterator() {
        let mesh = create_unit_square_mesh();
        let expected_edges: Vec<EdgeRef> = vec![
            EdgeRef::new(0, &mesh), 
            EdgeRef::new(1, &mesh), 
            EdgeRef::new(2, &mesh), 
            EdgeRef::new(3, &mesh), 
            EdgeRef::new(5, &mesh)
        ];

        assert_eq!(expected_edges.len(), mesh.edges().count());
        
        let pairs = mesh.edges().zip(expected_edges.iter());

        for pair in pairs {
            assert_eq!(pair.0, *pair.1);
        }
    }

    // Corners iter macro

    #[test]
    fn corners_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![11, 2, 5, 8];
        let mut corners: Vec<usize> = Vec::new();

        corners_around_vertex(&mesh, 4, |corner_index| corners.push(*corner_index));

        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![10, 0];
        let mut corners: Vec<usize> = Vec::new();

        corners_around_vertex(&mesh, 0, |corner_index| corners.push(*corner_index));

        assert_eq!(corners, expected_corners);
    }

    // Vertices iter macro

    #[test]
    fn vertices_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices: Vec<usize> = vec![0, 1, 2, 3];
        let mut vertices: Vec<usize> = Vec::new();
        vertices_around_vertex(&mesh, 4, |vertex_index| vertices.push(*vertex_index));
    
        assert_eq!(vertices, expected_vertices);
    }

    #[test]
    fn vertices_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_vertices: Vec<usize> = vec![3, 4, 1];
        let mut vertices: Vec<usize> = Vec::new();
        vertices_around_vertex(&mesh, 0, |vertex_index| vertices.push(*vertex_index));
    
        assert_eq!(vertices, expected_vertices);
    }

    // Faces iter macro

    #[test]
    fn faces_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces: Vec<usize> = vec![10, 1, 4, 7];
        let mut faces: Vec<usize> = Vec::new();
        faces_around_vertex(&mesh, 4, |face_index| faces.push(*face_index));
    
        assert_eq!(faces, expected_faces);
    }

    #[test]
    fn faces_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_faces: Vec<usize> = vec![9, 1];
        let mut faces: Vec<usize> = Vec::new();
        faces_around_vertex(&mesh, 0, |face_index| faces.push(*face_index));
    
        assert_eq!(faces, expected_faces);
    }

}
