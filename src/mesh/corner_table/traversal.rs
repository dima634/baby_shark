use std::collections::HashSet;

use super::{corner_table::CornerTable, connectivity::{traits::{Corner, Vertex}}};

///
/// Can be used to traverse corner table topology
/// 
pub struct CornerWalker<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>,
    corner: &'a TCorner
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerWalker<'a, TCorner, TVertex> {
    pub fn from_corner(table: &'a CornerTable<TCorner, TVertex>, corner: &'a TCorner) -> Self { 
        return Self {
            table, 
            corner 
        }; 
    }

    /// Moves to next corner
    #[inline]
    pub fn next(&mut self) -> &mut Self {
        self.corner = self.table.get_corner(self.corner.get_next_corner_index()).unwrap();
        return self;
    }
    
    /// Moves to opposite corner if exist, otherwise corner stays still
    #[inline]
    pub fn opposite(&mut self) -> &mut Self {
        if let Some(opposite) = self.corner.get_opposite_corner_index() {
            self.corner = self.table.get_corner(opposite).unwrap();
        }
        else {
            debug_assert!(false, "Moving to not existing corner");
        }

        return self;
    }

    /// Moves to previous corner. Shorthand for next().next()
    #[inline]
    pub fn previous(&mut self) -> &mut Self {
        return self.next().next();
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

    /// Returns next corner
    #[inline]
    pub fn get_next_corner(&self) -> &TCorner {
        return self.table.get_corner(self.corner.get_next_corner_index()).unwrap(); 
    }

    /// Returns previous corner
    #[inline]
    pub fn get_previous_corner(&self) -> &TCorner {
        return self.table.get_corner(self.get_next_corner().get_next_corner_index()).unwrap(); 
    }

    /// Returns opposite corner
    #[inline]
    pub fn get_opposite_corner(&self) -> Option<&TCorner> {
        if let Some(opposite) = self.corner.get_opposite_corner_index() {
            return Some(self.table.get_corner(opposite).unwrap());
        }
        else {
            return None;
        }
    }

    /// Returns current corner
    #[inline]
    pub fn get_corner(&self) -> &TCorner {
        return self.corner;
    }

    /// Returns vertex of current corner
    #[inline]
    pub fn get_vertex(&self) -> &TVertex {
        return self.table.get_vertex(self.corner.get_vertex_index()).unwrap();
    }
}

///
/// Iterator over faces of corner table. Face is returned as one of its corners.
///
pub struct CornerTableFacesIter<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>,
    corner_index: usize
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerTableFacesIter<'a, TCorner, TVertex> {
    pub fn new(corner_table: &'a CornerTable<TCorner, TVertex>) -> Self {
        return Self {
            table: corner_table,
            corner_index: 0
        };
    }
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornerTableFacesIter<'a, TCorner, TVertex> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let next_index = self.corner_index;
        self.corner_index += 3;

        match self.table.get_corner(next_index) {
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
/// Iterator over vertices of mesh
/// 
pub struct CornerTableVerticesIter<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>,
    vertex_index: usize
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerTableVerticesIter<'a, TCorner, TVertex> {
    pub fn new(table: &'a CornerTable<TCorner, TVertex>) -> Self {
        return Self { 
            table,
            vertex_index: 0
        };
    }
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornerTableVerticesIter<'a, TCorner, TVertex> {
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
pub struct CornerTableEdgesIter<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>,
    corner_index: usize,
    visited: HashSet<usize>
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerTableEdgesIter<'a, TCorner, TVertex> {
    pub fn new(table: &'a CornerTable<TCorner, TVertex>) -> Self {
        return Self {
            table,
            corner_index: 0,
            visited: HashSet::with_capacity(table.corners.len())
        };
    }
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornerTableEdgesIter<'a, TCorner, TVertex> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        let next_index = self.corner_index;
        self.corner_index += 1;

        match self.table.get_corner(next_index) {
            Some(next) => {
                // Skip deleted and visited edges
                if self.visited.contains(&next_index) || next.is_deleted() {
                    return self.next();
                }

                // Visit current
                self.visited.insert(next_index);

                // Visit opposite, it is referencing same edge as current
                if let Some(opposite) = next.get_opposite_corner_index() {
                    self.visited.insert(opposite);
                }

                return Some(next_index);
            },
            None => return None,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::{corner_table::test_helpers::create_unit_square_mesh, traits::Mesh};

    #[test]
    fn edges_iterator() {
        let mesh = create_unit_square_mesh();
        let expected_edges: Vec<usize> = vec![0, 1, 2, 3, 5];

        assert_eq!(expected_edges.len(), mesh.edges().count());
        
        let pairs = mesh.edges().zip(expected_edges.iter());

        for pair in pairs {
            assert_eq!(pair.0, *pair.1);
        }
    }
}
