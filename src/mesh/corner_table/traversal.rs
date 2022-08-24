use crate::mesh::traits::good_mesh;
use super::{corner_table::CornerTable, connectivity::{traits::{Corner, Vertex}, flags::clear_visited}};

///
/// Can be used to traverse corner table topology
/// 
pub struct CornerWalker<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>,
    corner_index: usize
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerWalker<'a, TCorner, TVertex> {
    /// Creates walker starting at given corner
    pub fn from_corner(table: &'a CornerTable<TCorner, TVertex>, corner_index: usize) -> Self { 
        return Self {
            table, 
            corner_index
        }; 
    }

    /// Creates walker starting at random corner of given vertex
    pub fn from_vertex(table: &'a CornerTable<TCorner, TVertex>, vertex_index: usize) -> Self { 
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
    #[inline]
    pub fn next(&mut self) -> &mut Self {
        self.corner_index = self.get_corner().get_next_corner_index();
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
        return self.table.get_corner(self.get_corner().get_next_corner_index()).unwrap(); 
    }

    /// Returns previous corner index
    #[inline]
    pub fn get_previous_corner_index(&self) -> usize {
        return self.get_next_corner().get_next_corner_index(); 
    }

    /// Returns previous corner
    #[inline]
    pub fn get_previous_corner(&self) -> &TCorner {
        return self.table.get_corner(self.get_previous_corner_index()).unwrap(); 
    }

    /// Returns opposite corner
    #[inline]
    pub fn get_opposite_corner(&self) -> Option<&TCorner> {
        if let Some(opposite) = self.get_corner().get_opposite_corner_index() {
            return Some(self.table.get_corner(opposite).unwrap());
        }
        else {
            return None;
        }
    }

    /// Returns current corner
    #[inline]
    pub fn get_corner(&self) -> &TCorner {
        return self.table.get_corner(self.corner_index).unwrap();
    }

    /// Returns current corner index
    #[inline]
    pub fn get_corner_index(&self) -> usize {
        return self.corner_index;
    }

    /// Returns vertex of current corner
    #[inline]
    pub fn get_vertex(&self) -> &TVertex {
        return self.table.get_vertex(self.get_corner().get_vertex_index()).unwrap();
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
    corner_index: usize
}

impl<'a, TCorner: Corner, TVertex: Vertex> CornerTableEdgesIter<'a, TCorner, TVertex> {
    pub fn new(table: &'a CornerTable<TCorner, TVertex>) -> Self {
        clear_visited(table.corners.iter());
        return Self {
            table,
            corner_index: 0
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
                if next.is_visited() || next.is_deleted() {
                    return self.next();
                }

                // Visit current
                next.set_visited(true);

                // Visit opposite, it is referencing same edge as current
                if let Some(opposite_index) = next.get_opposite_corner_index() {
                    self.table.get_corner(opposite_index).unwrap().set_visited(true);
                }

                return Some(next_index);
            },
            None => return None,
        }
    }
}

///
/// Iterator over corners around vertex
/// 
pub struct CornersAroundVertexIter<'a, TCorner: Corner, TVertex: Vertex> {
    started_at_corner: usize,
    done: bool,
    swing_left: bool,
    walker: CornerWalker<'a, TCorner, TVertex>
}

// Alias 
pub type VC<'a, TCorner, TVertex> = CornersAroundVertexIter<'a, TCorner, TVertex>;

impl<'a, TCorner: Corner, TVertex: Vertex> CornersAroundVertexIter<'a, TCorner, TVertex> {
    pub fn new(table: &'a CornerTable<TCorner, TVertex>, vertex_index: usize) -> Self { 
        let corner_index = table.get_vertex(vertex_index).unwrap().get_corner_index();
        return Self {
            started_at_corner: corner_index,
            done: false,
            swing_left: true,
            walker: CornerWalker::from_corner(table, corner_index)
        };
    }

    fn next_left(&mut self) -> Option<usize> {
        let corner_index = self.walker.get_corner_index();

        if !self.walker.can_swing_left() {
            self.swing_left = false;
            self.walker.set_current_corner(self.started_at_corner);

            if self.walker.can_swing_right() {
                self.walker.swing_right();
            } else {
                self.done = true;
            }
        } else if self.walker.swing_left().get_corner_index() == self.started_at_corner {
            self.done = true;
            self.swing_left = false;
        }

        return Some(corner_index);
    }
    
    fn next_right(&mut self) -> Option<usize> {
        if self.done {
            return None;
        }

        let corner_index = self.walker.get_corner_index();

        if !self.walker.can_swing_right() || self.walker.swing_right().get_corner_index() == self.started_at_corner {
            self.done = true;
        }

        return Some(corner_index);
    }
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornersAroundVertexIter<'a, TCorner, TVertex> {
    type Item = usize;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.swing_left {
            return self.next_left();
        } else {
            return self.next_right();
        }
    }
}

///
/// Iterates over corner of given vertex with given index
/// 
/// ## Arguments
/// * reference to corner table
/// * vertex index
/// * variable to write corner index
/// * code to execute for each corner (corner index is written to 3rd arg)
/// 
/// ## Example
/// ```ignore
/// let mesh = create_unit_cross_square_mesh();
/// let mut corners: Vec<usize> = Vec::new();
/// let mut corner_index;
/// corners_around_vertex!(&mesh, 0, corner_index, corners.push(corner_index));
/// ```
/// 
#[macro_export]
macro_rules! corners_around_vertex {
    ($table:expr, $vertex:expr, $corner:expr, $expression:expr) => {
        #[allow(unused_imports)]
        use crate::mesh::corner_table::connectivity::traits::Vertex;
        use crate::mesh::corner_table::traversal::CornerWalker;

        let start_index = $table.get_vertex($vertex).unwrap().get_corner_index();
        let mut walker = CornerWalker::from_corner($table, start_index);

        loop {
            $corner = walker.get_corner_index();
            $expression;

            if !walker.can_swing_left() || walker.swing_left().get_corner_index() == start_index {
                break;
            }
        }

        if !walker.can_swing_left() && walker.can_swing_right() {
            walker.swing_right();
            loop {
                $corner = walker.get_corner_index();
                $expression;
    
                if !walker.can_swing_right() || walker.swing_right().get_corner_index() == start_index {
                    break;
                }
            }
        }
    };
}

///
/// Shorthand to collect all corners of vertex using [`corners_around_vertex`] macro
/// 
#[inline]
pub fn corners_around_vertex<TCorner: Corner, TVertex: Vertex>(corner_table: &CornerTable<TCorner, TVertex>, vertex_index: usize) -> Vec<usize> {
    let mut corners: Vec<usize> = Vec::with_capacity(good_mesh::INTERIOR_VERTEX_VALENCE);
    let mut corner_index;

    corners_around_vertex!(corner_table, vertex_index, corner_index, corners.push(corner_index));

    return corners;
}

#[cfg(test)]
mod tests {
    use crate::mesh::{corner_table::{test_helpers::{create_unit_square_mesh, create_unit_cross_square_mesh}, traversal::VC}, traits::Mesh};
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

    #[test]
    fn corners_around_internal_vertex_iter() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![11, 2, 5, 8];
        let corners: Vec<usize> = VC::new(&mesh, 4).collect();

        assert_eq!(corners.len(), expected_corners.len());
        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_boundary_vertex_iter() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![10, 0];
        let corners: Vec<usize> = VC::new(&mesh, 0).collect();

        assert_eq!(corners.len(), expected_corners.len());
        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_internal_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![11, 2, 5, 8];
        let mut corners: Vec<usize> = Vec::new();
        let mut corner_index;

        corners_around_vertex!(&mesh, 4, corner_index, corners.push(corner_index));

        assert_eq!(corners, expected_corners);
    }

    #[test]
    fn corners_around_boundary_vertex_macro() {
        let mesh = create_unit_cross_square_mesh();
        let expected_corners: Vec<usize> = vec![10, 0];
        let mut corners: Vec<usize> = Vec::new();
        let mut corner_index;

        corners_around_vertex!(&mesh, 0, corner_index, corners.push(corner_index));

        assert_eq!(corners, expected_corners);
    }
}
