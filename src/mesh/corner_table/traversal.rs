use super::{corner_table::CornerTable, connectivity::traits::{Corner, Vertex}};

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
    type Item = &'a TCorner;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.table.get_corner(self.corner_index);
            self.corner_index += 3;

            if !(next.is_some() && next.unwrap().is_deleted()) {
                return next;
            }
        }
    }
}

pub struct CornerTableVerticesIter<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornerTableVerticesIter<'a, TCorner, TVertex> {
    type Item = &'a TVertex;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}
pub struct CornerTableEdgesIter<'a, TCorner: Corner, TVertex: Vertex> {
    table: &'a CornerTable<TCorner, TVertex>
}

impl<'a, TCorner: Corner, TVertex: Vertex> Iterator for CornerTableEdgesIter<'a, TCorner, TVertex> {
    type Item = &'a TCorner;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}