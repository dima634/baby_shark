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

    #[inline]
    pub fn next(&mut self) -> &mut Self {
        self.corner = self.table.get_corner(self.corner.get_next_corner_index()).unwrap();
        return self;
    }
    
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

    #[inline]
    pub fn previous(&mut self) -> &mut Self {
        return self.next().next();
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