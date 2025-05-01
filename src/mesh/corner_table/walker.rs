use crate::geometry::traits::RealNumber;
use super::*;

/// Helper structure for traversing corners of a mesh.
pub struct CornerWalker<'a, S: RealNumber> {
    table: &'a CornerTable<S>,
    corner: CornerId,
}

impl<'a, TScalar: RealNumber> CornerWalker<'a, TScalar> {
    /// Creates walker starting at given corner
    #[inline]
    pub fn from_corner(table: &'a CornerTable<TScalar>, corner: CornerId) -> Self {
        debug_assert!(!table[corner].is_deleted());
        Self { table, corner }
    }

    /// Creates walker starting at random corner of given vertex
    #[inline]
    pub fn from_vertex(table: &'a CornerTable<TScalar>, vertex_id: VertexId) -> Self {
        Self::from_corner(table, table[vertex_id].corner())
    }

    /// Jumps to given corner
    #[inline]
    pub fn set_current_corner(&mut self, corner: CornerId) -> &mut Self {
        self.corner = corner;
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Moves to next corner
    #[allow(clippy::should_implement_trait)]
    #[inline]
    pub fn move_to_next(&mut self) -> &mut Self {
        self.corner = self.corner.next();
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Moves to opposite corner if exist, otherwise corner stays still
    #[inline]
    pub fn move_to_opposite(&mut self) -> &mut Self {
        if let Some(opposite) = self.corner().opposite_corner() {
            self.corner = opposite;
        } else {
            debug_assert!(false, "Moving to not existing corner");
        }

        debug_assert!(!self.table[self.corner].is_deleted());

        self
    }

    /// Moves to previous corner. Shorthand for next().next()
    #[inline]
    pub fn move_to_previous(&mut self) -> &mut Self {
        self.corner = self.corner_id().previous();
        debug_assert!(!self.table[self.corner].is_deleted());
        self
    }

    /// Swings to right around corner vertex
    #[inline]
    pub fn swing_right(&mut self) -> &mut Self {
        self.move_to_previous()
            .move_to_opposite()
            .move_to_previous()
    }

    /// Swings to right around corner vertex and returns `true` if it is possible to do so, `false` otherwise.
    /// If it is not possible to swing right walker stays at starting position.
    pub fn try_swing_right(&mut self) -> bool {
        self.move_to_previous();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_previous();
            true
        } else {
            self.move_to_next();
            false
        }
    }

    /// Swings to left around corner vertex
    #[inline]
    pub fn swing_left(&mut self) -> &mut Self {
        return self.move_to_next().move_to_opposite().move_to_next();
    }

    ///
    /// Trying to swing left and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing left walker stays at starting position.
    ///
    #[inline]
    pub fn swing_left_or_stay(&mut self) -> bool {
        self.move_to_next();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_next();
            true
        } else {
            self.move_to_previous();
            false
        }
    }

    ///
    /// Trying to swing right and returns `true` if operation succeeded, `false otherwise`.
    /// In the case when it is not possible to swing right walker stays at starting position.
    ///
    #[inline]
    pub fn swing_right_or_stay(&mut self) -> bool {
        self.move_to_previous();

        if let Some(opposite) = self.corner().opposite_corner() {
            self.set_current_corner(opposite);
            self.move_to_previous();
            true
        } else {
            self.move_to_next();
            false
        }
    }

    /// Returns next corner
    #[inline]
    pub fn next_corner(&self) -> &Corner {
        &self.table[self.corner.next()]
    }

    /// Returns previous corner index
    #[inline]
    pub fn previous_corner_id(&self) -> CornerId {
        self.corner_id().previous()
    }

    /// Returns previous corner
    #[inline]
    pub fn previous_corner(&self) -> &Corner {
        &self.table[self.corner.previous()]
    }

    /// Returns opposite corner
    #[inline]
    pub fn opposite_corner(&self) -> Option<&Corner> {
        self.corner()
            .opposite_corner()
            .map(|corner| &self.table[corner])
    }

    /// Returns current corner
    #[inline]
    pub fn corner(&self) -> &Corner {
        &self.table[self.corner]
    }

    /// Returns current corner index
    #[inline]
    pub fn corner_id(&self) -> CornerId {
        self.corner
    }

    /// Returns vertex of current corner
    #[inline]
    pub fn vertex(&self) -> &Vertex<TScalar> {
        &self.table[self.corner().vertex()]
    }
}

impl<S: RealNumber> CornerTable<S> {
    pub fn walker_from_corner(&self, corner: CornerId) -> CornerWalker<S> {
        CornerWalker::from_corner(self, corner)
    }
}
