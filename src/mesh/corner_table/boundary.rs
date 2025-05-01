use super::*;
use std::collections::HashSet;
use std::ops::ControlFlow;

/// A ring of boundary edges in a mesh.
#[derive(Debug, Clone, Copy)]
pub struct BoundaryRing(CornerId); // Corner opposite to boundary edge in the loop

impl<S: RealNumber> CornerTable<S> {
    /// Returns a vector of boundary rings in the mesh.
    pub fn boundary_rings(&self) -> Vec<BoundaryRing> {
        let mut rings = Vec::new();
        let mut visited = HashSet::new();

        for edge in self.edges() {
            if !visited.insert(edge.corner()) {
                continue;
            }

            let corner_id = edge.corner();
            let corner = &self[corner_id];

            if let None = corner.opposite_corner() {
                rings.push(BoundaryRing(corner_id));

                // Mark corners of the edges in the ring as visited
                self.boundary_edges(BoundaryRing(corner_id), |edge| {
                    visited.insert(edge.corner());
                    ControlFlow::Continue(())
                });
            }
        }

        rings
    }

    /// Visits all edges in a boundary ring.
    /// # Arguments
    /// * `ring` - The boundary ring to visit.
    /// * `visit` - A closure that will be called for each edge in the ring.
    pub fn boundary_edges(
        &self,
        ring: BoundaryRing,
        mut visit: impl FnMut(EdgeId) -> ControlFlow<(), ()>,
    ) {
        if let Some(_) = self[ring.0].opposite_corner() {
            return; // Not a boundary
        }

        if let ControlFlow::Break(_) = visit(EdgeId::new(ring.0)) {
            return;
        }

        let mut walker = CornerWalker::from_corner(self, ring.0);

        loop {
            walker.move_to_previous();
            while walker.try_swing_right() {}

            walker.move_to_previous();

            if walker.corner_id() == ring.0 {
                break; // We are back to the start
            }

            if let ControlFlow::Break(_) = visit(EdgeId::new(walker.corner_id())) {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::ops::ControlFlow;
    use super::test_helpers::*;

    #[test]
    fn test_boundary_rings() {
        let mesh = create_unit_square_mesh();
        assert_eq!(mesh.boundary_rings().len(), 1);
    }

    
    #[test]
    fn test_boundary_edges() {
        let mesh = create_unit_square_mesh();
        let ring = mesh.boundary_rings()[0];
        
        let mut count = 0;
        mesh.boundary_edges(ring, |_| {
            count += 1;
            ControlFlow::Continue(())
        });

        assert_eq!(count, 4);
    }
}
