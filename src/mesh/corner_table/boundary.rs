use std::ops::ControlFlow;

use crate::mesh::corner_table::traits::Flags;
use super::{flags::clear_visited, traversal::CornerWalker, *};

#[derive(Debug, Clone, Copy)]
pub struct BoundaryRing(CornerId); // Corner opposite to boundary edge in the loop

impl<T: RealNumber> CornerTable<T> {
    pub fn boundary_rings(&self) -> Vec<BoundaryRing> {
        clear_visited(self.corners.iter());

        let mut rings = Vec::new();

        for (idx, corner) in self.corners.iter().enumerate() {
            if corner.is_visited() || corner.is_deleted() {
                continue;
            }

            if let None = corner.opposite_corner() {
                let id = CornerId::new(idx);
                rings.push(BoundaryRing(id));
                
                // Mark corners of the edges in the ring as visited
                self.boundary_edges(BoundaryRing(id), |edge| {
                    self[edge.corner()].set_visited(true);
                    ControlFlow::Continue(())
                });
            }
        }

        rings
    }

    pub fn boundary_edges(&self, ring: BoundaryRing, mut visit: impl FnMut(EdgeId) -> ControlFlow<(), ()>) {
        if let Some(_)  = self[ring.0].opposite_corner() {
            return; // Not a boundary
        }

        if let ControlFlow::Break(_) = visit(EdgeId::new_boundary(ring.0)) {
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

            if let ControlFlow::Break(_) = visit(EdgeId::new_boundary(walker.corner_id())) {
                break;
            }
        }
    }
}
