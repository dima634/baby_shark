use nalgebra::Point2;

use crate::geometry::{traits::{RealNumber, Intersects}, primitives::line_segment2::LineSegment2, orientation::{orientation2d, Orientation}};

use super::{delaunay::{Triangulation2, self}, halfedge::{prev_halfedge, next_halfedge}};

pub struct ConstrainedTriangulation2<'points, TScalar: RealNumber> {
    delaunay: Triangulation2<'points, TScalar>,
    unsafe_flips: Vec<usize>
}

impl<'points, TScalar: RealNumber> ConstrainedTriangulation2<'points, TScalar> {
    pub fn new() -> Self {
        return Self { 
            delaunay: Triangulation2::new(),
            unsafe_flips: Vec::new()
        };
    }

    pub fn triangulate(&mut self, points: &'points [Point2<TScalar>]) {
        // Start from unconstrained delaunay triangulation and insert constrained edges on by one
        self.delaunay.triangulate(points);
    }

    fn insert_edge(&mut self, edge: Edge) -> Result<(), ()> {
        let halfedge = &self.delaunay.halfedge;

        // Find first intersection edge
        let mut intersected_edge = None;

        let edge_start_pos = self.delaunay.vertex_position(edge.start);
        let edge_end_pos = self.delaunay.vertex_position(edge.end);
        let edge_segment = LineSegment2::new(*edge_start_pos, *edge_end_pos);

        let start = halfedge.outgoing_halfedge(edge.start);
        let mut he = start;
        let mut border = false;
        
        loop {
            let e = next_halfedge(he);
            let (v1, v2) = halfedge.halfedge_vertices(e);
            let v1_pos = self.delaunay.vertex_position(v1);
            let v2_pos = self.delaunay.vertex_position(v2);
            let segment = LineSegment2::new(*v1_pos, *v2_pos);

            if segment.intersects_at(&edge_segment).is_some() {
                intersected_edge = Some(e);
                break;
            }

            if let Some(next) = halfedge.opposite_halfedge(prev_halfedge(he)) {
                if next == start {
                    break;
                }

                he = next;
            } else {
                border = true;
                break;
            }
        }

        if border && halfedge.opposite_halfedge(start).is_some() {
            he = halfedge.opposite_halfedge(start).unwrap();
            he = next_halfedge(he);
            
            loop {
                let e = next_halfedge(he);
                let (v1, v2) = halfedge.halfedge_vertices(e);
                let v1_pos = self.delaunay.vertex_position(v1);
                let v2_pos = self.delaunay.vertex_position(v2);
                let segment = LineSegment2::new(*v1_pos, *v2_pos);
    
                if segment.intersects_at(&edge_segment).is_some() {
                    intersected_edge = Some(e);
                    break;
                }
                
                if let Some(opp) = halfedge.opposite_halfedge(he) {
                    he = next_halfedge(opp);
                } else {
                    break;
                }
            }
        }

        if intersected_edge.is_none() {
            return Err(());
        }
        delaunay::debugging::save_to_stl(&self.delaunay, "test.stl".into());

        let intersected_edge = intersected_edge.unwrap();
        let (v1, v2) = self.delaunay.halfedge.halfedge_vertices(intersected_edge);
        println!("{}-{}", v1, v2);

        let mut he = self.delaunay.halfedge.flip_edge(intersected_edge);
        let (v1, v2) = self.delaunay.halfedge.halfedge_vertices(he);
        println!("{}-{}", v1, v2);
        delaunay::debugging::save_to_stl(&self.delaunay, "test.stl".into());

        he = self.next_intersected_edge(&edge_segment, he);

        loop {
            let (v1, v2) = self.delaunay.halfedge.halfedge_vertices(he);
            let (_, v3) = self.delaunay.halfedge.halfedge_vertices(next_halfedge(he));
            let (v4, _) = self.delaunay.halfedge.halfedge_vertices(prev_halfedge(self.delaunay.halfedge.opposite_halfedge(he).unwrap()));

            let v1_pos = self.delaunay.vertex_position(v1);
            let v2_pos = self.delaunay.vertex_position(v2);
            let v3_pos = self.delaunay.vertex_position(v3);
            let v4_pos = self.delaunay.vertex_position(v4);

            let safe_flip = 
                orientation2d(v3_pos, v4_pos, v2_pos) == Orientation::CounterClockwise && 
                orientation2d(v3_pos, v1_pos, v4_pos) == Orientation::CounterClockwise;

            println!("edge {}-{}", v1, v2);
            if !safe_flip {
                println!("not safe");
                self.unsafe_flips.push(he);

                he = self.delaunay.halfedge.opposite_halfedge(he).unwrap();
                let next_he = next_halfedge(he);

                if self.intersects(&edge_segment, next_he) {
                    he = next_halfedge(he)
                } else {
                    he = prev_halfedge(he);
                    debug_assert!(self.intersects(&edge_segment, he));
                }

                continue;
            }

            println!("flip");

            he = self.delaunay.halfedge.flip_edge(he);

            delaunay::debugging::save_to_stl(&self.delaunay, "test.stl".into());
            let (v1, _) = self.delaunay.halfedge.halfedge_vertices(he);

            if v1 == edge.end{
                break;
            }
            
            he = self.next_intersected_edge(&edge_segment, he);
        }

        while let Some(he) = self.unsafe_flips.pop() {
            self.delaunay.halfedge.flip_edge(he);
        }

        return Ok(());
    }

    fn next_intersected_edge(&self, edge: &LineSegment2<TScalar>, he: usize) -> usize {
        let prev = prev_halfedge(he);
            
        if self.intersects(edge, prev) {
            return prev;
        } else {
            let opp = self.delaunay.halfedge.opposite_halfedge(he).unwrap();
            debug_assert!(self.intersects(edge, next_halfedge(opp)));
            return next_halfedge(opp);
        }
    }

    fn intersects(&self, edge: &LineSegment2<TScalar>, he: usize) -> bool {
        let (v1, v2) = self.delaunay.halfedge.halfedge_vertices(he);
        let v1_pos = self.delaunay.vertex_position(v1);
        let v2_pos = self.delaunay.vertex_position(v2);
        let segment = LineSegment2::new(*v1_pos, *v2_pos);

        return segment.intersects_at(edge).is_some();
    }
}

struct Edge {
    start: usize,
    end: usize
}

#[cfg(test)]
mod tests {
    use std::iter::repeat_with;

    use nalgebra::Point2;
    use rand::{rngs::StdRng, SeedableRng, Rng};

    use crate::triangulation::delaunay;

    use super::ConstrainedTriangulation2;

    #[test]
    fn test_insert_edge() {
        let mut tri = ConstrainedTriangulation2::new();

        let mut rng = StdRng::seed_from_u64(rand::random());
        let points2d: Vec<Point2<f32>> = repeat_with(|| rng.gen())
            .map(|(x, y)| Point2::new(x, y))
            .take(100)
            .collect();

        tri.triangulate(&points2d);

        tri.insert_edge(super::Edge { start: 0, end: 8 }).expect("msg");
        
        delaunay::debugging::save_to_stl(&tri.delaunay, "test.stl".into());
    }
}
