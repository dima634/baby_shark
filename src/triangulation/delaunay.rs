use std::mem::swap;

use nalgebra::{Point2, Vector2};

use crate::geometry::{traits::{RealNumber, Intersects}, coordinates::{polar2, PolarPoint2}, primitives::{triangle2::Triangle2, line_segment2::LineSegment2}, orientation::{orientation2d, Orientation}};

// struct Vertex<TScalar: RealNumber>{
//     polar: PolarPoint2<TScalar>,
//     original_idx: usize
// }

// impl<TScalar: RealNumber> Vertex<TScalar> {
//     fn new(polar: PolarPoint2<TScalar>, original_idx: usize) -> Self {
//         return Self { polar, original_idx }; 
//     }
// }

///
/// "A faster circle-sweep Delaunay triangulation algorithm" by Ahmad Biniaz and Gholamhossein Dastghaibyfard
/// https://cglab.ca/~biniaz/papers/Sweep%20Circle.pdf
/// 
pub struct Triangulation2<'a, TScalar: RealNumber> {
    pole: Point2<TScalar>,
    points: &'a [Point2<TScalar>],
    polars: Vec<PolarPoint2<TScalar>>,
    frontier: Vec<usize>,
    triangles: Vec<usize>,
    he_twins: Vec<Option<usize>>
}

impl<'a, TScalar: RealNumber> Triangulation2<'a, TScalar> {
    pub fn triangulate(&mut self, points: &'a [Point2<TScalar>]) {
        self.points = points;
        self.reset();

        self.initialize();
    }

    fn initialize(&mut self) {
        // Compute polar coordinates of input point set
        self.pole = self.points.iter().fold(Point2::origin(), |acc, p| acc + p.coords) / num_traits::cast(2).unwrap();
        self.polars = self.points.iter()
            .map(|p| polar2(&self.pole, p))
            .collect();
        self.polars.sort_by(|a, b| a.partial_cmp_by_radius(&b).unwrap());

        // Init frontier
        let a = 0;
        let mut b = 1;
        let mut c = 2;

        if orientation2d(&self.points[a], &self.points[b], &self.points[c]) == Orientation::Clockwise {
            swap(&mut b, &mut c);
        }

        self.frontier.push(a);
        self.frontier.push(b);
        self.frontier.push(c);

        self.add_triangle(a, b, c, None, None, None);
    }

    fn triangulation(&mut self) {
        for p_idx in 3..self.points.len() {
            let (start, end) = self.project_on_frontier(p_idx);
            self.frontier.insert(start, p_idx);
            // self.add_triangle(start, p_idx, end, he1, he2, he3);
            
        }
    }

    fn project_on_frontier(&self, point_idx: usize) -> (usize, usize) {
        let projection = LineSegment2::new(self.points[point_idx], self.pole);
        
        for start in &self.frontier {
            let end = (start + 1) % self.frontier.len();
            let edge = LineSegment2::new(self.points[*start], self.points[end]);

            if let Some(_) = projection.intersects_at(&edge) {
                return (*start, end);
            }
        }

        unreachable!();
    }

    fn add_triangle(&mut self, a: usize, b: usize, c: usize, he1: Option<usize>, he2: Option<usize>, he3: Option<usize>) -> usize {
        let triangle_idx = self.triangles.len();

        self.triangles.push(a);
        self.triangles.push(b);
        self.triangles.push(c);

        self.he_twins.push(he1);
        self.he_twins.push(he2);
        self.he_twins.push(he3);

        return triangle_idx;
    }

    fn link(&mut self, he1: usize, he2: Option<usize>) {
        self.he_twins[he1] = he2;

        if let Some(he2) = he2 {
            self.he_twins[he2] = Some(he1);
        }
    }

    fn reset(&mut self) {
        self.triangles.clear();
        self.he_twins.clear();
    }
}
