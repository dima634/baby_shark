use std::{mem::swap, collections::{HashMap, VecDeque}, path::Path, cmp::Ordering};

use nalgebra::{Point2, Vector2, Point, Point3};
use nalgebra_glm::{min2, max2};
use num_traits::{cast, Float};
use svg::{node::{element::{path::Data, Group, Text, Circle}}, Document};

use crate::{
    geometry::{traits::{RealNumber, Intersects}, 
    primitives::{triangle2::{Triangle2, self}, line_segment2::LineSegment2}, 
    orientation::{orientation2d, Orientation, signed_angle}},
    mesh::polygon_soup::data_structure::PolygonSoup, 
    io::stl::StlWriter, 
    helpers::utils::{sort3, sort3_by}
};

// struct Vertex<TScalar: RealNumber>{
//     polar: PolarPoint2<TScalar>,
//     original_idx: usize
// }

// impl<TScalar: RealNumber> Vertex<TScalar> {
//     fn new(polar: PolarPoint2<TScalar>, original_idx: usize) -> Self {
//         return Self { polar, original_idx }; 
//     }
// }

#[derive(PartialEq, Eq, Debug)]
pub struct PolarPoint2<TScalar: RealNumber>(Point2<TScalar>);

impl<TScalar: RealNumber> PolarPoint2<TScalar> {
    pub fn new(radius: TScalar, angle: TScalar) -> Self {
        return Self(Point2::new(radius, angle));
    }

    #[inline]
    pub fn radius_squared(&self) -> TScalar {
        return self.0.x;
    }

    #[inline]
    pub fn angle(&self) -> TScalar {
        return self.0.y;
    }

    #[inline]
    pub fn partial_cmp_by_radius(&self, other: &Self) -> Option<Ordering> {
        let radius_ord = self.radius_squared().partial_cmp(&other.radius_squared());

        return radius_ord.map(|ord| match ord {
            Ordering::Less => Ordering::Less,
            Ordering::Greater => Ordering::Greater,
            Ordering::Equal => self.angle().partial_cmp(&other.angle()).unwrap()
        });
    }
}

///
/// `x` is radius and `y` is CW angle
/// 
pub fn polar2<TScalar: RealNumber>(pole: &Point2<TScalar>, point: &Point2<TScalar>) -> PolarPoint2<TScalar> {
    let radius_squared = (point - pole).norm();
    let mut acos = Float::atan2(point.y - pole.y, point.x - pole.x);

    if acos.is_negative() {
        acos += TScalar::two_pi();
    }

    return PolarPoint2::new(radius_squared, acos);
}

#[derive(Debug)]
struct FrontierEdge {
    halfedge: usize,
    f_start: usize,
    f_end: usize
}

#[inline]
fn next_halfedge(he: usize) -> usize {
    return if (he % 3) == 2 { he - 2 } else { he + 1 };
}

#[inline]
fn prev_halfedge(he: usize) -> usize {
    return if (he % 3) == 0 { he + 2 } else { he - 1 };
}

#[derive(Debug)]
struct Vertex<TScalar: RealNumber> {
    cartesian: Point2<TScalar>,
    radius_squared: TScalar,
    angle: TScalar
}

impl<TScalar: RealNumber> Vertex<TScalar> {
    #[inline]
    fn angle(&self) -> TScalar {
        return self.angle;
    }
    
    #[inline]
    fn radius_squared(&self) -> TScalar {
        return self.radius_squared;
    }
}

///
/// "A faster circle-sweep Delaunay triangulation algorithm" by Ahmad Biniaz and Gholamhossein Dastghaibyfard
/// https://cglab.ca/~biniaz/papers/Sweep%20Circle.pdf
/// 
#[derive(Debug)]
pub struct Triangulation2<TScalar: RealNumber> {
    pole: Point2<TScalar>,              // origin if polar coordinates, center of bbox
    frontier: VecDeque<usize>,
    vertices: Vec<Vertex<TScalar>>,
    outgoing_he: Vec<usize>,                // halfedges outgoing from vertex
    triangles: Vec<usize>,                  // list of triangle indices
    he_twins: Vec<Option<usize>>,           // list of halfedge twins
    stack: Vec<usize>
}

impl<TScalar: RealNumber> Triangulation2<TScalar> {
    pub fn new() -> Self {
        return Self {
            pole: Point2::origin(),
            frontier: VecDeque::new(),
            vertices: Vec::new(),
            he_twins: Vec::new(),
            outgoing_he: Vec::new(),
            triangles: Vec::new(),
            stack: Vec::new()
        };
    }

    #[inline]
    pub fn triangles(&self) -> &Vec<usize> {
        return &self.triangles;
    }

    #[inline]
    pub fn v_pos(&self, idx: usize) -> &Point2<TScalar> {
        return &self.vertices[idx].cartesian;
    }

    pub fn triangulate(&mut self, points: &[Point2<TScalar>]) {
        self.triangles.clear();
        self.he_twins.clear();
        self.outgoing_he.clear();
        self.vertices.clear();
        self.frontier.clear();

        let estimated_halfedges_count = points.len() * 6;
        self.triangles.reserve(estimated_halfedges_count);
        self.he_twins.reserve(estimated_halfedges_count);
        self.outgoing_he.resize(points.len(), usize::MAX);
        // self.vertices.resize(points.len(), value);
        self.frontier.reserve(points.len() / 10);


        self.initialize(points);
        self.triangulation();
        self.finalize();
        // self.save("final.stl".into());

        // self.is_delaunay();
    }

    fn initialize(&mut self, points: &[Point2<TScalar>]) {
        // Compute polar coordinates of input point set
        let min = points.iter().fold(Vector2::zeros(), |acc, p| min2(&acc, &p.coords));
        let max = points.iter().fold(Vector2::zeros(), |acc, p| max2(&acc, &p.coords));
        self.pole = ((min + max) * cast::<_, TScalar>(0.5).unwrap()).into();
        self.vertices = points.iter()
            .map(|p| Vertex { 
                cartesian: *p, 
                angle: TScalar::zero(),
                radius_squared: (p - self.pole).norm_squared()
            })
            .collect();
        self.vertices.sort_unstable_by(|a, b| a.radius_squared.partial_cmp(&b.radius_squared).unwrap());

        // Init frontier
        let mut a = 0;
        let mut b = 1;
        let mut c = 2;

        // if orientation2d(&self.mesh.vertices[a], &self.mesh.vertices[b], &self.mesh.vertices[c]) == Orientation::Clockwise {
        //     swap(&mut b, &mut c);
        // }

        // println!("{:?}", self.mesh.vertices);

        self.pole = (self.vertices[a].cartesian + self.vertices[b].cartesian.coords + self.vertices[c].cartesian.coords) / cast(3).unwrap();
        
        for vertex in &mut self.vertices {
            let mut angle = Float::atan2(vertex.cartesian.y - self.pole.y, vertex.cartesian.x - self.pole.x);

            if angle.is_negative() {
                angle += TScalar::two_pi();
            }

            vertex.angle = angle;
        }

        sort3_by(&mut a, &mut b, &mut c, |v| self.vertices[*v].angle());

        // println!("{:?}\t{:?}\t{:?}", self.vertices[a],self.vertices[b],self.vertices[c]);
        self.frontier.push_back(a);
        self.frontier.push_back(b);
        self.frontier.push_back(c);

        // self.save_svg("it.svg".into());

        // self.mesh.border.insert(0, (2, 1));
        // self.mesh.border.insert(1, (0, 2));
        // self.mesh.border.insert(2, (1, 0));

        self.add_triangle(a, b, c, None, None, None);
    }

    #[inline(never)]
    fn triangulation(&mut self) {
        for point in 3..self.vertices.len() {

            // self.save("init.stl".into());
            // self.save_svg("it.svg".into());
            let edge = self.project_on_frontier(point);

            self.add_triangle(self.frontier[edge.f_start], point, self.frontier[edge.f_end], None, None, Some(edge.halfedge));

            if edge.f_end == 0 {
                if self.vertices[point].angle() <= self.vertices[self.frontier[0]].angle() {
                    self.frontier.push_front(point);
                } else {
                    self.frontier.push_back(point);
                }
            } else {
                self.frontier.insert(edge.f_end, point);
            }
            

            // let a: Vec<_> = self.frontier.iter().map(|f| self.polars[*f].angle()).collect();
            // println!("{:?}", a);
            // assert!(self.frontier.windows(2).all(|w| self.polars[w[0]].angle() <= self.polars[w[1]].angle()));
            // self.replace_halfedge_in_frontier(edge.halfedge, he1, he2);


            self.legalize(edge.halfedge);
            // self.is_delaunay();

            // self.save("legal.stl".into());
            let mut f_point = edge.f_end;

            // println!("walk");

            self.walf_left(&mut f_point);
            // self.save("walk1.stl".into());

            // self.is_delaunay();

            self.walk_right(&mut f_point);
            // self.save("walk2.stl".into());

            // self.is_delaunay();

            self.remove_basin_left(&mut f_point);
            self.remove_basin_right(f_point);

            
            // self.save("end.stl".into());

            // self.is_delaunay();

            // println!("{},{},{}", edge.point_start, point, edge.point_end);
        }
    }

    // fn replace_halfedge_in_frontier(&mut self, replace: usize, he1: usize, he2: usize) {
    //     let (prev, next) = self.mesh.border.remove(&replace).unwrap();

    //     self.mesh.border.insert(he1, (prev, he2));
    //     self.mesh.border.insert(he2, (he1, next));
    //     self.mesh.border.insert(next, (he2, self.mesh.border[&next].1));
    //     self.mesh.border.insert(prev, (self.mesh.border[&prev].0, he1));
    // }

    // fn replace_halfedges_in_frontier(&mut self, replace1: usize, replace2: usize, he: usize) {
    //     let (prev, _) = self.mesh.border.remove(&replace1).unwrap();
    //     let (_, next) = self.mesh.border.remove(&replace2).unwrap();

    //     self.mesh.border.insert(he, (prev, next));
    //     self.mesh.border.insert(prev, (self.mesh.border[&prev].0, he));
    //     self.mesh.border.insert(next, (he, self.mesh.border[&next].1));
    // }

    fn finalize(&mut self) {
        self.remove_basin::<true>(0);
    }

    fn remove_basin<const WALK_LEFT: bool>(&mut self, f: usize) {
        let mut f1 = f;

        loop {
            let f2 = next_vertex::<WALK_LEFT>(f1, self.frontier.len());
            let f3 = next_vertex::<WALK_LEFT>(f2, self.frontier.len());
            
            let v1 = self.frontier[f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &self.vertices[v1].cartesian;
            let v2_pos = &self.vertices[v2].cartesian;
            let v3_pos = &self.vertices[v3].cartesian;

            if orientation2d(v1_pos, v2_pos, v3_pos) == Orientation::Clockwise {
                if WALK_LEFT {
                    let v1v2 = self.outgoing_border_halfedge(v1);
                    let v2v3 = self.outgoing_border_halfedge(v2);
                    self.add_triangle(v3, v2, v1, v2v3, v1v2, None);

                    self.legalize(v1v2.unwrap());
                    self.legalize(v2v3.unwrap());
                } else {
                    let v2v1 = self.outgoing_border_halfedge(v2);
                    let v3v2 = self.outgoing_border_halfedge(v3);
                    self.add_triangle(v1, v2, v3, v2v1, v3v2, None);

                    self.legalize(v2v1.unwrap());
                    self.legalize(v3v2.unwrap());
                }

                self.frontier.remove(f2);
                f1 = next_vertex::<false>(f1, self.frontier.len());
            } else {
                f1 = f2;
            }

            if f3 == f {
                break;
            }
        }
    }

    #[inline(never)]
    fn walf_left(&mut self, start_point: &mut usize) {
        loop {
            let f1 = *start_point;
            let f2 = (*start_point + 1) % self.frontier.len();
            let f3 = (*start_point + 2) % self.frontier.len();

            let v1 = self.frontier[f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &self.vertices[v1].cartesian;
            let v2_pos = &self.vertices[v2].cartesian;
            let v3_pos = &self.vertices[v3].cartesian;

            let v2v1 = v1_pos - v2_pos;
            let v2v3 = v3_pos - v2_pos;

            let signed_angle = signed_angle(&v2v1, &v2v3);

            if signed_angle >= TScalar::frac_pi_2() {
                break;
            }

            // println!("insert left");

            let next_he = self.outgoing_border_halfedge(v2);
            let current_he = self.outgoing_border_halfedge(v1);

            debug_assert!(current_he.is_some());
            debug_assert!(next_he.is_some());

            self.add_triangle(v3, v2, v1, next_he, current_he, None);
            self.frontier.remove(f2);

            if *start_point > f2 {
                *start_point -= 1;
            }

            self.legalize(current_he.unwrap());
            self.legalize(next_he.unwrap());

            // self.replace_halfedges_in_frontier(current_he, next_he, face + 2);
            //current_he = face + 2;
        }
    }

    fn walk_right(&mut self, mut f1: &mut usize) {
        loop {
            let f2 = (*f1 + self.frontier.len() - 1) % self.frontier.len();
            let f3 = (*f1 + self.frontier.len() - 2) % self.frontier.len();

            let v1 = self.frontier[*f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &self.vertices[v1].cartesian;
            let v2_pos = &self.vertices[v2].cartesian;
            let v3_pos = &self.vertices[v3].cartesian;

            let v2v1 = v1_pos - v2_pos;
            let v2v3 = v3_pos - v2_pos;

            let signed_angle = signed_angle(&v2v3, &v2v1);

            if signed_angle >= TScalar::frac_pi_2() {
                break;
            }

            // println!("insert right");

            let current_he = self.outgoing_border_halfedge(v2);
            let prev_he = self.outgoing_border_halfedge(v3);

            self.add_triangle(v1, v2, v3, current_he, prev_he, None);

            self.frontier.remove(f2);

            if *f1 > f2 {
                *f1 -= 1;
            }

            self.legalize(current_he.unwrap());
            self.legalize(prev_he.unwrap());

            // self.replace_halfedges_in_frontier(prev_he, current_he, face + 2);
            // current_he = face + 2;
        }
    }

    fn remove_basin_right(&mut self, f1: usize) {
        let f3 = (f1 + self.frontier.len() - 2) % self.frontier.len();
        
        let f1_p = &self.vertices[self.frontier[f1]];
        let f3_p = &self.vertices[self.frontier[f3]];

        let delta_r = f1_p.radius_squared() - f3_p.radius_squared();
        let delta_o = f1_p.angle() - f3_p.angle();

        let basin_factor = delta_r / (f3_p.radius_squared() * delta_o);
        let is_basin = basin_factor > num_traits::cast(2).unwrap();

        if is_basin {
            // println!("Found basin right");

            let f_basin_start = f3;
            // let mut f_basin_bottom = (f_basin_start + self.frontier.len() - 1) % self.frontier.len();
            // let mut f_basin_bottom_next = (f_basin_bottom + self.frontier.len() - 1) % self.frontier.len();

            // while self.polars[self.frontier[f_basin_bottom]].radius() > self.polars[self.frontier[f_basin_bottom_next]].radius() {
            //     f_basin_bottom = f_basin_bottom_next;
            //     f_basin_bottom_next = (f_basin_bottom_next + self.frontier.len() - 1) % self.frontier.len();
            // }

            let mut f_basin_end = f3;
            let mut f_start = f3;

            loop {
                let f_mid = (f_start + self.frontier.len() - 1) % self.frontier.len();
                let f_end = (f_start + self.frontier.len() - 2) % self.frontier.len();

                let f_start_p = &self.vertices[self.frontier[f_start]].cartesian;
                let f_mid_p = &self.vertices[self.frontier[f_mid]].cartesian;
                let f_end_p = &self.vertices[self.frontier[f_end]].cartesian;

                if orientation2d(f_start_p, f_mid_p, f_end_p) == Orientation::Clockwise {
                    f_basin_end = f_mid;
                    break;
                }
        
                f_start = f_mid;
            }

            let mut f_mid = (f_basin_start + self.frontier.len() - 1) % self.frontier.len();
            if f_mid == f_basin_end {
                return;
            }
        
            let mut f_end = (f_basin_start + self.frontier.len() - 2) % self.frontier.len();
            let mut f_start = f_basin_start;

            // self.save("basin1.stl".into());
            loop {
                let v1 = self.frontier[f_start];
                let v2 = self.frontier[f_mid];
                let v3 = self.frontier[f_end];

                let v2v1 = self.outgoing_border_halfedge(v2);
                let v3v2 = self.outgoing_border_halfedge(v3);

                debug_assert!(v2v1.is_some());
                debug_assert!(v3v2.is_some());

                // println!("add");
                self.add_triangle(v1, v2, v3, v2v1, v3v2, None);
                self.frontier.remove(f_mid);
                
                if f_basin_end >= f_mid {
                    f_basin_end -= 1;
                }

                if f_end > f_mid {
                    f_end -= 1;
                }

                if f_start > f_mid {
                    f_start -= 1;
                }

                f_mid = f_end;
                f_end = (f_end + self.frontier.len() - 1) % self.frontier.len();

                self.legalize(v2v1.unwrap());
                self.legalize(v3v2.unwrap());

                if f_mid == f_basin_end {
                    break;
                }
            }
            // self.save("basin2.stl".into());
        } 
    }

    
    fn remove_basin_left(&mut self, f1: &mut usize) {
        let f3 = (*f1 + 2) % self.frontier.len();
        
        let f1_p = &self.vertices[self.frontier[*f1]];
        let f3_p = &self.vertices[self.frontier[f3]];

        let delta_r = f1_p.radius_squared() - f3_p.radius_squared();
        let delta_o = f1_p.angle() - f3_p.angle();

        let basin_factor = delta_r / (f3_p.radius_squared() * delta_o);
        let is_basin = basin_factor > num_traits::cast(2).unwrap();

        if is_basin {
            // println!("Found basin left");

            let f_basin_start = f3;
            // let mut f_basin_bottom = (f_basin_start + self.frontier.len() - 1) % self.frontier.len();
            // let mut f_basin_bottom_next = (f_basin_bottom + self.frontier.len() - 1) % self.frontier.len();

            // while self.polars[self.frontier[f_basin_bottom]].radius() > self.polars[self.frontier[f_basin_bottom_next]].radius() {
            //     f_basin_bottom = f_basin_bottom_next;
            //     f_basin_bottom_next = (f_basin_bottom_next + self.frontier.len() - 1) % self.frontier.len();
            // }

            let mut f_basin_end = f3;
            let mut f_start = f3;

            loop {
                let f_mid = (f_start + 1) % self.frontier.len();
                let f_end = (f_start + 2) % self.frontier.len();

                let f_start_p = &self.vertices[self.frontier[f_start]].cartesian;
                let f_mid_p = &self.vertices[self.frontier[f_mid]].cartesian;
                let f_end_p = &self.vertices[self.frontier[f_end]].cartesian;

                if orientation2d(f_start_p, f_mid_p, f_end_p) == Orientation::CounterClockwise {
                    f_basin_end = f_mid;
                    break;
                }
        
                f_start = f_mid;
            }

            let mut f_mid = (f_basin_start + 1) % self.frontier.len();
            if f_mid == f_basin_end {
                return;
            }
        
            let mut f_end = (f_basin_start + 2) % self.frontier.len();
            let mut f_start = f_basin_start;

            // self.save("basin1.stl".into());
            loop {
                let v1 = self.frontier[f_start];
                let v2 = self.frontier[f_mid];
                let v3 = self.frontier[f_end];

                let v2v3 = self.outgoing_border_halfedge(v2);
                let v1v2 = self.outgoing_border_halfedge(v1);

                debug_assert!(v2v3.is_some());
                debug_assert!(v1v2.is_some());

                // println!("add");
                self.add_triangle(v3, v2, v1, v2v3, v1v2, None);
                self.frontier.remove(f_mid);
                
                if f_basin_end >= f_mid {
                    f_basin_end -= 1;
                }

                if f_end > f_mid {
                    f_end -= 1;
                }

                if f_start > f_mid {
                    f_start -= 1;
                }

                if *f1 > f_mid {
                    *f1 -= 1;
                }

                f_mid = f_end;
                f_end = (f_end + 1) % self.frontier.len();

                self.legalize(v2v3.unwrap());
                self.legalize(v1v2.unwrap());

                if f_mid == f_basin_end {
                    break;
                }
            }
            // self.save("basin2.stl".into());
        }
    }


    fn legalize(&mut self, he: usize) {
        self.stack.push(he);

        while !self.stack.is_empty() {
            let he = self.stack.pop().unwrap();

            let opposite_he = self.opposite_halfedge(he);

            if opposite_he.is_none() {
                continue;
            }

            let (v1, v2) = self.halfedge_vertices(he);
            let (_, v3) = self.halfedge_vertices(next_halfedge(he));
            let (_, v4) = self.halfedge_vertices(next_halfedge(opposite_he.unwrap()));

            // let t1 = Triangle2::new(self.vertices[v1].cartesian, self.vertices[v2].cartesian, self.vertices[v3].cartesian);
            // let t2 = Triangle2::new(self.vertices[v2].cartesian, self.vertices[v1].cartesian, self.vertices[v4].cartesian);

            // let is_illegal =
            //     t1.is_inside_circum(&self.vertices[v4].cartesian) ||
            //     t2.is_inside_circum(&self.vertices[v3].cartesian);

            let is_illegal =
                triangle2::is_inside_circumcircle(&self.vertices[v1].cartesian, &self.vertices[v2].cartesian, &self.vertices[v3].cartesian, &self.vertices[v4].cartesian) ||
                triangle2::is_inside_circumcircle(&self.vertices[v2].cartesian, &self.vertices[v1].cartesian, &self.vertices[v4].cartesian, &self.vertices[v3].cartesian);

            if is_illegal {
                // println!("Flip {}:{}-{}", he, self.mesh.vertices[v1], self.mesh.vertices[v2]);
                // println!("Tri {:?}-{:?}", t1, t2);
                // println!();
                let he1 = prev_halfedge(he);
                let he2 = self.opposite_halfedge(he).unwrap();
                let he3 = prev_halfedge(he2);

                self.flip_edge(he);

                self.stack.push(he);
                self.stack.push(he1);
                self.stack.push(he2);
                self.stack.push(he3);

                // self.legalize(he);
                // self.legalize(he1);
                // self.legalize(he2);
                // self.legalize(he3);
            }
        }

    }

    /// Returns index of intersected halfedge
    fn project_on_frontier(&self, point_idx: usize) -> FrontierEdge {
        let ang = self.vertices[point_idx].angle();
        let edge_end = self.frontier.partition_point(|probe| self.vertices[*probe].angle() < ang) % self.frontier.len();
        let edge_start = (edge_end + self.frontier.len() - 1) % self.frontier.len();

        // println!("{:?}\t{:?}\t{:?}", 
        //     self.mesh.vertices[self.frontier[edge_start]],
        //     self.mesh.vertices[point_idx],
        //     self.mesh.vertices[self.frontier[edge_end]]);

        return FrontierEdge {
            halfedge: self.outgoing_border_halfedge(self.frontier[edge_start]).unwrap(),
            f_end: edge_end,
            f_start: edge_start
        };


        // let projection = LineSegment2::new(self.mesh.vertices[point_idx], self.pole);

        // let a: Vec<_> = self.frontier.iter().map(|f| self.polars[*f].angle()).collect();
        // println!("{:?}", a);
        
        // for edge_start in 0..self.frontier.len() {
        //     let edge_end = (edge_start + 1) % self.frontier.len();
        //      let edge = LineSegment2::new(self.mesh.vertices[self.frontier[edge_start]], self.mesh.vertices[self.frontier[edge_end]]);

        //     //println!("{:?}\n{:?}", projection, edge);

        //     if let Some(_) = projection.intersects_at(&edge) {
        //         // println!("{:?}\t{:?}\t{:?}", self.polars[self.frontier[edge_start]], self.polars[self.frontier[edge_end]], self.polars[point_idx]);
        //         return FrontierEdge {
        //             halfedge: self.mesh.outgoing_border_halfedge(self.frontier[edge_start]).unwrap(),
        //             point_end: edge_end,
        //             point_start: edge_start
        //         };
        //     }
        // }

        // self.save("unreachable.stl".into());
        unreachable!();
    }

    fn print_frontier(&self) {
        // let start_edge = self.mesh.border.iter().next().unwrap().1.1;
        // let mut current_edge = start_edge;

        // loop {
        //     let (start, end) = self.mesh.halfedge_vertices(current_edge);
        //     println!("{}:{}-{}", current_edge, start, end);

        //     current_edge = self.mesh.border[&current_edge].1;

        //     if current_edge == start_edge {
        //         break;
        //     }
        // }

        for start in 0..self.frontier.len() {
            let end = (start + 1) % self.frontier.len();
            let current_edge = self.outgoing_border_halfedge(self.frontier[start]).unwrap();
            println!("{}:{}-{}", current_edge, self.frontier[start], self.frontier[end]);
        }

        println!();
        println!();
    }

    fn save(&self, p: String) {
        let pts: Vec<_> = self.vertices.iter().map(|p| p.cartesian).map(|p| Point3::new(num_traits::cast(p.x).unwrap(), num_traits::cast(p.y).unwrap(), 0.0)).collect();


        let mut mesh = PolygonSoup::<f32>::new();
        for idx in (0..self.triangles.len()).step_by(3) {
            mesh.add_face(pts[self.triangles[idx]], pts[self.triangles[idx + 1]], pts[self.triangles[idx + 2]]);
        }
        let writer = StlWriter::new();
        writer.write_stl_to_file(&mesh, Path::new(&p)).expect("Must");
    }

    fn save_svg(&self, p: String) {
        let scale = 500.0;
        let height = 500.0;

        let lines = self.frontier.iter()
            .enumerate()
            .map(|(i, e_s)| {
                let e_e = self.frontier[(i + 1) % self.frontier.len()];
                let p_s = self.vertices[*e_s].cartesian;
                let p_e = self.vertices[e_e].cartesian;

                return (p_s, p_e);
            })
            .fold(Data::new(), |data, (s, e)| {
                return data
                    .move_to((
                        num_traits::cast::<TScalar, f64>(s.x).unwrap() * scale,
                        height-num_traits::cast::<TScalar, f64>(s.y).unwrap() * scale,
                    ))
                    .line_to((
                        num_traits::cast::<TScalar, f64>(e.x).unwrap() * scale,
                        height-num_traits::cast::<TScalar, f64>(e.y).unwrap() * scale,
                    ));
            });

        let points = self.vertices.iter()
            .map(|v| v.cartesian)
            .enumerate()
            .map(|(i, v)| (
                Text::new().add(svg::node::Text::new(format!(" {};{:.2};{:.2}", i, self.vertices[i].radius_squared(), self.vertices[i].angle()))) //format!("{i}; {}; {}", v.x, v.y)
                    .set("x", num_traits::cast::<TScalar, f64>(v.x).unwrap() * scale)
                    .set("y", height-num_traits::cast::<TScalar, f64>(v.y).unwrap() * scale)
                    .set("font-size", "3px"),
                Circle::new()
                    .set("r", 1)
                    .set("cx", num_traits::cast::<TScalar, f64>(v.x).unwrap() * scale)
                    .set("cy", height-num_traits::cast::<TScalar, f64>(v.y).unwrap() * scale)
                    .set("fill", "green")
                ))
            .fold(Group::new(), |group, (text, circle)| group.add(text).add(circle));

        let path = svg::node::element::Path::new()
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", 0.5)
            .set("d", lines);

        let pole = Circle::new()
            .set("r", 1)
            .set("cx", num_traits::cast::<TScalar, f64>(self.pole.x).unwrap() * scale)
            .set("cy", height-num_traits::cast::<TScalar, f64>(self.pole.y).unwrap() * scale)
            .set("fill", "red");
    
        let doc = Document::new()
            .add(path)
            .add(points)
            .add(pole);

        svg::save(Path::new(&p), &doc).unwrap();
    }

    pub fn is_delaunay(&self) -> bool {
        for he in 0..self.triangles.len() {
            let opposite_he = self.opposite_halfedge(he);

            if opposite_he.is_none() {
                continue;
            }

            let (v1, v2) = self.halfedge_vertices(he);
            let (_, v3) = self.halfedge_vertices(next_halfedge(he));
            let (_, v4) = self.halfedge_vertices(next_halfedge(opposite_he.unwrap()));

            let t1 = Triangle2::new(self.vertices[v1].cartesian, self.vertices[v2].cartesian, self.vertices[v3].cartesian);
            let t2 = Triangle2::new(self.vertices[v2].cartesian, self.vertices[v1].cartesian, self.vertices[v4].cartesian);

            let is_illegal =
                t1.is_inside_circum(&self.vertices[v4].cartesian) ||
                t2.is_inside_circum(&self.vertices[v3].cartesian);

            if is_illegal {
                return false;
            }
        }

        return true;
    }

    #[inline]
    fn halfedge_vertices(&self, he: usize) -> (usize, usize) {
        return (self.triangles[he], self.triangles[next_halfedge(he)]);
    }

    #[inline]
    fn opposite_halfedge(&self, he: usize) -> Option<usize> {
        return self.he_twins[he];
    }

    fn add_triangle(&mut self, v1: usize, v2: usize, v3: usize, he1: Option<usize>, he2: Option<usize>, he3: Option<usize>) -> usize {
        let triangle_idx = self.triangles.len();

        let t = Triangle2::new(
            Point2::new(self.vertices[v1].cartesian[0], self.vertices[v1].cartesian[1]), 
            Point2::new(self.vertices[v2].cartesian[0], self.vertices[v2].cartesian[1]), 
            Point2::new(self.vertices[v3].cartesian[0], self.vertices[v3].cartesian[1])
        );
        let orient = t.orientation();
        debug_assert!(orient == Orientation::CounterClockwise);

        self.triangles.push(v1);
        self.triangles.push(v2);
        self.triangles.push(v3);

        self.outgoing_he[v1] = triangle_idx;
        self.outgoing_he[v2] = triangle_idx + 1;
        self.outgoing_he[v3] = triangle_idx + 2;
        
        self.add_halfedge(he1);
        self.add_halfedge(he2);
        self.add_halfedge(he3);

        return triangle_idx;
    }

    fn add_halfedge(&mut self, opposite: Option<usize>) {
        self.he_twins.push(opposite);
        if let Some(opposite) = opposite {
            self.he_twins[opposite] = Some(self.he_twins.len() - 1);
        }
    }

    fn make_opposite(&mut self, he1: usize, he2: Option<usize>) {
        self.he_twins[he1] = he2;

        if let Some(he2) = he2 {
            self.he_twins[he2] = Some(he1);
        }
    }
    
    #[inline]
    fn outgoing_halfedge(&self, v: usize) -> usize {
        return self.outgoing_he[v];
    }

    #[inline]
    fn outgoing_border_halfedge(&self, v: usize) -> Option<usize> {
        let start_he = self.outgoing_halfedge(v);
        let mut current_he = start_he;

        loop {
            let twin = self.he_twins[current_he];

            if twin.is_none() {
                return Some(current_he);
            }

            current_he = next_halfedge(twin.unwrap());

            if start_he == current_he {
                return None;
            }
        }
    }

    fn flip_edge(&mut self, he: usize) {
        let he_next = next_halfedge(he);
        let he_opposite = self.opposite_halfedge(he).unwrap();
        let he_opposite_next = next_halfedge(he_opposite);

        let op1 = self.opposite_halfedge(he_next);
        let he_next_next = next_halfedge(he_next);
        let op2 = self.opposite_halfedge(he_next_next);
    
        let op3 = self.opposite_halfedge(he_opposite_next);
        let he_opposite_next_next = next_halfedge(he_opposite_next);
        let op4 = self.opposite_halfedge(he_opposite_next_next);

        let (v2, v3) = self.halfedge_vertices(he_next);
        let (v1, v4) = self.halfedge_vertices(he_opposite_next);

        self.triangles[he_next] = v4;
        self.triangles[he_opposite_next] = v3;

        self.make_opposite(he, op3);
        self.make_opposite(he_next, Some(he_opposite_next));
        self.make_opposite(he_next_next, op2);

        self.make_opposite(he_opposite, op1);
        self.make_opposite(he_opposite_next, Some(he_next));
        self.make_opposite(he_opposite_next_next, op4);

        // self.he_twins[he] = op3;
        // self.he_twins[he_next] = Some(he_opposite_next);
        // self.he_twins[he_next_next] = op2;

        // self.he_twins[he_opposite] = op1;
        // self.he_twins[he_opposite_next] = Some(he_next);
        // self.he_twins[he_opposite_next_next] = op4;


        self.outgoing_he[v1] = he;
        self.outgoing_he[v2] = he_opposite;
        self.outgoing_he[v3] = he_next_next;
        self.outgoing_he[v4] = he_opposite_next_next;

        // let t1 = Triangle2::new(
        //     Point2::new(self.vertices[v3][0], self.vertices[v3][1]), 
        //     Point2::new(self.vertices[v1][0], self.vertices[v1][1]), 
        //     Point2::new(self.vertices[v4][0], self.vertices[v4][1])
        // );

        // let t2 = Triangle2::new(
        //     Point2::new(self.vertices[v3][0], self.vertices[v3][1]), 
        //     Point2::new(self.vertices[v4][0], self.vertices[v4][1]), 
        //     Point2::new(self.vertices[v2][0], self.vertices[v2][1])
        // );

        // debug_assert!(t1.orientation() == Orientation::CounterClockwise);
        // debug_assert!(t2.orientation() == Orientation::CounterClockwise);

        // if op3.is_none() {
        //     self.replace_halfedge(he_opposite_next, he);
        // }

        // if op4.is_none() {
        //     self.replace_halfedge(he_opposite_next_next, he_opposite_next_next);
        // }
        
        // if op1.is_none() {
        //     self.replace_halfedge(he_next, he_opposite);
        // }
        
        // if op2.is_none() {
        //     self.replace_halfedge(he_next_next, );
        // }
    }

    // fn replace_halfedge(&mut self, replace: usize, new: usize) {
    //     let (prev, next) = self.border.remove(&replace).unwrap();

    //     self.border.insert(new, (prev, next));
    //     self.border.insert(next, (new, self.border[&next].1));
    //     self.border.insert(prev, (self.border[&prev].0, new));
    // }
}

#[inline]
fn next_vertex<const WALK_LEFT: bool>(vertex: usize, vertices_count: usize) -> usize {
    if WALK_LEFT {
        return (vertex + 1) % vertices_count;
    } else {
        return (vertex + vertices_count - 1) % vertices_count;
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point2;

    use super::Triangulation2;

    #[test]
    fn degenerate_case() {
        let mut triangulation = Triangulation2::new();
        triangulation.triangulate(&mut vec![
            Point2::new(1.0, 2.0),
            Point2::new(5.0, 1.0),
            Point2::new(8.0, 6.0),
            Point2::new(2.0, 8.0)
        ]);
    }
    
    #[test]
    fn test1() {
        let points = [[0.48984146, 0.4899361], [0.4463194, 0.45261556], [0.42847013, 0.42460257], [0.41823488, 0.57288224], [0.5913105, 0.45535183], [0.53855276, 0.5922733], [0.37710214, 0.5732515], [0.5043943, 0.6273088], [0.34420383, 0.51125544], [0.62980384, 0.44524848], [0.34035563, 0.43844408], [0.61331505, 0.35406935], [0.61050564, 0.34783804], [0.66835, 0.5447868], [0.32081836, 0.3943385], [0.4718566, 0.2961123], [0.58064073, 0.66067904], [0.6851884, 0.48713744], [0.29649615, 0.57779795], [0.63608783, 0.6429018], [0.46383494, 0.27027756], [0.70931464, 0.46491158], [0.7052269, 0.55955833], [0.54671764, 0.25920832], [0.6284604, 0.68279934], [0.3177119, 0.3153975], [0.42712665, 0.7265839], [0.56969875, 0.7230318], [0.49226338, 0.7405513], [0.4741112, 0.74244386], [0.2804165, 0.33925468], [0.29501998, 0.66089964], [0.6637637, 0.6773343], [0.46313453, 0.74667466], [0.71958226, 0.37372464], [0.2911582, 0.31229526], [0.43222797, 0.77797765], [0.71959144, 0.3079893], [0.76890755, 0.59576356], [0.24977851, 0.28614485], [0.3248073, 0.20827317], [0.16804123, 0.57080585], [0.15872717, 0.5225644], [0.21868831, 0.6842079], [0.35850996, 0.17893916], [0.26844138, 0.23370874], [0.18464863, 0.64404595], [0.18881321, 0.30391115], [0.13282919, 0.49237096], [0.5348515, 0.84225017], [0.7661881, 0.7145568], [0.82922363, 0.6159804], [0.8447025, 0.41357028], [0.80576605, 0.66857046], [0.65735865, 0.1653955], [0.4404143, 0.8562609], [0.12434751, 0.56717086], [0.8447379, 0.38467562], [0.4579938, 0.11322808], [0.10814023, 0.48565876], [0.66940445, 0.15512234], [0.18147635, 0.71670175], [0.17786211, 0.72111464], [0.12957686, 0.65061164], [0.5351382, 0.088799596], [0.6344292, 0.8699391], [0.8590333, 0.6721916], [0.39739162, 0.06835717], [0.32444948, 0.8887432], [0.114165425, 0.2647937], [0.16959798, 0.18581927], [0.039387226, 0.498999], [0.70789284, 0.87855], [0.06639743, 0.31261855], [0.33921427, 0.053124905], [0.3961032, 0.02758801], [0.11840737, 0.7837957], [0.014104009, 0.51041526], [0.6770156, 0.92879564], [0.1100536, 0.78631395], [0.73594517, 0.072675765], [0.9592681, 0.6101808], [0.9563696, 0.63878834], [0.9551344, 0.31495094], [0.6514476, 0.96371853], [0.860139, 0.14633244], [0.7776793, 0.91099083], [0.86620057, 0.14327657], [0.06995958, 0.18052047], [0.79034364, 0.059315145], [0.023816466, 0.22163707], [0.056708217, 0.16669017], [0.7203423, 0.9783333], [0.23453873, 0.9738017], [0.78757405, 0.022196889], [0.833493, 0.92673594], [0.1371184, 0.036289155], [0.021484733, 0.14100307], [0.9737798, 0.10746962], [0.95703167, 0.9444567]];
        let mut points2d: Vec<_> = points.iter().map(|p| Point2::new(p[0], p[1])).collect();

        let mut triangulation = Triangulation2::new();
        triangulation.triangulate(&mut points2d);
        triangulation.triangles();

        assert!(triangulation.is_delaunay());
    }
}
