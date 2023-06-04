use std::{collections::HashSet, hash::Hash, path::Path};

use nalgebra::Point2;

use crate::{
    geometry::{traits::{RealNumber, Intersects}, 
    primitives::line_segment2::LineSegment2, 
    orientation::{orientation2d, Orientation}}, 
    data_structures::linked_list::LinkedList
};

use self::debugging::save_to_svg;

use super::{
    delaunay::Triangulation2, 
    halfedge::{prev_halfedge, next_halfedge, HalfedgeMesh}
};

/// Error that happens when adding constraint to triangulation
#[derive(Debug, PartialEq, Eq)]
pub enum AddConstraintErr {
    ConstraintExist,
    EdgeIntersectsExistingConstraint,
    EdgeIntersectsPoint
}

///
/// 2d constrained triangulation. Triangulation is build on top of unconstrained delaunay triangulation by inserting constrained edges.
/// Therefore it is not necessarily a delaunay triangulation.
/// 
/// ## Example
/// ```
/// use nalgebra::Point2;
/// use baby_shark::triangulation::constrained_delaunay::ConstrainedTriangulation2;
/// 
/// let points = vec![
///     Point2::new(-3.0, 1.0),
///     Point2::new(0.0, 0.0),
///     Point2::new(0.0, 4.0),
///     Point2::new(3.0, 2.0),
///     Point2::new(6.0, 0.0),
///     Point2::new(6.0, 4.0),
///     Point2::new(9.0, 2.0)
/// ];
/// let mut tri = ConstrainedTriangulation2::new();
/// tri.set_points(&points);
/// tri.add_constrained_edge(0, 6);
/// tri.triangulate();
/// ```
/// 
/// Based on "A fast algorithm for generating constrained delaunay triangulations" by S. W. Sloan:
/// https://www.newcastle.edu.au/__data/assets/pdf_file/0019/22519/23_A-fast-algortithm-for-generating-constrained-Delaunay-triangulations.pdf
/// 
pub struct ConstrainedTriangulation2<TScalar: RealNumber> {
    delaunay: Triangulation2<TScalar>,
    intersected_edges: LinkedList<Edge>,
    constraints: HashSet<Edge>,
    points: Vec<Point2<TScalar>>
}

impl<TScalar: RealNumber> ConstrainedTriangulation2<TScalar> {
    /// Creates triangulator
    pub fn new() -> Self {
        return Self { 
            delaunay: Triangulation2::new(),
            intersected_edges: LinkedList::new(),
            constraints: HashSet::new(),
            points: Vec::new()
        };
    }

    /// Returns triangle indices
    #[inline]
    pub fn triangles(&self) -> &Vec<usize> {
        return self.delaunay.triangles();
    }

    /// Returns input points set
    #[inline]
    pub fn points(&self) -> &Vec<Point2<TScalar>> {
        return &self.points;
    }

    /// Set input points set
    #[inline]
    pub fn set_points(&mut self, points: &[Point2<TScalar>]) {
        self.points.clear();
        self.points.extend_from_slice(points);
    }

    /// Set input points set
    #[inline]
    pub fn with_points(mut self, points: &[Point2<TScalar>]) -> Self {
        self.set_points(points);
        return self;
    }

    /// Triangulate points set
    pub fn triangulate(&mut self) {
        if self.points().len() < 3 {
            return;
        }

        // Start from unconstrained delaunay triangulation and insert constrained edges on by one
        self.delaunay.triangulate(&self.points);

        for edge in self.constraints.iter().copied().collect::<Vec<_>>() {
            self.insert_edge(edge);
        }
    }

    /// Adds constrained edge given by indices of two points.
    /// Before adding constraint edge is check whether it doesn't intersect existing constrained edges and if it is not going through any vertices other than edge ends.
    /// If you are sure that edge meet this requirements use unchecked version [`add_constrained_edge_unchecked`]
    pub fn add_constrained_edge(&mut self, start: usize, end: usize) -> Result<(), AddConstraintErr> {
        let new_constraint = Edge { start, end };
        // let v1 = self.points()[new_constraint.start];
        // let v2 = self.points()[new_constraint.end];
        // let new_constraint_segment = LineSegment2::new(v1, v2);

        // // Validate that edge doesn't intersects existing constrained edges
        // for edge in &self.constraints {
        //     if edge == &new_constraint {
        //         return Err(AddConstraintErr::ConstraintExist);
        //     }

        //     let sharing_vertex =
        //         new_constraint.start == edge.start || 
        //         new_constraint.start == edge.end ||
        //         new_constraint.end == edge.start ||
        //         new_constraint.end == edge.end;

        //     if sharing_vertex {
        //         continue;
        //     }

        //     let edge_segment = LineSegment2::new(self.points()[edge.start], self.points()[edge.end]);

        //     if edge_segment.intersects_at(&new_constraint_segment).is_some() {
        //         return Err(AddConstraintErr::EdgeIntersectsExistingConstraint);
        //     }
        // }

        // // Check if edge is not on the path of three coplanar points
        // for pi in 0..self.points().len() {
        //     if pi == new_constraint.start || pi == new_constraint.end {
        //         continue;
        //     }

        //     if new_constraint_segment.contains_point(&self.points()[pi]) {
        //         return Err(AddConstraintErr::EdgeIntersectsPoint);
        //     }
        // }

        self.constraints.insert(new_constraint);
        
        return Ok(());
    }


    /// Adds constrained edge given by indices of two points but doesn't check whether edge is valid
    #[inline]
    pub fn add_constrained_edge_unchecked(&mut self, start: usize, end: usize) {
        let new_constraint = Edge { start, end };
        self.constraints.insert(new_constraint);
    }

    /// Inserts edge into existing triangulation by flipping edges intersected by it
    fn insert_edge(&mut self, mut edge: Edge) {
        self.intersected_edges.clear();

        println!("{:?}", edge);
        save_to_svg(self, Path::new("test.svg"));

        if self.mesh().connection_halfedge(edge.start, edge.end).is_some() {
            return;
        }

        // Find first intersection edge
        let mut intersected_edge = None;

        let edge_start_pos = self.points[edge.start];
        let edge_end_pos   = self.points[edge.end];
        let edge_segment = LineSegment2::new(edge_start_pos, edge_end_pos);

        let start = self.mesh().outgoing_halfedge(edge.start);
        let mut he = start;
        let mut border = false;
        
        // Find first edge intersected by constrained
        loop {
            let e = next_halfedge(he);
            let (v1, v2) = self.mesh().halfedge_vertices(e);
            let v1_pos = self.points[v1];
            let v2_pos = self.points[v2];
            let segment = LineSegment2::new(v1_pos, v2_pos);

            if segment.intersects_at(&edge_segment).is_some() {
                intersected_edge = Some(e);
                break;
            }

            if let Some(next) = self.mesh().opposite_halfedge(prev_halfedge(he)) {
                if next == start {
                    break;
                }

                he = next;
            } else {
                border = true;
                break;
            }
        }

        if border && self.mesh().opposite_halfedge(start).is_some() {
            he = self.mesh().opposite_halfedge(start).unwrap();
            he = next_halfedge(he);
            
            loop {
                let e = next_halfedge(he);
                let (v1, v2) = self.mesh().halfedge_vertices(e);
                let v1_pos = self.points[v1];
                let v2_pos = self.points[v2];
                let segment = LineSegment2::new(v1_pos, v2_pos);
    
                if segment.intersects_at(&edge_segment).is_some() {
                    intersected_edge = Some(e);
                    break;
                }
                
                if let Some(opp) = self.mesh().opposite_halfedge(he) {
                    he = next_halfedge(opp);
                } else {
                    break;
                }
            }
        }

        if intersected_edge.is_none() {
            return;
        }

        let mut intersected_edge = intersected_edge.unwrap();

        // Collect intersected edges
        loop {
            let (v1, v2) = self.mesh().halfedge_vertices(intersected_edge);

            self.intersected_edges.push_back(Edge{ start: v1, end: v2 });

            let opposite = self.mesh().opposite_halfedge(intersected_edge).unwrap();

            // Next intersected edge is either previous or next halfedge
            let next_he = next_halfedge(opposite);

            if let Some(intersection) = self.intersects(&edge_segment, next_he) {
                match intersection {
                    Intersection::OnLine(point) => {
                        intersected_edge = next_he;

                        let (v1, v2) = self.mesh().halfedge_vertices(intersected_edge);
                        let e = Edge::new(v1, v2);
                        println!("{}-{}", v1, v2);
                        
                        if self.constraints.contains(&e) {
                            self.points.push(point);
                            let v = self.points.len() - 1;
                            self.mesh_mut().split_edge(intersected_edge, v);
                            // save_to_svg(self, Path::new("test.svg"));

                            self.insert_edge(Edge::new(edge.start, v));
                            self.insert_edge(Edge::new(v, edge.end));

                            return;
                        }
                    },
                    Intersection::AtEnd => {
                        // Split constrained edge at intersection
                        let (_, v_end) = self.mesh().halfedge_vertices(next_he);

                        if v_end == edge.end {
                            break;
                        }

                        self.insert_edge(Edge{start: v_end, end: edge.end});
                        edge.end = v_end;
                        break;
                    },
                }
            } else {
                intersected_edge = prev_halfedge(opposite);
                let intersection = self.intersects(&edge_segment, intersected_edge).unwrap();

                match intersection {
                    Intersection::OnLine(point) => {
                        let (v1, v2) = self.mesh().halfedge_vertices(intersected_edge);
                        let e = Edge::new(v1, v2);
                        
                        if self.constraints.contains(&e) {
                            self.points.push(point);
                            let v = self.points.len() - 1;
                            self.mesh_mut().split_edge(intersected_edge, v);

                            self.insert_edge(Edge::new(edge.start, v));
                            self.insert_edge(Edge::new(v, edge.end));

                            return;
                        }
                    },
                    Intersection::AtEnd => {
                        // Split constrained edge at intersection
                        let (v_start, _) = self.mesh().halfedge_vertices(next_he);

                        if v_start == edge.end {
                            break;
                        }

                        self.insert_edge(Edge{start: v_start, end: edge.end});
                        edge.end = v_start;
                        break;
                    },
                }
            }
        }

        // Flip edges that are intersection constrained until there are no intersections
        while let Some(intersected_edge) = self.intersected_edges.pop_front().cloned() {
            let he = self.mesh().connection_halfedge(intersected_edge.start, intersected_edge.end).unwrap();

            let (v1, v2) = self.mesh().halfedge_vertices(he);
            let (_, v3)  = self.mesh().halfedge_vertices(next_halfedge(he));
            let (v4, _)  = self.mesh().halfedge_vertices(prev_halfedge(self.mesh().opposite_halfedge(he).unwrap()));

            let v1_pos = &self.points()[v1];
            let v2_pos = &self.points()[v2];
            let v3_pos = self.points()[v3];
            let v4_pos = self.points()[v4];

            // Skip edge if flipping it will invert face normal
            let safe_flip = 
                orientation2d(&v3_pos, &v4_pos, v2_pos) == Orientation::CounterClockwise && 
                orientation2d(&v3_pos, v1_pos, &v4_pos) == Orientation::CounterClockwise;

            if !safe_flip {
                self.intersected_edges.push_back(intersected_edge);
                continue;
            }

            self.mesh_mut().flip_edge(he);
            save_to_svg(self, Path::new("test.svg"));

            // Flip on ends of constrained edge?
            if v3 == edge.end || v4 == edge.end || v3 == edge.start || v4 == edge.start {
                continue;
            }

            // Check if flipped edge still intersects constrained
            let flipped_edge = LineSegment2::new(v3_pos, v4_pos);
            if flipped_edge.intersects_at(&edge_segment).is_some() {
                self.intersected_edges.push_back(Edge { start: v3, end: v4 });
            }
        }
    }

    /// Checks whether edge given by halfedge intersects line segment
    fn intersects(&self, edge: &LineSegment2<TScalar>, he: usize) -> Option<Intersection<TScalar>> {
        let (v1, v2) = self.mesh().halfedge_vertices(he);
        let v1_pos = self.points[v1];
        let v2_pos = self.points[v2];
        let segment = LineSegment2::new(v1_pos, v2_pos);

        return segment.intersects_line_segment2_at(edge)
            .map(|(t, _)| {
                if t.is_one() || t.is_zero() {
                    return Intersection::AtEnd;
                } else {
                    return Intersection::OnLine(segment.at(t));
                }
            });
    }

    /// Returns reference to underlying mesh representation
    #[inline]
    fn mesh(&self) -> &HalfedgeMesh {
        return &self.delaunay.mesh;
    }

    /// Returns mutable reference to underlying mesh representation
    #[inline]
    fn mesh_mut(&mut self) -> &mut HalfedgeMesh {
        return &mut self.delaunay.mesh;
    }
}

#[derive(PartialEq, Eq)]
enum Intersection<TScalar: RealNumber> {
    OnLine(Point2<TScalar>),
    AtEnd
}

#[derive(Clone, Copy, Debug)]
struct Edge {
    start: usize,
    end: usize
}

impl Edge {
    #[inline]
    fn new(start: usize, end: usize) -> Self {
        return Self{start, end};
    }
}

impl Hash for Edge {
    #[inline]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.start ^ self.end).hash(state);
    }
}

impl PartialEq for Edge {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        return 
            (self.start == other.start && self.end == other.end) ||
            (self.start == other.end && self.end == other.start);
    }
}

impl Eq for Edge {}

mod debugging {
    use std::path::Path;

    use nalgebra::{Point2, Vector2};
    use num_traits::cast;
    use svg::{node::element::{Text, Circle, Group, path::Data}, Document};

    use crate::geometry::traits::RealNumber;

    use super::ConstrainedTriangulation2;

    #[allow(dead_code)]
    pub fn save_to_svg<TScalar: RealNumber>(tri: &ConstrainedTriangulation2<TScalar>, file: &Path) {
        let scale = 100.0;
        let min: Point2<TScalar> = tri.points().iter().fold(Point2::origin(), |acc, p| acc.coords.inf(&p.coords).into());
        let size: Point2<TScalar> = (tri.points().iter().fold(Vector2::zeros(), |acc, p| acc.sup(&p.coords)) - min.coords).into();

        let constraints = tri.constraints.iter()
            .fold(Data::new(), |data, edge| {
                let v1 = tri.points()[edge.start];
                let v2 = tri.points()[edge.end];

                return data
                    .move_to((
                        cast::<TScalar, f64>(v1.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v1.y - min.y).unwrap() * scale
                    ))
                    .line_to((
                        cast::<TScalar, f64>(v2.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v2.y - min.y).unwrap() * scale
                    ));
            });

        let triangles = tri.triangles()
            .chunks(3)
            .fold(Data::new(), |data, face| {
                let v1 = tri.points()[face[0]];
                let v2 = tri.points()[face[1]];
                let v3 = tri.points()[face[2]];

                return data
                    .move_to((
                        cast::<TScalar, f64>(v1.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v1.y - min.y).unwrap() * scale
                    ))
                    .line_to((
                        cast::<TScalar, f64>(v2.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v2.y - min.y).unwrap() * scale
                    ))
                    .move_to((
                        cast::<TScalar, f64>(v2.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v2.y - min.y).unwrap() * scale
                    ))
                    .line_to((
                        cast::<TScalar, f64>(v3.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v3.y - min.y).unwrap() * scale
                    ))
                    .move_to((
                        cast::<TScalar, f64>(v3.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v3.y - min.y).unwrap() * scale
                    ))
                    .line_to((
                        cast::<TScalar, f64>(v1.x - min.x).unwrap() * scale,
                        cast::<TScalar, f64>(v1.y - min.y).unwrap() * scale
                    ));
            });

        let triangles_path = svg::node::element::Path::new()
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", 0.5)
            .set("d", triangles);

        
        let constraints_path = svg::node::element::Path::new()
            .set("fill", "none")
            .set("stroke", "red")
            .set("stroke-width", 0.5)
            .set("d", constraints);

        let points = tri.points().iter()
            .enumerate()
            .map(|(i, v)| (
                Text::new()
                    .add(svg::node::Text::new(format!(" {}", i)))
                    .set("x", cast::<TScalar, f64>(v.x - min.x).unwrap() * scale)
                    .set("y", cast::<TScalar, f64>(v.y - min.y).unwrap() * scale)
                    .set("font-size", "16px"),
                Circle::new()
                    .set("r", 5)
                    .set("cx", cast::<TScalar, f64>(v.x - min.x).unwrap() * scale)
                    .set("cy", cast::<TScalar, f64>(v.y - min.y).unwrap() * scale)
                    .set("fill", "green")
                ))
            .fold(Group::new(), |group, (text, circle)| group.add(text).add(circle))
            .add(triangles_path)
            .add(constraints_path);
    
        let doc = Document::new()
            .set("height", size.y.to_f64().unwrap() * scale)
            .set("width", size.x.to_f64().unwrap() * scale)
            .add(points);

        svg::save(file, &doc).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use nalgebra::Point2;

    use crate::triangulation::constrained_delaunay::debugging::save_to_svg;

    use super::{ConstrainedTriangulation2, Edge};

    static POINTS: [Point2<f32>; 7] = [
        Point2::new(-3.0, 1.0),
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 4.0),
        Point2::new(3.0, 2.0),
        Point2::new(6.0, 0.0),
        Point2::new(6.0, 4.0),
        Point2::new(9.0, 2.0)
    ];

    #[test]
    fn test_constrained_delaunay_triangulation() {
        let constrained_edge = Edge{ start:0, end: 6 };
        let mut tri = ConstrainedTriangulation2::new();
        tri.set_points(&POINTS);
        tri.add_constrained_edge(constrained_edge.start, constrained_edge.end).expect("To be valid edge");
        tri.triangulate();
        
        let edge_exist = tri.triangles()
            .chunks(3)
            .any(|face| {
                let e1 = Edge{start: face[0], end: face[1]};
                let e2 = Edge{start: face[1], end: face[2]};
                let e3 = Edge{start: face[2], end: face[0]};

                return e1 == constrained_edge || e2 == constrained_edge || e3 == constrained_edge;
            });

        assert!(edge_exist, "Constrained edge should exist in triangulation");
    }

    #[test]
    fn test_constrained_delaunay_triangulation_add_edge_intersecting_edge() {
        let mut tri = ConstrainedTriangulation2::new();
        tri.set_points(&POINTS);
        tri.add_constrained_edge(0, 6).expect("To be valid edge");
        tri.add_constrained_edge(1, 5).expect("To be valid edge");
        tri.triangulate();
    }

    #[test]
    fn test_constrained_delaunay_triangulation_add_edge_intersecting_point() {
        let mut tri = ConstrainedTriangulation2::new();
        tri.set_points(&POINTS);
        tri.add_constrained_edge(0, 6).expect("To be valid edge");
        tri.add_constrained_edge(1, 3).expect("To be valid edge");
        tri.triangulate();

        save_to_svg(&tri, Path::new("test.svg"));
    }
}
