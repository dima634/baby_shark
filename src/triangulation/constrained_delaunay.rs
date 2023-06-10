use std::{collections::HashSet, hash::Hash};

use nalgebra::Point2;

use crate::{
    geometry::{traits::{RealNumber, Intersects}, 
    primitives::line_segment2::LineSegment2, 
    orientation::{orientation2d, Orientation}}, 
    data_structures::linked_list::LinkedList
};

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
/// let mut tri = ConstrainedTriangulation2::from_points(&points);
/// tri.insert_constrained_edge(0, 6);
/// tri.insert_constrained_edge(1, 2);
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

    /// Creates triangulator from set of points
    pub fn from_points(points: &[Point2<TScalar>]) -> Self {
        let mut delaunay = Triangulation2::new();
        delaunay.triangulate(points);

        return Self { 
            delaunay,
            intersected_edges: LinkedList::new(),
            constraints: HashSet::new(),
            points: Vec::from(points)
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
        self.constraints.clear();
        self.delaunay.triangulate(&self.points);
    }

    /// Adds constrained edge given by indices of two points. 
    /// Conflicting constraints are resolved as follows:
    /// * When new constrained edge intersects other constrained edge both edges are splitted into two at intersection points
    /// * When new constrained edge intersects point it is splitted into two at this point
    pub fn insert_constrained_edge(&mut self, start: usize, end: usize) {
        let new_constraint = Edge { start, end };
        self.constraints.insert(new_constraint);
        self.insert_edge(new_constraint);
    }

    /// Inserts edge into existing triangulation by flipping edges intersected by it
    fn insert_edge(&mut self, mut edge: Edge) {
        // Edge exist?
        if self.mesh().connection_halfedge(edge.start, edge.end).is_some() {
            return;
        }

        self.intersected_edges.clear();

        let edge_start_pos = self.points[edge.start];
        let edge_end_pos   = self.points[edge.end];
        let edge_segment = LineSegment2::new(edge_start_pos, edge_end_pos);

        let (mut intersected_he, intersection) = self.find_first_intersected_edge(&edge, &edge_segment);

        if let Intersection::OnLine(point) = intersection {
            let splitted = self.split_at_intersection_if_constrained(&edge, intersected_he, point);

            if splitted {
                return;
            }
        }

        // Collect intersected edges
        loop {
            // Insert edge to list of intersected ones
            let (v1, v2) = self.mesh().halfedge_vertices(intersected_he);
            self.intersected_edges.push_back(Edge{ start: v1, end: v2 });

            let opposite = self.mesh().opposite_halfedge(intersected_he).unwrap();

            // Next intersected edge is either previous or next halfedge
            let next_he = next_halfedge(opposite);

            if let Some(intersection) = self.intersects(&edge, &edge_segment, next_he) {
                intersected_he = next_he;

                match intersection {
                    Intersection::OnLine(point) => {
                        let splitted = self.split_at_intersection_if_constrained(&edge, intersected_he, point);

                        if splitted {
                            return;
                        }
                    },
                    Intersection::AtEnd => {
                        // Split constrained edge at intersection
                        let (_, v_end) = self.mesh().halfedge_vertices(intersected_he);

                        if v_end == edge.end {
                            break;
                        }

                        // Split original edge in two
                        let new_edge = Edge::new(v_end, edge.end);
                        edge.end = v_end;
                        self.constraints.insert(edge);
                        self.constraints.insert(new_edge);
                        self.insert_edge(edge);
                        self.insert_edge(new_edge);
    
                        return;
                    },
                }
            } else {
                intersected_he = prev_halfedge(opposite);
                let intersection = self.intersects(&edge, &edge_segment, intersected_he).unwrap();

                match intersection {
                    Intersection::OnLine(point) => {
                        let splitted = self.split_at_intersection_if_constrained(&edge, intersected_he, point);

                        if splitted {
                            return;
                        }
                    },
                    Intersection::AtEnd => {
                        // Split constrained edge at intersection
                        let (v_start, _) = self.mesh().halfedge_vertices(intersected_he);

                        if v_start == edge.end {
                            break;
                        }

                        // Split original edge in two
                        let new_edge = Edge::new(v_start, edge.end);
                        edge.end = v_start;
                        self.constraints.insert(edge);
                        self.constraints.insert(new_edge);
                        self.insert_edge(edge);
                        self.insert_edge(new_edge);

                        return;
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

    fn find_first_intersected_edge(&self, edge: &Edge, edge_segment: &LineSegment2<TScalar>) -> (usize, Intersection<TScalar>) {
        let mut intersected_edge = None;
        let mut intersection;

        let start = self.mesh().outgoing_halfedge(edge.start);
        let mut he = start;
        let mut border = false;
        
        // Find first edge intersected by constrained
        // Traverse edges around vertex and test for intersection
        loop {
            let current = next_halfedge(he);

            intersection = self.intersects(&edge, &edge_segment, current);
            if intersection.is_some() {
                intersected_edge = Some(current);
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
    
                intersection = self.intersects(&edge, &edge_segment, e);
                if intersection.is_some() {
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

        debug_assert!(intersected_edge.is_some(), "There has to be intersection");
        debug_assert!(intersection.is_some(), "There has to be intersection");

        return (intersected_edge.unwrap(), intersection.unwrap());
    }

    fn split_at_intersection_if_constrained(&mut self, edge: &Edge, intersected_he: usize, intersection_point: Point2<TScalar>) -> bool {
        let (v1, v2) = self.mesh().halfedge_vertices(intersected_he);
        let intersected_edge = Edge::new(v1, v2);
        
        if self.constraints.contains(&intersected_edge) {
            // Insert point of intersection into triangulation
            self.points.push(intersection_point);
            let v = self.points.len() - 1;
            self.mesh_mut().split_edge(intersected_he, v);

            let e1 = Edge::new(edge.start, v);
            let e2 = Edge::new(v, edge.end);

            // Update constrained edges after split
            self.constraints.insert(e1);
            self.constraints.insert(e2);
            self.constraints.insert(Edge::new(v1, v));
            self.constraints.insert(Edge::new(v, v2));

            // Insert edges created by splitting original into two
            self.insert_edge(e1);
            self.insert_edge(e2);

            return true;
        }

        return false;
    }

    /// Checks whether edge given by halfedge intersects line segment
    fn intersects(&self, edge: &Edge, edge_segment: &LineSegment2<TScalar>, he: usize) -> Option<Intersection<TScalar>> {
        let (v1, v2) = self.mesh().halfedge_vertices(he);
        let he_edge = Edge::new(v1, v2);

        if edge.sharing_vertex(&he_edge) {
            return Some(Intersection::AtEnd);
        }

        let v1_pos = self.points[v1];
        let v2_pos = self.points[v2];
        let segment = LineSegment2::new(v1_pos, v2_pos);

        return segment.intersects_line_segment2_at(edge_segment)
            .map(|(t, _)| {
                if t.is_zero() || t.is_one() {
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

impl From<(usize, usize)> for Edge {
    #[inline]
    fn from(value: (usize, usize)) -> Self {
        return Self {
            start: value.0,
            end: value.1
        };
    }
}

impl Edge {
    #[inline]
    fn new(start: usize, end: usize) -> Self {
        return Self{start, end};
    }

    #[inline]
    fn sharing_vertex(&self, other: &Self) -> bool {
        return self.start == other.start || self.start == other.end || self.end == other.start || self.end == other.end;
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
            (self.start == other.end   && self.end == other.start);
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
            .set("stroke-width", 1)
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
            .add(constraints_path)
            .add(triangles_path);
    
        let doc = Document::new()
            .set("height", size.y.to_f64().unwrap() * scale)
            .set("width", size.x.to_f64().unwrap() * scale)
            .add(points);

        svg::save(file, &doc).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::HashSet, iter::repeat_with};

    use nalgebra::Point2;
    use rand::{rngs::StdRng, SeedableRng, Rng, RngCore};

    use super::{ConstrainedTriangulation2, Edge};

    #[test]
    fn test_constrained_delaunay_triangulation() {
        let points: [Point2<f32>; 7] = [
            Point2::new(-3.0, 1.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 4.0),
            Point2::new(3.0, 2.0),
            Point2::new(6.0, 0.0),
            Point2::new(6.0, 4.0),
            Point2::new(9.0, 2.0)
        ];
        let constrained_edge = Edge{ start:0, end: 6 };
        let mut tri = ConstrainedTriangulation2::new();
        tri.set_points(&points);
        tri.insert_constrained_edge(constrained_edge.start, constrained_edge.end);
        
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
    fn test_two_intersecting_constrained_edges() {
        let points: Vec<Point2<f32>> = 
            [
                [0.036289513, 0.030899823], [0.8319869, 0.9282945], [0.632523, 0.7931476], [0.26751977, 0.81933427], [0.18547505, 0.19786644], 
                [0.83746547, 0.84567744], [0.07653844, 0.33284003], [0.69826245, 0.04364556], [0.0780437, 0.24317974], [0.011519253, 0.2217015], 
                [0.59603006, 0.21335393], [0.4317752, 0.6709788], [0.61111355, 0.58693725], [0.43586057, 0.12799275], [0.9518744, 0.21104646]
            ]
            .iter()
            .map(|[x, y]| Point2::new(*x, *y))
            .collect();

        let mut triangulation = ConstrainedTriangulation2::from_points(&points);

        let constraints = [
            Edge::new(12, 10),
            Edge::new(5, 9),
            Edge::new(14, 0),
            Edge::new(14, 6)
        ];

        let expected_constraints = [
            // Edge::new(12, 10),
            Edge::new(12, 15),
            Edge::new(15, 10),

            // Edge::new(5, 9),
            Edge::new(5, 16),
            Edge::new(16, 9),

            Edge::new(14, 0),

            // Edge::new(14, 6)
            Edge::new(6, 16),
            Edge::new(16, 15),
            Edge::new(15, 14),
        ];
        
        for edge in constraints {
            triangulation.insert_constrained_edge(edge.start, edge.end);
        }

        let existing_edges = triangulation.triangles()
            .chunks(3)
            .fold(HashSet::new(), |mut acc, face| {
                let e1 = Edge{start: face[0], end: face[1]};
                let e2 = Edge{start: face[1], end: face[2]};
                let e3 = Edge{start: face[2], end: face[0]};

                acc.insert(e1);
                acc.insert(e2);
                acc.insert(e3);

                return acc;
            });

        for edge in expected_constraints {
            assert!(existing_edges.contains(&edge), "Edge {:?} should exist in triangulation", edge);
        }
    }

    #[test]
    fn test_random() {
        let mut rng = StdRng::seed_from_u64(rand::random());
        let points2d: Vec<Point2<f32>> = 
            repeat_with(|| rng.gen())        
            .take(100)
            .map(|(x, y)| Point2::new(x, y))
            .collect();
    
        let mut triangulation = ConstrainedTriangulation2::from_points(&points2d);
    
        // Create random constrains
        for _ in 0..10 {
            let v1 = rng.next_u64() as usize % points2d.len();
            let v2 = rng.next_u64() as usize % points2d.len();
    
            if v1 == v2 {
                continue;
            }
    
            triangulation.insert_constrained_edge(v1, v2);
        }
    }

    #[test]
    fn test_edge_intersects_point() {
        let points: [Point2<f32>; 16] = [
            Point2::new(0.0, 1.0),
            Point2::new(0.0, 2.0),
            Point2::new(0.0, 3.0),
            Point2::new(0.0, 4.0),
            
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 2.0),
            Point2::new(1.0, 3.0),
            Point2::new(1.0, 4.0),
            
            Point2::new(2.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(2.0, 3.0),
            Point2::new(2.0, 4.0),
            
            Point2::new(3.0, 1.0),
            Point2::new(3.0, 2.0),
            Point2::new(3.0, 3.0),
            Point2::new(3.0, 4.0),
        ];
    
        let mut triangulation = ConstrainedTriangulation2::from_points(&points);
        triangulation.insert_constrained_edge(2, 8);

        let expected_constraints = [
            Edge::new(2, 5),
            Edge::new(5, 8),
        ];

        let existing_edges = triangulation.triangles()
            .chunks(3)
            .fold(HashSet::new(), |mut acc, face| {
                let e1 = Edge{start: face[0], end: face[1]};
                let e2 = Edge{start: face[1], end: face[2]};
                let e3 = Edge{start: face[2], end: face[0]};

                acc.insert(e1);
                acc.insert(e2);
                acc.insert(e3);

                return acc;
            });

        for edge in expected_constraints {
            assert!(existing_edges.contains(&edge), "Edge {:?} should exist in triangulation", edge);
        }
    }
}
