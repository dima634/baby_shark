use nalgebra::{Point2, Vector2};
use nalgebra_glm::{min2, max2};
use num_traits::cast;

use crate::{
    geometry::{
        traits::RealNumber, 
        primitives::triangle2::{Triangle2, self}, 
        orientation::{orientation2d, Orientation, signed_diamond_angle, signed_diamond_angle_between_vectors}
    },
    helpers::utils::sort3_by, 
    data_structures::linked_list::{LinkedList, Link}
};

use super::halfedge::{HalfedgeMesh, next_halfedge, prev_halfedge};

#[derive(Debug)]
struct FrontierEdge {
    halfedge: usize,
    f_start: Link,
    f_end: Link
}

#[derive(Debug)]
struct Vertex<TScalar: RealNumber> {
    radius_squared: TScalar,
    angle: TScalar,
    original_index: usize
}

///
/// 2D delaunay triangulation.
/// 
/// ## Example
/// ```
/// use nalgebra::Point2;
/// use baby_shark::triangulation::delaunay::Triangulation2;
/// 
/// let points = vec![
///     Point2::new(1.0, 2.0),
///     Point2::new(5.0, 1.0),
///     Point2::new(8.0, 6.0),
///     Point2::new(2.0, 8.0)
/// ];
/// let mut triangulation = Triangulation2::new().with_points(&points);
/// triangulation.triangulate();
/// ```
/// 
/// Based on "A faster circle-sweep Delaunay triangulation algorithm" by Ahmad Biniaz and Gholamhossein Dastghaibyfard:
/// https://cglab.ca/~biniaz/papers/Sweep%20Circle.pdf
/// 
#[derive(Debug)]
pub struct Triangulation2<TScalar: RealNumber> {
    pub(super) mesh: HalfedgeMesh,  // mesh
    pole: Point2<TScalar>,          // origin of polar coordinates, center of bbox
    vertices: Vec<Vertex<TScalar>>, 
    stack: Vec<usize>,

    // Frontier
    frontier: LinkedList<usize>,
    hash: Vec<Option<Link>>
}

impl<TScalar: RealNumber> Triangulation2<TScalar> {
    pub fn new() -> Self {
        return Self {
            pole: Point2::origin(),
            vertices: Vec::new(),
            mesh: HalfedgeMesh::new(),
            stack: Vec::new(),
            hash: Vec::new(),
            frontier: LinkedList::new()
        };
    }

    #[inline]
    pub fn triangles(&self) -> &Vec<usize> {
        return &self.mesh.triangles();
    }

    /// Triangulate set of 2d points
    pub fn triangulate(&mut self, points: &[Point2<TScalar>]) {
        self.mesh.clear();
        self.vertices.clear();
        self.hash.clear();
        self.frontier.clear();

        // Preallocate memory
        self.mesh.reserve(points.len() * 2, points.len());
        let hash_size = (points.len() as f32).sqrt() as usize;
        self.hash.resize(hash_size, None);
        self.frontier.reserve(points.len() / 10);

        self.initialize(points);
        self.triangulation(points);
        self.finalize(points);
    }

    /// Return `true` if triangulation satisfies delaunay condition, `false` otherwise
    pub fn is_delaunay(&self, points: &[Point2<TScalar>]) -> bool {
        for he in 0..self.triangles().len() {
            let opposite_he = self.mesh.opposite_halfedge(he);

            if opposite_he.is_none() {
                continue;
            }

            let (v1, v2) = self.mesh.halfedge_vertices(he);
            let (_, v3) = self.mesh.halfedge_vertices(next_halfedge(he));
            let (_, v4) = self.mesh.halfedge_vertices(next_halfedge(opposite_he.unwrap()));

            let t1 = Triangle2::new(points[v1], points[v2], points[v3]);
            let t2 = Triangle2::new(points[v2], points[v1], points[v4]);

            let is_illegal =
                t1.is_inside_circumcircle(&points[v4]) ||
                t2.is_inside_circumcircle(&points[v3]);

            if is_illegal {
                return false;
            }
        }

        return true;
    }

    /// Pick initial triangle and pole, compute polar coordinates (pseudo)
    fn initialize(&mut self, points: &[Point2<TScalar>]) {
        // Compute initial pole and distances from vertices to it
        // Vertices are going to be inserted in order of increasing radius
        let (min, max) = points.iter().fold((Vector2::zeros(), Vector2::zeros()), |(min, max), p| (min2(&min, &p.coords), max2(&max, &p.coords)));
        self.pole = ((min + max) * cast::<_, TScalar>(0.5).unwrap()).into();
        self.vertices = points.iter()
            .enumerate()
            .map(|(i, p)| Vertex { 
                angle: TScalar::zero(),
                radius_squared: (p - self.pole).norm_squared(),
                original_index: i
            })
            .collect();
        self.vertices.sort_unstable_by(|a, b| a.radius_squared.partial_cmp(&b.radius_squared).unwrap());

        // Init frontier
        let mut a = 0;
        let mut b = 1;
        let mut c = 2;

        // Move pole to center of first triangle
        let a_pos = points[self.vertices[a].original_index];
        let b_pos = points[self.vertices[b].original_index];
        let c_pos = points[self.vertices[c].original_index];
        let new_pole = (a_pos + b_pos.coords + c_pos.coords) / cast(3).unwrap();
        
        // Compute angles, they are used to project vertices on frontier
        for vertex in &mut self.vertices {
            let v_pos = points[vertex.original_index];
            vertex.angle = signed_diamond_angle(v_pos.y - new_pole.y, v_pos.x - new_pole.x);
        }

        sort3_by(&mut a, &mut b, &mut c, |v| self.vertices[*v].angle);

        self.frontier.push_back(a);
        self.frontier.push_back(b);
        self.frontier.push_back(c);

        self.add_triangle(a, b, c, None, None, None, points);
    }

    /// Triangulation step
    fn triangulation(&mut self, points: &[Point2<TScalar>]) {
        for point in 3..self.vertices.len() {
            let edge = self.project_on_frontier(point); 

            self.add_triangle(self.frontier[edge.f_start], point, self.frontier[edge.f_end], None, None, Some(edge.halfedge), points);

            // Insert new point to frontier and update hash
            let f_point = 
                if edge.f_end == self.frontier.head().unwrap() {
                    if self.vertices[point].angle <= self.vertices[self.frontier[edge.f_end]].angle {
                        self.frontier.push_front(point)
                    } else {
                        self.frontier.push_back(point)
                    }
                } else {
                    self.frontier.insert_before(edge.f_end, point)
                };
            self.insert_to_hash(point, f_point);

            self.legalize(edge.halfedge, points);

            self.walk_left(f_point, points);
            self.walk_right(f_point, points);
        }
    }

    /// Finalization step
    fn finalize(&mut self, points: &[Point2<TScalar>]) {
        // Walk over frontier and add triangle for left turn
        let mut f = self.frontier.head().unwrap();
        let mut f1 = f;

        loop {
            let f2 = self.frontier.next_circular(f1).unwrap();
            let f3 = self.frontier.next_circular(f2).unwrap();
            
            let v1 = self.frontier[f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &points[self.vertices[v1].original_index];
            let v2_pos = &points[self.vertices[v2].original_index];
            let v3_pos = &points[self.vertices[v3].original_index];

            if orientation2d(v1_pos, v2_pos, v3_pos) == Orientation::Clockwise {
                let v1v2 = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v1));
                let v2v3 = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v2));
                self.add_triangle(v3, v2, v1, v2v3, v1v2, None, points);

                self.legalize(v2v3.unwrap(), points);
                self.legalize(v1v2.unwrap(), points);

                self.frontier.remove(f2);

                f1 = self.frontier.prev_circular(f1).unwrap();
                f = f1;
            } else {
                f1 = f2;
            }

            // Reached start?
            if f3 == f {
                break;
            }
        }
    }

    /// Add triangles while angle between edge outgoing from `f1` and next edge is smaller that 90
    fn walk_left(&mut self, f1: Link, points: &[Point2<TScalar>]) {
        loop {
            let f2 = self.frontier.next_circular(f1).unwrap();
            let f3 = self.frontier.next_circular(f2).unwrap();

            let v1 = self.frontier[f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &points[self.vertices[v1].original_index];
            let v2_pos = &points[self.vertices[v2].original_index];
            let v3_pos = &points[self.vertices[v3].original_index];

            let v2v1 = v1_pos - v2_pos;
            let v2v3 = v3_pos - v2_pos;

            let signed_angle = signed_diamond_angle_between_vectors(&v2v1, &v2v3);

            if signed_angle >= TScalar::one() {
                break;
            }

            let next_he = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v2));
            let current_he = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v1));

            // Update frontier
            self.add_triangle(v3, v2, v1, next_he, current_he, None, points);
            self.remove_from_hash(f2);
            self.frontier.remove(f2);

            self.legalize(current_he.unwrap(), points);
            self.legalize(next_he.unwrap(), points);
        }
    }
    
    /// Add triangles while angle between edge ingoing to `f1` and previous edge is smaller that 90
    fn walk_right(&mut self, f1: Link, points: &[Point2<TScalar>]) {
        loop {
            let f2 = self.frontier.prev_circular(f1).unwrap();
            let f3 = self.frontier.prev_circular(f2).unwrap();

            let v1 = self.frontier[f1];
            let v2 = self.frontier[f2];
            let v3 = self.frontier[f3];

            let v1_pos = &points[self.vertices[v1].original_index];
            let v2_pos = &points[self.vertices[v2].original_index];
            let v3_pos = &points[self.vertices[v3].original_index];

            let v2v1 = v1_pos - v2_pos;
            let v2v3 = v3_pos - v2_pos;

            let signed_angle = signed_diamond_angle_between_vectors(&v2v3, &v2v1);

            if signed_angle >= TScalar::one() {
                break;
            }

            let current_he = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v2));
            let prev_he = self.mesh.outgoing_border_halfedge(self.vertex_point_index(v3));

            self.add_triangle(v1, v2, v3, current_he, prev_he, None, points);

            // Update frontier
            self.remove_from_hash(f2);
            self.frontier.remove(f2);

            self.legalize(current_he.unwrap(), points);
            self.legalize(prev_he.unwrap(), points);
        }
    }

    /// Legalize triangle that shares `he` edge by recursive flip
    fn legalize(&mut self, he: usize, points: &[Point2<TScalar>]) {
        self.stack.push(he);

        while let Some(he) = self.stack.pop() {
            let opposite_he = self.mesh.opposite_halfedge(he);

            if opposite_he.is_none() {
                continue;
            }

            let (v1, v2) = self.mesh.halfedge_vertices(he);
            let (_, v3) = self.mesh.halfedge_vertices(next_halfedge(he));
            let (_, v4) = self.mesh.halfedge_vertices(next_halfedge(opposite_he.unwrap()));

            let is_illegal =
                triangle2::is_inside_circumcircle(&points[v1], &points[v2], &points[v3], &points[v4]) ||
                triangle2::is_inside_circumcircle(&points[v2], &points[v1], &points[v4], &points[v3]);

            if is_illegal {
                let he1 = prev_halfedge(he);
                let he2 = self.mesh.opposite_halfedge(he).unwrap();
                let he3 = prev_halfedge(he2);

                self.mesh.flip_edge(he);

                self.stack.push(he);
                self.stack.push(he1);
                self.stack.push(he2);
                self.stack.push(he3);
            }
        }

    }

    /// Returns edge intersected by projection of point
    fn project_on_frontier(&self, point_idx: usize) -> FrontierEdge {
        let ang = self.vertices[point_idx].angle;
        let hash_key = self.hash(ang);
        let mut f_start = self.hash[hash_key];

        if f_start.is_none() {
            // Walk left and right to find frontier edge that is closest to point
            let mut left = hash_key as isize;
            let mut right = hash_key;

            loop {
                if left >= 0 && self.hash[left as usize].is_some() {
                    f_start = self.hash[left as usize];
                    break;
                }

                if right < self.hash.len() && self.hash[right].is_some() {
                    f_start = self.hash[right];
                    break;
                }

                left -= 1;
                right += 1;

                if left < 0 && right > self.hash.len() {
                    break;
                }
            }
        }

        let f_start = f_start.unwrap_or(self.frontier.head().unwrap());

        let v_start = &self.vertices[self.frontier[f_start]];

        if ang < v_start.angle {
            for f_end in self.frontier.before(f_start) {
                let f_start = self.frontier.prev_circular(f_end).unwrap();
                let v_start = &self.vertices[self.frontier[f_start]];
                let v_end = &self.vertices[self.frontier[f_end]];
                
                if v_start.angle < ang && v_end.angle > ang {
                    return FrontierEdge {
                        f_end,
                        f_start,
                        halfedge: self.mesh.outgoing_border_halfedge(self.vertex_point_index(self.frontier[f_start])).unwrap()
                    };
                }
            }
        } else {
            for f_start in self.frontier.after(f_start) {
                let f_end = self.frontier.next_circular(f_start).unwrap();
                let v_start = &self.vertices[self.frontier[f_start]];
                let v_end = &self.vertices[self.frontier[f_end]];
                
                if v_start.angle < ang && v_end.angle > ang {
                    return FrontierEdge {
                        f_end,
                        f_start,
                        halfedge: self.mesh.outgoing_border_halfedge(self.vertex_point_index(self.frontier[f_start])).unwrap()
                    };
                }
            }
        }

        return FrontierEdge {
            f_end: self.frontier.head().unwrap(),
            f_start: self.frontier.tail().unwrap(),
            halfedge: self.mesh.outgoing_border_halfedge(self.vertex_point_index(self.frontier[self.frontier.tail().unwrap()])).unwrap()
        };
    }

    ///
    /// Insert vertex to hash
    /// `vertex` - index of vertex to be inserted into hash
    /// `frontier_vertex` - position of `vertex` on frontier
    /// 
    fn insert_to_hash(&mut self, vertex: usize, frontier_vertex: Link) {
        let vertex = &self.vertices[vertex];
        let hash_key = self.hash(vertex.angle);
        let old = self.hash[hash_key];

        if let Some(old) = old {
            let old_vertex = &self.vertices[self.frontier[old]];
            
            if old_vertex.angle < vertex.angle {
                return;
            }
        }

        self.hash[hash_key] = Some(frontier_vertex);
    }

    /// Insert vertex from hash
    fn remove_from_hash(&mut self, frontier_vertex: Link) {
        let vertex = &self.vertices[self.frontier[frontier_vertex]];
        let hash_key = self.hash(vertex.angle);
        let old = self.hash[hash_key];

        if old == Some(frontier_vertex) {
            self.hash[hash_key] = None;
        }
    }

    /// Compute index in hash for given angle
    #[inline]
    fn hash(&self, angle: TScalar) -> usize {
        return (angle / cast(4.0).unwrap() * cast(self.hash.len()).unwrap()).to_usize().unwrap() % self.hash.len();
    }

    #[inline]
    fn add_triangle(&mut self, v1: usize, v2: usize, v3: usize, he1: Option<usize>, he2: Option<usize>, he3: Option<usize>, points: &[Point2<TScalar>]) -> usize {
        let t = Triangle2::new(
            points[self.vertices[v1].original_index], 
            points[self.vertices[v2].original_index], 
            points[self.vertices[v3].original_index]
        );
        let orient = t.orientation();
        debug_assert!(orient == Orientation::CounterClockwise);
        
        return self.mesh.add_triangle(
            self.vertices[v1].original_index, 
            self.vertices[v2].original_index, 
            self.vertices[v3].original_index, he1, he2, he3);
    }

    #[inline]
    fn vertex_point_index(&self, vertex: usize) -> usize {
        return self.vertices[vertex].original_index;
    }
}

#[cfg(debug_assertions)]
pub(super) mod debugging {
    use std::path::Path;

    use nalgebra::Point2;
    use svg::{
        node::element::{
            path::Data, 
            Group, 
            Text, 
            Circle
        }, 
        Document
    };

    use crate::{geometry::traits::RealNumber, mesh::polygon_soup::data_structure::PolygonSoup, io::stl::StlWriter};
    use super::Triangulation2;

    #[allow(dead_code)]
    pub fn save_to_svg<TScalar: RealNumber>(triangulation: &Triangulation2<TScalar>, points: &[Point2<TScalar>], p: String) {
        let scale = 2000.0;
        let height = 2000.0;

        let lines = triangulation.frontier.values()
            .zip(triangulation.frontier.values()
                    .skip(1)
                    .chain([triangulation.frontier[triangulation.frontier.head().unwrap()]]
                    .iter()))
            .map(|(e_s, e_e)| {
                let p_s = points[*e_s];
                let p_e = points[*e_e];

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

        let points = points.iter()
            .enumerate()
            .map(|(i, v)| (
                Text::new().add(svg::node::Text::new(format!(" {};{:.2};{:.2}", i, triangulation.vertices[i].radius_squared, triangulation.vertices[i].angle)))
                    .set("x", num_traits::cast::<TScalar, f64>(v.x).unwrap() * scale)
                    .set("y", height - num_traits::cast::<TScalar, f64>(v.y).unwrap() * scale)
                    .set("font-size", "16px"),
                Circle::new()
                    .set("r", 5)
                    .set("cx", num_traits::cast::<TScalar, f64>(v.x).unwrap() * scale)
                    .set("cy", height - num_traits::cast::<TScalar, f64>(v.y).unwrap() * scale)
                    .set("fill", "green")
                ))
            .fold(Group::new(), |group, (text, circle)| group.add(text).add(circle));

        let path = svg::node::element::Path::new()
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", 0.5)
            .set("d", lines);

        let pole = Circle::new()
            .set("r", 5)
            .set("cx", num_traits::cast::<TScalar, f64>(triangulation.pole.x).unwrap() * scale)
            .set("cy", height-num_traits::cast::<TScalar, f64>(triangulation.pole.y).unwrap() * scale)
            .set("fill", "red");
    
        let doc = Document::new()
            .set("height", height)
            .add(path)
            .add(points)
            .add(pole);

        svg::save(Path::new(&p), &doc).unwrap();
    }

    #[allow(dead_code)]
    pub fn save_to_stl<TScalar: RealNumber>(triangulation: &Triangulation2<TScalar>, points: &[Point2<TScalar>], p: String) {
        let mut mesh = PolygonSoup::<TScalar>::new();
        for idx in (0..triangulation.triangles().len()).step_by(3) {
            mesh.add_face(
                points[triangulation.triangles()[idx]].to_homogeneous().into(), 
                points[triangulation.triangles()[idx + 1]].to_homogeneous().into(), 
                points[triangulation.triangles()[idx + 2]].to_homogeneous().into()
            );
        }
        let writer = StlWriter::new();
        writer.write_stl_to_file(&mesh, Path::new(&p)).expect("Must save to file");
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point2;

    use super::Triangulation2;

    #[test]
    fn test_triangulate_degenerate_case() {
        let points = vec![
            Point2::new(1.0, 2.0),
            Point2::new(5.0, 1.0),
            Point2::new(8.0, 6.0),
            Point2::new(2.0, 8.0)
        ];
        let mut triangulation = Triangulation2::new();
        triangulation.triangulate(&points);
    }
    
    #[test]
    fn test_triangulate_uniform() {
        let points = [
            [0.48984146, 0.4899361], [0.4463194, 0.45261556], [0.42847013, 0.42460257], [0.41823488, 0.57288224], 
            [0.5913105, 0.45535183], [0.53855276, 0.5922733], [0.37710214, 0.5732515], [0.5043943, 0.6273088], 
            [0.34420383, 0.51125544], [0.62980384, 0.44524848], [0.34035563, 0.43844408], [0.61331505, 0.35406935], 
            [0.61050564, 0.34783804], [0.66835, 0.5447868], [0.32081836, 0.3943385], [0.4718566, 0.2961123], 
            [0.58064073, 0.66067904], [0.6851884, 0.48713744], [0.29649615, 0.57779795], [0.63608783, 0.6429018], 
            [0.46383494, 0.27027756], [0.70931464, 0.46491158], [0.7052269, 0.55955833], [0.54671764, 0.25920832], 
            [0.6284604, 0.68279934], [0.3177119, 0.3153975], [0.42712665, 0.7265839], [0.56969875, 0.7230318], 
            [0.49226338, 0.7405513], [0.4741112, 0.74244386], [0.2804165, 0.33925468], [0.29501998, 0.66089964], 
            [0.6637637, 0.6773343], [0.46313453, 0.74667466], [0.71958226, 0.37372464], [0.2911582, 0.31229526], 
            [0.43222797, 0.77797765], [0.71959144, 0.3079893], [0.76890755, 0.59576356], [0.24977851, 0.28614485], 
            [0.3248073, 0.20827317], [0.16804123, 0.57080585], [0.15872717, 0.5225644], [0.21868831, 0.6842079], 
            [0.35850996, 0.17893916], [0.26844138, 0.23370874], [0.18464863, 0.64404595], [0.18881321, 0.30391115], 
            [0.13282919, 0.49237096], [0.5348515, 0.84225017], [0.7661881, 0.7145568], [0.82922363, 0.6159804], 
            [0.8447025, 0.41357028], [0.80576605, 0.66857046], [0.65735865, 0.1653955], [0.4404143, 0.8562609], 
            [0.12434751, 0.56717086], [0.8447379, 0.38467562], [0.4579938, 0.11322808], [0.10814023, 0.48565876], 
            [0.66940445, 0.15512234], [0.18147635, 0.71670175], [0.17786211, 0.72111464], [0.12957686, 0.65061164], 
            [0.5351382, 0.088799596], [0.6344292, 0.8699391], [0.8590333, 0.6721916], [0.39739162, 0.06835717], 
            [0.32444948, 0.8887432], [0.114165425, 0.2647937], [0.16959798, 0.18581927], [0.039387226, 0.498999], 
            [0.70789284, 0.87855], [0.06639743, 0.31261855], [0.33921427, 0.053124905], [0.3961032, 0.02758801], 
            [0.11840737, 0.7837957], [0.014104009, 0.51041526], [0.6770156, 0.92879564], [0.1100536, 0.78631395], 
            [0.73594517, 0.072675765], [0.9592681, 0.6101808], [0.9563696, 0.63878834], [0.9551344, 0.31495094], 
            [0.6514476, 0.96371853], [0.860139, 0.14633244], [0.7776793, 0.91099083], [0.86620057, 0.14327657], 
            [0.06995958, 0.18052047], [0.79034364, 0.059315145], [0.023816466, 0.22163707], [0.056708217, 0.16669017],
            [0.7203423, 0.9783333], [0.23453873, 0.9738017], [0.78757405, 0.022196889], [0.833493, 0.92673594], 
            [0.1371184, 0.036289155], [0.021484733, 0.14100307], [0.9737798, 0.10746962], [0.95703167, 0.9444567]
        ];

        let points2d: Vec<_> = points.iter().map(|p| Point2::new(p[0], p[1])).collect();
        let mut triangulation = Triangulation2::new();
        triangulation.triangulate(&points2d);
        triangulation.triangles();

        assert!(triangulation.is_delaunay(&points2d));
    }
}
