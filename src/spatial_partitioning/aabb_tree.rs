use nalgebra::Vector3;
use num_traits::*;

use crate::{
    geometry::{
        primitives::{box3::Box3, plane3::Plane3, triangle3::Triangle3},
        traits::{ClosestPoint3, HasBBox3, RealNumber},
    },
    helpers::aliases::Vec3,
    mesh::traits::Mesh,
};

#[derive(Debug, PartialEq, Clone, Copy)]
enum NodeType {
    Leaf,
    Branch,
}

#[derive(Debug, Clone, Copy)]
struct BinaryNode<TScalar: RealNumber> {
    node_type: NodeType,
    left: usize,  // For child nodes (left, right) is range of objects contained in node,
    right: usize, // for leaf nodes these are indices of child nodes
    bbox: Box3<TScalar>,
}

impl<TScalar: RealNumber> BinaryNode<TScalar> {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        return self.node_type == NodeType::Leaf;
    }
}

///
/// Bounding volume hierarchy of axis aligned bounding boxes
///
/// ## Generic parameters
/// * `TObject` - type of objects that going to be saved in tree
///
/// ## Example
/// ```ignore
/// let aabb = AABBTree::from_mesh(&mesh)
///     .with_min_objects_per_leaf(10)
///     .with_max_depth(10)
///     .top_down::<MedianCut>();
/// ```
///
#[derive(Debug)]
pub struct AABBTree<TObject>
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber,
{
    nodes: Vec<BinaryNode<TObject::ScalarType>>, // root is last element
    objects: Vec<(TObject, Box3<TObject::ScalarType>)>,
    min_objects_per_leaf: usize,
    max_depth: usize,
}

impl<TObject> AABBTree<TObject>
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber,
{
    ///
    /// Create new AABB tree from objects. This method is not finishing construction of tree.
    /// To finish tree construction it should be chained with call of construction strategy ([top_down](AABBTree) etc)
    ///
    pub fn new(objects: Vec<TObject>) -> Self {
        return Self {
            nodes: Vec::new(),
            min_objects_per_leaf: 10,
            max_depth: 40,
            objects: objects
                .into_iter()
                .map(|obj| {
                    let bbox = obj.bbox();
                    return (obj, bbox);
                })
                .collect(),
        };
    }

    pub fn empty() -> Self {
        return Self {
            nodes: Vec::new(),
            min_objects_per_leaf: 10,
            max_depth: 40,
            objects: Vec::new(),
        };
    }

    /// Set minimal objects count per leaf node. Default value is 10
    pub fn with_min_objects_per_leaf(mut self, min_objects_per_leaf: usize) -> Self {
        self.min_objects_per_leaf = min_objects_per_leaf;
        return self;
    }

    /// Set max depth of tree. Default value is 40
    pub fn with_max_depth(mut self, max_depth: usize) -> Self {
        self.max_depth = max_depth;
        return self;
    }

    pub fn depth(&self) -> usize {
        if self.nodes.is_empty() {
            return 0;
        }

        self.node_depth(self.nodes.len() - 1)
    }

    ///
    /// Constructs AABB tree using top-down building strategy.
    /// This strategy is fast but not always produce best tree.
    ///
    /// ## Generic arguments
    /// * `TPartition` - partitioning strategy used to split two sets of objects into subnodes (see [MedianCut])
    ///
    pub fn top_down<TPartition: PartitionStrategy<TObject>>(mut self) -> Self {
        self.nodes.clear();

        if !self.objects.is_empty() {
            self.top_down_build_node(0, self.objects.len(), 1, &mut TPartition::default());
        }

        return self;
    }

    /// Traverse leaf node of tree
    #[inline]
    pub fn traverse<TFunc>(&self, visit: &mut TFunc)
    where
        TFunc: FnMut(
            (
                &[(TObject, Box3<TObject::ScalarType>)],
                &Box3<TObject::ScalarType>,
            ),
        ),
    {
        self.visit_node(self.nodes.len() - 1, visit);
    }

    /// Recursively visit tree node
    fn visit_node<TFunc>(&self, node_index: usize, visit: &mut TFunc)
    where
        TFunc: FnMut(
            (
                &[(TObject, Box3<TObject::ScalarType>)],
                &Box3<TObject::ScalarType>,
            ),
        ),
    {
        let node = &self.nodes[node_index];

        match node.node_type {
            NodeType::Leaf => {
                let objects = &self.objects[node.left..node.right];
                visit((objects, &node.bbox));
            }
            NodeType::Branch => {
                self.visit_node(node.left, visit);
                self.visit_node(node.right, visit);
            }
        }
    }

    /// Build tree node (leaf or branch) from set of objects
    fn top_down_build_node<TPartition: PartitionStrategy<TObject>>(
        &mut self,
        first: usize,
        last: usize,
        depth: usize,
        partition_strategy: &mut TPartition,
    ) -> usize {
        if depth >= self.max_depth || last - first <= self.min_objects_per_leaf {
            // println!("Leaf node: depth: {}, objects: {}", depth, last - first);
            // Create leaf node when number of objects is small
            return self.leaf_node_from_objects(first, last);
        } else {
            // Split set of objects
            let subset = &mut self.objects[first..last];
            let split_at_result =
                Self::split(subset, partition_strategy).map(|split_at| split_at + first);

            match split_at_result {
                Some(split_at) => {
                    // Create branch node if split succeeded
                    let left =
                        self.top_down_build_node(first, split_at, depth + 1, partition_strategy);
                    let right =
                        self.top_down_build_node(split_at, last, depth + 1, partition_strategy);

                    let mut bbox = self.nodes[left].bbox;
                    bbox.union_box(&self.nodes[right].bbox);

                    let node = BinaryNode {
                        bbox,
                        node_type: NodeType::Branch,
                        left,
                        right,
                    };

                    self.nodes.push(node);

                    return self.nodes.len() - 1;
                }
                None => {
                    // println!("SPLIT FAILED");
                    // Create leaf node if split failed
                    let node = self.leaf_node_from_objects(first, last);

                    // if self.nodes[node].bbox.size_max() > TObject::ScalarType::from_f64(100.0).unwrap() {
                    //     std::io::stdin().read_line(&mut String::new());
                    // }

                    return node;
                }
            }
        }
    }

    fn split<TPartition: PartitionStrategy<TObject>>(
        objects: &mut [(TObject, Box3<TObject::ScalarType>)],
        partition_strategy: &mut TPartition,
    ) -> Option<usize> {
        // Split by biggest dimension first
        let bbox = objects
            .iter()
            .fold(objects[0].1, |acc, (_, bbox)| acc + bbox);

        let mut split_axises = vec![
            (bbox.size_x(), SplitAxis::X),
            (bbox.size_y(), SplitAxis::Y),
            (bbox.size_z(), SplitAxis::Z),
        ];

        // Sort by bbox size along split axis
        split_axises.sort_by(|(size1, _), (size2, _)| size2.partial_cmp(size1).unwrap());
        Self::sort_along_axis_and_try_split(objects, split_axises[0].1, partition_strategy, &bbox)
            .or_else(|| {
                Self::sort_along_axis_and_try_split(
                    objects,
                    split_axises[1].1,
                    partition_strategy,
                    &bbox,
                )
            })
            .or_else(|| {
                Self::sort_along_axis_and_try_split(
                    objects,
                    split_axises[2].1,
                    partition_strategy,
                    &bbox,
                )
            })
    }

    fn sort_along_axis_and_try_split<TPartition: PartitionStrategy<TObject>>(
        objects: &mut [(TObject, Box3<TObject::ScalarType>)],
        axis: SplitAxis,
        partition_strategy: &mut TPartition,
        objects_bbox: &Box3<TObject::ScalarType>,
    ) -> Option<usize> {
        let axis_idx = axis.as_usize();
        objects.sort_by(|(_, bbox1), (_, bbox2)| {
            bbox1.get_center()[axis_idx]
                .partial_cmp(&bbox2.get_center()[axis_idx])
                .unwrap()
        });

        partition_strategy.split(objects, axis, objects_bbox)
    }

    /// Create leaf node from set of objects
    fn leaf_node_from_objects(&mut self, first: usize, last: usize) -> usize {
        // Compute bounding box of set of objects
        let (_, mut bbox) = self.objects[first];
        for i in first + 1..last {
            bbox.union_box(&self.objects[i].1);
        }

        let node = BinaryNode {
            bbox,
            node_type: NodeType::Leaf,
            left: first,
            right: last,
        };

        self.nodes.push(node);

        return self.nodes.len() - 1;
    }

    fn node_depth(&self, idx: usize) -> usize {
        let node = &self.nodes[idx];

        match node.node_type {
            NodeType::Leaf => 1,
            NodeType::Branch => 1 + self.node_depth(node.left).max(self.node_depth(node.right)),
        }
    }

    fn node_bounds(&self, idx: usize) -> (usize, usize) {
        let node = &self.nodes[idx];

        match node.node_type {
            NodeType::Leaf => (node.left, node.right),
            NodeType::Branch => {
                let left = self.node_bounds(node.left);
                let right = self.node_bounds(node.right);

                (left.0.min(right.0), left.1.max(right.1))
            }
        }
    }
}

impl<TScalar: RealNumber> AABBTree<Triangle3<TScalar>> {
    ///
    /// Create new AABB tree from faces of triangular mesh. This method is not finishing construction of tree.
    /// To finish tree construction it should be chained with call of construction strategy ([top_down](AABBTree) etc)
    ///
    pub fn from_mesh<TMesh: Mesh<ScalarType = TScalar>>(mesh: &TMesh) -> Self {
        let faces: Vec<Triangle3<TScalar>> = mesh
            .faces()
            .map(|face| mesh.face_positions(&face))
            .collect();

        return Self::new(faces);
    }
}

impl<TObject> AABBTree<TObject>
where
    TObject: HasBBox3 + ClosestPoint3,
    TObject::ScalarType: RealNumber,
{
    pub fn closest_point(
        &self,
        point: &Vec3<TObject::ScalarType>,
        max_distance: TObject::ScalarType,
    ) -> Option<Vec3<TObject::ScalarType>> {
        let max_distance_square = max_distance * max_distance;

        let mut stack = Vec::with_capacity(self.max_depth);
        stack.push(self.nodes.last().unwrap());

        let mut closest_point = Vec3::zeros();
        let mut distance_squared = Float::infinity();

        while !stack.is_empty() {
            let top = stack.pop().unwrap();

            if top.is_leaf() {
                for (obj, _) in &self.objects[top.left..top.right + 1] {
                    let new_closest = obj.closest_point(point);
                    let new_distance = (new_closest - point).norm_squared();

                    if new_distance < distance_squared {
                        distance_squared = new_distance;
                        closest_point = new_closest;
                    }
                }
            } else {
                let left = &self.nodes[top.left];
                let right = &self.nodes[top.right];

                if left.bbox.contains_point(point)
                    || left.bbox.squared_distance(point) < max_distance_square
                {
                    stack.push(left);
                }

                if right.bbox.contains_point(point)
                    || right.bbox.squared_distance(point) < max_distance_square
                {
                    stack.push(right);
                }
            }
        }

        if distance_squared.is_infinite() {
            return None;
        }

        return Some(closest_point);
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SplitAxis {
    X,
    Y,
    Z,
}

impl SplitAxis {
    pub fn as_usize(&self) -> usize {
        match self {
            SplitAxis::X => 0,
            SplitAxis::Y => 1,
            SplitAxis::Z => 2,
        }
    }
}

///
/// Partition strategy trait. Partition strategy is used to split set of object into two parts during AABB tree construction.
/// See [AABBTree]
///
pub trait PartitionStrategy<TObject: HasBBox3>: Default {
    ///
    /// Splits set of objects into two parts. Returns index of split.
    /// This method can rearrange elements with indices between `first` and `last`.
    /// But it is not allowed to mutate element outside that slice or add/remove elements to objects vector.
    ///
    fn split(
        &mut self,
        objects: &[(TObject, Box3<TObject::ScalarType>)],
        axis: SplitAxis,
        objects_bbox: &Box3<TObject::ScalarType>,
    ) -> Option<usize>;
}

///
/// A simple partitioning method - median-cut algorithm. Here, the set is divided in
/// two equal-size parts with respect to their projection along the selected axis, resulting
/// in a balanced tree.
///
#[derive(Default)]
pub struct MedianCut;

impl<TObject> PartitionStrategy<TObject> for MedianCut
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber,
{
    fn split(
        &mut self,
        objects: &[(TObject, Box3<TObject::ScalarType>)],
        axis: SplitAxis,
        objects_bbox: &Box3<TObject::ScalarType>,
    ) -> Option<usize> {
        if objects.is_empty() {
            return None;
        }

        let split_at = objects.len() / 2;

        if !check_split(axis.as_usize(), &objects_bbox, objects, split_at) {
            return None;
        }

        // Return medial point
        Some(split_at)
    }
}

#[derive(Debug, Default)]
pub struct Area;

impl<TObject> PartitionStrategy<TObject> for Area
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber,
{
    fn split(
        &mut self,
        objects: &[(TObject, Box3<<TObject>::ScalarType>)],
        axis: SplitAxis,
        objects_bbox: &Box3<<TObject>::ScalarType>,
    ) -> Option<usize> {
        if objects.is_empty() {
            return None;
        }

        let centroid_bounds = objects
            .iter()
            .fold(Box3::empty(), |acc, (_, bbox)| acc + &bbox.get_center());

        let axis = axis.as_usize();
        let empty_bucket = Bucket::<TObject::ScalarType> {
            primitives_count: 0,
            bbox: Box3::empty(),
        };
        const NUM_BUCKETS: usize = 12;
        let mut buckets = [empty_bucket; NUM_BUCKETS];
        let num_buckets = TObject::ScalarType::from_usize(NUM_BUCKETS).unwrap();

        // Put objects into `NUM_BUCKETS` buckets and compute bounds of each bucket
        for (_, bbox) in objects {
            let center = bbox.get_center();
            let bucket_idx = (num_buckets * centroid_bounds.offset(&center)[axis])
                .to_usize()
                .unwrap()
                .min(NUM_BUCKETS - 1);

            let bucket = &mut buckets[bucket_idx];
            bucket.primitives_count += 1;
            bucket.bbox.union_box(bbox);
        }

        let mut costs = [TObject::ScalarType::zero(); NUM_BUCKETS - 1];

        // Compute bucket cost
        for i in 0..NUM_BUCKETS - 1 {
            let mut b0 = Box3::empty();
            let mut b1 = Box3::empty();

            let mut b0_count = 0;
            let mut b1_count = 0;

            for j in 0..=i {
                b0.union_box(&buckets[j].bbox);
                b0_count += buckets[j].primitives_count;
            }

            for j in i + 1..NUM_BUCKETS {
                b1.union_box(&buckets[j].bbox);
                b1_count += buckets[j].primitives_count;
            }

            let b0_count = TObject::ScalarType::from_usize(b0_count).unwrap();
            let b1_count = TObject::ScalarType::from_usize(b1_count).unwrap();
            let c = TObject::ScalarType::from_f64(0.125).unwrap();

            costs[i] = c + (b0_count * b0.area() + b1_count * b1.area()) / objects_bbox.area();
        }

        let (min_cost_bucket_idx, _) = costs
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        let leaf_cost = TObject::ScalarType::from_usize(objects.len()).unwrap();

        if costs[min_cost_bucket_idx] < leaf_cost {
            let split_at = objects.iter().position(|(_, bbox)| {
                let center = bbox.get_center();
                let bucket_idx = (num_buckets * centroid_bounds.offset(&center)[axis])
                    .to_usize()
                    .unwrap()
                    .min(NUM_BUCKETS - 1);
                bucket_idx > min_cost_bucket_idx
            });

            return split_at;
        }

        None
    }
}

#[derive(Debug, Clone, Copy)]
struct Bucket<T: RealNumber> {
    primitives_count: usize,
    bbox: Box3<T>,
}

fn check_split<TObject: HasBBox3>(
    axis: usize,
    parent_bbox: &Box3<TObject::ScalarType>,
    objects: &[(TObject, Box3<TObject::ScalarType>)],
    split_at: usize,
) -> bool
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber,
{
    let mut split_axis = Vector3::<TObject::ScalarType>::zeros();
    split_axis[axis] = One::one();
    let split_point = (objects[split_at].1.get_center() + objects[split_at - 1].1.get_center())
        * TObject::ScalarType::from_f32(0.5).unwrap();
    let plane = Plane3::new(split_axis, split_point[axis]);

    // Test whether all objects intersects plane
    let all_object_intersects_plane = objects
        .iter()
        .all(|(_, bbox)| bbox.intersects_plane3(&plane));

    if all_object_intersects_plane {
        return false;
    }

    let first_child_box = objects
        .iter()
        .take(split_at)
        .fold(objects[0].1, |acc, (_, bbox)| acc + bbox);
    let first_child_volume = first_child_box.volume();

    let second_child_box = objects
        .iter()
        .skip(split_at)
        .fold(objects[split_at].1, |acc, (_, bbox)| acc + bbox);
    let second_child_volume = second_child_box.volume();

    let parent_volume = parent_bbox.volume();
    let threshold = TObject::ScalarType::from_f64(0.8).unwrap();

    if first_child_volume / parent_volume > threshold
        && second_child_volume / parent_volume > threshold
    {
        return false;
    }

    true
}

pub mod winding_numbers {
    use std::f32::consts::PI;

    use num_traits::Float;

    use crate::{
        geometry::{primitives::triangle3::Triangle3, traits::RealNumber},
        helpers::aliases::{Mat3f, Vec3, Vec3f},
        mesh::traits::Mesh,
    };

    use super::{AABBTree, Area, BinaryNode, MedianCut, NodeType};

    pub fn solid_angle<T: RealNumber>(tri: &Triangle3<T>, q: &Vec3<T>) -> T {
        let mut qa = tri.p1() - q;
        let mut qb = tri.p2() - q;
        let mut qc = tri.p3() - q;

        let a_length = qa.norm();
        let b_length = qb.norm();
        let c_length = qc.norm();

        let zero = T::zero();

        // If any triangle vertices are coincident with query,
        // query is on the surface, which we treat as no solid angle.
        if a_length == zero || b_length == zero || c_length == zero {
            return zero;
        }

        // Normalize the vectors
        qa /= a_length;
        qb /= b_length;
        qc /= c_length;

        let numerator = qa.dot(&(qb - qa).cross(&(qc - qa)));

        // If numerator is 0, regardless of denominator, query is on the
        // surface, which we treat as no solid angle.
        if numerator == zero {
            return zero;
        }

        let denominator = T::one() + qa.dot(&qb) + qa.dot(&qc) + qb.dot(&qc);

        return Float::atan2(numerator, denominator) * T::from_f32(2.0).unwrap();
    }

    pub fn winding_number<'tri>(
        triangles: impl Iterator<Item = &'tri Triangle3<f32>>,
        point: &Vec3f,
    ) -> f32 {
        let mut wn = 0.0;

        for tri in triangles {
            wn += solid_angle(&tri, point);
        }

        wn / (4.0 * PI)
    }

    pub struct WindingNumbers {
        tree: AABBTree<Triangle3<f32>>,
        nodes_data: Vec<NodeData>,
    }

    impl WindingNumbers {
        pub fn from_mesh<'a, T: Mesh<ScalarType = f32>>(mesh: &'a T) -> Self {
            let mut tree = AABBTree::from_mesh(mesh)
                .with_min_objects_per_leaf(3)
                .top_down::<Area>();

            let nodes_data = compute_tree_coeffs(&mut tree);

            Self { tree, nodes_data }
        }

        pub fn from_triangles(triangles: Vec<Triangle3<f32>>) -> Self {
            let mut tree = AABBTree::new(triangles).top_down::<MedianCut>();

            let nodes_data = compute_tree_coeffs(&mut tree);

            Self { tree, nodes_data }
        }

        pub fn approximate(&self, point: &Vec3f, accuracy_scale: f32) -> f32 {
            if self.tree.nodes.is_empty() {
                return 0.0;
            }

            self.fast_wn(self.tree.nodes.len() - 1, point, accuracy_scale)
        }

        fn fast_wn(&self, root: usize, point: &Vec3f, accuracy_scale: f32) -> f32 {
            let node_data = &self.nodes_data[root];
            let dist = (point - node_data.dipole_center).norm();

            if dist > node_data.radius * accuracy_scale {
                let (ord1, ord2) = hessians(&node_data.dipole_center, point);
                return node_data.order1_coefficients.dot(&ord1)
                    + node_data.order2_coefficients.dot(&ord2);
            }

            let BinaryNode {
                left,
                right,
                node_type,
                ..
            } = self.tree.nodes[root];

            match node_type {
                NodeType::Leaf => {
                    let tris = self.tree.objects[left..right].iter().map(|(o, _)| o);
                    winding_number(tris, point)
                }
                NodeType::Branch => {
                    let left_wn = self.fast_wn(left, point, accuracy_scale);
                    let right_wn = self.fast_wn(right, point, accuracy_scale);

                    left_wn + right_wn
                }
            }
        }
    }

    struct InitData {
        area_weighted_normal: Vec3f,
        area_weighted_center: Vec3f,
        order1_sum: Mat3f,
        total_area: f32,
        dipole_center: Vec3f,
    }

    #[derive(Debug, Default, Clone, Copy)]
    struct NodeData {
        order1_coefficients: Vec3f,
        order2_coefficients: Mat3f,
        // order3_coefficients: Vec3f,
        radius: f32,
        dipole_center: Vec3f,
    }

    fn compute_tree_coeffs(tree: &mut AABBTree<Triangle3<f32>>) -> Vec<NodeData> {
        if tree.nodes.is_empty() {
            return vec![];
        }

        let mut data = Vec::with_capacity(tree.nodes.len());
        data.resize(tree.nodes.len(), NodeData::default());
        compute_node_data(tree, tree.nodes.len() - 1, &mut data);

        data
    }

    fn compute_node_data(
        tree: &AABBTree<Triangle3<f32>>,
        idx: usize,
        data: &mut Vec<NodeData>,
    ) -> InitData {
        let node = &tree.nodes[idx];
        let node_data = match node.node_type {
            NodeType::Leaf => leaf_data(tree, node),
            NodeType::Branch => branch_data(tree, node, data),
        };

        let dist_to_min_sq = (node.bbox.get_min() - node_data.dipole_center).norm_squared();
        let dist_to_max_sq = (node.bbox.get_max() - node_data.dipole_center).norm_squared();
        let radius = dist_to_min_sq.max(dist_to_max_sq).sqrt();

        data[idx] = NodeData {
            radius,
            order1_coefficients: node_data.area_weighted_normal,
            order2_coefficients: node_data.order1_sum
                - node_data.dipole_center * node_data.area_weighted_normal.transpose(),
            dipole_center: node_data.dipole_center,
        };

        node_data
    }

    fn leaf_data(tree: &AABBTree<Triangle3<f32>>, node: &BinaryNode<f32>) -> InitData {
        let mut area_weighted_normal = Vec3f::zeros();
        let mut area_weighted_center = Vec3f::zeros();
        let mut order1_sum = Mat3f::zeros();
        let mut total_area = 0.0f32;

        for t in node.left..node.right {
            let (tri, _) = &tree.objects[t];
            let n = match tri.try_get_normal() {
                Some(n) => n,
                None => continue, // Skip degenerate triangles
            };
            let area = tri.get_area();

            total_area += area;
            area_weighted_normal += area * n;

            let c = tri.center();
            order1_sum += area * c * n.transpose();
            area_weighted_center += area * c;
        }

        InitData {
            area_weighted_normal,
            area_weighted_center,
            total_area,
            order1_sum,
            dipole_center: area_weighted_center / total_area,
        }
    }

    fn branch_data(
        tree: &AABBTree<Triangle3<f32>>,
        node: &BinaryNode<f32>,
        data: &mut Vec<NodeData>,
    ) -> InitData {
        let left_data = compute_node_data(tree, node.left, data);
        let right_data = compute_node_data(tree, node.right, data);

        let order1_sum = left_data.order1_sum + right_data.order1_sum;
        let area_weighted_normal = left_data.area_weighted_normal + right_data.area_weighted_normal;
        let area_weighted_center = left_data.area_weighted_center + right_data.area_weighted_center;
        let total_area = left_data.total_area + right_data.total_area;
        let dipole_center =
            (left_data.area_weighted_center + right_data.area_weighted_center) / total_area;

        InitData {
            area_weighted_normal,
            area_weighted_center,
            dipole_center,
            total_area,
            order1_sum,
        }
    }

    fn hessians(dipole: &Vec3f, query_point: &Vec3f) -> (Vec3f, Mat3f) {
        let r = dipole - query_point;
        let r2 = r.norm_squared();
        let r1 = r2.sqrt();
        let r3 = r2 * r1;
        let ord1_den = 4.0 * PI * r3;
        let ord1_den_inv = 1.0 / ord1_den;
        let ord1 = r * ord1_den_inv;

        let r5 = r3 * r2;
        let ord2 = Mat3f::identity() * ord1_den_inv - 3.0 * r * r.transpose() / (4.0 * PI * r5);

        (ord1, ord2)
    }
}
