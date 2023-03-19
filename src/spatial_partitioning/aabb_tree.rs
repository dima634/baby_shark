use nalgebra::{Vector3, Point3};
use num_traits::{Float, One};

use crate::{
    geometry::{
        traits::{
            HasBBox3, 
            ClosestPoint3, 
            RealNumber
        }, 
        primitives::{
            box3::Box3, 
            triangle3::Triangle3, 
            plane3::Plane3
        }
    }, 
    mesh::traits::Mesh
};

#[derive(PartialEq)]
enum NodeType {
    Leaf,
    Branch
}

struct BinaryNode<TScalar: RealNumber> {
    node_type: NodeType,
    left: usize,
    right: usize,
    bbox: Box3<TScalar>
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
pub struct AABBTree<TObject>
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber
{
    nodes: Vec<BinaryNode<TObject::ScalarType>>,
    objects: Vec<(TObject, Box3<TObject::ScalarType>)>,
    min_objects_per_leaf: usize,
    max_depth: usize
}

impl<TObject> AABBTree<TObject>
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber
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
            objects: objects.into_iter()
                .map(|obj| { 
                    let bbox = obj.bbox(); 
                    return (obj, bbox) 
                })
                .collect()
        };
    }

    pub fn empty() -> Self {
        return Self { 
            nodes: Vec::new(),
            min_objects_per_leaf: 10,
            max_depth: 40,
            objects: Vec::new()
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

    /// 
    /// Constructs AABB tree using top-down building strategy.
    /// This strategy is fast but not always produce best tree.
    /// 
    /// ## Generic arguments
    /// * `TPartition` - partitioning strategy used to split two sets of objects into subnodes (see [MedianCut])
    /// 
    pub fn top_down<TPartition: PartitionStrategy<TObject>>(mut self) -> Self {
        self.nodes.clear();
        self.top_down_build_node(0, self.objects.len() - 1, 1, &mut TPartition::default());
        return self;
    }

    /// Traverse leaf node of tree
    #[inline]
    pub fn traverse<TFunc>(&self, visit: &mut TFunc) 
    where
        TFunc: FnMut((&[(TObject, Box3<TObject::ScalarType>)], &Box3<TObject::ScalarType>))
    {
        self.visit_node(self.nodes.len() - 1, visit);
    }

    /// Recursively visit tree node
    fn visit_node<TFunc>(&self, node_index: usize, visit: &mut TFunc) 
    where
        TFunc: FnMut((&[(TObject, Box3<TObject::ScalarType>)], &Box3<TObject::ScalarType>))
    {
        let node = &self.nodes[node_index];

        match node.node_type {
            NodeType::Leaf => {
                let objects = &self.objects[node.left..node.right];
                visit((objects, &node.bbox));
            },
            NodeType::Branch => {
                self.visit_node(node.left, visit);
                self.visit_node(node.right, visit);
            },
        }
    }

    /// Build tree node (leaf or branch) from set of objects
    fn top_down_build_node<TPartition: PartitionStrategy<TObject>>(&mut self, first: usize, last: usize, depth: usize, partition_strategy: &mut TPartition) -> usize {
        if depth >= self.max_depth || last - first <= self.min_objects_per_leaf {
            // Create leaf node when number of objects is small
            return self.leaf_node_from_objects(first, last);
        } else {
            // Split set of objects
            let split_at_result = partition_strategy.split(&mut self.objects, first, last);

            match split_at_result {
                Ok(split_at) => {
                    // Create branch node if split succeeded
                    let left = self.top_down_build_node(first, split_at - 1, depth + 1, partition_strategy);
                    let right = self.top_down_build_node(split_at, last, depth + 1, partition_strategy);
        
                    let mut bbox = self.nodes[left].bbox;
                    bbox.add_box3(&self.nodes[right].bbox);
        
                    let node = BinaryNode {
                        bbox,
                        node_type: NodeType::Branch,
                        left,
                        right
                    };
                    
                    self.nodes.push(node);
                    
                    return self.nodes.len() - 1;
                },
                Err(_) => {
                    // Create leaf node if split failed
                    return self.leaf_node_from_objects(first, last);
                },
            }
        }
    }

    /// Create leaf node from set of objects
    fn leaf_node_from_objects(&mut self, first: usize, last: usize) -> usize {
        // Compute bounding box of set of objects
        let (_, mut bbox) = self.objects[first];
        for i in first + 1..last {
            bbox.add_box3(&self.objects[i].1);
        }

        let node = BinaryNode {
            bbox,
            node_type: NodeType::Leaf,
            left: first,
            right: last
        };

        self.nodes.push(node);

        return self.nodes.len() - 1;
    }
}

impl<TScalar: RealNumber> AABBTree<Triangle3<TScalar>>{
    /// 
    /// Create new AABB tree from faces of triangular mesh. This method is not finishing construction of tree.
    /// To finish tree construction it should be chained with call of construction strategy ([top_down](AABBTree) etc)
    /// 
    pub fn from_mesh<TMesh: Mesh<ScalarType = TScalar>>(mesh: &TMesh) -> Self {
        let faces: Vec<Triangle3<TScalar>> = mesh.faces()
            .map(|face| mesh.face_positions(&face))
            .collect();

        return Self::new(faces);
    }
}

impl<TObject> AABBTree<TObject>
where
    TObject: HasBBox3 + ClosestPoint3,
    TObject::ScalarType: RealNumber 
{
    pub fn closest_point(&self, point: &Point3<TObject::ScalarType>, max_distance: TObject::ScalarType) -> Option<Point3<TObject::ScalarType>> {
        let max_distance_square = max_distance * max_distance;

        let mut stack = Vec::with_capacity(self.max_depth);
        stack.push(self.nodes.last().unwrap());

        let mut closest_point = Point3::origin();
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

                if left.bbox.contains_point(point) || left.bbox.squared_distance(point) < max_distance_square {
                    stack.push(left);
                }

                if right.bbox.contains_point(point) || right.bbox.squared_distance(point) < max_distance_square {
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
    fn split(&mut self, objects: &mut Vec<(TObject, Box3<TObject::ScalarType>)>, first: usize, last: usize) -> Result<usize, &'static str>;
}

///
/// A simple partitioning method - median-cut algorithm. Here, the set is divided in
/// two equal-size parts with respect to their projection along the selected axis, resulting
/// in a balanced tree.
/// 
#[derive(Default)]
pub struct MedianCut {}

impl MedianCut {
    fn try_split_by_axis<TObject>(axis: usize, objects: &mut [(TObject, Box3<TObject::ScalarType>)], first: usize, last: usize) -> Result<usize, &'static str> 
    where
        TObject: HasBBox3,
        TObject::ScalarType: RealNumber
    {
        // Sort along split axis
        objects[first..last].sort_by(|(_, bbox1), (_, bbox2)| bbox1.get_center()[axis].partial_cmp(&bbox2.get_center()[axis]).unwrap());

        let mut split_axis = Vector3::<TObject::ScalarType>::zeros();
        split_axis[axis] = One::one();
        let (_, split_bbox) = &objects[(first + last) / 2];
        let split_point = split_bbox.get_center();
        let plane = Plane3::new(split_axis, split_point[axis]);

        // Test whether all objects intersects plane
        let all_object_intersects_plane = objects[first..last].iter().all(|(_, bbox)| bbox.intersects_plane3(&plane));

        if all_object_intersects_plane {
            return Err("All objects are intersecting split plane");
        }

        // Test whether all objects lies on same side of plane
        let first_sign: TObject::ScalarType = plane.distance(&objects[first].1.get_center());
        let mut are_on_same_side = true;

        for (_, bbox) in objects.iter().take(last).skip(first + 1) {
            let sign = plane.distance(&bbox.get_center()).signum();

            // On different sides?
            if first_sign != sign {
                are_on_same_side = false;
                break;
            }
        }

        if are_on_same_side {
            return Err("All objects are on same side of plane");
        }

        // Return medial point
        return Ok((first + last) / 2);
    }
}

impl<TObject> PartitionStrategy<TObject> for MedianCut     
where
    TObject: HasBBox3,
    TObject::ScalarType: RealNumber
{
    fn split(&mut self, objects: &mut Vec<(TObject, Box3<TObject::ScalarType>)>, first: usize, last: usize) -> Result<usize, &'static str> {
        // Split by biggest dimension first
        let mut bbox = objects[first].1;

        objects.iter()
            .take(last + 1)
            .skip(first + 1)
            .for_each(|(_, obj_bbox)| { 
                bbox.add_box3(obj_bbox); 
            });

        let mut split_axises = vec![
            (bbox.size_x(), 0),
            (bbox.size_y(), 1),
            (bbox.size_z(), 2)
        ];

        // Sort by bbox size along split axis
        split_axises.sort_by(|(size1, _), (size2, _)| size2.partial_cmp(size1).unwrap());

        // Try to split by X/Y/Z until one succeeded
        return Self::try_split_by_axis(split_axises[0].1, objects, first, last)
            .or_else(|_| Self::try_split_by_axis(split_axises[1].1, objects, first, last))
            .or_else(|_| Self::try_split_by_axis(split_axises[2].1, objects, first, last));
    }
}
