use nalgebra::Vector3;

pub trait TreeNode {
    /// Number of tiles in one dimension on current level
    const BRANCHING: usize;
    /// Total number of tiles in one dimension
    const BRANCHING_TOTAL: usize;
    /// Total number of voxels in node
    const SIZE: usize;

    fn at(&self, index: Vector3<usize>) -> bool;
    fn insert(&mut self, index: Vector3<usize>);
    fn remove(&mut self, index: Vector3<usize>);
    fn is_empty(&self) -> bool;
}

pub trait HasChild {
    type Child: TreeNode;
}
