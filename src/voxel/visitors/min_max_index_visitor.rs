use crate::{
    helpers::aliases::Vec3i,
    voxel::{Tile, TreeNode, Visitor},
};
use std::marker::PhantomData;

struct MinMaxIdx<T: TreeNode<Value = f32>> {
    min: Vec3i,
    max: Vec3i,
    _tree: PhantomData<T>,
}

impl<T: TreeNode<Value = f32>> MinMaxIdx<T> {
    fn new() -> Self {
        Self {
            min: Vec3i::new(isize::MAX, isize::MAX, isize::MAX),
            max: Vec3i::new(isize::MIN, isize::MIN, isize::MIN),
            _tree: PhantomData,
        }
    }
}

impl<T: TreeNode<Value = f32>> Visitor<T::Leaf> for MinMaxIdx<T> {
    fn tile(&mut self, tile: Tile<<T::Leaf as TreeNode>::Value>) {
        let size = tile.size as isize;
        let current_max = tile.origin + Vec3i::new(size, size, size);
        self.min = nalgebra_glm::min2(&self.min, &tile.origin);
        self.max = nalgebra_glm::max2(&self.max, &current_max);
    }

    fn dense(&mut self, dense: &T::Leaf) {
        let size = T::Leaf::resolution() as isize;
        let current_max = dense.origin() + Vec3i::new(size, size, size);
        self.min = nalgebra_glm::min2(&self.min, &dense.origin());
        self.max = nalgebra_glm::max2(&self.max, &current_max);
    }
}
