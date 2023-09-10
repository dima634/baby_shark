use std::collections::HashMap;

use bitvec::index;
use nalgebra::Vector3;

use crate::mesh::traits::Mesh;

use super::{Accessor, TreeNode};

pub fn to_mesh<TMesh: Mesh, TGrid: TreeNode>(grid: &TGrid) -> TMesh {
    let mut vertices: Vec<isize> = Vec::new();
    let mut indices: Vec<usize> = Vec::new();
    let mut index_vertex_map = HashMap::<Vector3<isize>, usize>::new();

    grid.voxels(|voxel| {
        let top = voxel + Vector3::new(0, 1, 0);
        let bottom = voxel + Vector3::new(0, -1, 0);
        let left = voxel + Vector3::new(-1, 0, 0);
        let right = voxel + Vector3::new(1, 0, 0);
        let front = voxel + Vector3::new(0, 0, 1);
        let back = voxel + Vector3::new(0, 0, -1);
    });

    todo!()
}
