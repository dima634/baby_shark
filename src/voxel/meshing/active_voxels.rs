use crate::voxel::{utils::CUBE_OFFSETS, Tile, TreeNode, Visitor};
use nalgebra::Vector3;

pub struct ActiveVoxelsMesher {
    vertices: Vec<Vector3<isize>>,
    box_vertices: [Vector3<isize>; 8],
}

impl ActiveVoxelsMesher {
    /// Returns a list where each tree consecutive vertices form a triangle
    #[allow(dead_code)]
    pub(super) fn mesh(&mut self, grid: &impl TreeNode) -> Vec<Vector3<isize>> {
        self.vertices.clear();

        let mut visitor = ActiveVoxelsVisitor { grid, mesher: self };

        grid.visit_leafs(&mut visitor);
        std::mem::take(&mut self.vertices)
    }

    fn test_voxel(&mut self, voxel: Vector3<isize>, grid: &impl TreeNode) {
        if grid.at(&voxel).is_none() {
            return;
        }

        let top_index = voxel + Vector3::new(0, 0, 1);
        let bottom_index = voxel + Vector3::new(0, 0, -1);
        let left_index = voxel + Vector3::new(-1, 0, 0);
        let right_index = voxel + Vector3::new(1, 0, 0);
        let front_index = voxel + Vector3::new(0, 1, 0);
        let back_index = voxel + Vector3::new(0, -1, 0);

        let top = grid.at(&top_index).is_some();
        let bottom = grid.at(&bottom_index).is_some();
        let left = grid.at(&left_index).is_some();
        let right = grid.at(&right_index).is_some();
        let front = grid.at(&front_index).is_some();
        let back = grid.at(&back_index).is_some();

        if !top {
            let faces = [
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[6],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !bottom {
            let faces = [
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[2],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !left {
            let faces = [
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[3],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !right {
            let faces = [
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[2],
                voxel + self.box_vertices[6],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !front {
            let faces = [
                voxel + self.box_vertices[2],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[7],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !back {
            let faces = [
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[0],
            ];

            self.vertices.extend_from_slice(&faces);
        }
    }
}

impl Default for ActiveVoxelsMesher {
    #[inline]
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            box_vertices: CUBE_OFFSETS,
        }
    }
}

struct ActiveVoxelsVisitor<'a, T: TreeNode> {
    grid: &'a T,
    mesher: &'a mut ActiveVoxelsMesher,
}

impl<T: TreeNode> Visitor<T::Leaf> for ActiveVoxelsVisitor<'_, T> {
    fn tile(&mut self, tile: Tile<<T>::Value>) {
        // Test only boundary voxels
        for i in 0..tile.size {
            for j in 0..tile.size {
                let left = tile.origin + Vector3::new(0, i, j).cast();
                let right = tile.origin + Vector3::new(tile.size - 1, i, j).cast();

                let top = tile.origin + Vector3::new(i, j, tile.size - 1).cast();
                let bottom = tile.origin + Vector3::new(i, j, 0).cast();

                let front = tile.origin + Vector3::new(i, tile.size - 1, j).cast();
                let back = tile.origin + Vector3::new(i, 0, j).cast();

                self.mesher.test_voxel(left, self.grid);
                self.mesher.test_voxel(right, self.grid);
                self.mesher.test_voxel(top, self.grid);
                self.mesher.test_voxel(bottom, self.grid);
                self.mesher.test_voxel(front, self.grid);
                self.mesher.test_voxel(back, self.grid);
            }
        }
    }

    fn dense(&mut self, dense: &T::Leaf) {
        let size = T::Leaf::resolution();
        let origin = dense.origin();

        // Test all voxels in the node
        for x in 0..size {
            for y in 0..size {
                for z in 0..size {
                    let voxel = origin + Vector3::new(x, y, z).cast();

                    self.mesher.test_voxel(voxel, self.grid);
                }
            }
        }
    }
}
