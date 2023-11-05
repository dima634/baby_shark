use nalgebra::{Point3, Vector3};

use crate::{
    geometry::primitives::box3::Box3,
    voxel::{Grid, Leaf, TreeNode},
};

pub struct CubesMesher<'a, T: Grid> {
    grid: &'a T,
    vertices: Vec<Vector3<isize>>,
    box_vertices: [Vector3<isize>; 8],
}

impl<'a, T: Grid> CubesMesher<'a, T> {
    pub fn new(grid: &'a T) -> Self {
        let bbox = Box3::new(Point3::new(0, 0, 0), Point3::new(1, 1, 1));

        let box_vertices = [
            bbox.vertex(0).coords,
            bbox.vertex(1).coords,
            bbox.vertex(2).coords,
            bbox.vertex(3).coords,
            bbox.vertex(4).coords,
            bbox.vertex(5).coords,
            bbox.vertex(6).coords,
            bbox.vertex(7).coords,
        ];

        Self {
            grid,
            box_vertices,
            vertices: Vec::new(),
        }
    }

    ///
    /// Returns a list where each tree consecutive vertices form a triangle.
    /// 
    pub fn mesh(&mut self) -> Vec<Vector3<isize>> {
        self.vertices.clear();

        for leaf in self.grid.leafs() {
            match leaf {
                Leaf::Tile(tile) => {
                    // Test only boundary voxels
                    for i in 0..tile.size {
                        for j in 0..tile.size {
                            let left = tile.origin + Vector3::new(0, i, j).cast();
                            let right = tile.origin + Vector3::new(tile.size - 1, i, j).cast();

                            let top = tile.origin + Vector3::new(i, j, tile.size - 1).cast();
                            let bottom = tile.origin + Vector3::new(i, j, 0).cast();

                            let front = tile.origin + Vector3::new(i, tile.size - 1, j).cast();
                            let back = tile.origin + Vector3::new(i, 0, j).cast();

                            self.test_voxel(left);
                            self.test_voxel(right);
                            self.test_voxel(top);
                            self.test_voxel(bottom);
                            self.test_voxel(front);
                            self.test_voxel(back);
                        }
                    }
                }
                Leaf::Dense(node) => {
                    let size = T::LeafNode::resolution();
                    let origin = node.origin();

                    // Test all voxels in the node
                    for x in 0..size {
                        for y in 0..size {
                            for z in 0..size {
                                let voxel = origin + Vector3::new(x, y, z).cast();

                                self.test_voxel(voxel);
                            }
                        }
                    }
                }
            }
        }

        self.vertices.clone()
    }

    fn test_voxel(&mut self, voxel: Vector3<isize>) {
        if self.grid.at(&voxel).is_none() {
            return;
        }

        let top_index = voxel + Vector3::new(0, 0, 1);
        let bottom_index = voxel + Vector3::new(0, 0, -1);
        let left_index = voxel + Vector3::new(-1, 0, 0);
        let right_index = voxel + Vector3::new(1, 0, 0);
        let front_index = voxel + Vector3::new(0, 1, 0);
        let back_index = voxel + Vector3::new(0, -1, 0);

        let top = self.grid.at(&top_index).is_some();
        let bottom = self.grid.at(&bottom_index).is_some();
        let left = self.grid.at(&left_index).is_some();
        let right = self.grid.at(&right_index).is_some();
        let front = self.grid.at(&front_index).is_some();
        let back = self.grid.at(&back_index).is_some();

        if !top {
            let faces = [
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[7],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !bottom {
            let faces = [
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[2],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[1],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !left {
            let faces = [
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[2],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !right {
            let faces = [
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[3],
                voxel + self.box_vertices[7],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !front {
            let faces = [
                voxel + self.box_vertices[2],
                voxel + self.box_vertices[6],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[2],
                voxel + self.box_vertices[7],
                voxel + self.box_vertices[3],
            ];

            self.vertices.extend_from_slice(&faces);
        }

        if !back {
            let faces = [
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[5],
                voxel + self.box_vertices[4],
                voxel + self.box_vertices[0],
                voxel + self.box_vertices[1],
                voxel + self.box_vertices[5],
            ];

            self.vertices.extend_from_slice(&faces);
        }
    }
}
