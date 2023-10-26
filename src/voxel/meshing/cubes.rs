use std::collections::HashMap;

use nalgebra::{Vector3, Point3};

use crate::{mesh::traits::Mesh, geometry::primitives::box3::Box3, algo::utils::cast, voxel::{TreeNode, Grid, Leaf}};

pub struct CubesMeshing<'a, T: Grid> {
    grid: &'a T,
    vertices: Vec<Vector3<isize>>,
    indices: Vec<usize>,
    index_vertex_map: HashMap<Vector3<isize>, usize>,
    v: [Vector3<isize>; 8],
}

impl<'a, T: Grid> CubesMeshing<'a, T> {
    pub fn new(grid: &'a T) -> Self {
        let bbox = Box3::new(
            Point3::new(0, 0, 0), 
            Point3::new(1, 1, 1)
        );

        let v_indices = [
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
            v: v_indices,
            vertices: Vec::new(),
            indices: Vec::new(),
            index_vertex_map: HashMap::new(),
        }
    }

    pub fn mesh<TMesh: Mesh>(&mut self) -> TMesh {
        self.reset();

        for leaf in self.grid.leafs() {
            match leaf {
                Leaf::Tile(tile) => {
                    // Test only boundary voxels
                    for i in 0..tile.size {
                        for j in 0..tile.size {
                            let left  = tile.origin + Vector3::new(0, i, j).cast();
                            let right = tile.origin + Vector3::new(tile.size - 1, i, j).cast();
                            
                            let top    = tile.origin + Vector3::new(i, j, tile.size - 1).cast();
                            let bottom = tile.origin + Vector3::new(i, j, 0).cast();
                            
                            let front = tile.origin + Vector3::new(i, tile.size - 1, j).cast();
                            let back  = tile.origin + Vector3::new(i, 0, j).cast();

                            self.test_voxel(left);
                            self.test_voxel(right);
                            self.test_voxel(top);
                            self.test_voxel(bottom);
                            self.test_voxel(front);
                            self.test_voxel(back);
                        }
                    }
                },
                Leaf::Node(node) => {
                    let size = T::LeafNode::resolution();
                    let origin = node.origin();

                    // Test all voxels in the node
                    for x in 0..size {
                        for y in 0..size {
                            for z in 0..size {
                                let voxel = origin + Vector3::new(x, y, z).cast();
                                
                                todo!();
                                if self.grid.at(&voxel).is_some() {
                                    continue;
                                }

                                self.test_voxel(voxel);
                            }
                        }
                    }
                },
            }
        }

        let vertices: Vec<_> = self.vertices.iter().map(|v| cast(&v).into()).collect();
        let mesh = TMesh::from_vertices_and_indices(vertices.as_slice(), &self.indices);

        mesh
    }

    fn test_voxel(&mut self, voxel: Vector3<isize>) {
        let top_index     = voxel + Vector3::new(0, 0, 1);
        let bottom_index  = voxel + Vector3::new(0, 0, -1);
        let left_index    = voxel + Vector3::new(-1, 0, 0);
        let right_index   = voxel + Vector3::new(1, 0, 0);
        let front_index   = voxel + Vector3::new(0, 1, 0);
        let back_index    = voxel + Vector3::new(0, -1, 0);

        todo!();

        let top     = self.grid.at(&top_index).is_some();
        let bottom  = self.grid.at(&bottom_index).is_some();
        let left    = self.grid.at(&left_index).is_some();
        let right   = self.grid.at(&right_index).is_some();
        let front   = self.grid.at(&front_index).is_some();
        let back    = self.grid.at(&back_index).is_some();

        if !top {
            let faces = [
                voxel + self.v[4],
                voxel + self.v[7],
                voxel + self.v[6],

                voxel + self.v[4],
                voxel + self.v[5],
                voxel + self.v[7],
            ];

            self.add_faces(&faces);
        }

        if !bottom {
            let faces = [
                voxel + self.v[0],
                voxel + self.v[2],
                voxel + self.v[3],

                voxel + self.v[0],
                voxel + self.v[3],
                voxel + self.v[1],
            ];

            self.add_faces(&faces);
        }

        if !left {
            let faces = [
                voxel + self.v[0],
                voxel + self.v[4],
                voxel + self.v[6],

                voxel + self.v[0],
                voxel + self.v[6],
                voxel + self.v[2],
            ];

            self.add_faces(&faces);
        }

        if !right {
            let faces = [
                voxel + self.v[1],
                voxel + self.v[7],
                voxel + self.v[5],

                voxel + self.v[1],
                voxel + self.v[3],
                voxel + self.v[7],
            ];

            self.add_faces(&faces);
        }

        if !front {
            let faces = [
                voxel + self.v[2],
                voxel + self.v[6],
                voxel + self.v[7],

                voxel + self.v[2],
                voxel + self.v[7],
                voxel + self.v[3],
            ];

            self.add_faces(&faces);
        }

        if !back {
            let faces = [
                voxel + self.v[0],
                voxel + self.v[5],
                voxel + self.v[4],

                voxel + self.v[0],
                voxel + self.v[1],
                voxel + self.v[5],
            ];

            self.add_faces(&faces);
        }
    }

    fn add_faces(&mut self, faces: &[Vector3<isize>]) {
        for vertex in faces {
            let idx = self.get_or_insert_vertex(*vertex);
            self.indices.push(idx);
        }
    }

    fn get_or_insert_vertex(&mut self, vertex: Vector3<isize>) -> usize {
        if let Some(idx) = self.index_vertex_map.get(&vertex) {
            *idx
        } else {
            let idx = self.vertices.len();
            self.vertices.push(vertex);
            self.index_vertex_map.insert(vertex, idx);

            idx
        }
    }

    fn reset(&mut self) {
        self.indices.clear();
        self.vertices.clear();
        self.index_vertex_map.clear();
    }
}
