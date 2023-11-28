use std::mem::MaybeUninit;

use nalgebra::Vector3;

use crate::voxel::{Grid, Leaf, Tile, TreeNode, Accessor, Scalar, sdf::Sdf};

use self::lookup_table::{Cube, LOOKUP_TABLE, Edge};

pub struct Vertex<T> {
    pub index: Vector3<isize>,
    pub value: T,
}

pub trait MarchingCubes {
    type Value;

    fn interpolate(&self, v1: &Vertex<Self::Value>, v2: &Vertex<Self::Value>) -> Vector3<f32>;
    fn is_inside(&self, idx: &Self::Value) -> bool;
    fn cubes<T: FnMut(Vector3<isize>)> (&self, func: T);
    fn at(&self, index: &Vector3<isize>) -> Option<Self::Value>;
}

// impl<T: Grid<Value = ()>> MarchingCubes for T {
//     #[inline]
//     fn interpolate(&self, v1: Vertex<Self::Value>, v2: Vertex<Self::Value>) -> Vector3<f32> {
//         (v1.index + v2.index).cast() / 2.0
//     }
// }

pub struct MarchingCubesMesher<'a, T: MarchingCubes> {
    grid: &'a T,
    vertices: Vec<Vector3<f32>>,
}

impl<'a, T: MarchingCubes> MarchingCubesMesher<'a, T> {
    pub fn new(grid: &'a T) -> Self {
        Self { 
            grid,
            vertices: Vec::new(),
        }
    }

    pub fn mesh(&mut self) -> Vec<Vector3<f32>> {
        self.grid.cubes(|i| self.handle_cube(i));
        self.vertices.clone()
    }

    fn handle_cube(&mut self, v: Vector3<isize>) {
    
        let mut cube = MCube::from_voxel(v, self.grid);

        if cube.is_none() {
            return;
        }

        let cube = cube.unwrap();
        let triangles = LOOKUP_TABLE[(cube.id()) as usize];

        for i in (0..triangles.len()).step_by(3) {
            let e1 = triangles[i];
            let e2 = triangles[i + 1];
            let e3 = triangles[i + 2];

            let v1 = self.interpolate(e1, &cube);
            let v2 = self.interpolate(e2, &cube);
            let v3 = self.interpolate(e3, &cube);
    
            self.vertices.push(v1);
            self.vertices.push(v2);
            self.vertices.push(v3);
        }
    }
    
    fn interpolate(&mut self, e: Edge, cube: &MCube<T::Value>) -> Vector3<f32> {
        let v1 = cube.vertex(e.v1 as usize);
        let v2 = cube.vertex(e.v2 as usize);
        let mid = self.grid.interpolate(v1, v2);

        mid
    }
}

struct MCube<T> {
    bits: u8,
    vertices: [Vertex<T>; 8],
}

impl<T> MCube<T> {
    fn from_voxel<TGrid: MarchingCubes<Value = T>>(voxel: Vector3<isize>, grid: &TGrid) -> Option<Self> {
        let mut vertices: [Vertex<T>; 8] = unsafe { MaybeUninit::uninit().assume_init() }; 
        let mut bits = 0;

        let vertex_indices = [
            voxel,
            voxel + Vector3::new(1, 0, 0),
            voxel + Vector3::new(1, 1, 0),
            voxel + Vector3::new(0, 1, 0),
            voxel + Vector3::new(0, 0, 1),
            voxel + Vector3::new(1, 0, 1),
            voxel + Vector3::new(1, 1, 1),
            voxel + Vector3::new(0, 1, 1),
        ];

        for i in 0..vertex_indices.len() {
            let index = vertex_indices[i];
            let value = grid.at(&index);

            if value.is_none() {
                return None;
            }

            let value = value.unwrap();
            let inside = grid.is_inside(&value);
            
            if inside {
                bits |= 1 << i;
            }

            vertices[i] = Vertex { 
                index, 
                value,
            };
        }

        Some(Self {
            bits,
            vertices,
        })
    }

    #[inline]
    fn id(&self) -> u8 {
        self.bits
    }

    #[inline]
    fn vertex(&self, vertex: usize) -> &Vertex<T> {
        &self.vertices[vertex]
    }
}

mod lookup_table {
    use std::{ops::Not, collections::BTreeMap};

    #[derive(Debug, Clone, Copy)]
    pub struct Edge {
        pub v1: u8,
        pub v2: u8,
    }

    pub const LOOKUP_TABLE: [&[Edge]; 256] = [
        [].as_slice(),
        [EP4, EP9, EP1].as_slice(),
        [EP1, EP10, EP2].as_slice(),
        [EP2, EP9, EP10, EP2, EP4, EP9].as_slice(),
        [EP2, EP12, EP3].as_slice(),
        [EP9, EP1, EP4, EP2, EP12, EP3].as_slice(),
        [EP3, EP10, EP12, EP3, EP1, EP10].as_slice(),
        [EP3, EP4, EP12, EP4, EP9, EP12, EP9, EP10, EP12].as_slice(),
        [EP3, EP11, EP4].as_slice(),
        [EP1, EP11, EP9, EP1, EP3, EP11].as_slice(),
        [EP11, EP4, EP3, EP1, EP10, EP2].as_slice(),
        [EP2, EP3, EP10, EP3, EP11, EP10, EP11, EP9, EP10].as_slice(),
        [EP4, EP12, EP11, EP4, EP2, EP12].as_slice(),
        [EP1, EP2, EP9, EP2, EP12, EP9, EP12, EP11, EP9].as_slice(),
        [EP4, EP1, EP11, EP1, EP10, EP11, EP10, EP12, EP11].as_slice(),
        [EP12, EP11, EP9, EP12, EP9, EP10].as_slice(),
        [EP8, EP5, EP9].as_slice(),
        [EP4, EP5, EP1, EP4, EP8, EP5].as_slice(),
        [EP8, EP5, EP9, EP10, EP2, EP1].as_slice(),
        [EP5, EP10, EP8, EP10, EP2, EP8, EP2, EP4, EP8].as_slice(),
        [EP2, EP12, EP3, EP8, EP5, EP9].as_slice(),
        [EP5, EP4, EP8, EP5, EP1, EP4, EP3, EP2, EP12].as_slice(),
        [EP3, EP10, EP12, EP3, EP1, EP10, EP5, EP9, EP8].as_slice(),
        [EP3, EP4, EP8, EP3, EP10, EP12, EP3, EP8, EP10, EP10, EP8, EP5].as_slice(),
        [EP3, EP11, EP4, EP8, EP5, EP9].as_slice(),
        [EP11, EP8, EP3, EP8, EP5, EP3, EP5, EP1, EP3].as_slice(),
        [EP11, EP4, EP3, EP8, EP5, EP9, EP1, EP10, EP2].as_slice(),
        [EP11, EP2, EP3, EP10, EP2, EP11, EP10, EP11, EP8, EP10, EP8, EP5].as_slice(),
        [EP12, EP4, EP2, EP12, EP11, EP4, EP9, EP8, EP5].as_slice(),
        [EP11, EP2, EP12, EP2, EP11, EP5, EP11, EP8, EP5, EP2, EP5, EP1].as_slice(),
        [EP9, EP8, EP5, EP11, EP10, EP12, EP11, EP1, EP10, EP4, EP1, EP11].as_slice(),
        [EP5, EP10, EP8, EP8, EP10, EP11, EP11, EP10, EP12].as_slice(),
        [EP5, EP6, EP10].as_slice(),
        [EP4, EP9, EP1, EP5, EP6, EP10].as_slice(),
        [EP1, EP6, EP2, EP1, EP5, EP6].as_slice(),
        [EP9, EP5, EP4, EP5, EP6, EP4, EP6, EP2, EP4].as_slice(),
        [EP5, EP6, EP10, EP12, EP3, EP2].as_slice(),
        [EP9, EP1, EP4, EP5, EP6, EP10, EP2, EP12, EP3].as_slice(),
        [EP6, EP12, EP5, EP12, EP3, EP5, EP3, EP1, EP5].as_slice(),
        [EP9, EP3, EP4, EP12, EP3, EP9, EP12, EP9, EP5, EP12, EP5, EP6].as_slice(),
        [EP3, EP11, EP4, EP5, EP6, EP10].as_slice(),
        [EP11, EP1, EP3, EP11, EP9, EP1, EP10, EP5, EP6].as_slice(),
        [EP6, EP1, EP5, EP6, EP2, EP1, EP4, EP3, EP11].as_slice(),
        [EP9, EP3, EP11, EP3, EP9, EP6, EP9, EP5, EP6, EP3, EP6, EP2].as_slice(),
        [EP4, EP12, EP11, EP4, EP2, EP12, EP6, EP10, EP5].as_slice(),
        [EP10, EP5, EP6, EP9, EP12, EP11, EP9, EP2, EP12, EP1, EP2, EP9].as_slice(),
        [EP4, EP1, EP5, EP4, EP12, EP11, EP4, EP5, EP12, EP12, EP5, EP6].as_slice(),
        [EP6, EP12, EP5, EP5, EP12, EP9, EP9, EP12, EP11].as_slice(),
        [EP8, EP10, EP9, EP8, EP6, EP10].as_slice(),
        [EP10, EP1, EP6, EP1, EP4, EP6, EP4, EP8, EP6].as_slice(),
        [EP1, EP9, EP2, EP9, EP8, EP2, EP8, EP6, EP2].as_slice(),
        [EP8, EP6, EP2, EP8, EP2, EP4].as_slice(),
        [EP8, EP10, EP9, EP8, EP6, EP10, EP2, EP12, EP3].as_slice(),
        [EP2, EP12, EP3, EP6, EP4, EP8, EP6, EP1, EP4, EP10, EP1, EP6].as_slice(),
        [EP1, EP12, EP3, EP12, EP1, EP8, EP1, EP9, EP8, EP12, EP8, EP6].as_slice(),
        [EP3, EP4, EP12, EP12, EP4, EP6, EP6, EP4, EP8].as_slice(),
        [EP10, EP8, EP6, EP10, EP9, EP8, EP11, EP4, EP3].as_slice(),
        [EP10, EP1, EP3, EP10, EP8, EP6, EP10, EP3, EP8, EP8, EP3, EP11].as_slice(),
        [EP4, EP3, EP11, EP2, EP8, EP6, EP2, EP9, EP8, EP1, EP9, EP2].as_slice(),
        [EP11, EP8, EP3, EP3, EP8, EP2, EP2, EP8, EP6].as_slice(),
        [EP2, EP11, EP4, EP2, EP12, EP11, EP8, EP10, EP9, EP6, EP10, EP8].as_slice(),
        [EP11, EP8, EP6, EP11, EP6, EP12, EP10, EP1, EP2].as_slice(),
        [EP6, EP12, EP11, EP6, EP11, EP8, EP4, EP1, EP9].as_slice(),
        [EP6, EP12, EP11, EP6, EP11, EP8].as_slice(),
        [EP6, EP7, EP12].as_slice(),
        [EP4, EP9, EP1, EP6, EP7, EP12].as_slice(),
        [EP1, EP10, EP2, EP6, EP7, EP12].as_slice(),
        [EP9, EP2, EP4, EP9, EP10, EP2, EP12, EP6, EP7].as_slice(),
        [EP2, EP7, EP3, EP2, EP6, EP7].as_slice(),
        [EP7, EP2, EP6, EP7, EP3, EP2, EP1, EP4, EP9].as_slice(),
        [EP10, EP6, EP1, EP6, EP7, EP1, EP7, EP3, EP1].as_slice(),
        [EP10, EP4, EP9, EP4, EP10, EP7, EP10, EP6, EP7, EP4, EP7, EP3].as_slice(),
        [EP6, EP7, EP12, EP11, EP4, EP3].as_slice(),
        [EP1, EP11, EP9, EP1, EP3, EP11, EP7, EP12, EP6].as_slice(),
        [EP10, EP2, EP1, EP6, EP7, EP12, EP3, EP11, EP4].as_slice(),
        [EP12, EP6, EP7, EP10, EP11, EP9, EP10, EP3, EP11, EP2, EP3, EP10].as_slice(),
        [EP7, EP11, EP6, EP11, EP4, EP6, EP4, EP2, EP6].as_slice(),
        [EP1, EP2, EP6, EP1, EP11, EP9, EP1, EP6, EP11, EP11, EP6, EP7].as_slice(),
        [EP10, EP4, EP1, EP11, EP4, EP10, EP11, EP10, EP6, EP11, EP6, EP7].as_slice(),
        [EP7, EP11, EP6, EP6, EP11, EP10, EP10, EP11, EP9].as_slice(),
        [EP9, EP8, EP5, EP7, EP12, EP6].as_slice(),
        [EP4, EP5, EP1, EP4, EP8, EP5, EP6, EP7, EP12].as_slice(),
        [EP1, EP10, EP2, EP9, EP8, EP5, EP6, EP7, EP12].as_slice(),
        [EP6, EP7, EP12, EP8, EP2, EP4, EP8, EP10, EP2, EP5, EP10, EP8].as_slice(),
        [EP2, EP7, EP3, EP2, EP6, EP7, EP8, EP5, EP9].as_slice(),
        [EP8, EP1, EP4, EP8, EP5, EP1, EP2, EP7, EP3, EP6, EP7, EP2].as_slice(),
        [EP5, EP9, EP8, EP1, EP7, EP3, EP1, EP6, EP7, EP10, EP6, EP1].as_slice(),
        [EP3, EP4, EP8, EP3, EP8, EP7, EP5, EP10, EP6].as_slice(),
        [EP3, EP11, EP4, EP12, EP6, EP7, EP8, EP5, EP9].as_slice(),
        [EP7, EP12, EP6, EP3, EP5, EP1, EP3, EP8, EP5, EP11, EP8, EP3].as_slice(),
        [EP11, EP4, EP3, EP2, EP1, EP10, EP8, EP5, EP9, EP7, EP12, EP6].as_slice(),
        [EP2, EP3, EP12, EP10, EP6, EP5, EP7, EP11, EP8].as_slice(),
        [EP8, EP5, EP9, EP6, EP4, EP2, EP6, EP11, EP4, EP7, EP11, EP6].as_slice(),
        [EP1, EP2, EP6, EP1, EP6, EP5, EP7, EP11, EP8].as_slice(),
        [EP4, EP1, EP9, EP11, EP8, EP7, EP5, EP10, EP6].as_slice(),
        [EP10, EP6, EP5, EP8, EP7, EP11].as_slice(),
        [EP5, EP12, EP10, EP5, EP7, EP12].as_slice(),
        [EP12, EP5, EP7, EP12, EP10, EP5, EP9, EP1, EP4].as_slice(),
        [EP12, EP2, EP7, EP2, EP1, EP7, EP1, EP5, EP7].as_slice(),
        [EP12, EP2, EP4, EP12, EP5, EP7, EP12, EP4, EP5, EP5, EP4, EP9].as_slice(),
        [EP2, EP10, EP3, EP10, EP5, EP3, EP5, EP7, EP3].as_slice(),
        [EP1, EP4, EP9, EP3, EP5, EP7, EP3, EP10, EP5, EP2, EP10, EP3].as_slice(),
        [EP5, EP7, EP3, EP5, EP3, EP1].as_slice(),
        [EP9, EP5, EP4, EP4, EP5, EP3, EP3, EP5, EP7].as_slice(),
        [EP5, EP12, EP10, EP5, EP7, EP12, EP3, EP11, EP4].as_slice(),
        [EP3, EP9, EP1, EP3, EP11, EP9, EP5, EP12, EP10, EP7, EP12, EP5].as_slice(),
        [EP3, EP11, EP4, EP7, EP1, EP5, EP7, EP2, EP1, EP12, EP2, EP7].as_slice(),
        [EP9, EP5, EP7, EP9, EP7, EP11, EP12, EP2, EP3].as_slice(),
        [EP2, EP11, EP4, EP11, EP2, EP5, EP2, EP10, EP5, EP11, EP5, EP7].as_slice(),
        [EP7, EP11, EP9, EP7, EP9, EP5, EP1, EP2, EP10].as_slice(),
        [EP4, EP1, EP11, EP11, EP1, EP7, EP7, EP1, EP5].as_slice(),
        [EP7, EP11, EP9, EP7, EP9, EP5].as_slice(),
        [EP8, EP7, EP9, EP7, EP12, EP9, EP12, EP10, EP9].as_slice(),
        [EP10, EP7, EP12, EP7, EP10, EP4, EP10, EP1, EP4, EP7, EP4, EP8].as_slice(),
        [EP1, EP12, EP2, EP7, EP12, EP1, EP7, EP1, EP9, EP7, EP9, EP8].as_slice(),
        [EP12, EP2, EP7, EP7, EP2, EP8, EP8, EP2, EP4].as_slice(),
        [EP8, EP7, EP3, EP8, EP10, EP9, EP8, EP3, EP10, EP10, EP3, EP2].as_slice(),
        [EP8, EP7, EP3, EP8, EP3, EP4, EP2, EP10, EP1].as_slice(),
        [EP8, EP7, EP9, EP9, EP7, EP1, EP1, EP7, EP3].as_slice(),
        [EP3, EP4, EP8, EP3, EP8, EP7].as_slice(),
        [EP11, EP4, EP3, EP9, EP12, EP10, EP9, EP7, EP12, EP8, EP7, EP9].as_slice(),
        [EP10, EP1, EP3, EP10, EP3, EP12, EP11, EP8, EP7].as_slice(),
        [EP12, EP2, EP3, EP7, EP11, EP8, EP4, EP1, EP9].as_slice(),
        [EP2, EP3, EP12, EP7, EP11, EP8].as_slice(),
        [EP2, EP10, EP9, EP2, EP9, EP4, EP8, EP7, EP11].as_slice(),
        [EP1, EP2, EP10, EP7, EP11, EP8].as_slice(),
        [EP7, EP11, EP8, EP9, EP4, EP1].as_slice(),
        [EP7, EP11, EP8].as_slice(),
        [EP7, EP8, EP11].as_slice(),
        [EP7, EP8, EP11, EP9, EP1, EP4].as_slice(),
        [EP1, EP10, EP2, EP7, EP8, EP11].as_slice(),
        [EP2, EP9, EP10, EP2, EP4, EP9, EP8, EP11, EP7].as_slice(),
        [EP2, EP12, EP3, EP7, EP8, EP11].as_slice(),
        [EP12, EP3, EP2, EP7, EP8, EP11, EP4, EP9, EP1].as_slice(),
        [EP10, EP3, EP1, EP10, EP12, EP3, EP11, EP7, EP8].as_slice(),
        [EP11, EP7, EP8, EP12, EP9, EP10, EP12, EP4, EP9, EP3, EP4, EP12].as_slice(),
        [EP3, EP8, EP4, EP3, EP7, EP8].as_slice(),
        [EP8, EP9, EP7, EP9, EP1, EP7, EP1, EP3, EP7].as_slice(),
        [EP8, EP3, EP7, EP8, EP4, EP3, EP2, EP1, EP10].as_slice(),
        [EP2, EP3, EP7, EP2, EP9, EP10, EP2, EP7, EP9, EP9, EP7, EP8].as_slice(),
        [EP12, EP7, EP2, EP7, EP8, EP2, EP8, EP4, EP2].as_slice(),
        [EP12, EP1, EP2, EP9, EP1, EP12, EP9, EP12, EP7, EP9, EP7, EP8].as_slice(),
        [EP12, EP1, EP10, EP1, EP12, EP8, EP12, EP7, EP8, EP1, EP8, EP4].as_slice(),
        [EP8, EP9, EP7, EP7, EP9, EP12, EP12, EP9, EP10].as_slice(),
        [EP7, EP9, EP11, EP7, EP5, EP9].as_slice(),
        [EP4, EP11, EP1, EP11, EP7, EP1, EP7, EP5, EP1].as_slice(),
        [EP7, EP9, EP11, EP7, EP5, EP9, EP1, EP10, EP2].as_slice(),
        [EP4, EP10, EP2, EP10, EP4, EP7, EP4, EP11, EP7, EP10, EP7, EP5].as_slice(),
        [EP9, EP7, EP5, EP9, EP11, EP7, EP12, EP3, EP2].as_slice(),
        [EP3, EP2, EP12, EP1, EP7, EP5, EP1, EP11, EP7, EP4, EP11, EP1].as_slice(),
        [EP1, EP12, EP3, EP1, EP10, EP12, EP7, EP9, EP11, EP5, EP9, EP7].as_slice(),
        [EP5, EP10, EP12, EP5, EP12, EP7, EP3, EP4, EP11].as_slice(),
        [EP9, EP4, EP5, EP4, EP3, EP5, EP3, EP7, EP5].as_slice(),
        [EP7, EP5, EP1, EP7, EP1, EP3].as_slice(),
        [EP1, EP10, EP2, EP5, EP3, EP7, EP5, EP4, EP3, EP9, EP4, EP5].as_slice(),
        [EP2, EP3, EP10, EP10, EP3, EP5, EP5, EP3, EP7].as_slice(),
        [EP9, EP4, EP2, EP9, EP7, EP5, EP9, EP2, EP7, EP7, EP2, EP12].as_slice(),
        [EP12, EP7, EP2, EP2, EP7, EP1, EP1, EP7, EP5].as_slice(),
        [EP12, EP7, EP5, EP12, EP5, EP10, EP9, EP4, EP1].as_slice(),
        [EP5, EP10, EP12, EP5, EP12, EP7].as_slice(),
        [EP10, EP5, EP6, EP8, EP11, EP7].as_slice(),
        [EP4, EP9, EP1, EP11, EP7, EP8, EP5, EP6, EP10].as_slice(),
        [EP1, EP6, EP2, EP1, EP5, EP6, EP7, EP8, EP11].as_slice(),
        [EP8, EP11, EP7, EP4, EP6, EP2, EP4, EP5, EP6, EP9, EP5, EP4].as_slice(),
        [EP2, EP12, EP3, EP10, EP5, EP6, EP7, EP8, EP11].as_slice(),
        [EP9, EP1, EP4, EP3, EP2, EP12, EP5, EP6, EP10, EP8, EP11, EP7].as_slice(),
        [EP7, EP8, EP11, EP5, EP3, EP1, EP5, EP12, EP3, EP6, EP12, EP5].as_slice(),
        [EP3, EP4, EP11, EP12, EP7, EP6, EP8, EP9, EP5].as_slice(),
        [EP3, EP8, EP4, EP3, EP7, EP8, EP5, EP6, EP10].as_slice(),
        [EP5, EP6, EP10, EP7, EP1, EP3, EP7, EP9, EP1, EP8, EP9, EP7].as_slice(),
        [EP7, EP4, EP3, EP7, EP8, EP4, EP1, EP6, EP2, EP5, EP6, EP1].as_slice(),
        [EP2, EP3, EP7, EP2, EP7, EP6, EP8, EP9, EP5].as_slice(),
        [EP6, EP10, EP5, EP2, EP8, EP4, EP2, EP7, EP8, EP12, EP7, EP2].as_slice(),
        [EP1, EP2, EP10, EP9, EP5, EP8, EP6, EP12, EP7].as_slice(),
        [EP4, EP1, EP5, EP4, EP5, EP8, EP6, EP12, EP7].as_slice(),
        [EP9, EP5, EP8, EP7, EP6, EP12].as_slice(),
        [EP7, EP6, EP11, EP6, EP10, EP11, EP10, EP9, EP11].as_slice(),
        [EP4, EP10, EP1, EP6, EP10, EP4, EP6, EP4, EP11, EP6, EP11, EP7].as_slice(),
        [EP7, EP6, EP2, EP7, EP9, EP11, EP7, EP2, EP9, EP9, EP2, EP1].as_slice(),
        [EP7, EP6, EP11, EP11, EP6, EP4, EP4, EP6, EP2].as_slice(),
        [EP12, EP3, EP2, EP11, EP10, EP9, EP11, EP6, EP10, EP7, EP6, EP11].as_slice(),
        [EP10, EP1, EP2, EP6, EP12, EP7, EP3, EP4, EP11].as_slice(),
        [EP1, EP9, EP11, EP1, EP11, EP3, EP7, EP6, EP12].as_slice(),
        [EP6, EP12, EP7, EP11, EP3, EP4].as_slice(),
        [EP9, EP6, EP10, EP6, EP9, EP3, EP9, EP4, EP3, EP6, EP3, EP7].as_slice(),
        [EP10, EP1, EP6, EP6, EP1, EP7, EP7, EP1, EP3].as_slice(),
        [EP7, EP6, EP2, EP7, EP2, EP3, EP1, EP9, EP4].as_slice(),
        [EP2, EP3, EP7, EP2, EP7, EP6].as_slice(),
        [EP9, EP4, EP2, EP9, EP2, EP10, EP12, EP7, EP6].as_slice(),
        [EP1, EP2, EP10, EP6, EP12, EP7].as_slice(),
        [EP4, EP1, EP9, EP6, EP12, EP7].as_slice(),
        [EP6, EP12, EP7].as_slice(),
        [EP6, EP11, EP12, EP6, EP8, EP11].as_slice(),
        [EP6, EP11, EP12, EP6, EP8, EP11, EP4, EP9, EP1].as_slice(),
        [EP11, EP6, EP8, EP11, EP12, EP6, EP10, EP2, EP1].as_slice(),
        [EP4, EP10, EP2, EP4, EP9, EP10, EP6, EP11, EP12, EP8, EP11, EP6].as_slice(),
        [EP11, EP3, EP8, EP3, EP2, EP8, EP2, EP6, EP8].as_slice(),
        [EP4, EP9, EP1, EP8, EP2, EP6, EP8, EP3, EP2, EP11, EP3, EP8].as_slice(),
        [EP11, EP3, EP1, EP11, EP6, EP8, EP11, EP1, EP6, EP6, EP1, EP10].as_slice(),
        [EP10, EP6, EP8, EP10, EP8, EP9, EP11, EP3, EP4].as_slice(),
        [EP3, EP12, EP4, EP12, EP6, EP4, EP6, EP8, EP4].as_slice(),
        [EP3, EP9, EP1, EP9, EP3, EP6, EP3, EP12, EP6, EP9, EP6, EP8].as_slice(),
        [EP2, EP1, EP10, EP4, EP6, EP8, EP4, EP12, EP6, EP3, EP12, EP4].as_slice(),
        [EP8, EP9, EP10, EP8, EP10, EP6, EP2, EP3, EP12].as_slice(),
        [EP6, EP8, EP4, EP6, EP4, EP2].as_slice(),
        [EP1, EP2, EP9, EP9, EP2, EP8, EP8, EP2, EP6].as_slice(),
        [EP10, EP6, EP1, EP1, EP6, EP4, EP4, EP6, EP8].as_slice(),
        [EP8, EP9, EP10, EP8, EP10, EP6].as_slice(),
        [EP6, EP5, EP12, EP5, EP9, EP12, EP9, EP11, EP12].as_slice(),
        [EP6, EP5, EP1, EP6, EP11, EP12, EP6, EP1, EP11, EP11, EP1, EP4].as_slice(),
        [EP10, EP2, EP1, EP12, EP9, EP11, EP12, EP5, EP9, EP6, EP5, EP12].as_slice(),
        [EP4, EP11, EP12, EP4, EP12, EP2, EP6, EP5, EP10].as_slice(),
        [EP11, EP5, EP9, EP5, EP11, EP2, EP11, EP3, EP2, EP5, EP2, EP6].as_slice(),
        [EP6, EP5, EP1, EP6, EP1, EP2, EP4, EP11, EP3].as_slice(),
        [EP11, EP3, EP1, EP11, EP1, EP9, EP10, EP6, EP5].as_slice(),
        [EP3, EP4, EP11, EP5, EP10, EP6].as_slice(),
        [EP3, EP9, EP4, EP5, EP9, EP3, EP5, EP3, EP12, EP5, EP12, EP6].as_slice(),
        [EP6, EP5, EP12, EP12, EP5, EP3, EP3, EP5, EP1].as_slice(),
        [EP9, EP4, EP1, EP5, EP10, EP6, EP2, EP3, EP12].as_slice(),
        [EP5, EP10, EP6, EP12, EP2, EP3].as_slice(),
        [EP9, EP4, EP5, EP5, EP4, EP6, EP6, EP4, EP2].as_slice(),
        [EP1, EP2, EP6, EP1, EP6, EP5].as_slice(),
        [EP4, EP1, EP9, EP5, EP10, EP6].as_slice(),
        [EP5, EP10, EP6].as_slice(),
        [EP5, EP8, EP10, EP8, EP11, EP10, EP11, EP12, EP10].as_slice(),
        [EP9, EP1, EP4, EP10, EP11, EP12, EP10, EP8, EP11, EP5, EP8, EP10].as_slice(),
        [EP12, EP8, EP11, EP8, EP12, EP1, EP12, EP2, EP1, EP8, EP1, EP5].as_slice(),
        [EP12, EP2, EP4, EP12, EP4, EP11, EP9, EP5, EP8].as_slice(),
        [EP2, EP11, EP3, EP8, EP11, EP2, EP8, EP2, EP10, EP8, EP10, EP5].as_slice(),
        [EP11, EP3, EP4, EP8, EP9, EP5, EP1, EP2, EP10].as_slice(),
        [EP11, EP3, EP8, EP8, EP3, EP5, EP5, EP3, EP1].as_slice(),
        [EP3, EP4, EP11, EP8, EP9, EP5].as_slice(),
        [EP5, EP8, EP4, EP5, EP12, EP10, EP5, EP4, EP12, EP12, EP4, EP3].as_slice(),
        [EP3, EP12, EP10, EP3, EP10, EP1, EP5, EP8, EP9].as_slice(),
        [EP5, EP8, EP4, EP5, EP4, EP1, EP3, EP12, EP2].as_slice(),
        [EP2, EP3, EP12, EP8, EP9, EP5].as_slice(),
        [EP5, EP8, EP10, EP10, EP8, EP2, EP2, EP8, EP4].as_slice(),
        [EP8, EP9, EP5, EP10, EP1, EP2].as_slice(),
        [EP4, EP1, EP5, EP4, EP5, EP8].as_slice(),
        [EP8, EP9, EP5].as_slice(),
        [EP10, EP9, EP11, EP10, EP11, EP12].as_slice(),
        [EP4, EP11, EP1, EP1, EP11, EP10, EP10, EP11, EP12].as_slice(),
        [EP1, EP9, EP2, EP2, EP9, EP12, EP12, EP9, EP11].as_slice(),
        [EP4, EP11, EP12, EP4, EP12, EP2].as_slice(),
        [EP2, EP10, EP3, EP3, EP10, EP11, EP11, EP10, EP9].as_slice(),
        [EP11, EP3, EP4, EP1, EP2, EP10].as_slice(),
        [EP1, EP9, EP11, EP1, EP11, EP3].as_slice(),
        [EP3, EP4, EP11].as_slice(),
        [EP3, EP12, EP4, EP4, EP12, EP9, EP9, EP12, EP10].as_slice(),
        [EP3, EP12, EP10, EP3, EP10, EP1].as_slice(),
        [EP9, EP4, EP1, EP2, EP3, EP12].as_slice(),
        [EP2, EP3, EP12].as_slice(),
        [EP2, EP10, EP9, EP2, EP9, EP4].as_slice(),
        [EP1, EP2, EP10].as_slice(),
        [EP4, EP1, EP9].as_slice(),
        [].as_slice(),
    ];

    const V1: u8 = 1 << 0;
    const V2: u8 = 1 << 1;
    const V3: u8 = 1 << 2;
    const V4: u8 = 1 << 3;
    const V5: u8 = 1 << 4;
    const V6: u8 = 1 << 5;
    const V7: u8 = 1 << 6;
    const V8: u8 = 1 << 7;
    
    const E1: EdgeInternal = EdgeInternal::new(V1 | V2);
    const E2: EdgeInternal = EdgeInternal::new(V2 | V3);
    const E3: EdgeInternal = EdgeInternal::new(V3 | V4);
    const E4: EdgeInternal = EdgeInternal::new(V4 | V1);
    const E5: EdgeInternal = EdgeInternal::new(V5 | V6);
    const E6: EdgeInternal = EdgeInternal::new(V6 | V7);
    const E7: EdgeInternal = EdgeInternal::new(V7 | V8);
    const E8: EdgeInternal = EdgeInternal::new(V8 | V5);
    const E9: EdgeInternal = EdgeInternal::new(V1 | V5);
    const E10: EdgeInternal = EdgeInternal::new(V2 | V6);
    const E11: EdgeInternal = EdgeInternal::new(V4 | V8);
    const E12: EdgeInternal = EdgeInternal::new(V3 | V7);

    const EP1: Edge = E1.into_edge();
    const EP2: Edge = E2.into_edge();
    const EP3: Edge = E3.into_edge();
    const EP4: Edge = E4.into_edge();
    const EP5: Edge = E5.into_edge();
    const EP6: Edge = E6.into_edge();
    const EP7: Edge = E7.into_edge();
    const EP8: Edge = E8.into_edge();
    const EP9: Edge = E9.into_edge();
    const EP10: Edge = E10.into_edge();
    const EP11: Edge = E11.into_edge();
    const EP12: Edge = E12.into_edge();

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    struct Bits {
        bits: u8,
    }

    impl Bits {
        #[inline]
        fn get(&self, bit: u8) -> bool {
            let mask = 1 << bit;
            self.bits & mask != 0
        }

        #[inline]
        fn set(&mut self, bit: u8, value: bool) {
            let mask = 1 << bit;

            if value {
                self.bits |= mask;
            } else {
                self.bits &= !mask;
            }
        }
    }

    impl Not for Bits {
        type Output = Self;

        #[inline]
        fn not(self) -> Self::Output {
            Self { bits: !self.bits }
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct Cube(Bits);

    impl Cube {
        pub const fn new(vertices: u8) -> Self {
            Self(Bits { bits: vertices })
        }

        #[inline]
        fn reverse(&self) -> Self {
            Self(!self.0)
        }

        fn rotate_x(&self) -> Self {
            let mut rotated = Self(Bits { bits: 0 });
            rotated.0.set(0, self.0.get(4));
            rotated.0.set(1, self.0.get(5));
            rotated.0.set(2, self.0.get(1));
            rotated.0.set(3, self.0.get(0));
            rotated.0.set(4, self.0.get(7));
            rotated.0.set(5, self.0.get(6));
            rotated.0.set(6, self.0.get(2));
            rotated.0.set(7, self.0.get(3));

            rotated
        }

        fn rotate_y(&self) -> Self {
            let mut rotated = Self(Bits { bits: 0 });
            rotated.0.set(0, self.0.get(4));
            rotated.0.set(1, self.0.get(0));
            rotated.0.set(2, self.0.get(3));
            rotated.0.set(3, self.0.get(7));
            rotated.0.set(4, self.0.get(5));
            rotated.0.set(5, self.0.get(1));
            rotated.0.set(6, self.0.get(2));
            rotated.0.set(7, self.0.get(6));

            rotated
        }

        fn rotate_z(&self) -> Self {
            let back   = 
                ((self.0.bits << 1) & 0b00001111) |
                ((self.0.bits & 0b00001000) >> 3);
            let front  = 
                ((self.0.bits & 0b11110000) << 1) | 
                ((self.0.bits & 0b10000000) >> 3);
            Self(Bits { bits: back | front })
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct EdgeInternal(Cube);

    impl EdgeInternal {
        #[inline]
        const fn new(vertices: u8) -> Self {
            Self(Cube::new(vertices))
        }

        #[inline]
        fn rotate_x(&self) -> Self {
            Self(self.0.rotate_x())
        }

        #[inline]
        fn rotate_y(&self) -> Self {
            Self(self.0.rotate_y())
        }

        #[inline]
        fn rotate_z(&self) -> Self {
            Self(self.0.rotate_z())
        }

        const fn v1(&self) -> u8 {
            let bits = self.0.0.bits;
            assert!(bits != 0);

            let mut v = V1;
            let mut i = 0;

            while bits & v == 0 {
                v <<= 1;
                i += 1;
            }

            i
        }

        const fn v2(&self) -> u8 {
            let bits = self.0.0.bits;

            let mut i = self.v1() + 1;
            let mut v = 1 << i;

            while bits & v == 0 {
                v <<= 1;
                i += 1;
            }

            i
        }

        const fn into_edge(&self) -> Edge {
            Edge {
                v1: self.v1(),
                v2: self.v2(),
            }
        }
    }

    #[derive(Debug, Clone)]
    struct Pattern {
        cube: Cube,
        triangles: Vec<EdgeInternal>,
        root_pattern: u8,
    }

    impl Pattern {
        fn rotate_x(&mut self) {
            self.cube = self.cube.rotate_x();
            self.triangles = self.triangles.iter().map(|e| e.rotate_x()).collect();
        }

        fn rotate_y(&mut self) {
            self.cube = self.cube.rotate_y();
            self.triangles = self.triangles.iter().map(|e| e.rotate_y()).collect();
        }

        fn rotate_z(&mut self) {
            self.cube = self.cube.rotate_z();
            self.triangles = self.triangles.iter().map(|e| e.rotate_z()).collect();
        }

        fn reverse(&self) -> Self {
            let cube = self.cube.reverse();
            let mut triangles = self.triangles.clone();

            for i in (0..triangles.len()).step_by(3) {
                triangles.swap(i + 1, i + 2);
            }

            Self {
                cube,
                triangles,
                root_pattern: self.root_pattern,
            }
        }
    }

    fn generate_lookup_table() -> BTreeMap<Cube, Pattern> {
        let patterns = [
            Pattern {
                cube: Cube::new(0),
                triangles: vec![],
                root_pattern: 0,
            },
            Pattern {
                cube: Cube::new(V1),
                triangles: [E9, E1, E4].into(),
                root_pattern: 1,
            },
            Pattern {
                cube: Cube::new(V1 | V2),
                triangles: [E9, E2, E4, E9, E10, E2].into(),
                root_pattern: 2,
            },
            Pattern {
                cube: Cube::new(V1 | V3),
                triangles: [E9, E1, E4, E2, E12, E3].into(),
                root_pattern: 3,
            },
            Pattern {
                cube: Cube::new(V1 | V7),
                triangles: [E9, E1, E4, E12, E6, E7].into(),
                root_pattern: 4,
            },
            Pattern {
                cube: Cube::new(V2 | V6 | V5),
                triangles: [E1, E9, E2, E9, E8, E2, E8, E6, E2].into(),
                root_pattern: 5,
            },
            Pattern {
                cube: Cube::new(V1 | V2 | V7),
                triangles: [E9, E2, E4, E9, E10, E2, E12, E6, E7].into(),
                root_pattern: 6,
            },
            Pattern {
                cube: Cube::new(V4 | V2 | V7),
                triangles: [E4, E3, E11, E1, E10, E2, E12, E6, E7].into(),
                root_pattern: 7,
            },
            Pattern {
                cube: Cube::new(V1 | V2 | V6 | V5),
                triangles: [E6, E2, E4, E6, E4, E8].into(),
                root_pattern: 8,
            },
            Pattern {
                cube: Cube::new(V1 | V8 | V6 | V5),
                triangles: [E7, E4, E11, E1, E4, E7, E1, E7, E6, E1, E6, E10].into(),
                root_pattern: 9,
            },
            Pattern {
                cube: Cube::new(V1 | V4 | V6 | V7),
                triangles: [E9, E3, E11, E9, E1, E3, E12, E5, E7, E10, E5, E12].into(),
                root_pattern: 10,
            },
            Pattern {
                cube: Cube::new(V1 | V5 | V6 | V7),
                triangles: [E8, E1, E4, E1, E8, E12, E8, E7, E12, E1, E12, E10].into(),
                root_pattern: 11,
            },
            Pattern {
                cube: Cube::new(V4 | V2 | V6 | V5),
                triangles: [E4, E3, E11, E2, E8, E6, E2, E9, E8, E1, E9, E2].into(),
                root_pattern: 12,
            },
            Pattern {
                cube: Cube::new(V1 | V8 | V3 | V6),
                triangles: [E1, E4, E9, E8, E11, E7, E2, E12, E3, E10, E5, E6].into(),
                root_pattern: 13,
            },
            Pattern {
                cube: Cube::new(V8 | V2 | V6 | V5),
                triangles: [E1, E9, E11, E1, E6, E2, E1, E11, E6, E6, E11, E7].into(),
                root_pattern: 14,
            },
        ];

        let mut map = BTreeMap::new();

        for pattern in patterns {
            rotate_and_insert(pattern.clone(), &mut map);
            rotate_and_insert(pattern.reverse(), &mut map);
        }

        map
    }

    fn rotate_and_insert(mut pattern: Pattern, map: &mut BTreeMap<Cube, Pattern>) {
        for _ in 0..4 {
            pattern.rotate_x();

            for _ in 0..4 {
                pattern.rotate_y();

                for _ in 0..4 {
                    pattern.rotate_z();

                    if map.contains_key(&pattern.cube) {
                        continue;
                    }

                    map.insert(pattern.cube, pattern.clone());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    // use super::*;

    // #[test]
    // fn test_generate_lookup_table() {
    //     let table = generate_lookup_table();
    //     assert_eq!(table.len(), 256);
    // }
}
