use std::{collections::{BTreeMap, HashSet}, rc::Rc, mem::swap, ops::Not};

use nalgebra::Vector3;

use crate::{geometry::{traits::RealNumber, primitives::triangle3::Triangle3}, mesh::traits::Mesh, voxel::{TreeNode, InternalNode, Grid, Leaf, Tile, meshing::bool_grid::BoolGrid}, algo::{merge_points::merge_points, utils::cast}};

use super::bool_grid::intersection_grid;

const V1: u8 = 1 << 0;
const V2: u8 = 1 << 1;
const V3: u8 = 1 << 2;
const V4: u8 = 1 << 3;
const V5: u8 = 1 << 4;
const V6: u8 = 1 << 5;
const V7: u8 = 1 << 6;
const V8: u8 = 1 << 7;

const E1: Edge  = Edge::new(V1 | V2);
const E2: Edge  = Edge::new(V2 | V3);
const E3: Edge  = Edge::new(V3 | V4);
const E4: Edge  = Edge::new(V4 | V1);
const E5: Edge  = Edge::new(V5 | V6);
const E6: Edge  = Edge::new(V6 | V7);
const E7: Edge  = Edge::new(V7 | V8);
const E8: Edge  = Edge::new(V8 | V5);
const E9: Edge  = Edge::new(V1 | V5);
const E10: Edge = Edge::new(V2 | V6);
const E11: Edge = Edge::new(V4 | V8);
const E12: Edge = Edge::new(V3 | V7);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct BitSet {
    bits: u8,
}

impl BitSet {
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



impl Not for BitSet {
    type Output = Self;

    #[inline]
    fn not(self) -> Self::Output {
        Self { bits: !self.bits }
    }
}


#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Cube(BitSet);

impl Cube {
    const fn new(vertices: u8) -> Self {
        Self(BitSet { bits: vertices })
    }

    #[inline]
    fn reverse(&self) -> Self {
        Self(!self.0)
    }

    fn rotate_x(&self) -> Self {
        let mut rotated = Self(BitSet { bits: 0 });
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
        let mut rotated = Self(BitSet { bits: 0 });
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
        Self(BitSet { bits: back | front })
    }
}

#[derive(Debug, Clone, Copy)]
struct Edge(Cube);

impl Edge {
    const fn new(vertices: u8) -> Self {
        Self(Cube::new(vertices))
    }

    fn rotate_x(&self) -> Self {
        Self(self.0.rotate_x())
    }

    fn rotate_y(&self) -> Self {
        Self(self.0.rotate_y())
    }

    fn rotate_z(&self) -> Self {
        Self(self.0.rotate_z())
    }

    fn v1(&self) -> u8 {
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

    fn v2(&self) -> u8 {
        let bits = self.0.0.bits;

        let mut i = self.v1() + 1;
        let mut v = 1 << i;

        while bits & v == 0 {
            v <<= 1;
            i += 1;
        }

        i
    }
}

#[derive(Debug, Clone)]
struct Pattern {
    cube: Cube,
    triangles: Vec<Edge>,
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

        Self { cube, triangles, root_pattern: self.root_pattern }
    }
}

fn generate_lookup_table() -> BTreeMap<Cube, Pattern> {
    let patterns = [
        Pattern { cube: Cube::new(0),                 triangles: vec![],                                                        root_pattern: 0 },
        Pattern { cube: Cube::new(V1),                triangles: [E9, E1, E4].into(),                                           root_pattern: 1 },
        Pattern { cube: Cube::new(V1 | V2),           triangles: [E9, E2, E4, E9, E10, E2].into(),                              root_pattern: 2 },
        Pattern { cube: Cube::new(V1 | V3),           triangles: [E9, E1, E4, E2, E12, E3].into(),                              root_pattern: 3 },
        Pattern { cube: Cube::new(V1 | V7),           triangles: [E9, E1, E4, E12, E6, E7].into(),                              root_pattern: 4 },
        Pattern { cube: Cube::new(V2 | V6 | V5),      triangles: [E1, E9, E2, E9, E8, E2, E8, E6, E2].into(),                   root_pattern: 5 },
        Pattern { cube: Cube::new(V1 | V2 | V7),      triangles: [E9, E2, E4, E9, E10, E2, E12, E6, E7].into(),                 root_pattern: 6 },
        Pattern { cube: Cube::new(V4 | V2 | V7),      triangles: [E4, E3, E11, E1, E10, E2, E12, E6, E7].into(),                root_pattern: 7 },
        Pattern { cube: Cube::new(V1 | V2 | V6 | V5), triangles: [E6, E2, E4, E6, E4, E8].into(),                               root_pattern: 8 },
        Pattern { cube: Cube::new(V1 | V8 | V6 | V5), triangles: [E7, E4, E11, E1, E4, E7, E1, E7, E6, E1, E6, E10].into(),     root_pattern: 9 },
        Pattern { cube: Cube::new(V1 | V4 | V6 | V7), triangles: [E9, E3, E11, E9, E1, E3, E12, E5, E7, E10, E5, E12].into(),   root_pattern: 10 },
        Pattern { cube: Cube::new(V1 | V5 | V6 | V7), triangles: [E8, E1, E4, E1, E8, E12, E8, E7, E12, E1, E12, E10].into(),   root_pattern: 11 },
        Pattern { cube: Cube::new(V4 | V2 | V6 | V5), triangles: [E4, E3, E11, E2, E8, E6, E2, E9, E8, E1, E9, E2].into(),      root_pattern: 12 },
        Pattern { cube: Cube::new(V1 | V8 | V3 | V6), triangles: [E1, E4, E9, E8, E11, E7, E2, E12, E3, E10, E5, E6].into(),    root_pattern: 13 },
        Pattern { cube: Cube::new(V8 | V2 | V6 | V5), triangles: [E1, E9, E11, E1, E6, E2, E1, E11, E6, E6, E11, E7].into(),    root_pattern: 14 },
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
                // println!("{:08b}", cube.0);

                if map.contains_key(&pattern.cube) {
                    continue;
                }

                map.insert(pattern.cube, pattern.clone());
            }

            // println!();
        }
    }
}

pub fn  marching_cubes<TMesh: Mesh, TGrid: Grid>(grid: &TGrid) -> TMesh {
    let mut vertices: Vec<Vector3<f32>> = Vec::new();

    let lookup_table = generate_lookup_table();
    let int_grid = intersection_grid(grid);

    println!("TRAVERSE LEAFS");

    let mut i = 0;
    let mut j = 0;

    // grid.traverse_leafs(&mut |leaf| {
    //     let tile = match leaf {
    //         Leaf::Tile(t) => t, 
    //         Leaf::Node(n) => Tile {
    //             origin: n.origin(),
    //             size: n.size_t(),
    //         },
    //     };

    //     j += tile.size * tile.size * tile.size;
    // });

    int_grid.traverse_leafs(&mut |leaf| {
        let tile = match leaf {
            Leaf::Tile(t) => t,
            Leaf::Node(n) => Tile {
                origin: *n.origin(),
                size: n.size_t(),
            },
        };

        let max = tile.origin + Vector3::new(tile.size, tile.size, tile.size).cast();
        i += tile.size * tile.size * tile.size;
        for x in tile.origin.x..max.x {
            for y in tile.origin.y..max.y {
                for z in tile.origin.z..max.z {
                    let v = Vector3::new(x, y, z);
                    handle_cube(v, grid, &lookup_table, &mut vertices);
                }
            }
        }
    });

    
    println!("src = {j}");
    println!("i = {i}");
    println!("TRAVERSE LEAFS ============");

    // grid.voxels(&mut |v1_idx| {
    //     voxels.insert(v1_idx);
        
    //     for x in -1..=1 {
    //         for y in -1..=1 {
    //             for z in -1..=1 {
    //                 let v = v1_idx + Vector3::new(x, y, z);
    //                 voxels.insert(v);
    //             }
    //         }
    //     }
    // });

    // for v in voxels {
    //     handle_cube(v, grid, &lookup_table, &mut vertices);
    // }



    let vertices: Vec<_> = vertices.into_iter().map(|v| cast(&v).into()).collect();
    let indexed = merge_points(&vertices);
    
    TMesh::from_vertices_and_indices(indexed.points.as_slice(), &indexed.indices)
}

fn handle_cube<TGrid: TreeNode>(v: Vector3<isize>, grid: &TGrid, lookup_table: &BTreeMap<Cube, Pattern>, vertices: &mut Vec<Vector3<f32>>) {
    let vertex_indices = [
        v,
        v + Vector3::new(1, 0, 0),
        v + Vector3::new(1, 1, 0),
        v + Vector3::new(0, 1, 0),
        v + Vector3::new(0, 0, 1),
        v + Vector3::new(1, 0, 1),
        v + Vector3::new(1, 1, 1),
        v + Vector3::new(0, 1, 1),
    ];

    let mut cube = Cube::new(0);

    for i in 0..vertex_indices.len() {
        if grid.at(&vertex_indices[i]) {
            cube.0.set(i as u8, true);
        }
    }

    let pattern = lookup_table.get(&cube).unwrap();
    // println!("Voxel = {:?}; Pattern = {}", v1_idx, pattern.root_pattern);

    if pattern.root_pattern != 2 {
        //return;
    }

    for i in (0..pattern.triangles.len()).step_by(3) {
        let e1 = pattern.triangles[i];
        let e2 = pattern.triangles[i + 1];
        let e3 = pattern.triangles[i + 2];

        let v1 = interpolate(e1, &vertex_indices); 
        let v2 = interpolate(e2, &vertex_indices);
        let v3 = interpolate(e3, &vertex_indices);

        vertices.push(v1);
        vertices.push(v2);
        vertices.push(v3);

        Triangle3::normal(&v1.into(), &v2.into(), &v3.into());

        
        // println!("Add face = {:?}; {:?}; {:?}", v1, v2, v3);
    }

    // println!()
}

fn interpolate(e: Edge, vertices: &[Vector3<isize>]) -> Vector3<f32> {
    let v1_idx = e.v1() as usize;
    let v2_idx = e.v2() as usize;

    // println!("{} - {}", v1_idx, v2_idx);

    let v1 = vertices[v1_idx].cast();    
    let v2 = vertices[v2_idx].cast();

    let mid = (v1 + v2) / 2.0;

    mid
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_lookup_table() {
        let table = generate_lookup_table();
        assert_eq!(table.len(), 256);
    }

    #[test]
    fn test_marching_cubes() {
        let table = generate_lookup_table();
        assert_eq!(table.len(), 256);
    }
}
