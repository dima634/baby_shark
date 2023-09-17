use std::{collections::BTreeMap, rc::Rc, mem::swap, ops::Index};

use crate::geometry::traits::RealNumber;

const V1: u8 = 1 << 0;
const V2: u8 = 1 << 1;
const V3: u8 = 1 << 2;
const V4: u8 = 1 << 3;
const V5: u8 = 1 << 4;
const V6: u8 = 1 << 5;
const V7: u8 = 1 << 6;
const V8: u8 = 1 << 7;

const E1: Edge  = Edge(V1 | V2);
const E2: Edge  = Edge(V2 | V3);
const E3: Edge  = Edge(V3 | V4);
const E4: Edge  = Edge(V4 | V1);
const E5: Edge  = Edge(V5 | V6);
const E6: Edge  = Edge(V6 | V7);
const E7: Edge  = Edge(V7 | V8);
const E8: Edge  = Edge(V8 | V5);
const E9: Edge  = Edge(V1 | V5);
const E10: Edge = Edge(V2 | V6);
const E11: Edge = Edge(V4 | V8);
const E12: Edge = Edge(V3 | V7);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Cube(u8);

impl Cube {
    #[inline]
    fn get(&self, bit: u8) -> bool {
        let mask = 1 << bit;
        self.0 & mask != 0
    }

    #[inline]
    fn set(&mut self, bit: u8, value: bool) {
        let mask = 1 << bit;

        if value {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
    }

    #[inline]
    fn reverse(&self) -> Self {
        Self(!self.0)
    }

    fn rotate_x(&self) -> Self {
        let mut rotated = Self(0);
        rotated.set(0, self.get(4));
        rotated.set(1, self.get(5));
        rotated.set(2, self.get(1));
        rotated.set(3, self.get(0));
        rotated.set(4, self.get(7));
        rotated.set(5, self.get(6));
        rotated.set(6, self.get(2));
        rotated.set(7, self.get(3));

        rotated
    }

    fn rotate_y(&self) -> Self {
        let mut rotated = Self(0);
        rotated.set(0, self.get(4));
        rotated.set(1, self.get(0));
        rotated.set(2, self.get(3));
        rotated.set(3, self.get(7));
        rotated.set(4, self.get(5));
        rotated.set(5, self.get(1));
        rotated.set(6, self.get(2));
        rotated.set(7, self.get(6));

        rotated
    }

    fn rotate_z(&self) -> Self {
        let back   = 
            ((self.0 << 1) & 0b00001111) |
            ((self.0 & 0b00001000) >> 3);
        let front  = 
            ((self.0 & 0b11110000) << 1) | 
            ((self.0 & 0b10000000) >> 3);
        Self(back | front)
    }
}

#[derive(Debug, Clone, Copy)]
struct Edge(u8);

impl Edge {
    fn rotate(&self) -> Self {
        // THIS IS WRONG
        if self.0 & 0b10000000 != 0 {
            Self(
                (self.0 << 1) | 
                ((self.0 & 0b00000001) << 6)
            )
        } else {
            Self(self.0 << 1)
        }
    }
}

fn generate_lookup_table() -> BTreeMap<Cube, Vec<Edge>> {
    let patterns = [
        (Cube(0),                 vec![]),
        (Cube(V1),                [E9, E4, E1].into()),
        // (Cube(V1 | V2),           [E9, E4, E2, E9, E2, E10].into()),
        // (Cube(V1 | V3),           [E9, E4, E1, E2, E3, E12].into()),
        // (Cube(V1 | V7),           [E9, E4, E1, E12, E7, E6].into()),
        // (Cube(V2 | V6 | V5),      [E1, E2, E9, E9, E2, E8, E8, E2, E6].into()),
        // (Cube(V1 | V2 | V7),      [E9, E4, E2, E9, E2, E10, E12, E7, E6].into()),
        // (Cube(V4 | V2 | V7),      [E4, E11, E3, E1, E2, E10, E12, E7, E6].into()),
        // (Cube(V1 | V2 | V6 | V5), [E6, E2, E4, E6, E4, E8].into()),
        // (Cube(V1 | V8 | V6 | V5), [E7, E11, E4, E1, E7, E4, E1, E6, E7, E1, E10, E6].into()), 
        // (Cube(V1 | V4 | V6 | V7), [E9, E11, E3, E9, E3, E1, E12, E7, E5, E10, E12, E5].into()),
        // (Cube(V1 | V5 | V6 | V7), [E8, E4, E1, E1, E12, E8, E8, E12, E7, E1, E10, E12].into()),
        // (Cube(V4 | V2 | V6 | V5), [E4, E11, E3, E2, E6, E8, E2, E8, E9, E1, E2, E9].into()),
        // (Cube(V1 | V8 | V3 | V6), [E1, E9, E4, E8, E7, E11, E2, E3, E12, E10, E6, E5].into()),
        // (Cube(V8 | V2 | V6 | V5), [E1, E11, E9, E1, E2, E6, E1, E6, E11, E6, E7, E11].into()),
    ];

    let mut map = BTreeMap::new();

    for (cube, mut triangles) in patterns {
        rotate_and_insert(cube, triangles.clone(), &mut map);
        
        let cube = cube.reverse();
        for i in (0..triangles.len()).step_by(3) {
            triangles.swap(i + 1, i + 2);
        }

        rotate_and_insert(cube, triangles.clone(), &mut map);
    }
    
    map
}

fn rotate_and_insert(mut cube: Cube, mut triangles: Vec<Edge>, map: &mut BTreeMap<Cube, Vec<Edge>>) {
    // for _ in 0..4 {
    //     cube = cube.rotate_x();
        
    //     for _ in 0..4 {
    //         cube = cube.rotate_y();

    //         for _ in 0..4 {
    //             cube = cube.rotate_z();
    //             //println!("{:08b}", cube.0);

    //             if map.contains_key(&cube) {
    //                 assert!(false);
    //             }

    //             //map.insert(cube, triangles.clone());
    //         }

    //         //println!();
    //     }
    // }

    for _ in 0..4 {
        cube = cube.rotate_x();
        
        for _ in 0..4 {
            cube = cube.rotate_y();

            for _ in 0..4 {
                cube = cube.rotate_z();
                println!("{:08b}", cube.0);

                if map.contains_key(&cube) {
                    continue;
                }

                map.insert(cube, triangles.clone());
            }

            println!();
        }
    }
       
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_lookup_table() {
        let table = generate_lookup_table();
        println!("{:#?}", table.len());

        for key in table.keys() {
            println!("{} - {:08b}", key.0, key.0);
        }
    }
}
