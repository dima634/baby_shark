use std::{mem::MaybeUninit, fmt::Debug, ops::Index};

use nalgebra::Vector3;
use tabled::grid::config;

use crate::{voxel::{Grid, Leaf, Tile, TreeNode, Accessor, Scalar, sdf::Sdf}, geometry::primitives::triangle3::Triangle3, helpers::aliases::Vec3f};

use super::lookup_table::{CASES, Edge, TILING_1, TILING_EDGES_1, TILING_EDGES_2, TILING_EDGES_3_2, TILING_EDGES_3_1, TEST_3, TEST_6, TEST_10, TEST_12, TEST_4, TILING_EDGES_4_1, TILING_EDGES_4_2, TILING_5, TILING_EDGES_5, TILING_EDGES_6_2, TILING_6_1_1, TILING_6_1_2, TILING_EDGES_6_1_1, TILING_EDGES_6_1_2, TEST_7, TILING_EDGES_7_1, TILING_EDGES_7_2, TILING_EDGES_7_3, TILING_EDGES_7_4_2, TILING_EDGES_7_4_1, TILING_EDGES_8, TILING_EDGES_9, TILING_10_1_1_, TILING_EDGES_10_1_1_, TILING_EDGES_10_1_2, TILING_EDGES_10_2, TILING_EDGES_10_1_1, TILING_EDGES_10_2_, TILING_EDGES_11, TILING_EDGES_12_1_1_, TILING_EDGES_12_1_2, TILING_EDGES_12_2, TILING_EDGES_12_2_, TILING_EDGES_12_1_1, TEST_13, SUBCONFIG_13, TILING_EDGES_13_1, TILING_EDGES_13_2, TILING_EDGES_13_3};

use super::lookup_table::*;

#[derive(Debug)]
pub struct Vertex<T: Debug + Copy + Into<f32>> {
    pub index: Vector3<isize>,
    pub value: T,
}

pub trait MarchingCubes {
    type Value: Debug + Copy + Into<f32>;

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
    v12: Vec3f,
}

impl<'a, T: MarchingCubes> MarchingCubesMesher<'a, T> {
    pub fn new(grid: &'a T) -> Self {
        Self { 
            grid,
            vertices: Vec::new(),
            v12: Vec3f::zeros(),
        }
    }

    pub fn mesh(&mut self) -> Vec<Vector3<f32>> {
        self.grid.cubes(|i| self.handle_cube(i));
        self.vertices.clone()
    }

    
    fn handle_cube(&mut self, v: Vector3<isize>) {
        let cube = match MCube::from_voxel(v, self.grid) {
            Some(cube) => cube,
            None => return,
        };

        let [case, config] = CASES[cube.id() as usize];
        let config = config as usize;

        match case {
            0 => return,
            1 => self.add_faces(&TILING_EDGES_1[config], &cube),
            2 => self.add_faces(&TILING_EDGES_2[config], &cube),
            3 => {
                if Self::test_face(TEST_3[config], &cube) {
                    self.add_faces(&TILING_EDGES_3_2[config], &cube);
                } else {
                    self.add_faces(&TILING_EDGES_3_1[config], &cube);
                }
            },
            4 => {
                if self.test_interior(case, config, TEST_4[config], &cube) {
                    self.add_faces(&TILING_EDGES_4_1[config], &cube);
                } else {
                    self.add_faces(&TILING_EDGES_4_2[config], &cube);
                }
            }
            5 => self.add_faces(&TILING_EDGES_5[config], &cube),
            6 => {
                if Self::test_face(TEST_6[config][0], &cube) {
                    self.add_faces(&TILING_EDGES_6_2[config], &cube);
                } else {
                    if self.test_interior(case, config, TEST_6[config][1], &cube) {
                        self.add_faces(&TILING_EDGES_6_1_1[config], &cube);
                    } else {
                        self.add_faces(&TILING_EDGES_6_1_2[config], &cube);
                    }
                }
            },
            7 => {
                let mut subconfig = 0;

                if Self::test_face(TEST_7[config][0], &cube) {
                    subconfig += 1;
                }
                
                if Self::test_face(TEST_7[config][1], &cube) {
                    subconfig += 2;
                }
                
                if Self::test_face(TEST_7[config][2], &cube) {
                    subconfig += 4;
                }

                match subconfig {
                    0 => self.add_faces(&TILING_EDGES_7_1[config], &cube),
                    1 => self.add_faces(&TILING_EDGES_7_2[config][0], &cube),
                    2 => self.add_faces(&TILING_EDGES_7_2[config][1], &cube),
                    3 => {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_7_3[config][0], &cube);
                    },
                    4 => self.add_faces(&TILING_EDGES_7_2[config][2], &cube),
                    5 => {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_7_3[config][1], &cube);
                    },
                    6 => {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_7_3[config][2], &cube);
                    },
                    7 => {
                        if self.test_interior(case, config, TEST_7[config][3], &cube) {
                            self.add_faces(&TILING_EDGES_7_4_1[config], &cube);
                        } else {
                            self.add_faces(&TILING_EDGES_7_4_2[config], &cube);
                        }
                    },
                    _ => debug_assert!(false, "Marching cubes: impossible case {}", case),
                }
            },
            8 => self.add_faces(&TILING_EDGES_8[config], &cube),
            9 => self.add_faces(&TILING_EDGES_9[config], &cube),
            10 => {
                if Self::test_face(TEST_10[config][0], &cube) {
                    if Self::test_face(TEST_10[config][1], &cube) {
                        if self.test_interior(case, config, -TEST_10[config][2], &cube) {
                            self.add_faces(&TILING_EDGES_10_1_1_[config], &cube);
                        } else {
                            self.add_faces(&TILING_EDGES_10_1_2[5 - config], &cube);
                        }
                    } else {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_10_2[config], &cube);
                    }
                } else {
                    if Self::test_face(TEST_10[config][1], &cube) {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_10_2_[config], &cube);
                    } else {
                        if self.test_interior(case, config, TEST_10[config][2], &cube) {
                            self.add_faces(&TILING_EDGES_10_1_1[config], &cube);
                        } else {
                            self.add_faces(&TILING_EDGES_10_1_2[config], &cube);
                        }
                    }
                }    
            },
            11 => self.add_faces(&TILING_EDGES_11[config], &cube),
            12 => {
                if Self::test_face(TEST_12[config][0], &cube) {
                    if Self::test_face(TEST_12[config][1], &cube) {
                        if self.test_interior(case, config, -TEST_12[config][2], &cube) {
                            self.add_faces(&TILING_EDGES_12_1_1_[config], &cube);
                        } else {
                            self.add_faces(&TILING_EDGES_12_1_2[23 - config], &cube);
                        }
                    } else {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_12_2[config], &cube);
                    }
                } else {
                    if Self::test_face(TEST_12[config][1], &cube) {
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_12_2_[config], &cube);
                    } else {
                        if self.test_interior(case, config, TEST_12[config][2], &cube) {
                            self.add_faces(&TILING_EDGES_12_1_1[config], &cube);
                        } else {
                            self.add_faces(&TILING_EDGES_12_1_2[config], &cube);
                        }
                    }
                }
            },
            13 => {
                let mut subconfig = 0;

                if Self::test_face(TEST_13[config][0], &cube) {
                    subconfig += 1;
                }

                if Self::test_face(TEST_13[config][1], &cube) {
                    subconfig += 2;
                }

                if Self::test_face(TEST_13[config][2], &cube) {
                    subconfig += 4;
                }

                if Self::test_face(TEST_13[config][3], &cube) {
                    subconfig += 8;
                }

                if Self::test_face(TEST_13[config][4], &cube) {
                    subconfig += 16;
                }
                
                if Self::test_face(TEST_13[config][5], &cube) {
                    subconfig += 32;
                }
                
                match SUBCONFIG_13[subconfig] {
                    0 => { // 13.1
                        self.add_faces(&TILING_EDGES_13_1[config], &cube);
                    },
                    1..=6 => { // 13.2
                        self.add_faces(&TILING_EDGES_13_2[config][subconfig - 1], &cube);
                    },
                    7..=18 => { // 13.3
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_13_3[config][subconfig - 7], &cube);
                    },
                    19..=22 => { // 13.3
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_13_4[config][subconfig - 19], &cube);
                    },
                    23..=26 => { // 13.5
                        if config == 0 {
                            if self.interior_test_case13(&cube) {
                                self.add_faces(&TILING_EDGES_13_5_1[0][subconfig - 23], &cube);
                            } else {
                                self.add_faces(&TILING_EDGES_13_5_2[1][subconfig - 23], &cube);
                            
                            }
                        } else {
                            if self.interior_test_case13(&cube) {
                                self.add_faces(&TILING_EDGES_13_5_1[1][subconfig - 23], &cube);
                            } else {
                                self.add_faces(&TILING_EDGES_13_5_2[0][subconfig - 23], &cube);
                            }
                        }
                    },
                    27..=38 => { // 13.3
                        self.v12 = self.c_vertex(&cube);
                        self.add_faces(&TILING_EDGES_13_3_[config][subconfig - 27], &cube);
                    },
                    39..=44 => { // 13.2
                        self.add_faces(&TILING_EDGES_13_2_[config][subconfig - 39], &cube);
                    },
                    45 => { // 13.1
                        self.add_faces(&TILING_EDGES_13_1_[config], &cube);
                    },
                    _ => debug_assert!(false, "Marching Cubes: Impossible case 13?"),
                }
            }
            14 => self.add_faces(&TILING_EDGES_14[config], &cube),
            _ => debug_assert!(false, "Marching cubes: impossible case {}", case),
        }
    }

    fn interior_test_case13(&self, cube: &MCube<T::Value>) -> bool {
        let _cube = [
            cube.val(0),
            cube.val(1),
            cube.val(2),
            cube.val(3),
            cube.val(4),
            cube.val(5),
            cube.val(6),
            cube.val(7),
        ];
        let mut At1 = 0.0;
        let mut Bt1 = 0.0;
        let mut Ct1 = 0.0;
        let mut Dt1 = 0.0;
        let mut At2 = 0.0;
        let mut Bt2 = 0.0;
        let mut Ct2 = 0.0;
        let mut Dt2 = 0.0;
        let mut a = 0.0;
        let mut b = 0.0;
        let mut c = 0.0;

        a = (_cube[0] - _cube[1]) * (_cube[7] - _cube[6])
            - (_cube[4] - _cube[5]) * (_cube[3] - _cube[2]);
        b = _cube[6] * (_cube[0] - _cube[1]) + _cube[1] * (_cube[7] - _cube[6])
            - _cube[2] * (_cube[4] - _cube[5])
            - _cube[5] * (_cube[3] - _cube[2]);

        c = _cube[1]*_cube[6] - _cube[5]*_cube[2];

        let delta = b*b - 4.0*a*c;

        let t1 = (-b + delta.sqrt())/(a + a);
        let t2 = (-b - delta.sqrt())/(a + a);

        // println!("t1 = {}, t2 = {}", t1, t2);

        if t1 < 1.0 && t1 > 0.0 && t2 < 1.0 && t2 > 0.0 {
            At1 = _cube[1] + (_cube[0] - _cube[1]) * t1;
            Bt1 = _cube[5] + (_cube[4] - _cube[5]) * t1;
            Ct1 = _cube[6] + (_cube[7] - _cube[6]) * t1;
            Dt1 = _cube[2] + (_cube[3] - _cube[2]) * t1;

            let x1 = (At1 - Dt1)/(At1 + Ct1 - Bt1 - Dt1);
            let y1 = (At1 - Bt1)/(At1 + Ct1 - Bt1 - Dt1);

            At2 = _cube[1] + (_cube[0] - _cube[1]) * t2;
            Bt2 = _cube[5] + (_cube[4] - _cube[5]) * t2;
            Ct2 = _cube[6] + (_cube[7] - _cube[6]) * t2;
            Dt2 = _cube[2] + (_cube[3] - _cube[2]) * t2;

            let x2 = (At2 - Dt2)/(At2 + Ct2 - Bt2 - Dt2);
            let y2 = (At2 - Bt2)/(At2 + Ct2 - Bt2 - Dt2);

            if x1 < 1.0 && x1 > 0.0 && x2 < 1.0 && x2 > 0.0 && y1 < 1.0 && y1 > 0.0 && y2 < 1.0 && y2 > 0.0 {
                false
            } else {
                true
            }
        } else {
            true
        }
    }

    fn add_faces(&mut self, edges: &[Edge], cube: &MCube<T::Value>) {
        for i in (0..edges.len()).step_by(3) {
            let e1 = edges[i];
            let e3 = edges[i + 1];
            let e2 = edges[i + 2];

            let v1 = self.interpolate(e1, cube);
            let v2 = self.interpolate(e2, cube);
            let v3 = self.interpolate(e3, cube);

            self.vertices.push(v1);
            self.vertices.push(v2);
            self.vertices.push(v3);
        }
    }
    
    fn interpolate(&mut self, e: Edge, cube: &MCube<T::Value>) -> Vec3f {
        if e.is_special_edge() {
            return self.v12;
        }

        let v1 = cube.v(e.v1 as usize);
        let v2 = cube.v(e.v2 as usize);
        let mid = self.grid.interpolate(v1, v2);

        mid
    }

    fn test_face(face: i8, cube: &MCube<T::Value>) -> bool {
        let (a, b, c, d) = match face {
            -1 | 1 => (
                cube.v(0).value.into(),
                cube.v(4).value.into(),
                cube.v(5).value.into(),
                cube.v(1).value.into(),
            ),
            -2 | 2 => (
                cube.v(1).value.into(),
                cube.v(5).value.into(),
                cube.v(6).value.into(),
                cube.v(2).value.into(),
            ),
            -3 | 3 => (
                cube.v(2).value.into(),
                cube.v(6).value.into(),
                cube.v(7).value.into(),
                cube.v(3).value.into(),
            ),
            -4 | 4 => (
                cube.v(3).value.into(),
                cube.v(7).value.into(),
                cube.v(4).value.into(),
                cube.v(0).value.into(),
            ),
            -5 | 5 => (
                cube.v(0).value.into(),
                cube.v(3).value.into(),
                cube.v(2).value.into(),
                cube.v(1).value.into(),
            ),
            -6 | 6 => (
                cube.v(4).value.into(),
                cube.v(7).value.into(),
                cube.v(6).value.into(),
                cube.v(5).value.into(),
            ),
            _ => {
                debug_assert!(false, "Invalid face {}", face);
                return false;
            },
        };

        let val = a * c - b * d;

        if val.abs() < f32::EPSILON {
            return face >= 0;
        }

        face as f32 * a * val >= 0.0
    }

    fn test_interior(&self, case: i8, config: usize, s: i8, cube: &MCube<T::Value>) -> bool {
        let cube = [
            cube.val(0),
            cube.val(1),
            cube.val(2),
            cube.val(3),
            cube.val(4),
            cube.val(5),
            cube.val(6),
            cube.val(7),
        ];
        match case {
            4 => {
                let mut amb_face = 1;
                let mut edge = Self::interior_ambiguity(amb_face, s, &cube);
                let mut inter_amb = Self::interior_ambiguity_verification(edge, &cube);

                amb_face = 2;
                edge = Self::interior_ambiguity(amb_face, s, &cube);
                inter_amb += Self::interior_ambiguity_verification(edge, &cube);

                amb_face = 5;
                edge = Self::interior_ambiguity(amb_face, s, &cube);
                inter_amb += Self::interior_ambiguity_verification(edge, &cube);

                inter_amb != 0
            },
            6 => {
                let amb_face = TEST_6[config][0].abs();
                let edge = Self::interior_ambiguity(amb_face, s, &cube);
                let inter_amb = Self::interior_ambiguity_verification(edge, &cube);

                inter_amb != 0
            },
            7 => {
                let s = -s;

                let mut amb_face = 1;
                let mut edge = Self::interior_ambiguity(amb_face, s, &cube);
                let mut inter_amb = Self::interior_ambiguity_verification(edge, &cube);

                amb_face = 2;
                edge = Self::interior_ambiguity(amb_face, s, &cube);
                inter_amb += Self::interior_ambiguity_verification(edge, &cube);

                amb_face = 5;
                edge = Self::interior_ambiguity(amb_face, s, &cube);
                inter_amb += Self::interior_ambiguity_verification(edge, &cube);

                inter_amb != 0
            },
            10 => {
                let amb_face = TEST_10[config][0].abs();
                let edge = Self::interior_ambiguity(amb_face, s, &cube);
                let inter_amb = Self::interior_ambiguity_verification(edge, &cube);

                inter_amb != 0
            },
            12 => {
                let mut amb_face = TEST_12[config][0].abs();
                let mut edge = Self::interior_ambiguity(amb_face, s, &cube);
                let mut inter_amb = Self::interior_ambiguity_verification(edge, &cube);
    
                amb_face = TEST_12[config][1].abs();
                edge = Self::interior_ambiguity(amb_face, s, &cube);
                inter_amb += Self::interior_ambiguity_verification(edge, &cube);
    
                inter_amb != 0
            },
            _ => {
                debug_assert!(false, "Invalid case {}", case);
                false
            },
        }
    }

    fn interior_ambiguity(face: i8, s: i8, cube: &[f32; 8]) -> i8 {
        let s = s as f32;
        let mut edge = 0;

        match face {
            1 | 3 => {
                if cube[1] * s > 0.0 && cube[7] * s > 0.0 {
                    edge = 4;
                }

                if cube[0] * s > 0.0 && cube[6] * s > 0.0 {
                    edge = 5;
                }

                if cube[3] * s > 0.0 && cube[5] * s > 0.0 {
                    edge = 6;
                }

                if cube[2] * s > 0.0 && cube[4] * s > 0.0 {
                    edge = 7;
                }
            },
            2 | 4 => {
                if cube[1] * s > 0.0 && cube[7] * s > 0.0 {
                    edge = 0;
                }

                if cube[2] * s > 0.0 && cube[4] * s > 0.0 {
                    edge = 1;
                }

                if cube[3] * s > 0.0 && cube[5] * s > 0.0 {
                    edge = 2;
                }

                if cube[0] * s > 0.0 && cube[6] * s > 0.0 {
                    edge = 3;
                }
            },
            5 | 6 | 0 => {
                if cube[0] * s > 0.0 && cube[6] * s > 0.0 {
                    edge = 8;
                }

                if cube[1] * s > 0.0 && cube[7] * s > 0.0 {
                    edge = 9;
                }

                if cube[2] * s > 0.0 && cube[4] * s > 0.0 {
                    edge = 10;
                }

                if cube[3] * s > 0.0 && cube[5] * s > 0.0 {
                    edge = 11;
                }
            },
            _ => {
                debug_assert!(false, "Invalid face {}", face);
            },
        };

        edge
    }

    fn interior_ambiguity_verification(edge: i8, cube: &[f32; 8]) -> i8 {    
        match edge {
            0 => {
                let a = (cube[0] - cube[1]) * (cube[7] - cube[6]) - (cube[4] - cube[5]) * (cube[3] - cube[2]);
                let b = cube[6] * (cube[0] - cube[1]) + cube[1] * (cube[7] - cube[6]) - cube[2] * (cube[4] - cube[5]) - cube[5] * (cube[3] - cube[2]);
        
                if a > 0.0 {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }
        
                let at = cube[1] + (cube[0] - cube[1]) * t;
                let bt = cube[5] + (cube[4] - cube[5]) * t;
                let ct = cube[6] + (cube[7] - cube[6]) * t;
                let dt = cube[2] + (cube[3] - cube[2]) * t;
        
                let verify = at * ct - bt * dt;
        
                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }, 
            1 => {
                let a = (cube[3] - cube[2]) * (cube[4] - cube[5])
                        - (cube[0] - cube[1]) * (cube[7] - cube[6]);
                let b = cube[5] * (cube[3] - cube[2]) + cube[2] * (cube[4] - cube[5])
                        - cube[6] * (cube[0] - cube[1])
                        - cube[1] * (cube[7] - cube[6]);
        
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[2] + (cube[3] - cube[2]) * t;
                let Bt = cube[1] + (cube[0] - cube[1]) * t;
                let Ct = cube[5] + (cube[4] - cube[5]) * t;
                let Dt = cube[6] + (cube[7] - cube[6]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            2 => {
                let a = (cube[2] - cube[3]) * (cube[5] - cube[4])
                        - (cube[6] - cube[7]) * (cube[1] - cube[0]);
                let b = cube[4] * (cube[2] - cube[3]) + cube[3] * (cube[5] - cube[4])
                        - cube[0] * (cube[6] - cube[7])
                        - cube[7] * (cube[1] - cube[0]);
                
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[3] + (cube[2] - cube[3]) * t;
                let Bt = cube[7] + (cube[6] - cube[7]) * t;
                let Ct = cube[4] + (cube[5] - cube[4]) * t;
                let Dt = cube[0] + (cube[1] - cube[0]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            3 => {
                let a = (cube[1] - cube[0]) * (cube[6] - cube[7])
                        - (cube[2] - cube[3]) * (cube[5] - cube[4]);
                let b = cube[7] * (cube[1] - cube[0]) + cube[0] * (cube[6] - cube[7])
                        - cube[4] * (cube[2] - cube[3])
                        - cube[3] * (cube[5] - cube[4]);
                
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);

                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[0] + (cube[1] - cube[0]) * t;
                let Bt = cube[3] + (cube[2] - cube[3]) * t;
                let Ct = cube[7] + (cube[6] - cube[7]) * t;
                let Dt = cube[4] + (cube[5] - cube[4]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            4 => {
                let a = (cube[2] - cube[1]) * (cube[7] - cube[4])
                        - (cube[3] - cube[0]) * (cube[6] - cube[5]);
                let b = cube[4] * (cube[2] - cube[1]) + cube[1] * (cube[7] - cube[4])
                        - cube[5] * (cube[3] - cube[0])
                        - cube[0] * (cube[6] - cube[5]);
        
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[1] + (cube[2] - cube[1]) * t;
                let Bt = cube[0] + (cube[3] - cube[0]) * t;
                let Ct = cube[4] + (cube[7] - cube[4]) * t;
                let Dt = cube[5] + (cube[6] - cube[5]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            5 => {
                let a = (cube[3] - cube[0]) * (cube[6] - cube[5])
                        - (cube[2] - cube[1]) * (cube[7] - cube[4]);
                let b = cube[5] * (cube[3] - cube[0]) + cube[0] * (cube[6] - cube[5])
                        - cube[4] * (cube[2] - cube[1])
                        - cube[1] * (cube[7] - cube[4]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }

                let At = cube[0] + (cube[3] - cube[0]) * t;
                let Bt = cube[1] + (cube[2] - cube[1]) * t;
                let Ct = cube[5] + (cube[6] - cube[5]) * t;
                let Dt = cube[4] + (cube[7] - cube[4]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            6 => {
                let a = (cube[0] - cube[3]) * (cube[5] - cube[6])
                        - (cube[4] - cube[7]) * (cube[1] - cube[2]);
                let b = cube[6] * (cube[0] - cube[3]) + cube[3] * (cube[5] - cube[6])
                        - cube[2] * (cube[4] - cube[7])
                        - cube[7] * (cube[1] - cube[2]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }

                let At = cube[3] + (cube[0] - cube[3]) * t;
                let Bt = cube[7] + (cube[4] - cube[7]) * t;
                let Ct = cube[6] + (cube[5] - cube[6]) * t;
                let Dt = cube[2] + (cube[1] - cube[2]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            7 => {
                let a = (cube[1] - cube[2]) * (cube[4] - cube[7])
                        - (cube[0] - cube[3]) * (cube[5] - cube[6]);
                let b = cube[7] * (cube[1] - cube[2]) + cube[2] * (cube[4] - cube[7])
                        - cube[6] * (cube[0] - cube[3])
                        - cube[3] * (cube[5] - cube[6]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[2] + (cube[1] - cube[2]) * t;
                let Bt = cube[3] + (cube[0] - cube[3]) * t;
                let Ct = cube[7] + (cube[4] - cube[7]) * t;
                let Dt = cube[6] + (cube[5] - cube[6]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }

                if (verify < 0.0) {
                    return 1;
                }
            },
            8 => {
                let a = (cube[4] - cube[0]) * (cube[6] - cube[2])
                        - (cube[7] - cube[3]) * (cube[5] - cube[1]);
                let b = cube[2] * (cube[4] - cube[0]) + cube[0] * (cube[6] - cube[2])
                        - cube[1] * (cube[7] - cube[3])
                        - cube[3] * (cube[5] - cube[1]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[0] + (cube[4] - cube[0]) * t;
                let Bt = cube[3] + (cube[7] - cube[3]) * t;
                let Ct = cube[2] + (cube[6] - cube[2]) * t;
                let Dt = cube[1] + (cube[5] - cube[1]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }
                if (verify < 0.0) {
                    return 1;
                }
            },
            9 => {
                let a = (cube[5] - cube[1]) * (cube[7] - cube[3])
                        - (cube[4] - cube[0]) * (cube[6] - cube[2]);
                let b = cube[3] * (cube[5] - cube[1]) + cube[1] * (cube[7] - cube[3])
                        - cube[2] * (cube[4] - cube[0])
                        - cube[0] * (cube[6] - cube[2]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[1] + (cube[5] - cube[1]) * t;
                let Bt = cube[0] + (cube[4] - cube[0]) * t;
                let Ct = cube[3] + (cube[7] - cube[3]) * t;
                let Dt = cube[2] + (cube[6] - cube[2]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }
                if (verify < 0.0) {
                    return 1;
                }
            },
            10 => {
                let a = (cube[6] - cube[2]) * (cube[4] - cube[0])
                        - (cube[5] - cube[1]) * (cube[7] - cube[3]);
                let b = cube[0] * (cube[6] - cube[2]) + cube[2] * (cube[4] - cube[0])
                        - cube[3] * (cube[5] - cube[1])
                        - cube[1] * (cube[7] - cube[3]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[2] + (cube[6] - cube[2]) * t;
                let Bt = cube[1] + (cube[5] - cube[1]) * t;
                let Ct = cube[0] + (cube[4] - cube[0]) * t;
                let Dt = cube[3] + (cube[7] - cube[3]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }
                if (verify < 0.0) {
                    return 1;
                }
            },
            11 => {
                let a = (cube[7] - cube[3]) * (cube[5] - cube[1])
                        - (cube[6] - cube[2]) * (cube[4] - cube[0]);
                let b = cube[1] * (cube[7] - cube[3]) + cube[3] * (cube[5] - cube[1])
                        - cube[0] * (cube[6] - cube[2])
                        - cube[2] * (cube[4] - cube[0]);
                if (a > 0.0) {
                    return 1;
                }
        
                let t = -b / (2.0 * a);
                if (t < 0.0 || t > 1.0) {
                    return 1;
                }
        
                let At = cube[3] + (cube[7] - cube[3]) * t;
                let Bt = cube[2] + (cube[6] - cube[2]) * t;
                let Ct = cube[1] + (cube[5] - cube[1]) * t;
                let Dt = cube[0] + (cube[4] - cube[0]) * t;
        
                let verify = At * Ct - Bt * Dt;
        
                if (verify > 0.0) {
                    return 0;
                }
                if (verify < 0.0) {
                    return 1;
                }
            },
            _ => debug_assert!(false, "Invalid edge {}", edge)
        };
        
        debug_assert!(false, "Not reachable");
        0
    }

    fn c_vertex(&self, cube: &MCube<T::Value>) -> Vec3f {
        let intersections = [
            cube.interpolate(0, 1, self.grid),
            cube.interpolate(0, 5, self.grid),
            cube.interpolate(5, 4, self.grid),
            cube.interpolate(4, 0, self.grid),

            cube.interpolate(3, 2, self.grid),
            cube.interpolate(2, 6, self.grid),
            cube.interpolate(6, 7, self.grid),
            cube.interpolate(7, 3, self.grid),

            cube.interpolate(1, 2, self.grid),
            cube.interpolate(0, 3, self.grid),
            cube.interpolate(4, 7, self.grid),
            cube.interpolate(5, 6, self.grid),
        ];

        let count = intersections.iter().filter(|e| e.is_some()).count();
        let sum: Vec3f = intersections.into_iter()
            .filter_map(|e| e)
            .sum();

        sum / count as f32
    }
}

// fn print_edge<T:Debug + >(e: Edge, cube: &MCube<T>) {
//     let v1 = cube.vertex(e.v1 as usize);
//     let v2 = cube.vertex(e.v2 as usize);
    
//     println!("Edge: {:?}\t{:?}", v1.value, v2.value);
// }

#[derive(Debug)]
struct MCube<T: Debug + Copy + Into<f32>> {
    bits: u8,
    vertices: [Vertex<T>; 8],
}

impl<T: Debug + Copy + Into<f32>> MCube<T> {
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

    fn interpolate<TGrid: MarchingCubes<Value = T>>(&self, v1: usize, v2: usize, grid: &TGrid) -> Option<Vec3f> {
        let v1_inside = grid.is_inside(&self.vertices[v1].value);
        let v2_inside = grid.is_inside(&self.vertices[v2].value);

        if v1_inside == v2_inside {
            return None;
        }

        Some(grid.interpolate(&self.vertices[v1], &self.vertices[v2]))
    }

    #[inline]
    fn id(&self) -> u8 {
        self.bits
    }

    #[inline]
    fn v(&self, vertex: usize) -> &Vertex<T> {
        &self.vertices[vertex]
    }

    #[inline]
    fn val(&self, vertex: usize) -> f32 {
        self.vertices[vertex].value.into()
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
