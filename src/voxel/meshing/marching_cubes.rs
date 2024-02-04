use std::{fmt::Debug, ops::Index};

use nalgebra::Vector3;

use crate::{
    helpers::aliases::{Vec3f, Vec3i},
    voxel::utils::CUBE_OFFSETS, geometry::primitives::triangle3::Triangle3,
};

use super::lookup_table::*;

#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub index: Vec3i,
    pub value: f32,
}

pub trait MarchingCubes {
    type Value;

    fn cubes<T: FnMut(Vector3<isize>)>(&self, func: T);
    fn at(&self, idx: &Vector3<isize>) -> Option<f32>;
}

pub struct MarchingCubesMesher<'a, T: MarchingCubes> {
    grid: &'a T,
    vertices: Vec<Vector3<f32>>,
    voxel_size: f32,
    v12: Vec3f,
    cube: Cube,
    case: i8,
    config: usize,
}

impl<'a, T: MarchingCubes> MarchingCubesMesher<'a, T> {
    pub fn new(grid: &'a T, voxel_size: f32) -> Self {
        Self {
            grid,
            vertices: Vec::new(),
            v12: Vec3f::zeros(),
            cube: Default::default(),
            case: 0,
            config: 0,
            voxel_size,
        }
    }

    pub fn mesh(&mut self) -> Vec<Vector3<f32>> {
        self.grid.cubes(|i| self.handle_cube(i));
        self.vertices.clone()
    }

    fn handle_cube(&mut self, v: Vector3<isize>) {
        self.cube = match Cube::from_voxel(v, self.grid) {
            Some(cube) => cube,
            None => return,
        };

        let [cube_case, cube_config] = CASES[self.cube.id() as usize];
        self.case = cube_case;
        self.config = cube_config as usize;

        match self.case {
            0 => return,
            1 => self.add_faces(&TILING_EDGES_1[self.config]),
            2 => self.add_faces(&TILING_EDGES_2[self.config]),
            3 => {
                if self.test_face(TEST_3[self.config]) {
                    self.add_faces(&TILING_EDGES_3_2[self.config]);
                } else {
                    self.add_faces(&TILING_EDGES_3_1[self.config]);
                }
            }
            4 => {
                if self.test_interior(TEST_4[self.config]) {
                    self.add_faces(&TILING_EDGES_4_1[self.config]);
                } else {
                    self.add_faces(&TILING_EDGES_4_2[self.config]);
                }
            }
            5 => self.add_faces(&TILING_EDGES_5[self.config]),
            6 => {
                if self.test_face(TEST_6[self.config][0]) {
                    self.add_faces(&TILING_EDGES_6_2[self.config]);
                } else {
                    if self.test_interior(TEST_6[self.config][1]) {
                        self.add_faces(&TILING_EDGES_6_1_1[self.config]);
                    } else {
                        self.add_faces(&TILING_EDGES_6_1_2[self.config]);
                    }
                }
            }
            7 => {
                let mut subconfig = 0;

                if self.test_face(TEST_7[self.config][0]) {
                    subconfig += 1;
                }

                if self.test_face(TEST_7[self.config][1]) {
                    subconfig += 2;
                }

                if self.test_face(TEST_7[self.config][2]) {
                    subconfig += 4;
                }

                match subconfig {
                    0 => self.add_faces(&TILING_EDGES_7_1[self.config]),
                    1 => self.add_faces(&TILING_EDGES_7_2[self.config][0]),
                    2 => self.add_faces(&TILING_EDGES_7_2[self.config][1]),
                    3 => {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_7_3[self.config][0]);
                    }
                    4 => self.add_faces(&TILING_EDGES_7_2[self.config][2]),
                    5 => {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_7_3[self.config][1]);
                    }
                    6 => {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_7_3[self.config][2]);
                    }
                    7 => {
                        if self.test_interior(TEST_7[self.config][3]) {
                            self.add_faces(&TILING_EDGES_7_4_1[self.config]);
                        } else {
                            self.add_faces(&TILING_EDGES_7_4_2[self.config]);
                        }
                    }
                    _ => debug_assert!(
                        false,
                        "Marching cubes: impossible case {}, config {}",
                        self.case, self.config
                    ),
                }
            }
            8 => self.add_faces(&TILING_EDGES_8[self.config]),
            9 => self.add_faces(&TILING_EDGES_9[self.config]),
            10 => {
                if self.test_face(TEST_10[self.config][0]) {
                    if self.test_face(TEST_10[self.config][1]) {
                        if self.test_interior(-TEST_10[self.config][2]) {
                            self.add_faces(&TILING_EDGES_10_1_1_[self.config]);
                        } else {
                            self.add_faces(&TILING_EDGES_10_1_2[5 - self.config]);
                        }
                    } else {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_10_2[self.config]);
                    }
                } else {
                    if self.test_face(TEST_10[self.config][1]) {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_10_2_[self.config]);
                    } else {
                        if self.test_interior(TEST_10[self.config][2]) {
                            self.add_faces(&TILING_EDGES_10_1_1[self.config]);
                        } else {
                            self.add_faces(&TILING_EDGES_10_1_2[self.config]);
                        }
                    }
                }
            }
            11 => self.add_faces(&TILING_EDGES_11[self.config]),
            12 => {
                if self.test_face(TEST_12[self.config][0]) {
                    if self.test_face(TEST_12[self.config][1]) {
                        if self.test_interior(-TEST_12[self.config][2]) {
                            self.add_faces(&TILING_EDGES_12_1_1_[self.config]);
                        } else {
                            self.add_faces(&TILING_EDGES_12_1_2[23 - self.config]);
                        }
                    } else {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_12_2[self.config]);
                    }
                } else {
                    if self.test_face(TEST_12[self.config][1]) {
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_12_2_[self.config]);
                    } else {
                        if self.test_interior(TEST_12[self.config][2]) {
                            self.add_faces(&TILING_EDGES_12_1_1[self.config]);
                        } else {
                            self.add_faces(&TILING_EDGES_12_1_2[self.config]);
                        }
                    }
                }
            }
            13 => {
                let mut subconfig = 0;

                if self.test_face(TEST_13[self.config][0]) {
                    subconfig += 1;
                }

                if self.test_face(TEST_13[self.config][1]) {
                    subconfig += 2;
                }

                if self.test_face(TEST_13[self.config][2]) {
                    subconfig += 4;
                }

                if self.test_face(TEST_13[self.config][3]) {
                    subconfig += 8;
                }

                if self.test_face(TEST_13[self.config][4]) {
                    subconfig += 16;
                }

                if self.test_face(TEST_13[self.config][5]) {
                    subconfig += 32;
                }

                let offset = SUB_CONFIG_13[subconfig] as usize; // -1 is not matched below so we can safely cast to usize

                match SUB_CONFIG_13[subconfig] {
                    0 => {
                        // 13.1
                        self.add_faces(&TILING_EDGES_13_1[self.config]);
                    }
                    1..=6 => {
                        // 13.2
                        self.add_faces(&TILING_EDGES_13_2[self.config][offset - 1]);
                    }
                    7..=18 => {
                        // 13.3
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_13_3[self.config][offset - 7]);
                    }
                    19..=22 => {
                        // 13.3
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_13_4[self.config][offset - 19]);
                    }
                    23..=26 => {
                        // 13.5
                        if self.config == 0 {
                            if self.interior_test_case13() {
                                self.add_faces(&TILING_EDGES_13_5_1[0][offset - 23]);
                            } else {
                                self.add_faces(&TILING_EDGES_13_5_2[1][offset - 23]);
                            }
                        } else {
                            if self.interior_test_case13() {
                                self.add_faces(&TILING_EDGES_13_5_1[1][offset - 23]);
                            } else {
                                self.add_faces(&TILING_EDGES_13_5_2[0][offset - 23]);
                            }
                        }
                    }
                    27..=38 => {
                        // 13.3
                        self.compute_c_vertex();
                        self.add_faces(&TILING_EDGES_13_3_[self.config][offset - 27]);
                    }
                    39..=44 => {
                        // 13.2
                        self.add_faces(&TILING_EDGES_13_2_[self.config][offset - 39]);
                    }
                    45 => {
                        // 13.1
                        self.add_faces(&TILING_EDGES_13_1_[self.config]);
                    }
                    _ => debug_assert!(
                        false,
                        "Marching Cubes: Impossible case {}, config {}",
                        self.case, self.config
                    ),
                }
            }
            14 => self.add_faces(&TILING_EDGES_14[self.config]),
            _ => debug_assert!(false, "Marching cubes: impossible case {}", self.case),
        }
    }

    fn add_faces(&mut self, edges: &[Edge]) {
        for i in (0..edges.len()).step_by(3) {
            let e1 = edges[i];
            let e3 = edges[i + 1];
            let e2 = edges[i + 2];

            let v1 = self.intersection(&e1);
            let v2 = self.intersection(&e2);
            let v3 = self.intersection(&e3);

            if Triangle3::is_degenerate(&v1, &v2, &v3) {
                continue;
            }

            self.vertices.push(v1 * self.voxel_size);
            self.vertices.push(v2 * self.voxel_size);
            self.vertices.push(v3 * self.voxel_size);
        }
    }

    #[inline]
    fn intersection(&self, edge: &Edge) -> Vec3f {
        if edge.is_special_edge() {
            return self.v12;
        }

        self.cube.interpolate(edge.v1 as usize, edge.v2 as usize)
    }

    fn interior_test_case13(&self) -> bool {
        let a = (self.cube[0] - self.cube[1]) * (self.cube[7] - self.cube[6])
            - (self.cube[4] - self.cube[5]) * (self.cube[3] - self.cube[2]);
        let b = self.cube[6] * (self.cube[0] - self.cube[1])
            + self.cube[1] * (self.cube[7] - self.cube[6])
            - self.cube[2] * (self.cube[4] - self.cube[5])
            - self.cube[5] * (self.cube[3] - self.cube[2]);

        let c = self.cube[1] * self.cube[6] - self.cube[5] * self.cube[2];

        let delta = b * b - 4.0 * a * c;

        let t1 = (-b + delta.sqrt()) / (a + a);
        let t2 = (-b - delta.sqrt()) / (a + a);

        if t1 < 1.0 && t1 > 0.0 && t2 < 1.0 && t2 > 0.0 {
            let a_t1 = self.cube[1] + (self.cube[0] - self.cube[1]) * t1;
            let b_t1 = self.cube[5] + (self.cube[4] - self.cube[5]) * t1;
            let c_t1 = self.cube[6] + (self.cube[7] - self.cube[6]) * t1;
            let d_t1 = self.cube[2] + (self.cube[3] - self.cube[2]) * t1;

            let x1 = (a_t1 - d_t1) / (a_t1 + c_t1 - b_t1 - d_t1);
            let y1 = (a_t1 - b_t1) / (a_t1 + c_t1 - b_t1 - d_t1);

            let a_t2 = self.cube[1] + (self.cube[0] - self.cube[1]) * t2;
            let b_t2 = self.cube[5] + (self.cube[4] - self.cube[5]) * t2;
            let c_t2 = self.cube[6] + (self.cube[7] - self.cube[6]) * t2;
            let d_t2 = self.cube[2] + (self.cube[3] - self.cube[2]) * t2;

            let x2 = (a_t2 - d_t2) / (a_t2 + c_t2 - b_t2 - d_t2);
            let y2 = (a_t2 - b_t2) / (a_t2 + c_t2 - b_t2 - d_t2);

            if x1 < 1.0
                && x1 > 0.0
                && x2 < 1.0
                && x2 > 0.0
                && y1 < 1.0
                && y1 > 0.0
                && y2 < 1.0
                && y2 > 0.0
            {
                false
            } else {
                true
            }
        } else {
            true
        }
    }

    fn test_face(&self, face: i8) -> bool {
        let (a, b, c, d) = match face {
            -1 | 1 => (self.cube[0], self.cube[4], self.cube[5], self.cube[1]),
            -2 | 2 => (self.cube[1], self.cube[5], self.cube[6], self.cube[2]),
            -3 | 3 => (self.cube[2], self.cube[6], self.cube[7], self.cube[3]),
            -4 | 4 => (self.cube[3], self.cube[7], self.cube[4], self.cube[0]),
            -5 | 5 => (self.cube[0], self.cube[3], self.cube[2], self.cube[1]),
            -6 | 6 => (self.cube[4], self.cube[7], self.cube[6], self.cube[5]),
            _ => {
                debug_assert!(false, "Invalid face {}", face);
                return false;
            }
        };

        let val = a * c - b * d;

        if val.abs() < f32::EPSILON {
            return face >= 0;
        }

        face as f32 * a * val >= 0.0
    }

    fn test_interior(&self, face: i8) -> bool {
        match self.case {
            4 => {
                let mut amb_face = 1;
                let mut edge = self.interior_ambiguity(amb_face, face);
                let mut inter_amb = self.interior_ambiguity_verification(edge);

                amb_face = 2;
                edge = self.interior_ambiguity(amb_face, face);
                inter_amb += self.interior_ambiguity_verification(edge);

                amb_face = 5;
                edge = self.interior_ambiguity(amb_face, face);
                inter_amb += self.interior_ambiguity_verification(edge);

                inter_amb != 0
            }
            6 => {
                let amb_face = TEST_6[self.config][0].abs();
                let edge = self.interior_ambiguity(amb_face, face);
                let inter_amb = self.interior_ambiguity_verification(edge);

                inter_amb != 0
            }
            7 => {
                let s = -face;

                let mut amb_face = 1;
                let mut edge = self.interior_ambiguity(amb_face, s);
                let mut inter_amb = self.interior_ambiguity_verification(edge);

                amb_face = 2;
                edge = self.interior_ambiguity(amb_face, s);
                inter_amb += self.interior_ambiguity_verification(edge);

                amb_face = 5;
                edge = self.interior_ambiguity(amb_face, s);
                inter_amb += self.interior_ambiguity_verification(edge);

                inter_amb != 0
            }
            10 => {
                let amb_face = TEST_10[self.config][0].abs();
                let edge = self.interior_ambiguity(amb_face, face);
                let inter_amb = self.interior_ambiguity_verification(edge);

                inter_amb != 0
            }
            12 => {
                let mut amb_face = TEST_12[self.config][0].abs();
                let mut edge = self.interior_ambiguity(amb_face, face);
                let mut inter_amb = self.interior_ambiguity_verification(edge);

                amb_face = TEST_12[self.config][1].abs();
                edge = self.interior_ambiguity(amb_face, face);
                inter_amb += self.interior_ambiguity_verification(edge);

                inter_amb != 0
            }
            _ => {
                debug_assert!(false, "Invalid case {}", self.case);
                false
            }
        }
    }

    fn interior_ambiguity(&self, amb_face: i8, face: i8) -> i8 {
        let face = face as f32;
        let mut edge = 0;

        match amb_face {
            1 | 3 => {
                if self.cube[1] * face > 0.0 && self.cube[7] * face > 0.0 {
                    edge = 4;
                }

                if self.cube[0] * face > 0.0 && self.cube[6] * face > 0.0 {
                    edge = 5;
                }

                if self.cube[3] * face > 0.0 && self.cube[5] * face > 0.0 {
                    edge = 6;
                }

                if self.cube[2] * face > 0.0 && self.cube[4] * face > 0.0 {
                    edge = 7;
                }
            }
            2 | 4 => {
                if self.cube[1] * face > 0.0 && self.cube[7] * face > 0.0 {
                    edge = 0;
                }

                if self.cube[2] * face > 0.0 && self.cube[4] * face > 0.0 {
                    edge = 1;
                }

                if self.cube[3] * face > 0.0 && self.cube[5] * face > 0.0 {
                    edge = 2;
                }

                if self.cube[0] * face > 0.0 && self.cube[6] * face > 0.0 {
                    edge = 3;
                }
            }
            5 | 6 | 0 => {
                if self.cube[0] * face > 0.0 && self.cube[6] * face > 0.0 {
                    edge = 8;
                }

                if self.cube[1] * face > 0.0 && self.cube[7] * face > 0.0 {
                    edge = 9;
                }

                if self.cube[2] * face > 0.0 && self.cube[4] * face > 0.0 {
                    edge = 10;
                }

                if self.cube[3] * face > 0.0 && self.cube[5] * face > 0.0 {
                    edge = 11;
                }
            }
            _ => {
                debug_assert!(false, "Invalid face {}", amb_face);
            }
        };

        edge
    }

    fn interior_ambiguity_verification(&self, edge: i8) -> i8 {
        match edge {
            0 => {
                let a = (self.cube[0] - self.cube[1]) * (self.cube[7] - self.cube[6])
                    - (self.cube[4] - self.cube[5]) * (self.cube[3] - self.cube[2]);
                let b = self.cube[6] * (self.cube[0] - self.cube[1])
                    + self.cube[1] * (self.cube[7] - self.cube[6])
                    - self.cube[2] * (self.cube[4] - self.cube[5])
                    - self.cube[5] * (self.cube[3] - self.cube[2]);

                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let at = self.cube[1] + (self.cube[0] - self.cube[1]) * t;
                let bt = self.cube[5] + (self.cube[4] - self.cube[5]) * t;
                let ct = self.cube[6] + (self.cube[7] - self.cube[6]) * t;
                let dt = self.cube[2] + (self.cube[3] - self.cube[2]) * t;

                let verify = at * ct - bt * dt;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            1 => {
                let a = (self.cube[3] - self.cube[2]) * (self.cube[4] - self.cube[5])
                    - (self.cube[0] - self.cube[1]) * (self.cube[7] - self.cube[6]);
                let b = self.cube[5] * (self.cube[3] - self.cube[2])
                    + self.cube[2] * (self.cube[4] - self.cube[5])
                    - self.cube[6] * (self.cube[0] - self.cube[1])
                    - self.cube[1] * (self.cube[7] - self.cube[6]);

                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[2] + (self.cube[3] - self.cube[2]) * t;
                let b_t = self.cube[1] + (self.cube[0] - self.cube[1]) * t;
                let c_t = self.cube[5] + (self.cube[4] - self.cube[5]) * t;
                let d_t = self.cube[6] + (self.cube[7] - self.cube[6]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            2 => {
                let a = (self.cube[2] - self.cube[3]) * (self.cube[5] - self.cube[4])
                    - (self.cube[6] - self.cube[7]) * (self.cube[1] - self.cube[0]);
                let b = self.cube[4] * (self.cube[2] - self.cube[3])
                    + self.cube[3] * (self.cube[5] - self.cube[4])
                    - self.cube[0] * (self.cube[6] - self.cube[7])
                    - self.cube[7] * (self.cube[1] - self.cube[0]);

                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);

                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[3] + (self.cube[2] - self.cube[3]) * t;
                let b_t = self.cube[7] + (self.cube[6] - self.cube[7]) * t;
                let c_t = self.cube[4] + (self.cube[5] - self.cube[4]) * t;
                let d_t = self.cube[0] + (self.cube[1] - self.cube[0]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            3 => {
                let a = (self.cube[1] - self.cube[0]) * (self.cube[6] - self.cube[7])
                    - (self.cube[2] - self.cube[3]) * (self.cube[5] - self.cube[4]);
                let b = self.cube[7] * (self.cube[1] - self.cube[0])
                    + self.cube[0] * (self.cube[6] - self.cube[7])
                    - self.cube[4] * (self.cube[2] - self.cube[3])
                    - self.cube[3] * (self.cube[5] - self.cube[4]);

                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);

                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[0] + (self.cube[1] - self.cube[0]) * t;
                let b_t = self.cube[3] + (self.cube[2] - self.cube[3]) * t;
                let c_t = self.cube[7] + (self.cube[6] - self.cube[7]) * t;
                let d_t = self.cube[4] + (self.cube[5] - self.cube[4]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            4 => {
                let a = (self.cube[2] - self.cube[1]) * (self.cube[7] - self.cube[4])
                    - (self.cube[3] - self.cube[0]) * (self.cube[6] - self.cube[5]);
                let b = self.cube[4] * (self.cube[2] - self.cube[1])
                    + self.cube[1] * (self.cube[7] - self.cube[4])
                    - self.cube[5] * (self.cube[3] - self.cube[0])
                    - self.cube[0] * (self.cube[6] - self.cube[5]);

                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[1] + (self.cube[2] - self.cube[1]) * t;
                let b_t = self.cube[0] + (self.cube[3] - self.cube[0]) * t;
                let c_t = self.cube[4] + (self.cube[7] - self.cube[4]) * t;
                let d_t = self.cube[5] + (self.cube[6] - self.cube[5]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            5 => {
                let a = (self.cube[3] - self.cube[0]) * (self.cube[6] - self.cube[5])
                    - (self.cube[2] - self.cube[1]) * (self.cube[7] - self.cube[4]);
                let b = self.cube[5] * (self.cube[3] - self.cube[0])
                    + self.cube[0] * (self.cube[6] - self.cube[5])
                    - self.cube[4] * (self.cube[2] - self.cube[1])
                    - self.cube[1] * (self.cube[7] - self.cube[4]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[0] + (self.cube[3] - self.cube[0]) * t;
                let b_t = self.cube[1] + (self.cube[2] - self.cube[1]) * t;
                let c_t = self.cube[5] + (self.cube[6] - self.cube[5]) * t;
                let d_t = self.cube[4] + (self.cube[7] - self.cube[4]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            6 => {
                let a = (self.cube[0] - self.cube[3]) * (self.cube[5] - self.cube[6])
                    - (self.cube[4] - self.cube[7]) * (self.cube[1] - self.cube[2]);
                let b = self.cube[6] * (self.cube[0] - self.cube[3])
                    + self.cube[3] * (self.cube[5] - self.cube[6])
                    - self.cube[2] * (self.cube[4] - self.cube[7])
                    - self.cube[7] * (self.cube[1] - self.cube[2]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[3] + (self.cube[0] - self.cube[3]) * t;
                let b_t = self.cube[7] + (self.cube[4] - self.cube[7]) * t;
                let c_t = self.cube[6] + (self.cube[5] - self.cube[6]) * t;
                let d_t = self.cube[2] + (self.cube[1] - self.cube[2]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            7 => {
                let a = (self.cube[1] - self.cube[2]) * (self.cube[4] - self.cube[7])
                    - (self.cube[0] - self.cube[3]) * (self.cube[5] - self.cube[6]);
                let b = self.cube[7] * (self.cube[1] - self.cube[2])
                    + self.cube[2] * (self.cube[4] - self.cube[7])
                    - self.cube[6] * (self.cube[0] - self.cube[3])
                    - self.cube[3] * (self.cube[5] - self.cube[6]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[2] + (self.cube[1] - self.cube[2]) * t;
                let b_t = self.cube[3] + (self.cube[0] - self.cube[3]) * t;
                let c_t = self.cube[7] + (self.cube[4] - self.cube[7]) * t;
                let d_t = self.cube[6] + (self.cube[5] - self.cube[6]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }

                if verify < 0.0 {
                    return 1;
                }
            }
            8 => {
                let a = (self.cube[4] - self.cube[0]) * (self.cube[6] - self.cube[2])
                    - (self.cube[7] - self.cube[3]) * (self.cube[5] - self.cube[1]);
                let b = self.cube[2] * (self.cube[4] - self.cube[0])
                    + self.cube[0] * (self.cube[6] - self.cube[2])
                    - self.cube[1] * (self.cube[7] - self.cube[3])
                    - self.cube[3] * (self.cube[5] - self.cube[1]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[0] + (self.cube[4] - self.cube[0]) * t;
                let b_t = self.cube[3] + (self.cube[7] - self.cube[3]) * t;
                let c_t = self.cube[2] + (self.cube[6] - self.cube[2]) * t;
                let d_t = self.cube[1] + (self.cube[5] - self.cube[1]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }
                if verify < 0.0 {
                    return 1;
                }
            }
            9 => {
                let a = (self.cube[5] - self.cube[1]) * (self.cube[7] - self.cube[3])
                    - (self.cube[4] - self.cube[0]) * (self.cube[6] - self.cube[2]);
                let b = self.cube[3] * (self.cube[5] - self.cube[1])
                    + self.cube[1] * (self.cube[7] - self.cube[3])
                    - self.cube[2] * (self.cube[4] - self.cube[0])
                    - self.cube[0] * (self.cube[6] - self.cube[2]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[1] + (self.cube[5] - self.cube[1]) * t;
                let b_t = self.cube[0] + (self.cube[4] - self.cube[0]) * t;
                let c_t = self.cube[3] + (self.cube[7] - self.cube[3]) * t;
                let d_t = self.cube[2] + (self.cube[6] - self.cube[2]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }
                if verify < 0.0 {
                    return 1;
                }
            }
            10 => {
                let a = (self.cube[6] - self.cube[2]) * (self.cube[4] - self.cube[0])
                    - (self.cube[5] - self.cube[1]) * (self.cube[7] - self.cube[3]);
                let b = self.cube[0] * (self.cube[6] - self.cube[2])
                    + self.cube[2] * (self.cube[4] - self.cube[0])
                    - self.cube[3] * (self.cube[5] - self.cube[1])
                    - self.cube[1] * (self.cube[7] - self.cube[3]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[2] + (self.cube[6] - self.cube[2]) * t;
                let b_t = self.cube[1] + (self.cube[5] - self.cube[1]) * t;
                let c_t = self.cube[0] + (self.cube[4] - self.cube[0]) * t;
                let d_t = self.cube[3] + (self.cube[7] - self.cube[3]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }
                if verify < 0.0 {
                    return 1;
                }
            }
            11 => {
                let a = (self.cube[7] - self.cube[3]) * (self.cube[5] - self.cube[1])
                    - (self.cube[6] - self.cube[2]) * (self.cube[4] - self.cube[0]);
                let b = self.cube[1] * (self.cube[7] - self.cube[3])
                    + self.cube[3] * (self.cube[5] - self.cube[1])
                    - self.cube[0] * (self.cube[6] - self.cube[2])
                    - self.cube[2] * (self.cube[4] - self.cube[0]);
                if a > 0.0 {
                    return 1;
                }

                let t = -b / (2.0 * a);
                if t < 0.0 || t > 1.0 {
                    return 1;
                }

                let a_t = self.cube[3] + (self.cube[7] - self.cube[3]) * t;
                let b_t = self.cube[2] + (self.cube[6] - self.cube[2]) * t;
                let c_t = self.cube[1] + (self.cube[5] - self.cube[1]) * t;
                let d_t = self.cube[0] + (self.cube[4] - self.cube[0]) * t;

                let verify = a_t * c_t - b_t * d_t;

                if verify > 0.0 {
                    return 0;
                }
                if verify < 0.0 {
                    return 1;
                }
            }
            _ => debug_assert!(false, "Invalid edge {}", edge),
        };

        debug_assert!(false, "Not reachable");
        0
    }

    fn compute_c_vertex(&mut self) {
        let intersections = [
            self.cube.interpolate_checked(0, 1),
            self.cube.interpolate_checked(0, 5),
            self.cube.interpolate_checked(5, 4),
            self.cube.interpolate_checked(4, 0),
            self.cube.interpolate_checked(3, 2),
            self.cube.interpolate_checked(2, 6),
            self.cube.interpolate_checked(6, 7),
            self.cube.interpolate_checked(7, 3),
            self.cube.interpolate_checked(1, 2),
            self.cube.interpolate_checked(0, 3),
            self.cube.interpolate_checked(4, 7),
            self.cube.interpolate_checked(5, 6),
        ];

        let count = intersections.iter().filter(|e| e.is_some()).count();
        let sum: Vec3f = intersections.into_iter().filter_map(|e| e).sum();

        self.v12 = sum / count as f32
    }
}

#[derive(Debug)]
struct Cube {
    id: u8,
    vertices: [Vertex; 8],
}

impl Default for Cube {
    #[inline]
    fn default() -> Self {
        let v = Vertex {
            index: Vec3i::zeros(),
            value: 0.0,
        };
        Self {
            id: 0,
            vertices: [v; 8],
        }
    }
}

impl Index<usize> for Cube {
    type Output = f32;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.vertices[index].value
    }
}

impl Cube {
    fn from_voxel<TGrid: MarchingCubes>(voxel: Vector3<isize>, grid: &TGrid) -> Option<Self> {
        let mut cube: Self = Default::default();

        let vertex_indices = CUBE_OFFSETS.map(|off| voxel + off);

        for i in 0..vertex_indices.len() {
            let index = vertex_indices[i];
            let value = match grid.at(&index) {
                Some(v) => v,
                None => return None,
            };

            if value < 0.0 {
                cube.id |= 1 << i;
            }

            cube.vertices[i] = Vertex { index, value };
        }

        Some(cube)
    }

    fn interpolate_checked(&self, v1: usize, v2: usize) -> Option<Vec3f> {
        let v1_val = &self.vertices[v1].value;
        let v2_val = &self.vertices[v2].value;

        if v1_val * v2_val > 0.0 {
            return None;
        }

        Some(self.interpolate(v1, v2))
    }

    fn interpolate(&self, v1: usize, v2: usize) -> Vec3f {
        let v1 = self.vertices[v1];
        let v2 = self.vertices[v2];

        let v1_val = v1.value.abs();
        let v2_val = v2.value.abs();
        let l = v1_val + v2_val;
        let dir = v2.index - v1.index;
        let t = v1.index.cast() + dir.cast() * v1_val / l;

        debug_assert!(
            t.iter().all(|v| v.is_finite()),
            "Marching cubes: intersection coordinates are not finite: {:?}",
            t
        );

        t
    }

    #[inline]
    fn id(&self) -> u8 {
        self.id
    }
}
