use std::{fmt::Debug, ops::Index};

use crate::{
    geometry::primitives::triangle3::Triangle3,
    helpers::aliases::{Vec3, Vec3f, Vec3i},
    voxel::{utils::CUBE_OFFSETS, Accessor, Sdf, SdfGrid, Tile, TreeNode, Visitor},
};

use super::lookup_table::*;

///
/// Corrected marching cubes 33.
/// 
/// Based on article: ["Practical considerations on Marching Cubes 33 topological correctness"](https://www.sci.utah.edu/~etiene/pdf/mc33.pdf)
/// 
pub struct MarchingCubesMesher {
    vertices: Vec<Vec3f>,
    voxel_size: f32,
    v12: Vec3f,
    cube: Cube,
    case: i8,
    config: usize,
    x_int: Box<<SdfGrid as TreeNode>::As<f32>>,
    y_int: Box<<SdfGrid as TreeNode>::As<f32>>,
    z_int: Box<<SdfGrid as TreeNode>::As<f32>>,
}

impl MarchingCubesMesher {
    #[inline]
    pub fn with_voxel_size(mut self, size: f32) -> Self {
        self.voxel_size = size;
        self
    }

    #[inline]
    pub fn set_voxel_size(&mut self, size: f32) -> &mut Self {
        self.voxel_size = size;
        self
    }

    pub fn mesh(&mut self, sdf: Sdf) -> Vec<Vec3f> {
        self.clear();

        let mut compute_intersections = ComputeEdgeIntersections {
            grid: sdf.grid(),
            x_int: self.x_int.as_mut(),
            y_int: self.y_int.as_mut(),
            z_int: self.z_int.as_mut(),
        };

        sdf.grid().visit_leafs(&mut compute_intersections);

        let mut cubes_visitor = CubesVisitor {
            grid: sdf.grid(),
            mc: self,
        };

        sdf.grid().visit_leafs(&mut cubes_visitor);

        // println!("Edges: {:?}", self.edges.values());

        self.vertices.clone()
    }

    fn clear(&mut self) {
        self.vertices.clear();
        self.x_int.clear();
        self.y_int.clear();
        self.z_int.clear();
    }

    fn handle_cube(&mut self, cube: Option<Cube>) {
        self.cube = match cube {
            Some(c) => c,
            None => return,
        };
        let [cube_case, cube_config] = CASES[self.cube.id as usize];
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

            let (v1, v2, v3) = match (
                self.intersection(e1),
                self.intersection(e2),
                self.intersection(e3),
            ) {
                (Some(v1), Some(v2), Some(v3)) => (v1, v2, v3),
                _ => {
                    debug_assert!(false, "Marching cubes: intersection should exist");
                    continue;
                }
            };

            if Triangle3::is_degenerate(&v1, &v2, &v3) {
                continue;
            }

            self.vertices.push(v1 * self.voxel_size);
            self.vertices.push(v2 * self.voxel_size);
            self.vertices.push(v3 * self.voxel_size);
        }
    }

    #[inline]
    fn intersection(&self, edge: Edge) -> Option<Vec3f> {
        if edge.is_special_edge() {
            return Some(self.v12);
        }

        let idx = &self.cube.vertices[edge.v1 as usize].index;

        let int = match edge.dir() {
            EdgeDir::X => Vec3f::new(*self.x_int.at(idx)?, idx.y as f32, idx.z as f32),
            EdgeDir::Y => Vec3f::new(idx.x as f32, *self.y_int.at(idx)?, idx.z as f32),
            EdgeDir::Z => Vec3f::new(idx.x as f32, idx.y as f32, *self.z_int.at(idx)?),
        };

        Some(int)
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
            self.intersection(Edge { v1: 0, v2: 1 }),
            self.intersection(Edge { v1: 1, v2: 2 }),
            self.intersection(Edge { v1: 3, v2: 2 }),
            self.intersection(Edge { v1: 0, v2: 3 }),
            self.intersection(Edge { v1: 4, v2: 5 }),
            self.intersection(Edge { v1: 5, v2: 6 }),
            self.intersection(Edge { v1: 7, v2: 6 }),
            self.intersection(Edge { v1: 4, v2: 7 }),
            self.intersection(Edge { v1: 0, v2: 4 }),
            self.intersection(Edge { v1: 1, v2: 5 }),
            self.intersection(Edge { v1: 2, v2: 6 }),
            self.intersection(Edge { v1: 3, v2: 7 }),
            self.intersection(Edge { v1: 0, v2: 0 }),
        ];

        let count = intersections.iter().filter(|e| e.is_some()).count();
        let sum: Vec3f = intersections.into_iter().filter_map(|e| e).sum();

        self.v12 = sum / count as f32
    }
}

impl Default for MarchingCubesMesher {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            v12: Vec3f::zeros(),
            cube: Default::default(),
            case: 0,
            config: 0,
            voxel_size: 1.0,
            x_int: <SdfGrid as TreeNode>::As::<f32>::empty(Vec3::zeros()),
            y_int: <SdfGrid as TreeNode>::As::<f32>::empty(Vec3::zeros()),
            z_int: <SdfGrid as TreeNode>::As::<f32>::empty(Vec3::zeros()),
        }
    }
}

struct CubesVisitor<'a> {
    grid: &'a SdfGrid,
    mc: &'a mut MarchingCubesMesher,
}

impl<'a> CubesVisitor<'a> {
    #[inline]
    fn cube(&self, voxel: Vec3i) -> Option<Cube> {
        Cube::from_voxel(voxel, self.grid)
    }
}

impl<T: TreeNode<Value = f32>> Visitor<T> for CubesVisitor<'_> {
    fn tile(&mut self, tile: Tile<T::Value>) {
        let o = tile.origin;
        let s = tile.size;

        // Test only boundary voxels
        for i in 0..s {
            for j in 0..s {
                let left = o + Vec3::new(0, i, j).cast();
                let right = o + Vec3::new(tile.size - 1, i, j).cast();

                let top = o + Vec3::new(i, j, tile.size - 1).cast();
                let bottom = o + Vec3::new(i, j, 0).cast();

                let front = o + Vec3::new(i, tile.size - 1, j).cast();
                let back = o + Vec3::new(i, 0, j).cast();

                self.mc.handle_cube(self.cube(left));
                self.mc.handle_cube(self.cube(right));
                self.mc.handle_cube(self.cube(top));
                self.mc.handle_cube(self.cube(bottom));
                self.mc.handle_cube(self.cube(front));
                self.mc.handle_cube(self.cube(back));
            }
        }
    }

    fn dense(&mut self, dense: &T) {
        let min = dense.origin();
        let size = T::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);

        for x in min.x..max.x {
            for y in min.y..max.y {
                for z in min.z..max.z {
                    let voxel_idx = Vec3i::new(x, y, z);
                    let cube = self.cube(voxel_idx);
                    self.mc.handle_cube(cube);
                }
            }
        }
    }
}

struct ComputeEdgeIntersections<'a, T: TreeNode<Value = f32>> {
    grid: &'a T,
    x_int: &'a mut <SdfGrid as TreeNode>::As<f32>,
    y_int: &'a mut <SdfGrid as TreeNode>::As<f32>,
    z_int: &'a mut <SdfGrid as TreeNode>::As<f32>,
}

impl<'a, T: TreeNode<Value = f32>> ComputeEdgeIntersections<'a, T> {
    fn compute_intersection(&mut self, v1: &Vec3i, v2: &Vec3i, dir: EdgeDir) {
        if self.x_int.at(v2).is_some() {
            return;
        }

        let (v1_val, v2_val) = match (self.grid.at(v1), self.grid.at(v2)) {
            (Some(v1), Some(v2)) => (*v1, *v2),
            _ => return,
        };

        if v1_val * v2_val > 0.0 {
            return;
        }

        let v1_val_a = v1_val.abs().max(MIN_ABS_VERTEX_VALUE);
        let v2_val_a = v2_val.abs().max(MIN_ABS_VERTEX_VALUE);
        let l = v1_val_a + v2_val_a;
        let t = v1_val_a / l;

        match dir {
            EdgeDir::X => {
                let int = v1.x as f32 + t;
                self.x_int.insert(v1, int);
            }
            EdgeDir::Y => {
                let int = v1.y as f32 + t;
                self.y_int.insert(v1, int);
            }
            EdgeDir::Z => {
                let int = v1.z as f32 + t;
                self.z_int.insert(v1, int);
            }
        };
    }
}

impl<'a, T: TreeNode<Value = f32>> Visitor<T::Leaf> for ComputeEdgeIntersections<'a, T> {
    fn tile(&mut self, _tile: Tile<T::Value>) {
        todo!("Marching cubes: support for tiles");
    }

    fn dense(&mut self, dense: &T::Leaf) {
        let min = dense.origin();
        let size = T::Leaf::resolution() as isize;
        let max = Vec3i::new(min.x + size, min.y + size, min.z + size);

        for x in min.x..max.x {
            for y in min.y..max.y {
                for z in min.z..max.z {
                    let v = Vec3i::new(x, y, z);

                    let mut v1 = v;
                    v1.x += 1;
                    self.compute_intersection(&v, &v1, EdgeDir::X);

                    let mut v2 = v;
                    v2.y += 1;
                    self.compute_intersection(&v, &v2, EdgeDir::Y);

                    let mut v3 = v;
                    v3.z += 1;
                    self.compute_intersection(&v, &v3, EdgeDir::Z);
                }
            }
        }
    }
}

const MIN_ABS_VERTEX_VALUE: f32 = 1e-6;

#[derive(Debug, Clone, Copy)]
struct Vertex {
    index: Vec3i,
    value: f32,
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
    fn from_voxel(voxel: Vec3i, grid: &SdfGrid) -> Option<Self> {
        let mut cube: Self = Default::default();

        let vertex_indices = CUBE_OFFSETS.map(|off| voxel + off);

        for i in 0..vertex_indices.len() {
            let index = vertex_indices[i];
            let mut value: f32 = match grid.at(&index) {
                Some(v) => (*v).into(),
                None => return None,
            };

            if value.abs() < MIN_ABS_VERTEX_VALUE {
                value = MIN_ABS_VERTEX_VALUE.copysign(value);
            }

            if value < 0.0 {
                cube.id |= 1 << i; // inside
            }

            cube.vertices[i] = Vertex { index, value };
        }

        Some(cube)
    }
}
