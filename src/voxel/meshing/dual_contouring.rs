use crate::{
    helpers::aliases::{Vec3f, Vec3i, Vec3u},
    voxel::{utils::CUBE_OFFSETS, Grid, Leaf, Scalar, TreeNode},
};

pub struct DualContouringMesher<'a, T: Grid<Value = Scalar>> {
    vertices: Vec<Vec3f>,
    grid: &'a T,
}

impl<'a, T: Grid<Value = Scalar>> DualContouringMesher<'a, T> {
    pub fn new(grid: &'a T) -> Self {
        Self {
            vertices: Vec::new(),
            grid,
        }
    }

    pub fn mesh(&mut self) -> Vec<Vec3f> {
        self.vertices.clear();

        self.grid.traverse_leafs(&mut |leaf| {
            let (origin, size) = match leaf {
                Leaf::Tile(t) => (t.origin, t.size),
                Leaf::Dense(n) => (n.origin(), n.size_t()),
            };

            let max = origin + Vec3u::new(size, size, size).cast();
            for x in origin.x..max.x {
                for y in origin.y..max.y {
                    for z in origin.z..max.z {
                        let v = Vec3i::new(x, y, z);

                        self.handle_edge(v, v + Vec3i::new(1, 0, 0));
                        self.handle_edge(v, v + Vec3i::new(0, 1, 0));
                        self.handle_edge(v, v + Vec3i::new(0, 0, 1));
                    }
                }
            }
        });

        std::mem::take(&mut self.vertices)
    }

    fn handle_edge(&mut self, v1_idx: Vec3i, v2_idx: Vec3i) {
        let (v1, v2) = match (self.grid.at(&v1_idx), self.grid.at(&v2_idx)) {
            (Some(s), Some(e)) => (s, e),
            _ => return,
        };

        if v1.value * v2.value > 0.0 {
            return;
        }

        let flip = v2.value < 0.0;

        let edge_dir = (
            v1_idx.x == v2_idx.x,
            v1_idx.y == v2_idx.y,
            v1_idx.z == v2_idx.z,
        );

        // Cubes are listed in CCW order, aka normal == (v2 - v1)
        let cubes = match edge_dir {
            (true, true, false) => [
                Vec3i::new(v1_idx.x - 1, v1_idx.y, v1_idx.z),
                Vec3i::new(v1_idx.x - 1, v1_idx.y - 1, v1_idx.z),
                Vec3i::new(v1_idx.x, v1_idx.y - 1, v1_idx.z),
                v1_idx,
            ],
            (false, true, true) => [
                Vec3i::new(v1_idx.x, v1_idx.y - 1, v1_idx.z),
                Vec3i::new(v1_idx.x, v1_idx.y - 1, v1_idx.z - 1),
                Vec3i::new(v1_idx.x, v1_idx.y, v1_idx.z - 1),
                v1_idx,
            ],
            (true, false, true) => [
                Vec3i::new(v1_idx.x, v1_idx.y, v1_idx.z - 1),
                Vec3i::new(v1_idx.x - 1, v1_idx.y, v1_idx.z - 1),
                Vec3i::new(v1_idx.x - 1, v1_idx.y, v1_idx.z),
                v1_idx,
            ],
            _ => unreachable!("Invalid edge"),
        };

        let [mut v1, v2, mut v3, v4] = cubes.map(|c| self.vertex(c));

        if flip {
            std::mem::swap(&mut v1, &mut v3);
        }

        self.vertices.push(v3);
        self.vertices.push(v1);
        self.vertices.push(v2);

        self.vertices.push(v1);
        self.vertices.push(v3);
        self.vertices.push(v4);
    }

    fn vertex(&self, idx: Vec3i) -> Vec3f {
        let vertices: Vec<_> = CUBE_OFFSETS
            .iter()
            .map(|o| idx + o)
            .filter(|v| self.grid.at(v).is_some())
            .collect();

        let sum = vertices
            .iter()
            .fold(Vec3f::zeros(), |acc, v| acc + (*v).cast());
        sum / vertices.len() as f32
    }
}
