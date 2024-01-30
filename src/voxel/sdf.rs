use nalgebra::Vector3;

use crate::dynamic_vdb;

use super::{meshing::MarchingCubes, Grid, Leaf, Scalar, TreeNode, SMALL_SCALAR};

type DefaultGrid = dynamic_vdb!(Scalar, 5, 4, 3);

pub struct Sdf<T = DefaultGrid> where T: Grid<Value = Scalar> {
    pub grid: Box<T>,
}

impl<TGrid: Grid<Value = Scalar>> Sdf<TGrid> {
    ///
    /// Creates new SDF grid by evaluating given function on each grid point.
    /// Inside is negative.
    /// 
    pub fn from_fn<TFn: Fn(&Vector3<f32>) -> f32>(
        grid_size: usize,
        min: f32,
        max: f32,
        narrow_band_width: usize,
        func: TFn,
    ) -> Self {
        let mut grid = TGrid::empty(Vector3::zeros());

        let origin = Vector3::new(min, min, min);
        let spacing = (max - min) / grid_size as f32;
        let narrow_band_width = (narrow_band_width + 1) as f32 * spacing;

        for x in 0..grid_size {
            for y in 0..grid_size {
                for z in 0..grid_size {
                    let idx: Vector3<isize> = Vector3::new(x, y, z).cast();

                    let grid_point = origin + idx.cast() * spacing;
                    let value = func(&grid_point);

                    if value.abs() > narrow_band_width {
                        continue;
                    }

                    let val = Scalar { value };
                    grid.insert(&idx, val);
                }
            }
        }

        grid.prune(SMALL_SCALAR);

        Self { 
            grid
        }
    }
}

impl<T: Grid<Value = Scalar>> MarchingCubes for Sdf<T> {
    type Value = Scalar;

    fn cubes<TFn: FnMut(Vector3<isize>)> (&self, mut func: TFn) {
        self.grid.traverse_leafs(&mut |leaf| {
            let (origin, size) = match leaf {
                Leaf::Tile(t) => (t.origin, t.size),
                Leaf::Dense(n) => (n.origin(), n.size_t())
            };

            let max = origin + Vector3::new(size, size, size).cast();
            for x in origin.x..max.x {
                for y in origin.y..max.y {
                    for z in origin.z..max.z {
                        let v = Vector3::new(x, y, z);
                        func(v);
                    }
                }
            }
        });
    }

    #[inline]
    fn at(&self, idx: &Vector3<isize>) -> Option<f32> {
        self.grid.at(idx).map(|v| v.value)
    }
}


