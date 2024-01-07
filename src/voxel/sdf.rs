use nalgebra::Vector3;

use crate::dynamic_vdb;

use super::{Grid, Scalar, meshing::{MarchingCubes, Vertex}, Leaf, TreeNode, Tile};

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
                    
                    //println!("{:?}", idx);

                    let val = Scalar { value };
                    grid.insert(&idx, val);
                }
            }
        }

        // for idx in box_indices(0, grid_size as isize) {
        //     let grid_point = origin
        //         + Vector3::new(
        //             idx.x as f32 * spacing,
        //             idx.y as f32 * spacing,
        //             idx.z as f32 * spacing,
        //         );
        //     let mut value = func(&grid_point);

        //     if value > narrow_band_width {
        //         continue;
        //     }

        //     if value < 0.0 && value < -narrow_band_width {
        //         value = f32::MIN;
        //     }
            
        //     println!("{:?}", idx);

        //     grid.insert(&idx, Scalar { value });
        // }

        // grid.prune(SMALL_SCALAR);

        Self { 
            grid
        }
    }
}

impl<T: Grid<Value = Scalar>> MarchingCubes for Sdf<T> {
    type Value = Scalar;

    #[inline]
    fn interpolate(&self, v1: &Vertex<Self::Value>, v2: &Vertex<Self::Value>) -> Vector3<f32> {
        let v1_val = v1.value.value.abs();
        let v2_val = v2.value.value.abs();
        let l = v1_val + v2_val;
        let dir = v2.index - v1.index;
        let t  = v1.index.cast() + dir.cast() * v1_val / l;

        t

        // (v1.index.cast() + v2.index.cast()) * 0.5
    }

    #[inline]
    fn is_inside(&self, value: &Self::Value) -> bool {
        value.value < 0.0
    }

    #[inline]
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
    fn at(&self, index: &Vector3<isize>) -> Option<Self::Value> {
        self.grid.at(index).cloned()
    }
}


